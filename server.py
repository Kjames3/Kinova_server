#!/usr/bin/env python3
"""
Remote server for Kinova Gen3 7-DOF arm, Intel RealSense D435i, and OAK-D Pro W.

Run on the desktop:   python server.py
Access from any host: http://10.12.140.145:8080
API docs:             http://10.12.140.145:8080/docs

Environment variable overrides:
  PORT         Server port              (default 8080)
  KINOVA_IP    Kinova arm IP address    (default 192.168.1.10)
  KINOVA_PORT  Kinova TCP port          (default 10000)
  KINOVA_USER  Kinova username          (default admin)
  KINOVA_PASS  Kinova password          (default admin)
"""

import asyncio
import io
import logging
import os
import threading
import time
from contextlib import asynccontextmanager
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from pydantic import BaseModel

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s — %(message)s",
)
log = logging.getLogger("kinova-server")

# ── Configuration ─────────────────────────────────────────────────────────────
SERVER_HOST = "0.0.0.0"
SERVER_PORT = int(os.environ.get("PORT", 8080))
KINOVA_IP   = os.environ.get("KINOVA_IP",   "192.168.1.10")
KINOVA_PORT = int(os.environ.get("KINOVA_PORT", 10000))
KINOVA_USER = os.environ.get("KINOVA_USER", "admin")
KINOVA_PASS = os.environ.get("KINOVA_PASS", "admin")

# ── Shared state ──────────────────────────────────────────────────────────────
_frame_lock = threading.Lock()
_rs_rgb_frame:    Optional[bytes] = None
_rs_depth_frame:  Optional[bytes] = None
_oak_rgb_frame:   Optional[bytes] = None
_oak_depth_frame: Optional[bytes] = None

_robot_lock  = threading.Lock()
_robot_state: Dict[str, Any] = {"joints": [0.0] * 7, "pose": {}, "connected": False}

_robot_base    = None   # kortex BaseClient, set by kinova thread
_robot_cyclic  = None   # kortex BaseCyclicClient, set by kinova thread
_stop_event    = threading.Event()

_status: Dict[str, bool] = {"realsense": False, "oakd": False, "kinova": False}


# ─────────────────────────────────────────────────────────────────────────────
# Device threads
# ─────────────────────────────────────────────────────────────────────────────

def _realsense_thread() -> None:
    global _rs_rgb_frame, _rs_depth_frame
    try:
        import pyrealsense2 as rs
    except ImportError:
        log.warning("pyrealsense2 not installed — RealSense D435i disabled")
        return

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)

    try:
        pipeline.start(cfg)
        _status["realsense"] = True
        log.info("RealSense D435i started")
    except Exception as exc:
        log.error(f"RealSense start failed: {exc}")
        return

    colorizer = rs.colorizer()
    try:
        while not _stop_event.is_set():
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_img = np.asanyarray(color_frame.get_data())
            depth_img = np.asanyarray(colorizer.colorize(depth_frame).get_data())

            _, rgb_buf   = cv2.imencode(".jpg", color_img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            _, depth_buf = cv2.imencode(".jpg", depth_img, [cv2.IMWRITE_JPEG_QUALITY, 80])

            with _frame_lock:
                _rs_rgb_frame   = rgb_buf.tobytes()
                _rs_depth_frame = depth_buf.tobytes()
    except Exception as exc:
        log.error(f"RealSense error: {exc}")
    finally:
        pipeline.stop()
        _status["realsense"] = False
        log.info("RealSense D435i stopped")


def _oakd_thread() -> None:
    global _oak_rgb_frame, _oak_depth_frame
    try:
        import depthai as dai
    except ImportError:
        log.warning("depthai not installed — OAK-D Pro W disabled")
        return

    pipeline = dai.Pipeline()

    # RGB camera (center)
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setPreviewSize(640, 480)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setFps(30)

    # Stereo mono cameras
    mono_l = pipeline.create(dai.node.MonoCamera)
    mono_l.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_l.setBoardSocket(dai.CameraBoardSocket.CAM_B)

    mono_r = pipeline.create(dai.node.MonoCamera)
    mono_r.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_r.setBoardSocket(dai.CameraBoardSocket.CAM_C)

    # Stereo depth
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetType.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    mono_l.out.link(stereo.left)
    mono_r.out.link(stereo.right)

    # Output queues
    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    cam_rgb.preview.link(xout_rgb.input)

    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_depth.setStreamName("depth")
    stereo.depth.link(xout_depth.input)

    try:
        with dai.Device(pipeline) as device:
            _status["oakd"] = True
            log.info("OAK-D Pro W started")
            q_rgb   = device.getOutputQueue("rgb",   maxSize=4, blocking=False)
            q_depth = device.getOutputQueue("depth", maxSize=4, blocking=False)

            while not _stop_event.is_set():
                pkt_rgb   = q_rgb.tryGet()
                pkt_depth = q_depth.tryGet()

                if pkt_rgb is not None:
                    _, buf = cv2.imencode(".jpg", pkt_rgb.getCvFrame(),
                                         [cv2.IMWRITE_JPEG_QUALITY, 80])
                    with _frame_lock:
                        _oak_rgb_frame = buf.tobytes()

                if pkt_depth is not None:
                    raw   = pkt_depth.getFrame()
                    norm  = cv2.normalize(raw, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
                    color = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
                    _, buf = cv2.imencode(".jpg", color, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    with _frame_lock:
                        _oak_depth_frame = buf.tobytes()

                time.sleep(0.01)
    except Exception as exc:
        log.error(f"OAK-D error: {exc}")
    finally:
        _status["oakd"] = False
        log.info("OAK-D Pro W stopped")


def _kinova_thread() -> None:
    global _robot_base, _robot_cyclic
    try:
        from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
        from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
        from kortex_api.autogen.messages.Session_pb2 import CreateSessionInfo
        from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
        from kortex_api.SessionManager import SessionManager
        from kortex_api.TransportClientTcp import TransportClientTcp
    except ImportError:
        log.warning("kortex_api not installed — Kinova arm disabled")
        return

    transport = TransportClientTcp()
    router    = RouterClient(transport, RouterClientSendOptions())
    try:
        transport.connect(KINOVA_IP, KINOVA_PORT)
    except Exception as exc:
        log.error(f"Kinova connect failed ({KINOVA_IP}:{KINOVA_PORT}): {exc}")
        return

    session_info = CreateSessionInfo()
    session_info.username = KINOVA_USER
    session_info.password = KINOVA_PASS
    session_info.session_inactivity_timeout = 600_000  # 10 min

    session_manager = None
    try:
        session_manager = SessionManager(router)
        session_manager.CreateSession(session_info)
    except Exception as exc:
        log.error(f"Kinova session failed: {exc}")
        transport.disconnect()
        return

    _robot_base   = BaseClient(router)
    _robot_cyclic = BaseCyclicClient(router)
    _status["kinova"] = True
    log.info(f"Kinova Gen3 connected at {KINOVA_IP}:{KINOVA_PORT}")

    try:
        while not _stop_event.is_set():
            try:
                feedback = _robot_cyclic.RefreshFeedback()
                pose     = _robot_base.GetMeasuredCartesianPose()
                joints   = [feedback.actuators[i].position for i in range(7)]
                with _robot_lock:
                    _robot_state.update({
                        "joints": joints,
                        "pose": {
                            "x":       pose.x,
                            "y":       pose.y,
                            "z":       pose.z,
                            "theta_x": pose.theta_x,
                            "theta_y": pose.theta_y,
                            "theta_z": pose.theta_z,
                        },
                        "connected": True,
                    })
            except Exception as exc:
                log.warning(f"Kinova poll error: {exc}")
                with _robot_lock:
                    _robot_state["connected"] = False
            time.sleep(0.05)  # 20 Hz
    finally:
        _status["kinova"] = False
        with _robot_lock:
            _robot_state["connected"] = False
        _robot_base   = None
        _robot_cyclic = None
        try:
            if session_manager:
                session_manager.CloseSession()
            transport.disconnect()
        except Exception:
            pass
        log.info("Kinova Gen3 disconnected")


# ─────────────────────────────────────────────────────────────────────────────
# MJPEG streaming helper
# ─────────────────────────────────────────────────────────────────────────────

async def _mjpeg_stream(get_frame_fn, fps: int = 30):
    """Async MJPEG generator. Yields multipart HTTP chunks at the given FPS."""
    boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
    interval  = 1.0 / fps
    last: Optional[bytes] = None
    while True:
        with _frame_lock:
            frame = get_frame_fn()
        if frame and frame is not last:
            last = frame
            yield boundary + frame + b"\r\n"
        await asyncio.sleep(interval)


# ─────────────────────────────────────────────────────────────────────────────
# Pydantic request models
# ─────────────────────────────────────────────────────────────────────────────

class JointAngles(BaseModel):
    angles: List[float]  # 7 values, degrees

class CartesianPose(BaseModel):
    x: float
    y: float
    z: float
    theta_x: float
    theta_y: float
    theta_z: float


# ─────────────────────────────────────────────────────────────────────────────
# FastAPI app
# ─────────────────────────────────────────────────────────────────────────────

@asynccontextmanager
async def lifespan(app: FastAPI):
    threads = [
        threading.Thread(target=_realsense_thread, daemon=True, name="realsense"),
        threading.Thread(target=_oakd_thread,      daemon=True, name="oakd"),
        threading.Thread(target=_kinova_thread,    daemon=True, name="kinova"),
    ]
    for t in threads:
        t.start()
    log.info(f"Server listening on http://{SERVER_HOST}:{SERVER_PORT}")
    yield
    _stop_event.set()


app = FastAPI(title="Kinova Robot Server", version="1.0.0", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# ── Status ────────────────────────────────────────────────────────────────────

@app.get("/api/status", summary="Device availability")
def get_status():
    return JSONResponse(_status.copy())


# ── Camera streams ────────────────────────────────────────────────────────────

@app.get("/api/cameras/realsense/rgb", summary="RealSense D435i live RGB stream (MJPEG)")
def stream_rs_rgb():
    if not _status["realsense"]:
        raise HTTPException(503, "RealSense not available")
    return StreamingResponse(
        _mjpeg_stream(lambda: _rs_rgb_frame),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )

@app.get("/api/cameras/realsense/depth", summary="RealSense D435i live depth stream (MJPEG, colorized)")
def stream_rs_depth():
    if not _status["realsense"]:
        raise HTTPException(503, "RealSense not available")
    return StreamingResponse(
        _mjpeg_stream(lambda: _rs_depth_frame),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )

@app.get("/api/cameras/realsense/snapshot", summary="RealSense D435i single RGB JPEG")
def snapshot_rs():
    with _frame_lock:
        frame = _rs_rgb_frame
    if not frame:
        raise HTTPException(503, "No frame available")
    return StreamingResponse(io.BytesIO(frame), media_type="image/jpeg")

@app.get("/api/cameras/oakd/rgb", summary="OAK-D Pro W live RGB stream (MJPEG)")
def stream_oak_rgb():
    if not _status["oakd"]:
        raise HTTPException(503, "OAK-D not available")
    return StreamingResponse(
        _mjpeg_stream(lambda: _oak_rgb_frame),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )

@app.get("/api/cameras/oakd/depth", summary="OAK-D Pro W live depth stream (MJPEG, colorized)")
def stream_oak_depth():
    if not _status["oakd"]:
        raise HTTPException(503, "OAK-D not available")
    return StreamingResponse(
        _mjpeg_stream(lambda: _oak_depth_frame),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )

@app.get("/api/cameras/oakd/snapshot", summary="OAK-D Pro W single RGB JPEG")
def snapshot_oak():
    with _frame_lock:
        frame = _oak_rgb_frame
    if not frame:
        raise HTTPException(503, "No frame available")
    return StreamingResponse(io.BytesIO(frame), media_type="image/jpeg")


# ── Robot state ───────────────────────────────────────────────────────────────

@app.get("/api/robot/state", summary="Joint angles and end-effector pose")
def get_robot_state():
    with _robot_lock:
        return JSONResponse(_robot_state.copy())


# ── Robot control ─────────────────────────────────────────────────────────────

@app.post("/api/robot/stop", summary="Stop current motion immediately")
def robot_stop():
    if not _robot_base:
        raise HTTPException(503, "Kinova not connected")
    try:
        _robot_base.Stop()
        return {"ok": True}
    except Exception as exc:
        raise HTTPException(500, str(exc))


@app.post("/api/robot/home", summary="Move to the pre-saved Home pose")
def robot_home():
    if not _robot_base:
        raise HTTPException(503, "Kinova not connected")
    try:
        from kortex_api.autogen.messages import Base_pb2
        req = Base_pb2.RequestedActionType()
        req.action_type = Base_pb2.REACH_JOINT_ANGLES
        for action in _robot_base.ReadAllActions(req).action_list:
            if action.name == "Home":
                _robot_base.ExecuteActionFromReference(action.handle)
                return {"ok": True}
        raise HTTPException(404, "Home action not found on robot")
    except HTTPException:
        raise
    except Exception as exc:
        raise HTTPException(500, str(exc))


@app.post("/api/robot/retract", summary="Move to the pre-saved Retract pose")
def robot_retract():
    if not _robot_base:
        raise HTTPException(503, "Kinova not connected")
    try:
        from kortex_api.autogen.messages import Base_pb2
        req = Base_pb2.RequestedActionType()
        req.action_type = Base_pb2.REACH_JOINT_ANGLES
        for action in _robot_base.ReadAllActions(req).action_list:
            if action.name == "Retract":
                _robot_base.ExecuteActionFromReference(action.handle)
                return {"ok": True}
        raise HTTPException(404, "Retract action not found on robot")
    except HTTPException:
        raise
    except Exception as exc:
        raise HTTPException(500, str(exc))


@app.post("/api/robot/joints", summary="Move to target joint angles (degrees)")
def robot_move_joints(body: JointAngles):
    if not _robot_base:
        raise HTTPException(503, "Kinova not connected")
    if len(body.angles) != 7:
        raise HTTPException(400, "Exactly 7 joint angles required (degrees)")
    try:
        from kortex_api.autogen.messages import Base_pb2
        action = Base_pb2.Action()
        action.name = "api_joint_move"
        action.application_data = ""
        for i, angle in enumerate(body.angles):
            jnt = action.reach_joint_angles.joint_angles.joint_angles.add()
            jnt.joint_identifier = i
            jnt.value = angle
        _robot_base.ExecuteAction(action)
        return {"ok": True}
    except Exception as exc:
        raise HTTPException(500, str(exc))


@app.post("/api/robot/pose", summary="Move to target Cartesian pose (meters + degrees)")
def robot_move_pose(body: CartesianPose):
    if not _robot_base:
        raise HTTPException(503, "Kinova not connected")
    try:
        from kortex_api.autogen.messages import Base_pb2
        action = Base_pb2.Action()
        action.name = "api_pose_move"
        action.application_data = ""
        tp = action.reach_pose.target_pose
        tp.x, tp.y, tp.z = body.x, body.y, body.z
        tp.theta_x, tp.theta_y, tp.theta_z = body.theta_x, body.theta_y, body.theta_z
        _robot_base.ExecuteAction(action)
        return {"ok": True}
    except Exception as exc:
        raise HTTPException(500, str(exc))


# ── WebSocket: real-time robot state at 20 Hz ─────────────────────────────────

@app.websocket("/ws/robot")
async def ws_robot(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            with _robot_lock:
                state = _robot_state.copy()
            await ws.send_json(state)
            await asyncio.sleep(0.05)
    except WebSocketDisconnect:
        pass


# ── Web dashboard ─────────────────────────────────────────────────────────────

_DASHBOARD = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Kinova Robot Server</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:'Courier New',monospace;background:#111;color:#ddd;padding:20px}
  h1{color:#4fc3f7;margin-bottom:6px;font-size:1.35em}
  .sub{color:#777;font-size:.8em;margin-bottom:18px}
  .sub a{color:#4fc3f7;text-decoration:none}
  .sub a:hover{text-decoration:underline}
  .status-bar{display:flex;gap:12px;flex-wrap:wrap;margin-bottom:18px}
  .badge{padding:4px 12px;border-radius:12px;font-size:.78em;display:flex;align-items:center;gap:6px}
  .badge.ok{background:#1b5e20;color:#a5d6a7}
  .badge.err{background:#b71c1c;color:#ef9a9a}
  .dot{width:8px;height:8px;border-radius:50%}
  .dot.ok{background:#66bb6a}.dot.err{background:#ef5350}
  .grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(320px,1fr));gap:14px}
  .card{background:#1e1e1e;border:1px solid #2e2e2e;border-radius:8px;padding:14px}
  .card h3{color:#90caf9;font-size:.85em;margin-bottom:10px;letter-spacing:.05em}
  img.stream{width:100%;border-radius:4px;background:#000;min-height:180px;object-fit:contain}
  table{width:100%;border-collapse:collapse;font-size:.82em}
  td{padding:3px 6px;border-bottom:1px solid #252525}
  td:last-child{text-align:right;color:#4fc3f7}
  .controls{margin-top:12px;display:flex;gap:8px;flex-wrap:wrap}
  button{padding:5px 14px;border-radius:4px;border:none;cursor:pointer;font-size:.8em;font-family:inherit}
  .bp{background:#1565c0;color:#fff}.bp:hover{background:#1976d2}
  .bd{background:#c62828;color:#fff}.bd:hover{background:#d32f2f}
</style>
</head>
<body>
<h1>Kinova Robot Server</h1>
<p class="sub">
  10.12.140.145:8080&nbsp;&nbsp;·&nbsp;&nbsp;
  <a href="/docs" target="_blank">API docs</a>&nbsp;&nbsp;·&nbsp;&nbsp;
  <a href="/api/status" target="_blank">status JSON</a>
</p>

<div id="bar" class="status-bar"></div>

<div class="grid">
  <div class="card">
    <h3>REALSENSE D435i — RGB</h3>
    <img class="stream" src="/api/cameras/realsense/rgb" alt="feed unavailable">
  </div>
  <div class="card">
    <h3>REALSENSE D435i — DEPTH</h3>
    <img class="stream" src="/api/cameras/realsense/depth" alt="feed unavailable">
  </div>
  <div class="card">
    <h3>OAK-D PRO W — RGB</h3>
    <img class="stream" src="/api/cameras/oakd/rgb" alt="feed unavailable">
  </div>
  <div class="card">
    <h3>OAK-D PRO W — DEPTH</h3>
    <img class="stream" src="/api/cameras/oakd/depth" alt="feed unavailable">
  </div>
  <div class="card">
    <h3>KINOVA GEN3 — JOINT ANGLES</h3>
    <table><tbody id="joints"></tbody></table>
    <div class="controls">
      <button class="bp" onclick="cmd('home')">Home</button>
      <button class="bp" onclick="cmd('retract')">Retract</button>
      <button class="bd" onclick="cmd('stop')">&#9632; STOP</button>
    </div>
  </div>
  <div class="card">
    <h3>END-EFFECTOR POSE</h3>
    <table><tbody id="pose"></tbody></table>
  </div>
</div>

<script>
const host = location.host;

async function pollStatus() {
  try {
    const s = await fetch('/api/status').then(r=>r.json());
    document.getElementById('bar').innerHTML = Object.entries(s)
      .map(([k,v])=>`<div class="badge ${v?'ok':'err'}"><div class="dot ${v?'ok':'err'}"></div>${k}</div>`)
      .join('');
  } catch {}
}
pollStatus(); setInterval(pollStatus, 4000);

function connectWs() {
  const ws = new WebSocket(`ws://${host}/ws/robot`);
  ws.onmessage = ({data}) => {
    const {joints, pose} = JSON.parse(data);
    if (joints)
      document.getElementById('joints').innerHTML =
        joints.map((v,i)=>`<tr><td>Joint ${i+1}</td><td>${v.toFixed(3)}&deg;</td></tr>`).join('');
    if (pose && pose.x !== undefined)
      document.getElementById('pose').innerHTML = [
        ['X',  pose.x.toFixed(4)+' m'],['Y', pose.y.toFixed(4)+' m'],['Z', pose.z.toFixed(4)+' m'],
        ['&theta;x', pose.theta_x.toFixed(3)+'&deg;'],
        ['&theta;y', pose.theta_y.toFixed(3)+'&deg;'],
        ['&theta;z', pose.theta_z.toFixed(3)+'&deg;'],
      ].map(([k,v])=>`<tr><td>${k}</td><td>${v}</td></tr>`).join('');
  };
  ws.onclose = () => setTimeout(connectWs, 2000);
}
connectWs();

async function cmd(action) {
  try { await fetch(`/api/robot/${action}`, {method:'POST'}); }
  catch(e) { alert('Error: '+e); }
}
</script>
</body>
</html>"""


@app.get("/", response_class=HTMLResponse, include_in_schema=False)
def dashboard():
    return HTMLResponse(_DASHBOARD)


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    uvicorn.run(app, host=SERVER_HOST, port=SERVER_PORT, log_level="info")
