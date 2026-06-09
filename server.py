#!/usr/bin/env python3
"""
Kinova Gen3 7-DOF · RealSense D435i · OAK-D Pro W — Remote Server v2
Run:  python server.py
Open: http://10.12.140.145:8080

Env vars:
  PORT, SERVER_PASSWORD, KINOVA_IP, KINOVA_PORT, KINOVA_USER, KINOVA_PASS
"""

import asyncio
import hashlib
import io
import logging
import os
import queue as stdlib_queue
import secrets
import subprocess
import sys
import tempfile
import threading
import time
from contextlib import asynccontextmanager
from datetime import datetime, timedelta
from typing import Any, Dict, List, Optional, Set

import cv2
import numpy as np
import uvicorn
from fastapi import Depends, FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from pydantic import BaseModel

# ── Logging ───────────────────────────────────────────────────────────────────
_log_queue: stdlib_queue.Queue = stdlib_queue.Queue(maxsize=1000)

class _WsBroadcastHandler(logging.Handler):
    def emit(self, record: logging.LogRecord):
        msg = f"[{record.levelname}] {self.format(record)}"
        try:
            _log_queue.put_nowait(msg)
        except stdlib_queue.Full:
            pass

_ws_handler = _WsBroadcastHandler()
_ws_handler.setFormatter(logging.Formatter("%(message)s"))
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s — %(message)s",
    handlers=[logging.StreamHandler(), _ws_handler],
)
log = logging.getLogger("kinova-server")

# ── Config ────────────────────────────────────────────────────────────────────
SERVER_HOST     = "0.0.0.0"
SERVER_PORT     = int(os.environ.get("PORT", 8080))
SERVER_PASSWORD = os.environ.get("SERVER_PASSWORD", "kinova")
KINOVA_IP       = os.environ.get("KINOVA_IP",   "192.168.1.10")
KINOVA_PORT_NUM = int(os.environ.get("KINOVA_PORT", 10000))
KINOVA_USER     = os.environ.get("KINOVA_USER", "admin")
KINOVA_PASS     = os.environ.get("KINOVA_PASS", "admin")

# ── Auth ──────────────────────────────────────────────────────────────────────
_sessions: Dict[str, datetime] = {}
_PWD_HASH = hashlib.sha256(SERVER_PASSWORD.encode()).hexdigest()

def _new_token() -> str:
    tok = secrets.token_hex(32)
    _sessions[tok] = datetime.utcnow() + timedelta(hours=24)
    return tok

def _valid_token(tok: str) -> bool:
    exp = _sessions.get(tok)
    if not exp:
        return False
    if datetime.utcnow() > exp:
        del _sessions[tok]
        return False
    return True

def _auth(request: Request, token: str = "") -> str:
    auth_header = request.headers.get("Authorization", "")
    t = token or (auth_header[7:] if auth_header.startswith("Bearer ") else "")
    if not _valid_token(t):
        raise HTTPException(401, "Unauthorized — invalid or missing token")
    return t

# ── Frame buffers (JPEG) & raw data for point cloud ──────────────────────────
_frame_lock       = threading.Lock()
_rs_rgb_frame:    Optional[bytes] = None
_rs_depth_frame:  Optional[bytes] = None
_oak_rgb_frame:   Optional[bytes] = None
_oak_depth_frame: Optional[bytes] = None

# Raw arrays for 3-D point cloud generation
_rs_color_raw:   Optional[np.ndarray] = None   # H×W×3 RGB uint8
_rs_depth_raw:   Optional[np.ndarray] = None   # H×W float32 metres
_rs_intrinsics:  Optional[Dict]       = None   # fx fy ppx ppy
_oak_color_raw:  Optional[np.ndarray] = None   # 640×480×3 RGB uint8
_oak_depth_raw:  Optional[np.ndarray] = None   # 640×400 uint16 mm
_oak_intrinsics: Dict = {"fx": 452.0, "fy": 452.0, "ppx": 320.0, "ppy": 200.0}

# ── Robot state ───────────────────────────────────────────────────────────────
_robot_lock  = threading.Lock()
_robot_state: Dict[str, Any] = {
    "joints":    [0.0] * 7,
    "pose":      {"x": 0, "y": 0, "z": 0, "theta_x": 0, "theta_y": 0, "theta_z": 0},
    "gripper":   0.0,
    "connected": False,
}
_robot_base   = None
_robot_cyclic = None
_stop_event   = threading.Event()
_status: Dict[str, bool] = {"realsense": False, "oakd": False, "kinova": False}


# ─────────────────────────────────────────────────────────────────────────────
# RealSense D435i
# ─────────────────────────────────────────────────────────────────────────────
def _realsense_thread():
    global _rs_rgb_frame, _rs_depth_frame, _rs_color_raw, _rs_depth_raw, _rs_intrinsics
    try:
        import pyrealsense2 as rs
    except ImportError:
        log.warning("pyrealsense2 not installed — RealSense disabled")
        return

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)

    try:
        profile = pipeline.start(cfg)
        _status["realsense"] = True
        log.info("RealSense D435i started")
    except Exception as exc:
        log.error(f"RealSense start failed: {exc}")
        return

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale  = depth_sensor.get_depth_scale()
    align        = rs.align(rs.stream.color)
    colorizer    = rs.colorizer()

    try:
        while not _stop_event.is_set():
            frames         = pipeline.wait_for_frames(timeout_ms=1000)
            aligned        = align.process(frames)
            c = aligned.get_color_frame()
            d = aligned.get_depth_frame()
            if not c or not d:
                continue

            color_img = np.asanyarray(c.get_data())           # BGR
            depth_vis = np.asanyarray(colorizer.colorize(d).get_data())
            _, cb = cv2.imencode(".jpg", color_img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            _, db = cv2.imencode(".jpg", depth_vis, [cv2.IMWRITE_JPEG_QUALITY, 80])

            # Raw data for point cloud
            depth_m = np.asanyarray(d.get_data()).astype(np.float32) * depth_scale
            intr    = c.profile.as_video_stream_profile().get_intrinsics()

            with _frame_lock:
                _rs_rgb_frame   = cb.tobytes()
                _rs_depth_frame = db.tobytes()
                _rs_color_raw   = color_img[:, :, ::-1].copy()   # BGR→RGB
                _rs_depth_raw   = depth_m
                _rs_intrinsics  = {"fx": intr.fx, "fy": intr.fy,
                                   "ppx": intr.ppx, "ppy": intr.ppy}
    except Exception as exc:
        log.error(f"RealSense error: {exc}")
    finally:
        pipeline.stop()
        _status["realsense"] = False
        log.info("RealSense stopped")


# ─────────────────────────────────────────────────────────────────────────────
# OAK-D Pro W  (uses ImageManip resize to avoid preview issues)
# ─────────────────────────────────────────────────────────────────────────────
def _oakd_thread():
    global _oak_rgb_frame, _oak_depth_frame, _oak_color_raw, _oak_depth_raw, _oak_intrinsics
    try:
        import depthai as dai
    except ImportError:
        log.warning("depthai not installed — OAK-D disabled")
        return

    def _build_pipeline(dai):
        p = dai.Pipeline()

        # Color: full ISP → ImageManip resize → XLink
        cam = p.create(dai.node.ColorCamera)
        cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(30)

        manip = p.create(dai.node.ImageManip)
        manip.initialConfig.setResize(640, 480)
        manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
        manip.setMaxOutputFrameSize(640 * 480 * 3)
        cam.isp.link(manip.inputImage)

        xrgb = p.create(dai.node.XLinkOut)
        xrgb.setStreamName("rgb")
        manip.out.link(xrgb.input)

        # Stereo depth
        ml = p.create(dai.node.MonoCamera)
        ml.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        ml.setBoardSocket(dai.CameraBoardSocket.CAM_B)

        mr = p.create(dai.node.MonoCamera)
        mr.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mr.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        stereo = p.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetType.HIGH_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(False)
        ml.out.link(stereo.left)
        mr.out.link(stereo.right)

        xdepth = p.create(dai.node.XLinkOut)
        xdepth.setStreamName("depth")
        stereo.depth.link(xdepth.input)

        return p

    try:
        with dai.Device(_build_pipeline(dai)) as device:
            cams = [c.name for c in device.getConnectedCameras()]
            log.info(f"OAK-D Pro W started — cameras: {cams}")
            _status["oakd"] = True

            q_rgb   = device.getOutputQueue("rgb",   maxSize=4, blocking=False)
            q_depth = device.getOutputQueue("depth", maxSize=4, blocking=False)

            # Read calibration for point cloud deprojection
            try:
                calib = device.readCalibration()
                m = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, 640, 400)
                _oak_intrinsics = {"fx": m[0][0], "fy": m[1][1],
                                   "ppx": m[0][2], "ppy": m[1][2]}
                log.info(f"OAK-D intrinsics: fx={m[0][0]:.1f} fy={m[1][1]:.1f}")
            except Exception as exc:
                log.warning(f"OAK-D calibration read failed (using defaults): {exc}")

            time.sleep(1.5)  # allow pipeline to warm up

            while not _stop_event.is_set():
                pkt_rgb   = q_rgb.tryGet()
                pkt_depth = q_depth.tryGet()

                if pkt_rgb is not None:
                    frame = pkt_rgb.getCvFrame()
                    _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    with _frame_lock:
                        _oak_rgb_frame = buf.tobytes()
                        _oak_color_raw = frame[:, :, ::-1].copy()  # BGR→RGB

                if pkt_depth is not None:
                    raw   = pkt_depth.getFrame()                    # uint16, mm, 640×400
                    norm  = cv2.normalize(raw, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
                    color = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
                    _, buf = cv2.imencode(".jpg", color, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    with _frame_lock:
                        _oak_depth_frame = buf.tobytes()
                        _oak_depth_raw   = raw.copy()

                if pkt_rgb is None and pkt_depth is None:
                    time.sleep(0.005)

    except Exception as exc:
        log.error(f"OAK-D error: {exc}")
    finally:
        _status["oakd"] = False
        log.info("OAK-D stopped")


# ─────────────────────────────────────────────────────────────────────────────
# Kinova Gen3
# ─────────────────────────────────────────────────────────────────────────────
def _kinova_thread():
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
        transport.connect(KINOVA_IP, KINOVA_PORT_NUM)
    except Exception as exc:
        log.error(f"Kinova connect failed ({KINOVA_IP}:{KINOVA_PORT_NUM}): {exc}")
        return

    session_info = CreateSessionInfo()
    session_info.username = KINOVA_USER
    session_info.password = KINOVA_PASS
    session_info.session_inactivity_timeout = 600_000

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
    log.info(f"Kinova Gen3 connected at {KINOVA_IP}:{KINOVA_PORT_NUM}")

    try:
        while not _stop_event.is_set():
            try:
                fb   = _robot_cyclic.RefreshFeedback()
                pose = _robot_base.GetMeasuredCartesianPose()
                joints = [fb.actuators[i].position for i in range(7)]
                try:
                    gripper = fb.interconnect.gripper_feedback.motor[0].position / 100.0
                except Exception:
                    gripper = 0.0
                with _robot_lock:
                    _robot_state.update({
                        "joints": joints,
                        "pose": {
                            "x": pose.x, "y": pose.y, "z": pose.z,
                            "theta_x": pose.theta_x,
                            "theta_y": pose.theta_y,
                            "theta_z": pose.theta_z,
                        },
                        "gripper": gripper,
                        "connected": True,
                    })
            except Exception as exc:
                log.warning(f"Kinova poll: {exc}")
                with _robot_lock:
                    _robot_state["connected"] = False
            time.sleep(0.05)
    finally:
        _status["kinova"] = False
        _robot_base   = None
        _robot_cyclic = None
        with _robot_lock:
            _robot_state["connected"] = False
        try:
            if session_manager:
                session_manager.CloseSession()
            transport.disconnect()
        except Exception:
            pass
        log.info("Kinova disconnected")


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────
async def _mjpeg_stream(get_fn, fps: int = 30):
    boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
    interval  = 1.0 / fps
    last: Optional[bytes] = None
    while True:
        with _frame_lock:
            frame = get_fn()
        if frame and frame is not last:
            last = frame
            yield boundary + frame + b"\r\n"
        await asyncio.sleep(interval)

def _compute_pc_rs(max_pts: int = 8000):
    """Return (Nx3 float32 XYZ metres, Nx3 uint8 RGB) for the latest RealSense frame."""
    with _frame_lock:
        depth = _rs_depth_raw
        color = _rs_color_raw
        intr  = _rs_intrinsics
    if depth is None or color is None or intr is None:
        return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint8)

    h, w  = depth.shape
    step  = max(1, int(np.sqrt(h * w / max_pts)))
    ys, xs = np.mgrid[0:h:step, 0:w:step]
    ys, xs = ys.ravel(), xs.ravel()
    z = depth[ys, xs]

    valid = (z > 0.15) & (z < 6.0)
    ys, xs, z = ys[valid], xs[valid], z[valid]

    x = (xs - intr["ppx"]) * z / intr["fx"]
    y = (ys - intr["ppy"]) * z / intr["fy"]

    pts    = np.column_stack([x, y, z]).astype(np.float32)
    colors = color[ys, xs].astype(np.uint8)
    return pts, colors


def _compute_pc_oak(max_pts: int = 8000):
    """Return (Nx3 float32 XYZ metres, Nx3 uint8 RGB) for the latest OAK-D frame."""
    with _frame_lock:
        depth_mm = _oak_depth_raw
        color    = _oak_color_raw
        intr     = _oak_intrinsics
    if depth_mm is None or color is None:
        return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint8)

    dh, dw = depth_mm.shape        # 640×400
    ch, cw = color.shape[:2]       # 640×480

    step  = max(1, int(np.sqrt(dh * dw / max_pts)))
    ys, xs = np.mgrid[0:dh:step, 0:dw:step]
    ys, xs = ys.ravel(), xs.ravel()
    z = depth_mm[ys, xs].astype(np.float32) / 1000.0   # mm → m

    valid = (z > 0.15) & (z < 6.0)
    ys, xs, z = ys[valid], xs[valid], z[valid]

    x = (xs - intr["ppx"]) * z / intr["fx"]
    y = (ys - intr["ppy"]) * z / intr["fy"]

    pts    = np.column_stack([x, y, z]).astype(np.float32)
    cx     = (xs * cw / dw).astype(int).clip(0, cw - 1)
    cy     = (ys * ch / dh).astype(int).clip(0, ch - 1)
    colors = color[cy, cx].astype(np.uint8)
    return pts, colors


_log_subscribers: Set[asyncio.Queue] = set()

async def _broadcast_logs():
    while True:
        batch = []
        while True:
            try:
                batch.append(_log_queue.get_nowait())
            except stdlib_queue.Empty:
                break
        if batch:
            for msg in batch:
                for sub in list(_log_subscribers):
                    try:
                        sub.put_nowait(msg)
                    except asyncio.QueueFull:
                        pass
        await asyncio.sleep(0.1)


# ─────────────────────────────────────────────────────────────────────────────
# Pydantic models
# ─────────────────────────────────────────────────────────────────────────────
class LoginReq(BaseModel):
    password: str

class JointAngles(BaseModel):
    angles: List[float]

class CartesianPose(BaseModel):
    x: float; y: float; z: float
    theta_x: float; theta_y: float; theta_z: float

class GripperCmd(BaseModel):
    position: float  # 0.0–1.0

class ScriptRun(BaseModel):
    code: str


# ─────────────────────────────────────────────────────────────────────────────
# FastAPI app
# ─────────────────────────────────────────────────────────────────────────────
@asynccontextmanager
async def lifespan(app: FastAPI):
    asyncio.create_task(_broadcast_logs())
    threads = [
        threading.Thread(target=_realsense_thread, daemon=True, name="realsense"),
        threading.Thread(target=_oakd_thread,      daemon=True, name="oakd"),
        threading.Thread(target=_kinova_thread,    daemon=True, name="kinova"),
    ]
    for t in threads:
        t.start()
    log.info(f"Server ready — http://0.0.0.0:{SERVER_PORT}")
    yield
    _stop_event.set()


app = FastAPI(title="Kinova Robot Server", version="2.0.0", lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

# ── Auth ──────────────────────────────────────────────────────────────────────
@app.post("/api/auth/login", include_in_schema=False)
def login(body: LoginReq):
    if hashlib.sha256(body.password.encode()).hexdigest() != _PWD_HASH:
        raise HTTPException(401, "Invalid password")
    return {"token": _new_token()}

@app.post("/api/auth/logout", include_in_schema=False)
def logout(t: str = Depends(_auth)):
    _sessions.pop(t, None)
    return {"ok": True}

# ── Status & system ───────────────────────────────────────────────────────────
@app.get("/api/status")
def get_status(t: str = Depends(_auth)):
    return JSONResponse(_status.copy())

@app.get("/api/system")
def get_system(t: str = Depends(_auth)):
    try:
        import psutil
        return {"cpu": psutil.cpu_percent(interval=0.1), "mem": psutil.virtual_memory().percent}
    except ImportError:
        return {"cpu": 0, "mem": 0}

# ── Cameras ───────────────────────────────────────────────────────────────────
@app.get("/api/cameras/realsense/rgb")
def rs_rgb(t: str = Depends(_auth)):
    if not _status["realsense"]:
        raise HTTPException(503, "RealSense unavailable")
    return StreamingResponse(_mjpeg_stream(lambda: _rs_rgb_frame),
                             media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/api/cameras/realsense/depth")
def rs_depth(t: str = Depends(_auth)):
    if not _status["realsense"]:
        raise HTTPException(503, "RealSense unavailable")
    return StreamingResponse(_mjpeg_stream(lambda: _rs_depth_frame),
                             media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/api/cameras/realsense/snapshot")
def rs_snap(t: str = Depends(_auth)):
    with _frame_lock:
        f = _rs_rgb_frame
    if not f:
        raise HTTPException(503, "No frame")
    return StreamingResponse(io.BytesIO(f), media_type="image/jpeg")

@app.get("/api/cameras/oakd/rgb")
def oak_rgb(t: str = Depends(_auth)):
    if not _status["oakd"]:
        raise HTTPException(503, "OAK-D unavailable")
    return StreamingResponse(_mjpeg_stream(lambda: _oak_rgb_frame),
                             media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/api/cameras/oakd/depth")
def oak_depth(t: str = Depends(_auth)):
    if not _status["oakd"]:
        raise HTTPException(503, "OAK-D unavailable")
    return StreamingResponse(_mjpeg_stream(lambda: _oak_depth_frame),
                             media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/api/cameras/oakd/snapshot")
def oak_snap(t: str = Depends(_auth)):
    with _frame_lock:
        f = _oak_rgb_frame
    if not f:
        raise HTTPException(503, "No frame")
    return StreamingResponse(io.BytesIO(f), media_type="image/jpeg")

# ── Robot state ───────────────────────────────────────────────────────────────
@app.get("/api/robot/state")
def robot_state(t: str = Depends(_auth)):
    with _robot_lock:
        return JSONResponse(_robot_state.copy())

# ── Robot control ─────────────────────────────────────────────────────────────
def _need_robot():
    if not _robot_base:
        raise HTTPException(503, "Kinova not connected")

@app.post("/api/robot/stop")
def robot_stop(t: str = Depends(_auth)):
    _need_robot()
    try:
        _robot_base.Stop()
        log.info("Robot stopped")
        return {"ok": True}
    except Exception as e:
        raise HTTPException(500, str(e))

@app.post("/api/robot/home")
def robot_home(t: str = Depends(_auth)):
    _need_robot()
    try:
        from kortex_api.autogen.messages import Base_pb2
        req = Base_pb2.RequestedActionType()
        req.action_type = Base_pb2.REACH_JOINT_ANGLES
        for a in _robot_base.ReadAllActions(req).action_list:
            if a.name == "Home":
                _robot_base.ExecuteActionFromReference(a.handle)
                log.info("Moving to Home")
                return {"ok": True}
        raise HTTPException(404, "Home action not found")
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(500, str(e))

@app.post("/api/robot/retract")
def robot_retract(t: str = Depends(_auth)):
    _need_robot()
    try:
        from kortex_api.autogen.messages import Base_pb2
        req = Base_pb2.RequestedActionType()
        req.action_type = Base_pb2.REACH_JOINT_ANGLES
        for a in _robot_base.ReadAllActions(req).action_list:
            if a.name == "Retract":
                _robot_base.ExecuteActionFromReference(a.handle)
                log.info("Moving to Retract")
                return {"ok": True}
        raise HTTPException(404, "Retract action not found")
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(500, str(e))

@app.post("/api/robot/vertical")
def robot_vertical(t: str = Depends(_auth)):
    _need_robot()
    try:
        from kortex_api.autogen.messages import Base_pb2
        req = Base_pb2.RequestedActionType()
        req.action_type = Base_pb2.REACH_JOINT_ANGLES
        for a in _robot_base.ReadAllActions(req).action_list:
            if a.name.lower() in ("vertical", "zero"):
                _robot_base.ExecuteActionFromReference(a.handle)
                log.info("Moving to Vertical")
                return {"ok": True}
        # Fallback: move to hardcoded vertical joint config
        action = Base_pb2.Action()
        action.name = "api_vertical"
        action.application_data = ""
        for i, deg in enumerate([0, -90, 0, -90, 0, 0, 0]):
            j = action.reach_joint_angles.joint_angles.joint_angles.add()
            j.joint_identifier = i
            j.value = deg
        _robot_base.ExecuteAction(action)
        log.info("Moving to Vertical (hardcoded)")
        return {"ok": True}
    except Exception as e:
        raise HTTPException(500, str(e))

@app.post("/api/robot/joints")
def robot_joints(body: JointAngles, t: str = Depends(_auth)):
    _need_robot()
    if len(body.angles) != 7:
        raise HTTPException(400, "Need 7 joint angles (degrees)")
    try:
        from kortex_api.autogen.messages import Base_pb2
        action = Base_pb2.Action()
        action.name = "api_joints"
        action.application_data = ""
        for i, v in enumerate(body.angles):
            j = action.reach_joint_angles.joint_angles.joint_angles.add()
            j.joint_identifier = i
            j.value = v
        _robot_base.ExecuteAction(action)
        log.info(f"Joint move: {[round(a, 1) for a in body.angles]}")
        return {"ok": True}
    except Exception as e:
        raise HTTPException(500, str(e))

@app.post("/api/robot/pose")
def robot_pose(body: CartesianPose, t: str = Depends(_auth)):
    _need_robot()
    try:
        from kortex_api.autogen.messages import Base_pb2
        action = Base_pb2.Action()
        action.name = "api_pose"
        action.application_data = ""
        tp = action.reach_pose.target_pose
        tp.x, tp.y, tp.z = body.x, body.y, body.z
        tp.theta_x, tp.theta_y, tp.theta_z = body.theta_x, body.theta_y, body.theta_z
        _robot_base.ExecuteAction(action)
        log.info(f"Pose move: ({body.x:.3f}, {body.y:.3f}, {body.z:.3f})")
        return {"ok": True}
    except Exception as e:
        raise HTTPException(500, str(e))

@app.post("/api/robot/gripper")
def robot_gripper(body: GripperCmd, t: str = Depends(_auth)):
    _need_robot()
    pos = max(0.0, min(1.0, body.position))
    try:
        from kortex_api.autogen.messages import Base_pb2
        gc = Base_pb2.GripperCommand()
        gc.mode = Base_pb2.GRIPPER_POSITION
        f = gc.gripper.finger.add()
        f.finger_identifier = 1
        f.value = pos
        _robot_base.SendGripperCommand(gc)
        log.info(f"Gripper → {int(pos*100)}%")
        return {"ok": True}
    except Exception as e:
        raise HTTPException(500, str(e))

# ── WebSocket: robot state 20 Hz ──────────────────────────────────────────────
@app.websocket("/ws/robot")
async def ws_robot(ws: WebSocket, token: str = ""):
    if not _valid_token(token):
        await ws.close(code=4001)
        return
    await ws.accept()
    try:
        while True:
            with _robot_lock:
                state = _robot_state.copy()
            await ws.send_json(state)
            await asyncio.sleep(0.05)
    except WebSocketDisconnect:
        pass

# ── WebSocket: live logs ──────────────────────────────────────────────────────
@app.websocket("/ws/logs")
async def ws_logs(ws: WebSocket, token: str = ""):
    if not _valid_token(token):
        await ws.close(code=4001)
        return
    await ws.accept()
    q: asyncio.Queue = asyncio.Queue(maxsize=300)
    _log_subscribers.add(q)
    try:
        while True:
            msg = await q.get()
            await ws.send_text(msg)
    except WebSocketDisconnect:
        pass
    finally:
        _log_subscribers.discard(q)

# ── WebSocket: script terminal ────────────────────────────────────────────────
@app.websocket("/ws/terminal")
async def ws_terminal(ws: WebSocket, token: str = ""):
    if not _valid_token(token):
        await ws.close(code=4001)
        return
    await ws.accept()
    await ws.send_text("[INFO] Terminal ready. Send JSON: {\"code\": \"...your Python...\"}\n")
    await ws.send_text(f"[INFO] KINOVA_IP={KINOVA_IP}  KINOVA_PORT={KINOVA_PORT_NUM}\n")
    try:
        while True:
            raw = await ws.receive_text()
            try:
                import json
                code = json.loads(raw).get("code", "")
            except Exception:
                code = raw

            if not code.strip():
                continue

            await ws.send_text(f">>> Running ({len(code.splitlines())} lines)...\n")
            with tempfile.NamedTemporaryFile(
                mode="w", suffix=".py", delete=False, dir=tempfile.gettempdir()
            ) as f:
                f.write(code)
                tmpfile = f.name

            try:
                proc = await asyncio.create_subprocess_exec(
                    sys.executable, tmpfile,
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.STDOUT,
                    env={**os.environ,
                         "KINOVA_IP": KINOVA_IP,
                         "KINOVA_PORT": str(KINOVA_PORT_NUM),
                         "KINOVA_USER": KINOVA_USER,
                         "KINOVA_PASS": KINOVA_PASS},
                )
                try:
                    async for line in proc.stdout:
                        await ws.send_text(line.decode(errors="replace"))
                    await asyncio.wait_for(proc.wait(), timeout=120.0)
                    await ws.send_text(f"\n[Exit {proc.returncode}]\n")
                except asyncio.TimeoutError:
                    proc.kill()
                    await ws.send_text("\n[Timeout — script exceeded 120 s]\n")
            finally:
                try:
                    os.unlink(tmpfile)
                except OSError:
                    pass
    except WebSocketDisconnect:
        pass


# ── WebSocket: point cloud streams ───────────────────────────────────────────
import struct as _struct

@app.websocket("/ws/pointcloud/realsense")
async def ws_pc_realsense(ws: WebSocket, token: str = ""):
    if not _valid_token(token):
        await ws.close(code=4001)
        return
    await ws.accept()
    try:
        while True:
            pts, colors = await asyncio.get_event_loop().run_in_executor(
                None, _compute_pc_rs)
            n      = len(pts)
            header = _struct.pack("<I", n)
            await ws.send_bytes(header + pts.tobytes() + colors.tobytes())
            await asyncio.sleep(0.25)   # 4 Hz
    except WebSocketDisconnect:
        pass

@app.websocket("/ws/pointcloud/oakd")
async def ws_pc_oakd(ws: WebSocket, token: str = ""):
    if not _valid_token(token):
        await ws.close(code=4001)
        return
    await ws.accept()
    try:
        while True:
            pts, colors = await asyncio.get_event_loop().run_in_executor(
                None, _compute_pc_oak)
            n      = len(pts)
            header = _struct.pack("<I", n)
            await ws.send_bytes(header + pts.tobytes() + colors.tobytes())
            await asyncio.sleep(0.25)   # 4 Hz
    except WebSocketDisconnect:
        pass


# ─────────────────────────────────────────────────────────────────────────────
# Dashboard HTML
# ─────────────────────────────────────────────────────────────────────────────
_DASHBOARD = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Kinova Gen3 Server</title>
<!-- Three.js r128 (last version with global OrbitControls.js include) -->
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Courier New',monospace;background:#111;color:#d0d0d0;overflow:hidden;height:100vh;display:flex;flex-direction:column}

/* ── Header ── */
header{flex-shrink:0;height:38px;background:#0c0c0c;border-bottom:1px solid #252525;display:flex;align-items:center;padding:0 14px;gap:16px}
.h-title{color:#4fc3f7;font-size:.8em;font-weight:bold;letter-spacing:.12em}
.h-sub{color:#555;font-size:.72em;letter-spacing:.06em}
.h-metrics{display:flex;gap:14px;font-size:.72em;color:#666;margin-left:auto}
.h-metric span{color:#90caf9}
.h-status{font-size:.7em;padding:2px 10px;border-radius:10px;cursor:pointer}
.h-status.online{background:#1b3a1b;color:#81c784}
.h-status.offline{background:#3a1b1b;color:#e57373}

/* ── Layout ── */
.layout{flex:1;display:flex;overflow:hidden;min-height:0}

/* ── Sidebars ── */
.sl,.sr{flex-shrink:0;overflow-y:auto;padding:10px 12px}
.sl{width:228px;border-right:1px solid #1e1e1e}
.sr{width:274px;border-left:1px solid #1e1e1e;display:flex;flex-direction:column;gap:0}

/* ── Cameras main ── */
.cams{flex:1;overflow-y:auto;padding:8px 10px;display:flex;flex-direction:column;gap:8px}

/* ── Section labels ── */
.lbl{font-size:.62em;color:#555;letter-spacing:.12em;display:block;margin:10px 0 5px;border-bottom:1px solid #1e1e1e;padding-bottom:3px}
.lbl:first-child{margin-top:0}

/* ── Buttons ── */
.btn{padding:4px 10px;border:none;border-radius:3px;cursor:pointer;font-size:.74em;font-family:inherit;letter-spacing:.04em}
.bp{background:#0d47a1;color:#90caf9}.bp:hover{background:#1565c0}
.bg{background:#1b3a1b;color:#81c784}.bg:hover{background:#2e7d32}
.bn{background:#1e1e1e;color:#888;border:1px solid #2a2a2a}.bn:hover{background:#2a2a2a;color:#bbb}
.br{background:#3a0d0d;color:#ef9a9a}.br:hover{background:#c62828;color:#fff}
.btn-fw{width:100%;margin-top:5px;padding:6px}

/* ── Joint rows ── */
.jrow{display:flex;align-items:center;gap:5px;margin-bottom:3px}
.jlbl{font-size:.68em;color:#666;width:18px;flex-shrink:0}
.jrow input[type=range]{flex:1;accent-color:#4fc3f7;height:3px;cursor:pointer}
.jval{font-size:.7em;color:#4fc3f7;width:44px;text-align:right;flex-shrink:0}
.jrow input[type=range]::-webkit-slider-thumb{width:12px;height:12px}

/* ── Pose grid ── */
.pgrid{display:grid;grid-template-columns:26px 1fr 26px 1fr;gap:3px 5px;align-items:center;margin-bottom:5px}
.pgrid label{font-size:.68em;color:#666;text-align:right}
.pgrid input{background:#181818;border:1px solid #2a2a2a;color:#bbb;padding:2px 4px;border-radius:2px;font-size:.72em;font-family:inherit;width:100%}
.pgrid input:focus{outline:none;border-color:#1565c0}

/* ── Gripper ── */
.grip-row{display:flex;align-items:center;gap:6px;margin-bottom:5px}
.grip-row input[type=range]{flex:1;accent-color:#4fc3f7}
.grip-pct{font-size:.72em;color:#4fc3f7;width:32px;text-align:right}

/* ── Quick button rows ── */
.qrow{display:flex;gap:4px;margin-top:4px;flex-wrap:wrap}

/* ── Camera panels ── */
.cpanel{background:#181818;border:1px solid #222;border-radius:5px;overflow:hidden}
.chdr{background:#0e0e0e;padding:6px 10px;display:flex;align-items:center;gap:7px;font-size:.72em;color:#888;letter-spacing:.06em}
.chdr .ctitle{flex:1}
.sdot{width:7px;height:7px;border-radius:50%;background:#333;flex-shrink:0}
.sdot.on{background:#4caf50}
.cbody{display:flex;background:#000;height:200px}
.cbody img{object-fit:contain;background:#000}
.cam-rgb{flex:1;border-right:1px solid #111}
.cam-depth{width:38%}
.cam-off{flex:1;display:flex;align-items:center;justify-content:center;font-size:.72em;color:#333;letter-spacing:.08em}

/* ── Right sidebar sections ── */
.rs-sec{padding:10px 12px;border-bottom:1px solid #1a1a1a;flex-shrink:0}
.rs-sec.grow{flex:1;display:flex;flex-direction:column;min-height:0;overflow:hidden}

/* ── Command ── */
.cmd-row{display:flex;gap:5px;margin-bottom:5px}
.cmd-row input{flex:1;background:#181818;border:1px solid #2a2a2a;color:#bbb;padding:4px 7px;border-radius:3px;font-size:.74em;font-family:inherit}
.cmd-row input:focus{outline:none;border-color:#1565c0}

/* ── Log ── */
.log-box{flex:1;overflow-y:auto;background:#0a0a0a;border:1px solid #1a1a1a;border-radius:3px;padding:5px 7px;font-size:.68em;min-height:0}
.ll-i{color:#90caf9}.ll-w{color:#ffb74d}.ll-e{color:#ef5350}.ll-d{color:#555}

/* ── Terminal ── */
.term-input{width:100%;background:#0a0a0a;border:1px solid #1a1a1a;border-radius:3px 3px 0 0;color:#ccc;font-family:'Courier New',monospace;font-size:.72em;padding:6px;resize:none;height:90px}
.term-input:focus{outline:none;border-color:#1565c0}
.term-out{flex:1;overflow-y:auto;background:#0a0a0a;border:1px solid #1a1a1a;border-top:none;border-radius:0 0 3px 3px;padding:5px 7px;font-size:.68em;color:#4fc3f7;font-family:monospace;min-height:0}

/* ── Login overlay ── */
#lo{position:fixed;inset:0;background:#0a0a0ae8;display:flex;align-items:center;justify-content:center;z-index:999}
.lcard{background:#1a1a1a;border:1px solid #2e2e2e;border-radius:8px;padding:32px 28px;width:300px;text-align:center}
.lcard h2{color:#4fc3f7;font-size:1em;margin-bottom:6px;letter-spacing:.08em}
.lcard p{color:#666;font-size:.72em;margin-bottom:18px}
.lcard input{width:100%;background:#111;border:1px solid #333;color:#ddd;padding:8px 10px;border-radius:3px;font-family:inherit;font-size:.85em;margin-bottom:10px;text-align:center;letter-spacing:.1em}
.lcard input:focus{outline:none;border-color:#1565c0}
.lerr{color:#ef5350;font-size:.75em;margin-top:6px;min-height:16px}

/* ── Speed ── */
.spd-row{display:flex;align-items:center;gap:8px}
.spd-row input{flex:1;accent-color:#4fc3f7}
.spd-val{font-size:.72em;color:#4fc3f7;width:32px;text-align:right}

::-webkit-scrollbar{width:4px;height:4px}
::-webkit-scrollbar-track{background:#111}
::-webkit-scrollbar-thumb{background:#2a2a2a;border-radius:2px}
</style>
</head>
<body>

<!-- Login overlay -->
<div id="lo">
  <div class="lcard">
    <h2>KINOVA ROBOT SERVER</h2>
    <p>10.12.140.145:8080</p>
    <input type="password" id="lpass" placeholder="PASSWORD" onkeydown="if(event.key==='Enter')doLogin()">
    <button class="btn bp btn-fw" onclick="doLogin()">LOGIN</button>
    <div class="lerr" id="lerr"></div>
  </div>
</div>

<!-- App -->
<div id="app" style="display:none;flex-direction:column;height:100vh">

  <!-- Header -->
  <header>
    <div class="h-title">KINOVA GEN3</div>
    <div class="h-sub">7DOF &middot; 2F-140</div>
    <div class="h-metrics">
      <div class="h-metric">CPU <span id="hCpu">--</span>%</div>
      <div class="h-metric">MEM <span id="hMem">--</span>%</div>
    </div>
    <div id="hStatus" class="h-status offline">OFFLINE</div>
    <button class="btn bn" style="font-size:.65em" onclick="doLogout()">LOGOUT</button>
  </header>

  <!-- Main layout -->
  <div class="layout">

    <!-- ── Left sidebar ── -->
    <div class="sl">

      <span class="lbl">ROBOT ARM</span>
      <div class="qrow">
        <button class="btn bp" id="btnConnect" onclick="connectRobot()" style="flex:1">CONNECT</button>
      </div>

      <span class="lbl">JOINTS</span>
      <div id="jsliders"></div>
      <button class="btn bp btn-fw" onclick="sendJoints()">SEND TO ROBOT</button>
      <button class="btn bn btn-fw" onclick="syncJointsFromRobot()" style="margin-top:3px;font-size:.68em">SYNC FROM ROBOT</button>

      <span class="lbl">GRIPPER <span id="gPct" style="color:#4fc3f7">0%</span></span>
      <div class="grip-row">
        <input type="range" id="gSlider" min="0" max="100" value="0"
          onmousedown="activeSl='g'" onmouseup="activeSl=null"
          oninput="document.getElementById('gPct').textContent=this.value+'%'">
      </div>
      <div class="qrow">
        <button class="btn bg" style="flex:1" onclick="setGripper(0)">OPEN</button>
        <button class="btn bn" style="flex:1" onclick="setGripper(100)">CLOSE</button>
        <button class="btn bp" onclick="sendGripper()">SET</button>
      </div>

      <span class="lbl">END EFFECTOR POSE</span>
      <div class="pgrid">
        <label>X</label><input id="pX" type="number" step="0.001" value="0.400">
        <label>Y</label><input id="pY" type="number" step="0.001" value="0.000">
        <label>Z</label><input id="pZ" type="number" step="0.001" value="0.400">
        <label>RX</label><input id="pRX" type="number" step="0.1" value="180">
        <label>RY</label><input id="pRY" type="number" step="0.1" value="0">
        <label>RZ</label><input id="pRZ" type="number" step="0.1" value="90">
      </div>
      <button class="btn bp btn-fw" onclick="moveToPose()">MOVE TO POSE</button>

      <span class="lbl">SPEED <span id="spdVal" style="color:#4fc3f7">50%</span></span>
      <div class="spd-row">
        <input type="range" id="spdSlider" min="0" max="100" value="50"
          oninput="document.getElementById('spdVal').textContent=this.value+'%'">
      </div>

      <div class="qrow" style="margin-top:10px">
        <button class="btn bn" style="flex:1" onclick="apiPost('/api/robot/home')">HOME</button>
        <button class="btn bn" style="flex:1" onclick="apiPost('/api/robot/retract')">RETRACT</button>
        <button class="btn bn" style="flex:1" onclick="apiPost('/api/robot/vertical')">VERTICAL</button>
      </div>
      <button class="btn br btn-fw" style="margin-top:4px" onclick="apiPost('/api/robot/stop')">&#9632; EMERGENCY STOP</button>

    </div><!-- /sl -->

    <!-- ── Cameras ── -->
    <div class="cams">

      <!-- RealSense D435i -->
      <div class="cpanel">
        <div class="chdr">
          <span class="ctitle">GLOBAL D435i — REALSENSE</span>
          <div class="sdot" id="dot-realsense"></div>
          <button class="btn bn" onclick="toggleDepth('rs',this)">DEPTH ON</button>
          <button class="btn bg" onclick="startCam('rs')">START</button>
        </div>
        <div class="cbody" id="rsBody">
          <div class="cam-off">CLICK START TO STREAM</div>
        </div>
      </div>

      <!-- OAK-D Pro W -->
      <div class="cpanel">
        <div class="chdr">
          <span class="ctitle">OAK-D PRO W — STEREO</span>
          <div class="sdot" id="dot-oakd"></div>
          <button class="btn bn" onclick="toggleDepth('oak',this)">DEPTH ON</button>
          <button class="btn bg" onclick="startCam('oak')">START</button>
        </div>
        <div class="cbody" id="oakBody">
          <div class="cam-off">CLICK START TO STREAM</div>
        </div>
      </div>

      <!-- ── 3-D Viewers ── -->
      <div style="display:flex;gap:8px;flex-shrink:0;min-height:0">

        <!-- Kinova FK arm viewer -->
        <div class="cpanel" style="flex:1;min-width:0">
          <div class="chdr">
            <span class="ctitle">KINOVA GEN3 — FORWARD KINEMATICS</span>
            <div class="sdot on" style="background:#4fc3f7"></div>
          </div>
          <div id="armCanvas" style="height:340px;background:#0a0a0a;position:relative">
            <div style="position:absolute;bottom:5px;left:8px;font-size:.6em;color:#333;pointer-events:none">
              drag·orbit &nbsp; right-drag·pan &nbsp; scroll·zoom
            </div>
          </div>
        </div>

        <!-- Point cloud viewer -->
        <div class="cpanel" style="flex:1;min-width:0">
          <div class="chdr">
            <span class="ctitle">POINT CLOUD</span>
            <div class="sdot" id="dot-pc"></div>
            <select id="pcSource" style="background:#111;border:1px solid #2a2a2a;color:#aaa;font-size:.7em;padding:2px 5px;border-radius:2px">
              <option value="realsense">RealSense D435i</option>
              <option value="oakd">OAK-D Pro W</option>
            </select>
            <button class="btn bg" id="btnPcStream" onclick="togglePcStream()">&#9654; STREAM</button>
          </div>
          <div id="pcCanvas" style="height:340px;background:#0a0a0a;position:relative">
            <div id="pcOverlay" style="position:absolute;inset:0;display:flex;align-items:center;justify-content:center;font-size:.72em;color:#444;pointer-events:none">
              Press STREAM to start
            </div>
          </div>
        </div>

      </div><!-- /3d viewers -->

    </div><!-- /cams -->

    <!-- ── Right sidebar ── -->
    <div class="sr">

      <!-- Command -->
      <div class="rs-sec">
        <span class="lbl" style="margin-top:0">COMMAND</span>
        <div class="cmd-row">
          <input id="cmdIn" type="text" placeholder="home / open / close / stop ..." onkeydown="if(event.key==='Enter')sendCmd()">
          <button class="btn bp" onclick="sendCmd()">SEND</button>
        </div>
        <div class="qrow">
          <button class="btn bn" onclick="apiPost('/api/robot/home')">HOME</button>
          <button class="btn bg" onclick="setGripper(0)">OPEN</button>
          <button class="btn bn" onclick="setGripper(100)">CLOSE</button>
          <button class="btn br" onclick="apiPost('/api/robot/stop')">STOP</button>
        </div>
      </div>

      <!-- Log -->
      <div class="rs-sec grow" style="flex:1.2">
        <span class="lbl" style="margin-top:0;display:flex;align-items:center;justify-content:space-between">
          LOG
          <button class="btn bn" style="font-size:.6em;padding:1px 6px" onclick="document.getElementById('logBox').innerHTML=''">CLR</button>
        </span>
        <div id="logBox" class="log-box"></div>
      </div>

      <!-- Terminal -->
      <div class="rs-sec grow" style="flex:1">
        <span class="lbl" style="margin-top:0;display:flex;align-items:center;justify-content:space-between">
          PYTHON TERMINAL
          <button class="btn bp" style="font-size:.6em;padding:1px 8px" onclick="runScript()">&#9654; RUN</button>
        </span>
        <textarea id="termIn" class="term-input" placeholder="# Python script (runs on the server)&#10;print('hello from kinova')"></textarea>
        <div id="termOut" class="term-out"></div>
      </div>

    </div><!-- /sr -->

  </div><!-- /layout -->
</div><!-- /app -->

<script>
const TOK_KEY = 'kv2tok';
let TOKEN = localStorage.getItem(TOK_KEY) || '';
let wsRobot = null, wsLogs = null, wsTerm = null;
let activeSl = null;
let poseEditing = false;
let depthOn = {rs: true, oak: true};
let robotJoints = [0,0,0,0,0,0,0];

// ── Auth ──────────────────────────────────────────────────────────────────────
async function doLogin() {
  const pass = document.getElementById('lpass').value;
  const r = await fetch('/api/auth/login', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({password: pass})
  });
  if (r.ok) {
    TOKEN = (await r.json()).token;
    localStorage.setItem(TOK_KEY, TOKEN);
    showApp();
  } else {
    document.getElementById('lerr').textContent = 'Invalid password';
  }
}
function doLogout() {
  fetch(`/api/auth/logout?token=${TOKEN}`, {method:'POST'});
  localStorage.removeItem(TOK_KEY);
  location.reload();
}
async function checkAuth() {
  if (!TOKEN) { return; }
  const r = await fetch(`/api/status?token=${TOKEN}`);
  if (r.status === 401) { TOKEN = ''; localStorage.removeItem(TOK_KEY); return; }
  showApp();
}
function showApp() {
  document.getElementById('lo').style.display = 'none';
  const app = document.getElementById('app');
  app.style.display = 'flex';
  initJoints();
  connectRobotWs();
  connectLogsWs();
  connectTermWs();
  pollStatus();
  pollSystem();
  setInterval(pollStatus, 4000);
  setInterval(pollSystem, 3000);
}

// ── Joints ────────────────────────────────────────────────────────────────────
function initJoints() {
  const c = document.getElementById('jsliders');
  c.innerHTML = '';
  for (let i = 1; i <= 7; i++) {
    c.innerHTML += `<div class="jrow">
      <span class="jlbl">J${i}</span>
      <input type="range" id="j${i}" min="-180" max="180" value="0" step="0.5"
        onmousedown="activeSl='j${i}'" onmouseup="activeSl=null"
        ontouchstart="activeSl='j${i}'" ontouchend="activeSl=null"
        oninput="document.getElementById('jv${i}').textContent=parseFloat(this.value).toFixed(1)+'°'">
      <span class="jval" id="jv${i}">0.0°</span>
    </div>`;
  }
  // pose input focus tracking
  ['pX','pY','pZ','pRX','pRY','pRZ'].forEach(id => {
    const el = document.getElementById(id);
    if(el) { el.onfocus = ()=>poseEditing=true; el.onblur = ()=>poseEditing=false; }
  });
}
function syncJointsFromRobot() {
  robotJoints.forEach((v, i) => {
    const el = document.getElementById(`j${i+1}`);
    if (el) { el.value = v; document.getElementById(`jv${i+1}`).textContent = v.toFixed(1)+'°'; }
  });
}

// ── Robot WS ──────────────────────────────────────────────────────────────────
function connectRobotWs() {
  wsRobot = new WebSocket(`ws://${location.host}/ws/robot?token=${TOKEN}`);
  wsRobot.onmessage = ({data}) => {
    const s = JSON.parse(data);
    // Header status
    const st = document.getElementById('hStatus');
    st.textContent = s.connected ? 'ONLINE' : 'OFFLINE';
    st.className = 'h-status ' + (s.connected ? 'online' : 'offline');
    // Joints (read-only display if user dragging)
    if (s.joints) {
      robotJoints = s.joints;
      if (armReady) updateArm(s.joints);
      s.joints.forEach((v, i) => {
        if (activeSl !== `j${i+1}`) {
          const el = document.getElementById(`j${i+1}`);
          if (el) { el.value = v; document.getElementById(`jv${i+1}`).textContent = v.toFixed(1)+'°'; }
        }
      });
    }
    // Pose
    if (s.pose && s.pose.x !== undefined && !poseEditing) {
      document.getElementById('pX').value  = s.pose.x.toFixed(4);
      document.getElementById('pY').value  = s.pose.y.toFixed(4);
      document.getElementById('pZ').value  = s.pose.z.toFixed(4);
      document.getElementById('pRX').value = s.pose.theta_x.toFixed(2);
      document.getElementById('pRY').value = s.pose.theta_y.toFixed(2);
      document.getElementById('pRZ').value = s.pose.theta_z.toFixed(2);
    }
    // Gripper
    if (s.gripper !== undefined && activeSl !== 'g') {
      const pct = Math.round(s.gripper * 100);
      document.getElementById('gSlider').value = pct;
      document.getElementById('gPct').textContent = pct + '%';
    }
  };
  wsRobot.onclose = () => setTimeout(connectRobotWs, 2000);
}
function connectRobot() { /* connect/disconnect toggled via button — arm connects on server start */ }

// ── Logs WS ───────────────────────────────────────────────────────────────────
function connectLogsWs() {
  wsLogs = new WebSocket(`ws://${location.host}/ws/logs?token=${TOKEN}`);
  wsLogs.onmessage = ({data}) => appendLog(data);
  wsLogs.onclose = () => setTimeout(connectLogsWs, 3000);
}
function appendLog(msg) {
  const box = document.getElementById('logBox');
  const d = document.createElement('div');
  const ts = new Date().toLocaleTimeString('en-GB',{hour12:false});
  let cls = 'll-d';
  if (msg.includes('[INFO]'))  cls = 'll-i';
  if (msg.includes('[WARN'))   cls = 'll-w';
  if (msg.includes('[ERROR]') || msg.includes('[CRITICAL]')) cls = 'll-e';
  d.className = cls;
  d.textContent = `[${ts}] ${msg}`;
  box.appendChild(d);
  box.scrollTop = box.scrollHeight;
  while (box.children.length > 300) box.removeChild(box.firstChild);
}

// ── Terminal WS ───────────────────────────────────────────────────────────────
function connectTermWs() {
  wsTerm = new WebSocket(`ws://${location.host}/ws/terminal?token=${TOKEN}`);
  wsTerm.onmessage = ({data}) => {
    const out = document.getElementById('termOut');
    out.textContent += data;
    out.scrollTop = out.scrollHeight;
  };
  wsTerm.onclose = () => {
    document.getElementById('termOut').textContent += '\n[Disconnected — reconnecting...]\n';
    setTimeout(connectTermWs, 3000);
  };
}
function runScript() {
  const code = document.getElementById('termIn').value;
  if (!code.trim()) return;
  document.getElementById('termOut').textContent = '';
  if (wsTerm && wsTerm.readyState === WebSocket.OPEN)
    wsTerm.send(JSON.stringify({code}));
  else
    document.getElementById('termOut').textContent = '[Terminal not connected]\n';
}

// ── Camera helpers ────────────────────────────────────────────────────────────
function startCam(cam) {
  const t = TOKEN;
  if (cam === 'rs') {
    document.getElementById('rsBody').innerHTML =
      `<img class="cam-rgb" src="/api/cameras/realsense/rgb?token=${t}&_=${Date.now()}" alt="">` +
      (depthOn.rs ? `<img class="cam-depth" src="/api/cameras/realsense/depth?token=${t}&_=${Date.now()}" alt="">` : '');
  } else {
    document.getElementById('oakBody').innerHTML =
      `<img class="cam-rgb" src="/api/cameras/oakd/rgb?token=${t}&_=${Date.now()}" alt="">` +
      (depthOn.oak ? `<img class="cam-depth" src="/api/cameras/oakd/depth?token=${t}&_=${Date.now()}" alt="">` : '');
  }
}
function toggleDepth(cam, btn) {
  depthOn[cam] = !depthOn[cam];
  btn.textContent = depthOn[cam] ? 'DEPTH ON' : 'DEPTH OFF';
  btn.className = depthOn[cam] ? 'btn bn' : 'btn bp';
}

// ── Status & system ───────────────────────────────────────────────────────────
async function pollStatus() {
  try {
    const s = await fetch(`/api/status?token=${TOKEN}`).then(r=>r.json());
    Object.entries(s).forEach(([k,v]) => {
      const d = document.getElementById(`dot-${k}`);
      if (d) d.className = 'sdot' + (v ? ' on' : '');
    });
  } catch {}
}
async function pollSystem() {
  try {
    const s = await fetch(`/api/system?token=${TOKEN}`).then(r=>r.json());
    document.getElementById('hCpu').textContent = s.cpu.toFixed(0);
    document.getElementById('hMem').textContent = s.mem.toFixed(0);
  } catch {}
}

// ── Robot API calls ───────────────────────────────────────────────────────────
async function apiPost(path, body) {
  try {
    const r = await fetch(`${path}?token=${TOKEN}`, {
      method: 'POST',
      headers: body ? {'Content-Type':'application/json'} : {},
      body: body ? JSON.stringify(body) : undefined,
    });
    if (!r.ok) {
      const e = await r.json().catch(()=>({detail:'unknown error'}));
      appendLog(`[ERROR] ${path}: ${e.detail}`);
    }
  } catch(e) { appendLog(`[ERROR] ${e.message}`); }
}
async function sendJoints() {
  const angles = Array.from({length:7},(_,i)=>parseFloat(document.getElementById(`j${i+1}`).value));
  await apiPost('/api/robot/joints', {angles});
}
async function moveToPose() {
  await apiPost('/api/robot/pose', {
    x:       +document.getElementById('pX').value,
    y:       +document.getElementById('pY').value,
    z:       +document.getElementById('pZ').value,
    theta_x: +document.getElementById('pRX').value,
    theta_y: +document.getElementById('pRY').value,
    theta_z: +document.getElementById('pRZ').value,
  });
}
async function setGripper(pct) {
  document.getElementById('gSlider').value = pct;
  document.getElementById('gPct').textContent = pct + '%';
  await apiPost('/api/robot/gripper', {position: pct/100});
}
async function sendGripper() {
  const pct = parseInt(document.getElementById('gSlider').value);
  await apiPost('/api/robot/gripper', {position: pct/100});
}
async function sendCmd() {
  const el = document.getElementById('cmdIn');
  const t = el.value.trim().toLowerCase();
  el.value = '';
  if (!t) return;
  appendLog(`[CMD] ${t}`);
  if      (t.includes('home'))    await apiPost('/api/robot/home');
  else if (t.includes('retract')) await apiPost('/api/robot/retract');
  else if (t.includes('vertical'))await apiPost('/api/robot/vertical');
  else if (t.includes('stop'))    await apiPost('/api/robot/stop');
  else if (t.includes('open'))    await setGripper(0);
  else if (t.includes('close'))   await setGripper(100);
  else appendLog('[INFO] Commands: home · retract · vertical · stop · open · close');
}

// ═══════════════════════════════════════════════════════════════════════════
// 3-D VIEWERS
// ═══════════════════════════════════════════════════════════════════════════

// ── Kinova Gen3 modified-DH forward kinematics ────────────────────────────
// Parameters: [alpha_prev_rad, a_prev_m, d_m]  (a=0 for all Gen3 joints)
const DH = [
  [0,            0, 0.2848],
  [-Math.PI/2,   0, 0.0118],
  [ Math.PI/2,   0, 0.2506],
  [-Math.PI/2,   0, 0.0114],
  [ Math.PI/2,   0, 0.2085],
  [-Math.PI/2,   0, 0.0116],
  [ Math.PI/2,   0, 0.1059],
];

function mdhMat(alpha, a, d, theta) {
  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  const ct = Math.cos(theta), st = Math.sin(theta);
  const m = new THREE.Matrix4();
  m.set(
    ct,    -st,     0,    a,
    st*ca,  ct*ca, -sa,  -sa*d,
    st*sa,  ct*sa,  ca,   ca*d,
    0,      0,      0,    1
  );
  return m;
}

function forwardKinematics(deg) {
  const T = new THREE.Matrix4();
  const pos = [new THREE.Vector3()];
  DH.forEach(([alpha, a, d], i) => {
    T.multiply(mdhMat(alpha, a, d, deg[i] * Math.PI / 180));
    pos.push(new THREE.Vector3().setFromMatrixPosition(T));
  });
  return pos;   // 8 positions in DH (Z-up) frame
}

// ── Arm viewer setup ──────────────────────────────────────────────────────
let armReady = false;
let armRenderer, armCamera, armScene, armControls, armGroup;
let linkMeshes = [], jointMeshes = [], armLine;

function initArmViewer() {
  const container = document.getElementById('armCanvas');
  const w = container.clientWidth, h = container.clientHeight || 340;

  armRenderer = new THREE.WebGLRenderer({antialias: true});
  armRenderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  armRenderer.setSize(w, h);
  armRenderer.setClearColor(0x0a0a0a);
  container.appendChild(armRenderer.domElement);

  armCamera = new THREE.PerspectiveCamera(45, w / h, 0.001, 20);
  armCamera.position.set(0.7, 0.55, 0.7);

  armScene = new THREE.Scene();
  armScene.add(new THREE.AmbientLight(0xffffff, 0.45));
  const sun = new THREE.DirectionalLight(0xffffff, 0.9);
  sun.position.set(1, 2, 1.5);
  armScene.add(sun);

  // armGroup: rotate DH (Z-up) → Three.js (Y-up): Rx(-90°)
  armGroup = new THREE.Group();
  armGroup.rotation.x = -Math.PI / 2;
  armScene.add(armGroup);

  // Table surface
  const tg = new THREE.BoxGeometry(0.9, 0.015, 0.9);
  const tm = new THREE.MeshLambertMaterial({color: 0x1a2030});
  const table = new THREE.Mesh(tg, tm);
  table.position.set(0, 0, -0.008);
  armGroup.add(table);

  // Grid
  const grid = new THREE.GridHelper(0.9, 18, 0x222233, 0x1a1a28);
  grid.rotation.x = Math.PI / 2;   // XY plane in DH frame
  armGroup.add(grid);

  // Base cylinder
  const bg = new THREE.CylinderGeometry(0.055, 0.065, 0.08, 16);
  const bm = new THREE.MeshLambertMaterial({color: 0x4caf50});
  const base = new THREE.Mesh(bg, bm);
  base.position.set(0, 0, 0.04);
  base.rotation.x = Math.PI / 2;
  armGroup.add(base);

  // Pre-allocate link cylinders (unit length, scaled each frame)
  const linkMat = new THREE.MeshLambertMaterial({color: 0x1565c0});
  for (let i = 0; i < 7; i++) {
    const g = new THREE.CylinderGeometry(0.022, 0.022, 1, 8);
    const m = new THREE.Mesh(g, linkMat.clone());
    armGroup.add(m);
    linkMeshes.push(m);
  }

  // Pre-allocate joint spheres
  for (let i = 0; i <= 7; i++) {
    const r = i === 0 ? 0.04 : (i === 7 ? 0.022 : 0.030);
    const c = i === 0 ? 0x4caf50 : (i === 7 ? 0xffc107 : 0x4fc3f7);
    const g = new THREE.SphereGeometry(r, 12, 8);
    const m = new THREE.Mesh(g, new THREE.MeshLambertMaterial({color: c}));
    armGroup.add(m);
    jointMeshes.push(m);
  }

  armControls = new THREE.OrbitControls(armCamera, armRenderer.domElement);
  armControls.target.set(0, 0.3, 0);
  armControls.update();

  const ro = new ResizeObserver(() => {
    const w = container.clientWidth, h = container.clientHeight || 340;
    armCamera.aspect = w / h;
    armCamera.updateProjectionMatrix();
    armRenderer.setSize(w, h);
  });
  ro.observe(container);

  (function loop() {
    requestAnimationFrame(loop);
    armControls.update();
    armRenderer.render(armScene, armCamera);
  })();

  armReady = true;
  updateArm([0,0,0,0,0,0,0]);
}

const _dhY = new THREE.Vector3(0, 1, 0);
const _qTmp = new THREE.Quaternion();

function updateArm(deg) {
  const pos = forwardKinematics(deg);  // DH (Z-up) coordinates

  // Joint spheres
  pos.forEach((p, i) => {
    if (jointMeshes[i]) jointMeshes[i].position.set(p.x, p.y, p.z);
  });

  // Link cylinders between consecutive joints
  for (let i = 0; i < 7; i++) {
    const a = pos[i], b = pos[i + 1];
    const dir = new THREE.Vector3().subVectors(b, a);
    const len = dir.length();
    linkMeshes[i].position.copy(new THREE.Vector3().addVectors(a, b).multiplyScalar(0.5));
    linkMeshes[i].scale.set(1, len, 1);
    if (len > 1e-4) {
      _qTmp.setFromUnitVectors(_dhY, dir.normalize());
      linkMeshes[i].quaternion.copy(_qTmp);
    }
  }
}

// ── Point cloud viewer setup ──────────────────────────────────────────────
const MAX_PC = 12000;
let pcRenderer, pcCamera, pcScene, pcControls, pcGeom, pcPoints;
let pcReady = false, pcWs = null, pcStreaming = false;

function initPcViewer() {
  const container = document.getElementById('pcCanvas');
  const w = container.clientWidth, h = container.clientHeight || 340;

  pcRenderer = new THREE.WebGLRenderer({antialias: false});
  pcRenderer.setPixelRatio(1);
  pcRenderer.setSize(w, h);
  pcRenderer.setClearColor(0x080808);
  container.appendChild(pcRenderer.domElement);

  pcCamera = new THREE.PerspectiveCamera(60, w / h, 0.001, 50);
  pcCamera.position.set(0, 0, -0.4);
  pcCamera.lookAt(0, 0, 0.8);

  pcScene = new THREE.Scene();
  pcScene.add(new THREE.AxesHelper(0.25));

  // Pre-allocate BufferGeometry
  pcGeom = new THREE.BufferGeometry();
  const pos = new Float32Array(MAX_PC * 3);
  const col = new Float32Array(MAX_PC * 3).fill(0.3);
  pcGeom.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  pcGeom.setAttribute('color',    new THREE.BufferAttribute(col, 3));
  pcGeom.setDrawRange(0, 0);

  pcPoints = new THREE.Points(pcGeom,
    new THREE.PointsMaterial({size: 0.004, vertexColors: true, sizeAttenuation: true}));
  pcScene.add(pcPoints);

  pcControls = new THREE.OrbitControls(pcCamera, pcRenderer.domElement);
  pcControls.target.set(0, 0, 0.8);
  pcControls.update();

  const ro = new ResizeObserver(() => {
    const w = container.clientWidth, h = container.clientHeight || 340;
    pcCamera.aspect = w / h;
    pcCamera.updateProjectionMatrix();
    pcRenderer.setSize(w, h);
  });
  ro.observe(container);

  (function loop() {
    requestAnimationFrame(loop);
    pcControls.update();
    pcRenderer.render(pcScene, pcCamera);
  })();

  pcReady = true;
}

function updatePointCloud(posArr, colArr, count) {
  const n = Math.min(count, MAX_PC);
  const p = pcGeom.attributes.position.array;
  const c = pcGeom.attributes.color.array;
  // RealSense frame: x=right, y=down, z=forward → flip y for Y-up display
  for (let i = 0; i < n; i++) {
    p[i*3]   =  posArr[i*3];
    p[i*3+1] = -posArr[i*3+1];
    p[i*3+2] =  posArr[i*3+2];
    c[i*3]   = colArr[i*3]   / 255;
    c[i*3+1] = colArr[i*3+1] / 255;
    c[i*3+2] = colArr[i*3+2] / 255;
  }
  pcGeom.attributes.position.needsUpdate = true;
  pcGeom.attributes.color.needsUpdate    = true;
  pcGeom.setDrawRange(0, n);

  const overlay = document.getElementById('pcOverlay');
  if (overlay) overlay.style.display = 'none';
  const dot = document.getElementById('dot-pc');
  if (dot) dot.className = 'sdot on';
}

function togglePcStream() {
  if (pcStreaming) {
    pcStreaming = false;
    if (pcWs) { pcWs.close(); pcWs = null; }
    document.getElementById('btnPcStream').textContent = '▶ STREAM';
    document.getElementById('btnPcStream').className = 'btn bg';
    const dot = document.getElementById('dot-pc');
    if (dot) dot.className = 'sdot';
  } else {
    pcStreaming = true;
    document.getElementById('btnPcStream').textContent = '■ STOP';
    document.getElementById('btnPcStream').className = 'btn br';
    startPcWs();
  }
}

function startPcWs() {
  const src = document.getElementById('pcSource').value;
  pcWs = new WebSocket(`ws://${location.host}/ws/pointcloud/${src}?token=${TOKEN}`);
  pcWs.binaryType = 'arraybuffer';
  pcWs.onmessage = ({data}) => {
    const count = new DataView(data).getUint32(0, true);
    if (count === 0) return;
    const posF32 = new Float32Array(data, 4,          count * 3);
    const colU8  = new Uint8Array  (data, 4 + count*12, count * 3);
    updatePointCloud(posF32, colU8, count);
  };
  pcWs.onclose = () => {
    if (pcStreaming) setTimeout(startPcWs, 2000);
  };
}

// ── Init viewers after app is shown ──────────────────────────────────────
const _origShowApp = showApp;
showApp = function() {
  _origShowApp();
  setTimeout(() => {
    initArmViewer();
    initPcViewer();
  }, 100);
};

// ── Boot ──────────────────────────────────────────────────────────────────────
checkAuth();
</script>
</body>
</html>"""


@app.get("/", response_class=HTMLResponse, include_in_schema=False)
def dashboard():
    return HTMLResponse(_DASHBOARD)


if __name__ == "__main__":
    uvicorn.run(app, host=SERVER_HOST, port=SERVER_PORT, log_level="info")
