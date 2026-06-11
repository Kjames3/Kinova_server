"""ROS2 HTTP + WebSocket routes (PLAN 3.1, 3.1.2, 3.1.3)."""

from __future__ import annotations

import io
import json

from fastapi import APIRouter, Depends, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse

from .. import auth as _auth
from ..ros2.action_bridge import ACTIONS
from ..ros2.bridge import BRIDGE
from ..ros2.pointcloud_bridge import FUSED_PC
from ..ros2.processes import MANAGER
from .models import InsertReq, ProcessReq
from .ws import _accept

router = APIRouter()

_MJPEG = "multipart/x-mixed-replace; boundary=frame"


# ── discovery + generic echo ────────────────────────────────────────────────
@router.get("/api/ros2/status")
def ros2_status(t: str = Depends(_auth.auth)):
    return {
        "available": BRIDGE.available,
        "action_available": ACTIONS.action_available,
        "processes": MANAGER.status(),
    }


@router.get("/api/ros2/topics")
def ros2_topics(t: str = Depends(_auth.auth)):
    if not BRIDGE.available:
        raise HTTPException(503, "ROS2 bridge unavailable")
    return {"topics": BRIDGE.list_topics()}


@router.websocket("/ws/ros2/topic")
async def ws_topic(ws: WebSocket):
    if not await _accept(ws):
        return
    name = ws.query_params.get("name", "")
    ok, err = BRIDGE.add_topic_subscriber(name)
    if not ok:
        await ws.send_text(json.dumps({"error": err}))
        await ws.close()
        return
    bc = BRIDGE.topic_broadcaster(name)
    q = bc.subscribe(maxsize=20)
    try:
        while True:
            await ws.send_text(json.dumps(await q.get(), default=str))
    except WebSocketDisconnect:
        pass
    finally:
        bc.unsubscribe(q)
        BRIDGE.remove_topic_subscriber(name)


# ── image relay ─────────────────────────────────────────────────────────────
@router.get("/api/ros2/image")
async def ros2_image(name: str, t: str = Depends(_auth.auth)):
    """MJPEG relay of a ROS2 image topic. Pass ?name=/camera/color/image_raw."""
    ok, err = BRIDGE.add_image_subscriber(name)
    if not ok:
        raise HTTPException(503, err)
    channel = BRIDGE.image_channel(name)

    async def _gen():
        try:
            async for chunk in channel.frames():
                yield chunk
        finally:
            BRIDGE.remove_image_subscriber(name)

    return StreamingResponse(_gen(), media_type=_MJPEG)


# ── subprocess manager ──────────────────────────────────────────────────────
@router.post("/api/ros2/process/start")
async def process_start(body: ProcessReq, t: str = Depends(_auth.auth)):
    res = await MANAGER.start(body.process, body.args)
    if not res.get("ok"):
        raise HTTPException(400, res.get("error", "start failed"))
    return res


@router.post("/api/ros2/process/stop")
async def process_stop(body: ProcessReq, t: str = Depends(_auth.auth)):
    res = await MANAGER.stop(body.process)
    if not res.get("ok"):
        raise HTTPException(400, res.get("error", "stop failed"))
    return res


@router.get("/api/ros2/process/status")
def process_status(t: str = Depends(_auth.auth)):
    return MANAGER.status()


@router.websocket("/ws/ros2/process/logs")
async def ws_process_logs(ws: WebSocket):
    if not await _accept(ws):
        return
    name = ws.query_params.get("name", "")
    mp = MANAGER.get(name)
    if mp is None:
        await ws.send_text(f"[unknown process '{name}']")
        await ws.close()
        return
    q = mp.broadcaster.subscribe(maxsize=500)
    try:
        for line in list(mp.log_buffer):   # replay backlog
            await ws.send_text(line)
        while True:
            await ws.send_text(await q.get())
    except WebSocketDisconnect:
        pass
    finally:
        mp.broadcaster.unsubscribe(q)


# ── fused point cloud (PLAN GUI item 4) ─────────────────────────────────────
@router.websocket("/ws/pointcloud/fused")
async def ws_pc_fused(ws: WebSocket):
    if not await _accept(ws):
        return
    ok, err = FUSED_PC.add_subscriber()
    if not ok:
        await ws.close(code=1011)
        return
    q = FUSED_PC.broadcaster.subscribe(maxsize=2)
    try:
        while True:
            await ws.send_bytes(await q.get())
    except WebSocketDisconnect:
        pass
    finally:
        FUSED_PC.broadcaster.unsubscribe(q)
        FUSED_PC.remove_subscriber()


# ── insertion action ────────────────────────────────────────────────────────
@router.post("/api/ros2/insert/start")
def insert_start(body: InsertReq, t: str = Depends(_auth.auth)):
    fields = {
        "target_x": body.target_x,
        "target_y": body.target_y,
        "hover_above_top": body.hover_above_top,
        "dry_run": body.dry_run,
        "skip_home_move": body.skip_home_move,
    }
    # If dynamic TF requested and no explicit target, use the live fused centre.
    if body.use_dynamic_tf and body.target_x == 0.0 and body.target_y == 0.0:
        c = ACTIONS.latest_fusion.get("center")
        if c:
            fields["target_x"], fields["target_y"] = c["x"], c["y"]
    res = ACTIONS.start_insert(**fields)
    if not res.get("ok"):
        raise HTTPException(503, res.get("error", "insert failed"))
    return res


@router.post("/api/ros2/insert/cancel")
def insert_cancel(t: str = Depends(_auth.auth)):
    res = ACTIONS.cancel_insert()
    if not res.get("ok"):
        raise HTTPException(400, res.get("error", "cancel failed"))
    return res


@router.websocket("/ws/ros2/insert/feedback")
async def ws_insert_feedback(ws: WebSocket):
    if not await _accept(ws):
        return
    q = ACTIONS.feedback.subscribe(maxsize=50)
    try:
        while True:
            await ws.send_text(json.dumps(await q.get(), default=str))
    except WebSocketDisconnect:
        pass
    finally:
        ACTIONS.feedback.unsubscribe(q)


# ── fusion telemetry ────────────────────────────────────────────────────────
@router.websocket("/ws/ros2/fusion")
async def ws_fusion(ws: WebSocket):
    if not await _accept(ws):
        return
    if not ACTIONS.add_fusion_subscriber():
        await ws.send_text(json.dumps({"error": "ROS2 bridge unavailable"}))
        await ws.close()
        return
    q = ACTIONS.fusion.subscribe(maxsize=20)
    try:
        await ws.send_text(json.dumps(ACTIONS.latest_fusion, default=str))
        while True:
            await ws.send_text(json.dumps(await q.get(), default=str))
    except WebSocketDisconnect:
        pass
    finally:
        ACTIONS.fusion.unsubscribe(q)
        ACTIONS.remove_fusion_subscriber()
