"""Camera MJPEG + snapshot routes (event-driven, PLAN 2.2)."""

from __future__ import annotations

import io

from fastapi import APIRouter, Depends, HTTPException
from fastapi.responses import StreamingResponse

from .. import auth as _auth
from ..state import STATE

router = APIRouter()

_MJPEG = "multipart/x-mixed-replace; boundary=frame"


def _stream(channel):
    return StreamingResponse(channel.frames(), media_type=_MJPEG)


@router.get("/api/cameras/realsense/rgb")
async def rs_rgb(t: str = Depends(_auth.auth)):
    if not STATE.status["realsense"]:
        raise HTTPException(503, "RealSense unavailable")
    return _stream(STATE.realsense.rgb)


@router.get("/api/cameras/realsense/depth")
async def rs_depth(t: str = Depends(_auth.auth)):
    if not STATE.status["realsense"]:
        raise HTTPException(503, "RealSense unavailable")
    return _stream(STATE.realsense.depth)


@router.get("/api/cameras/realsense/snapshot")
async def rs_snap(t: str = Depends(_auth.auth)):
    f = await STATE.realsense.rgb.snapshot()
    if not f:
        raise HTTPException(503, "No frame")
    return StreamingResponse(io.BytesIO(f), media_type="image/jpeg")


@router.get("/api/cameras/oakd/rgb")
async def oak_rgb(t: str = Depends(_auth.auth)):
    if not STATE.status["oakd"]:
        raise HTTPException(503, "OAK-D unavailable")
    return _stream(STATE.oakd.rgb)


@router.get("/api/cameras/oakd/depth")
async def oak_depth(t: str = Depends(_auth.auth)):
    if not STATE.status["oakd"]:
        raise HTTPException(503, "OAK-D unavailable")
    return _stream(STATE.oakd.depth)


@router.get("/api/cameras/oakd/snapshot")
async def oak_snap(t: str = Depends(_auth.auth)):
    f = await STATE.oakd.rgb.snapshot()
    if not f:
        raise HTTPException(503, "No frame")
    return StreamingResponse(io.BytesIO(f), media_type="image/jpeg")
