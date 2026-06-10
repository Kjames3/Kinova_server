"""WebSocket endpoints: robot state, logs, script terminal, point clouds.

* ``/ws/robot``   — subscribes to the shared robot broadcaster (serialized once
  per tick by the pump in app.py, PLAN 2.2).
* ``/ws/logs``    — subscribes to the log broadcaster (PLAN 2.6).
* ``/ws/terminal``— runs a user Python script in a subprocess.
* ``/ws/pointcloud/{src}`` — subscribes to the shared per-source point-cloud
  task (PLAN 2.4); bytes are computed once and fanned out to all clients.

Tokens are read from ``Sec-WebSocket-Protocol`` (preferred) or ``?token=``.
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import sys
import tempfile

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from .. import auth as _auth
from .. import logs as _logs
from ..config import CFG
from ..state import STATE

router = APIRouter()
log = logging.getLogger("kinova-server")


async def _accept(ws: WebSocket):
    """Validate the token and accept, echoing the subprotocol if one was used."""
    tok = _auth.ws_token(ws, ws.query_params.get("token", ""))
    if tok is None:
        await ws.close(code=4001)
        return False
    proto = ws.headers.get("sec-websocket-protocol", "")
    subproto = proto.split(",")[0].strip() if proto else None
    await ws.accept(subprotocol=subproto)
    return True


@router.websocket("/ws/robot")
async def ws_robot(ws: WebSocket):
    if not await _accept(ws):
        return
    q = STATE.robot.broadcaster.subscribe()
    try:
        # send an immediate snapshot so the UI populates without waiting a tick
        await ws.send_text(json.dumps(STATE.robot.snapshot()))
        while True:
            await ws.send_text(await q.get())
    except WebSocketDisconnect:
        pass
    finally:
        STATE.robot.broadcaster.unsubscribe(q)


@router.websocket("/ws/logs")
async def ws_logs(ws: WebSocket):
    if not await _accept(ws):
        return
    q = _logs.subscribe()
    try:
        while True:
            await ws.send_text(await q.get())
    except WebSocketDisconnect:
        pass
    finally:
        _logs.unsubscribe(q)


@router.websocket("/ws/terminal")
async def ws_terminal(ws: WebSocket):
    if not await _accept(ws):
        return
    await ws.send_text('[INFO] Terminal ready. Send JSON: {"code": "...your Python..."}\n')
    await ws.send_text(f"[INFO] KINOVA_IP={CFG.kinova_ip}  KINOVA_PORT={CFG.kinova_port}\n")
    try:
        while True:
            raw = await ws.receive_text()
            try:
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
                         "KINOVA_IP": CFG.kinova_ip,
                         "KINOVA_PORT": str(CFG.kinova_port),
                         "KINOVA_USER": CFG.kinova_user,
                         "KINOVA_PASS": CFG.kinova_pass},
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


async def _pc_ws(ws: WebSocket, store):
    if not await _accept(ws):
        return
    q = store.pc_broadcaster.subscribe(maxsize=2)
    store.pc_subscribers += 1
    try:
        while True:
            await ws.send_bytes(await q.get())
    except WebSocketDisconnect:
        pass
    finally:
        store.pc_subscribers -= 1
        store.pc_broadcaster.unsubscribe(q)


@router.websocket("/ws/pointcloud/realsense")
async def ws_pc_realsense(ws: WebSocket):
    await _pc_ws(ws, STATE.realsense)


@router.websocket("/ws/pointcloud/oakd")
async def ws_pc_oakd(ws: WebSocket):
    await _pc_ws(ws, STATE.oakd)
