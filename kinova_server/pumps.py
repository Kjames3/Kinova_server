"""Shared background pumps (PLAN 2.2, 2.4).

* ``robot_pump`` serializes the robot state **once** per 50 ms tick and fans it
  out to every ``/ws/robot`` client, instead of each socket serializing itself.
* ``pc_pump`` computes a point cloud **once** per source when ≥1 subscriber is
  connected and broadcasts the same packed bytes to all of them.
"""

from __future__ import annotations

import asyncio
import json
import struct

from .pointcloud import compute_pc_oak, compute_pc_rs
from .state import STATE


async def robot_pump() -> None:
    while True:
        bc = STATE.robot.broadcaster
        if bc.subscriber_count:
            bc.publish(json.dumps(STATE.robot.snapshot()))
        await asyncio.sleep(0.05)


async def _pc_pump(store, compute_fn) -> None:
    loop = asyncio.get_running_loop()
    while True:
        if store.pc_subscribers > 0:
            pts, colors = await loop.run_in_executor(None, compute_fn)
            packed = struct.pack("<I", len(pts)) + pts.tobytes() + colors.tobytes()
            store.pc_broadcaster.publish(packed)
            await asyncio.sleep(0.25)   # 4 Hz
        else:
            await asyncio.sleep(0.1)


async def pc_pump_realsense() -> None:
    await _pc_pump(STATE.realsense, compute_pc_rs)


async def pc_pump_oakd() -> None:
    await _pc_pump(STATE.oakd, compute_pc_oak)
