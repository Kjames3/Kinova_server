"""ROS2 ``sensor_msgs/PointCloud2`` → viewer bridge (PLAN GUI item 4).

Subscribes to a ``PointCloud2`` topic (default ``/fused_pointcloud``, published
by the fusion node) and re-emits each cloud in the **same binary frame format**
the dashboard point-cloud viewer already parses:

    <uint32 count><count*3 float32 xyz><count*3 uint8 rgb>

so ``static/viewers.js`` needs no new parser. The subscription is ref-counted
(created on the first WS client, dropped on the last), mirroring the topic-echo
and per-camera point-cloud tasks. Best-effort QoS so it works with both
reliable and best-effort publishers.
"""

from __future__ import annotations

import logging
import struct

import numpy as np

from ..state import STATE, Broadcaster
from .bridge import BRIDGE

log = logging.getLogger("kinova-server")

MAX_PTS = 12000


def decode_pointcloud2(msg, max_pts: int = MAX_PTS):
    """Vectorised PointCloud2 → (Nx3 float32 xyz, Nx3 uint8 rgb)."""
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint8)

    pstep = msg.point_step
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    buf = buf[: n * pstep].reshape(n, pstep)
    offs = {f.name: f.offset for f in msg.fields}

    def f32(name):
        o = offs[name]
        return buf[:, o:o + 4].copy().view("<f4").ravel()

    try:
        x, y, z = f32("x"), f32("y"), f32("z")
    except KeyError:
        return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint8)
    pts = np.stack([x, y, z], axis=1)

    rgb_field = "rgb" if "rgb" in offs else ("rgba" if "rgba" in offs else None)
    if rgb_field is not None:
        o = offs[rgb_field]
        packed = buf[:, o:o + 4].copy().view("<u4").ravel()
        r = (packed >> 16) & 0xFF
        g = (packed >> 8) & 0xFF
        b = packed & 0xFF
        colors = np.stack([r, g, b], axis=1).astype(np.uint8)
    else:
        colors = np.full((n, 3), 180, np.uint8)

    valid = np.isfinite(pts).all(axis=1)
    pts, colors = pts[valid], colors[valid]

    m = len(pts)
    if m > max_pts:
        idx = np.linspace(0, m - 1, max_pts).astype(int)
        pts, colors = pts[idx], colors[idx]
    return pts.astype(np.float32), colors


class FusedPointCloud:
    def __init__(self, topic: str = "/fused_pointcloud") -> None:
        self.topic = topic
        self._sub = None
        self.count = 0
        self.broadcaster = Broadcaster()

    def add_subscriber(self):
        if not BRIDGE.available:
            return False, "ROS2 bridge unavailable"
        if self.count == 0:
            ok, err = self._subscribe()
            if not ok:
                return False, err
        self.count += 1
        return True, "ok"

    def remove_subscriber(self) -> None:
        self.count -= 1
        if self.count <= 0 and self._sub is not None:
            try:
                BRIDGE.node.destroy_subscription(self._sub)
            except Exception:
                pass
            self._sub = None
            self.count = 0

    def _subscribe(self):
        try:
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import PointCloud2
        except Exception as exc:
            return False, f"sensor_msgs unavailable: {exc}"
        try:
            self._sub = BRIDGE.node.create_subscription(
                PointCloud2, self.topic, self._cb, qos_profile_sensor_data)
            log.info(f"Subscribed to fused point cloud {self.topic}")
            return True, "ok"
        except Exception as exc:
            return False, str(exc)

    def _cb(self, msg) -> None:
        if self.count <= 0:
            return
        try:
            pts, colors = decode_pointcloud2(msg)
        except Exception as exc:
            log.debug(f"pointcloud decode {self.topic}: {exc}")
            return
        packed = struct.pack("<I", len(pts)) + pts.tobytes() + colors.tobytes()
        loop = STATE.loop
        if loop is not None:
            loop.call_soon_threadsafe(self.broadcaster.publish, packed)


FUSED_PC = FusedPointCloud()
