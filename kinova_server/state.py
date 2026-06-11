"""Shared runtime state — replaces the old module-level globals (PLAN 1, 2.1-2.4).

Everything the hardware threads, ROS2 bridge and HTTP routes share lives on a
single :data:`STATE` object made of small classes:

* :class:`StreamChannel` — one MJPEG stream (rgb or depth). Event-driven
  fan-out so clients wake exactly once per new frame, plus a subscriber count
  the producer reads to skip JPEG encoding nobody is watching.
* :class:`FrameStore`   — one camera: two stream channels + the raw arrays and
  intrinsics needed for point-cloud generation, published as atomic reference
  swaps (no lock needed — single writer, GIL-atomic).
* :class:`Broadcaster`  — serialize-once / fan-out-to-many for robot state.
* :class:`RobotState`   — the kortex feedback snapshot + client handles.
"""

from __future__ import annotations

import asyncio
import threading
from typing import Any, Dict, Optional, Set

import numpy as np


class StreamChannel:
    """A single MJPEG stream with event-driven fan-out and a subscriber count."""

    def __init__(self) -> None:
        self.jpeg: Optional[bytes] = None
        self.subscribers: int = 0          # read by producer thread (GIL-atomic)
        self._event: Optional[asyncio.Event] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def bind_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop
        self._event = asyncio.Event()

    def publish(self, jpeg: bytes) -> None:
        """Called from a camera thread: store the frame and wake waiters."""
        self.jpeg = jpeg
        loop, event = self._loop, self._event
        if loop is not None and event is not None:
            loop.call_soon_threadsafe(event.set)

    async def frames(self, fps: int = 30):
        """Async generator yielding raw JPEG bytes, one per new frame."""
        boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
        if self._event is None:
            self._event = asyncio.Event()
        self.subscribers += 1
        try:
            while True:
                await self._event.wait()
                self._event.clear()
                frame = self.jpeg
                if frame:
                    yield boundary + frame + b"\r\n"
        finally:
            self.subscribers -= 1

    async def snapshot(self, timeout: float = 2.0) -> Optional[bytes]:
        """One-shot grab: subscribe just long enough for one fresh encode.

        Without this, subscriber-aware encoding (PLAN 2.1) means no JPEG is
        produced unless an MJPEG stream is open, so a standalone snapshot would
        get nothing.
        """
        if self._event is None:
            self._event = asyncio.Event()
        self.subscribers += 1
        try:
            self._event.clear()
            try:
                await asyncio.wait_for(self._event.wait(), timeout)
            except asyncio.TimeoutError:
                pass
            return self.jpeg
        finally:
            self.subscribers -= 1


class FrameStore:
    """All buffers for one camera."""

    def __init__(self, name: str, intrinsics: Optional[Dict[str, float]] = None) -> None:
        self.name = name
        self.rgb = StreamChannel()
        self.depth = StreamChannel()

        # Raw data for point clouds — published as one atomic tuple swap.
        # (raw_color BGR uint8, raw_depth native, intrinsics)
        self._pc: tuple = (None, None, intrinsics)
        self.pc_subscribers: int = 0
        self.pc_broadcaster = Broadcaster()

    def bind_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self.rgb.bind_loop(loop)
        self.depth.bind_loop(loop)

    # ── point-cloud raw data (atomic reference swaps, no lock) ─────────────
    def publish_pc(self, raw_color, raw_depth, intrinsics) -> None:
        self._pc = (raw_color, raw_depth, intrinsics)

    def pc_data(self) -> tuple:
        return self._pc

    @property
    def pc_wanted(self) -> bool:
        return self.pc_subscribers > 0

    @property
    def jpeg_wanted(self) -> bool:
        """True if any MJPEG viewer or the point-cloud task needs frames."""
        return (
            self.rgb.subscribers > 0
            or self.depth.subscribers > 0
            or self.pc_subscribers > 0
        )


class Broadcaster:
    """Serialize-once, fan-out-to-many over asyncio queues."""

    def __init__(self) -> None:
        self._subs: Set[asyncio.Queue] = set()

    def subscribe(self, maxsize: int = 10) -> asyncio.Queue:
        q: asyncio.Queue = asyncio.Queue(maxsize=maxsize)
        self._subs.add(q)
        return q

    def unsubscribe(self, q: asyncio.Queue) -> None:
        self._subs.discard(q)

    @property
    def subscriber_count(self) -> int:
        return len(self._subs)

    def publish(self, item: Any) -> None:
        for q in list(self._subs):
            try:
                q.put_nowait(item)
            except asyncio.QueueFull:
                pass


class RobotState:
    """Kinova feedback snapshot plus the live client handles."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._data: Dict[str, Any] = {
            "joints": [0.0] * 7,
            "pose": {"x": 0, "y": 0, "z": 0, "theta_x": 0, "theta_y": 0, "theta_z": 0},
            "gripper": 0.0,
            "connected": False,
        }
        self.base = None          # kortex BaseClient
        self.cyclic = None        # kortex BaseCyclicClient
        self.broadcaster = Broadcaster()

    def update(self, **kw: Any) -> None:
        with self._lock:
            self._data.update(kw)

    def set_pose(self, pose: Dict[str, float]) -> None:
        with self._lock:
            self._data["pose"] = pose

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            d = dict(self._data)
            d["pose"] = dict(d["pose"])
            d["joints"] = list(d["joints"])
            return d


# End-effector registry (PLAN GUI item 7). ``tcp_offset`` is the tool-tip
# distance from the end_effector_link flange along ``tcp_axis`` (EE frame); the
# assembly-tip value (0.255 m, −X) comes from the pen_tip joint in the URDF.
END_EFFECTORS: Dict[str, Dict[str, Any]] = {
    "robotiq_2f_140": {
        "label": "Robotiq 2F-140", "has_gripper": True,
        "tcp_offset": 0.160, "tcp_axis": "z", "model": "gen3_robotiq_2f_140",
    },
    "assembly_tip": {
        "label": "Assembly Tip", "has_gripper": False,
        "tcp_offset": 0.255, "tcp_axis": "-x", "model": "gen3_assembly_tip",
    },
}


class AppState:
    """The one shared object. Hardware threads and routers read from here."""

    def __init__(self) -> None:
        self.realsense = FrameStore("realsense")
        self.oakd = FrameStore(
            "oakd", intrinsics={"fx": 452.0, "fy": 452.0, "ppx": 320.0, "ppy": 200.0}
        )
        self.robot = RobotState()
        self.status: Dict[str, bool] = {"realsense": False, "oakd": False, "kinova": False}
        self.end_effector: str = "robotiq_2f_140"   # PLAN GUI item 7
        self.stop_event = threading.Event()
        self.loop: Optional[asyncio.AbstractEventLoop] = None

    def bind_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self.loop = loop
        self.realsense.bind_loop(loop)
        self.oakd.bind_loop(loop)


STATE = AppState()
