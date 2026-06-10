"""In-server ROS2 bridge (PLAN 3.1).

A single ``rclpy`` node spins in its own daemon thread (same pattern as the
camera threads). It provides:

* topic discovery                — :meth:`Ros2Bridge.list_topics`
* generic JSON topic echo        — dynamic subscription, message → ordered dict,
  ref-counted (created on first WS subscriber, torn down on last disconnect)
* image relay                    — ``sensor_msgs/Image`` / ``CompressedImage``
  → cv_bridge → JPEG → a :class:`StreamChannel`, reusing the MJPEG generator

Waking asyncio WebSocket clients from the ROS callback thread always goes
through ``loop.call_soon_threadsafe`` (asyncio queues are not thread-safe).
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Dict, List, Optional, Tuple

from ..state import STATE, Broadcaster, StreamChannel

log = logging.getLogger("kinova-server")

_IMAGE_TYPES = {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"}


class _TopicSub:
    """Ref-counted generic subscription producing JSON dicts."""

    def __init__(self, handle, broadcaster: Broadcaster):
        self.handle = handle
        self.broadcaster = broadcaster
        self.count = 0
        self.last: Optional[dict] = None
        self.hz = 0.0
        self._stamps: List[float] = []


class _ImageSub:
    def __init__(self, handle, channel: StreamChannel):
        self.handle = handle
        self.channel = channel
        self.count = 0


class Ros2Bridge:
    def __init__(self) -> None:
        self.available = False
        self.node = None
        self._executor = None
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._topic_subs: Dict[str, _TopicSub] = {}
        self._image_subs: Dict[str, _ImageSub] = {}
        self._cv_bridge = None
        self._rclpy = None

    # ── lifecycle ──────────────────────────────────────────────────────────
    def start(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.executors import MultiThreadedExecutor
        except Exception as exc:
            log.warning(f"rclpy not importable — ROS2 bridge disabled ({exc})")
            return

        self._rclpy = rclpy
        try:
            if not rclpy.ok():
                rclpy.init()
            self.node = Node("kinova_server_bridge")
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self.node)
            self._thread = threading.Thread(
                target=self._spin, daemon=True, name="ros2-bridge"
            )
            self._thread.start()
            self.available = True
            log.info("ROS2 bridge node started (kinova_server_bridge)")
        except Exception as exc:
            log.error(f"ROS2 bridge start failed: {exc}")
            self.available = False

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception as exc:
            log.warning(f"ROS2 executor stopped: {exc}")

    def stop(self) -> None:
        if not self.available:
            return
        try:
            if self._executor:
                self._executor.shutdown()
            if self.node:
                self.node.destroy_node()
            if self._rclpy and self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            pass
        self.available = False

    # ── topic discovery ────────────────────────────────────────────────────
    def list_topics(self) -> List[dict]:
        if not self.available:
            return []
        out = []
        for name, types in self.node.get_topic_names_and_types():
            sub = self._topic_subs.get(name)
            out.append({
                "name": name,
                "types": types,
                "is_image": any(t in _IMAGE_TYPES for t in types),
                "hz": round(sub.hz, 1) if sub else 0.0,
                "subscribed": bool(sub or name in self._image_subs),
            })
        return sorted(out, key=lambda t: t["name"])

    def _resolve_type(self, name: str) -> Optional[str]:
        for tn, types in self.node.get_topic_names_and_types():
            if tn == name and types:
                return types[0]
        return None

    # ── generic JSON echo (ref-counted) ────────────────────────────────────
    def topic_broadcaster(self, name: str) -> Optional[Broadcaster]:
        sub = self._topic_subs.get(name)
        return sub.broadcaster if sub else None

    def add_topic_subscriber(self, name: str) -> Tuple[bool, str]:
        if not self.available:
            return False, "ROS2 bridge unavailable"
        with self._lock:
            sub = self._topic_subs.get(name)
            if sub is None:
                type_str = self._resolve_type(name)
                if not type_str:
                    return False, f"Topic {name} not found"
                try:
                    from rosidl_runtime_py.utilities import get_message
                    msg_cls = get_message(type_str)
                except Exception as exc:
                    return False, f"Cannot load type {type_str}: {exc}"
                bc = Broadcaster()
                sub = _TopicSub(None, bc)
                handle = self.node.create_subscription(
                    msg_cls, name, self._make_topic_cb(name), 10
                )
                sub.handle = handle
                self._topic_subs[name] = sub
            sub.count += 1
            return True, "ok"

    def remove_topic_subscriber(self, name: str) -> None:
        with self._lock:
            sub = self._topic_subs.get(name)
            if not sub:
                return
            sub.count -= 1
            if sub.count <= 0:
                try:
                    self.node.destroy_subscription(sub.handle)
                except Exception:
                    pass
                self._topic_subs.pop(name, None)

    def _make_topic_cb(self, name: str):
        from rosidl_runtime_py import message_to_ordereddict

        def _cb(msg):
            sub = self._topic_subs.get(name)
            if not sub:
                return
            try:
                data = message_to_ordereddict(msg)
            except Exception:
                return
            sub.last = data
            now = time.time()
            sub._stamps.append(now)
            sub._stamps = [t for t in sub._stamps if now - t < 2.0]
            if len(sub._stamps) > 1:
                sub.hz = (len(sub._stamps) - 1) / (sub._stamps[-1] - sub._stamps[0] + 1e-9)
            self._publish_threadsafe(sub.broadcaster, data)
        return _cb

    # ── image relay ────────────────────────────────────────────────────────
    def image_channel(self, name: str) -> Optional[StreamChannel]:
        sub = self._image_subs.get(name)
        return sub.channel if sub else None

    def add_image_subscriber(self, name: str) -> Tuple[bool, str]:
        if not self.available:
            return False, "ROS2 bridge unavailable"
        with self._lock:
            sub = self._image_subs.get(name)
            if sub is None:
                type_str = self._resolve_type(name)
                if type_str not in _IMAGE_TYPES:
                    return False, f"{name} is not an image topic ({type_str})"
                try:
                    from cv_bridge import CvBridge
                    from rosidl_runtime_py.utilities import get_message
                    if self._cv_bridge is None:
                        self._cv_bridge = CvBridge()
                    msg_cls = get_message(type_str)
                except Exception as exc:
                    return False, f"cv_bridge/type load failed: {exc}"
                channel = StreamChannel()
                if STATE.loop:
                    channel.bind_loop(STATE.loop)
                sub = _ImageSub(None, channel)
                compressed = type_str.endswith("CompressedImage")
                handle = self.node.create_subscription(
                    msg_cls, name, self._make_image_cb(name, compressed), 10
                )
                sub.handle = handle
                self._image_subs[name] = sub
            sub.count += 1
            return True, "ok"

    def remove_image_subscriber(self, name: str) -> None:
        with self._lock:
            sub = self._image_subs.get(name)
            if not sub:
                return
            sub.count -= 1
            if sub.count <= 0:
                try:
                    self.node.destroy_subscription(sub.handle)
                except Exception:
                    pass
                self._image_subs.pop(name, None)

    def _make_image_cb(self, name: str, compressed: bool):
        import cv2

        def _cb(msg):
            sub = self._image_subs.get(name)
            if not sub or sub.channel.subscribers <= 0:
                return
            try:
                if compressed:
                    img = self._cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
                else:
                    img = self._cv_bridge.imgmsg_to_cv2(msg, "bgr8")
                ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    sub.channel.publish(buf.tobytes())
            except Exception as exc:
                log.debug(f"image relay {name}: {exc}")
        return _cb

    # ── helpers ────────────────────────────────────────────────────────────
    def _publish_threadsafe(self, broadcaster: Broadcaster, item) -> None:
        loop = STATE.loop
        if loop is not None:
            loop.call_soon_threadsafe(broadcaster.publish, item)


BRIDGE = Ros2Bridge()


def start() -> None:
    BRIDGE.start()


def stop() -> None:
    BRIDGE.stop()
