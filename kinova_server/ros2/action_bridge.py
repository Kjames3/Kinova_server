"""Insertion action client + sensor-fusion telemetry (PLAN 3.1.3).

Reuses the spinning node from :mod:`kinova_server.ros2.bridge`.

* Action client on ``/insert_container`` (type
  ``kortex_bringup/action/InsertContainer``). Exposed via
  ``/api/ros2/insert/start`` and ``/api/ros2/insert/cancel``; live
  ``current_phase`` / ``progress`` feedback streams over
  ``/ws/ros2/insert/feedback``.
* Subscriptions to ``/fused_marker_square_center`` (PoseStamped) and
  ``/fused_corners`` (PoseArray); the extracted centre + corner count stream
  over ``/ws/ros2/fusion``.

The action interface must be built into the workspace (PLAN 3.0.1). If it is
not importable, :attr:`ActionBridge.action_available` is ``False`` and the
start endpoint returns a clear error.
"""

from __future__ import annotations

import logging
from typing import Optional

from ..state import STATE, Broadcaster
from .bridge import BRIDGE

log = logging.getLogger("kinova-server")

ACTION_NAME = "insert_container"
ACTION_TYPE = "kortex_bringup/action/InsertContainer"
CENTER_TOPIC = "/fused_marker_square_center"
CORNERS_TOPIC = "/fused_corners"


def _load_action_type():
    """Return the InsertContainer action class, or None if not built."""
    try:
        from kortex_bringup.action import InsertContainer
        return InsertContainer
    except Exception:
        pass
    try:
        from rosidl_runtime_py.utilities import get_action  # humble has this
        return get_action(ACTION_TYPE)
    except Exception:
        return None


class ActionBridge:
    def __init__(self) -> None:
        self._client = None
        self._action_cls = None
        self._goal_handle = None
        self.feedback = Broadcaster()
        self.fusion = Broadcaster()
        self._fusion_subs = []
        self._fusion_count = 0
        self.latest_fusion = {
            "center": None, "corners": 0, "corner_poses": [],
        }

    @property
    def action_available(self) -> bool:
        if not BRIDGE.available:
            return False
        if self._action_cls is None:
            self._action_cls = _load_action_type()
        return self._action_cls is not None

    # ── insertion action ───────────────────────────────────────────────────
    def _ensure_client(self) -> bool:
        if self._client is not None:
            return True
        if not self.action_available:
            return False
        from rclpy.action import ActionClient
        self._client = ActionClient(BRIDGE.node, self._action_cls, ACTION_NAME)
        return True

    def start_insert(self, **fields) -> dict:
        if not BRIDGE.available:
            return {"ok": False, "error": "ROS2 bridge unavailable"}
        if not self._ensure_client():
            return {"ok": False, "error": f"action interface '{ACTION_TYPE}' not built (see PLAN 3.0.1)"}
        if not self._client.wait_for_server(timeout_sec=2.0):
            return {"ok": False, "error": f"no action server on /{ACTION_NAME}"}

        goal = self._action_cls.Goal()
        for k, v in fields.items():
            if hasattr(goal, k):
                setattr(goal, k, v)

        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)
        log.info(f"InsertContainer goal sent: {fields}")
        return {"ok": True, "goal": fields}

    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self._emit_feedback({"event": "error", "message": str(exc)})
            return
        if not handle.accepted:
            self._emit_feedback({"event": "rejected", "message": "goal rejected"})
            return
        self._goal_handle = handle
        self._emit_feedback({"event": "accepted"})
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, msg) -> None:
        fb = msg.feedback
        self._emit_feedback({
            "event": "feedback",
            "current_phase": getattr(fb, "current_phase", ""),
            "progress": float(getattr(fb, "progress", 0.0)),
        })

    def _on_result(self, future) -> None:
        try:
            result = future.result().result
            self._emit_feedback({
                "event": "result",
                "success": bool(getattr(result, "success", False)),
                "message": getattr(result, "message", ""),
                "progress": 1.0,
            })
        except Exception as exc:
            self._emit_feedback({"event": "error", "message": str(exc)})
        finally:
            self._goal_handle = None

    def cancel_insert(self) -> dict:
        if self._goal_handle is None:
            return {"ok": False, "error": "no active goal"}
        self._goal_handle.cancel_goal_async()
        self._emit_feedback({"event": "cancel_requested"})
        log.info("InsertContainer cancel requested")
        return {"ok": True}

    def _emit_feedback(self, item: dict) -> None:
        loop = STATE.loop
        if loop:
            loop.call_soon_threadsafe(self.feedback.publish, item)

    # ── fusion telemetry (ref-counted) ─────────────────────────────────────
    def add_fusion_subscriber(self) -> bool:
        if not BRIDGE.available:
            return False
        if self._fusion_count == 0:
            self._subscribe_fusion()
        self._fusion_count += 1
        return True

    def remove_fusion_subscriber(self) -> None:
        self._fusion_count = max(0, self._fusion_count - 1)
        # keep subscriptions alive once created — cheap, and the panel reopens

    def _subscribe_fusion(self) -> None:
        try:
            from geometry_msgs.msg import PoseArray, PoseStamped
        except Exception as exc:
            log.warning(f"fusion subscribe failed: {exc}")
            return
        self._fusion_subs.append(BRIDGE.node.create_subscription(
            PoseStamped, CENTER_TOPIC, self._on_center, 10))
        self._fusion_subs.append(BRIDGE.node.create_subscription(
            PoseArray, CORNERS_TOPIC, self._on_corners, 10))
        log.info(f"Subscribed to fusion topics {CENTER_TOPIC}, {CORNERS_TOPIC}")

    def _on_center(self, msg) -> None:
        p, o = msg.pose.position, msg.pose.orientation
        self.latest_fusion["center"] = {
            "x": p.x, "y": p.y, "z": p.z,
            "qx": o.x, "qy": o.y, "qz": o.z, "qw": o.w,
        }
        self._emit_fusion()

    def _on_corners(self, msg) -> None:
        poses = [{"x": p.position.x, "y": p.position.y, "z": p.position.z} for p in msg.poses]
        self.latest_fusion["corners"] = len(poses)
        self.latest_fusion["corner_poses"] = poses
        self._emit_fusion()

    def _emit_fusion(self) -> None:
        loop = STATE.loop
        if loop:
            loop.call_soon_threadsafe(self.fusion.publish, dict(self.latest_fusion))


ACTIONS = ActionBridge()
