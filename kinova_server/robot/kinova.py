"""Kinova Gen3 driver thread + control helpers.

Poll split (PLAN 2.5): ``RefreshFeedback`` (joints + gripper) runs at 20 Hz;
the slower-changing ``GetMeasuredCartesianPose`` RPC runs at 5 Hz.

The control helpers raise ``RuntimeError`` on failure; the HTTP routes translate
that into an ``HTTPException``.
"""

from __future__ import annotations

import logging
import time

from ..config import CFG
from ..state import STATE

log = logging.getLogger("kinova-server")


def _protobuf_compat() -> None:
    """kortex_api ships an old protobuf that references names removed from
    ``collections`` in Python 3.9+ (e.g. ``collections.MutableMapping``). Alias
    them back from ``collections.abc`` so the generated ``*_pb2`` modules import
    on Python 3.10 (the Ubuntu 22.04 / ROS2 Humble default)."""
    import collections
    import collections.abc
    for _n in ("MutableMapping", "Mapping", "Sequence", "Callable", "Iterable",
               "MutableSequence", "Set", "MutableSet"):
        if not hasattr(collections, _n):
            setattr(collections, _n, getattr(collections.abc, _n))


def _import_transport():
    """Return the TCP transport class across kortex_api versions.

    2.6 wheel exposes ``TransportClientTcp``; newer builds renamed it to
    ``TCPTransport``."""
    try:
        from kortex_api.TransportClientTcp import TransportClientTcp
        return TransportClientTcp
    except ImportError:
        from kortex_api.TCPTransport import TCPTransport
        return TCPTransport


def kinova_thread() -> None:
    try:
        _protobuf_compat()
        from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
        from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
        from kortex_api.autogen.messages.Session_pb2 import CreateSessionInfo
        from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
        from kortex_api.SessionManager import SessionManager
        TransportClient = _import_transport()
    except ImportError:
        log.warning("kortex_api not installed — Kinova arm disabled")
        return

    transport = TransportClient()
    router = RouterClient(transport, RouterClientSendOptions())
    try:
        transport.connect(CFG.kinova_ip, CFG.kinova_port)
    except Exception as exc:
        log.error(f"Kinova connect failed ({CFG.kinova_ip}:{CFG.kinova_port}): {exc}")
        return

    session_info = CreateSessionInfo()
    session_info.username = CFG.kinova_user
    session_info.password = CFG.kinova_pass
    session_info.session_inactivity_timeout = 600_000

    session_manager = None
    try:
        session_manager = SessionManager(router)
        session_manager.CreateSession(session_info)
    except Exception as exc:
        log.error(f"Kinova session failed: {exc}")
        transport.disconnect()
        return

    STATE.robot.base = BaseClient(router)
    STATE.robot.cyclic = BaseCyclicClient(router)
    STATE.status["kinova"] = True
    log.info(f"Kinova Gen3 connected at {CFG.kinova_ip}:{CFG.kinova_port}")

    POSE_EVERY = 4  # 20 Hz feedback / 4 → ~5 Hz pose
    tick = 0
    try:
        while not STATE.stop_event.is_set():
            try:
                fb = STATE.robot.cyclic.RefreshFeedback()
                joints = [fb.actuators[i].position for i in range(7)]
                try:
                    gripper = fb.interconnect.gripper_feedback.motor[0].position / 100.0
                except Exception:
                    gripper = 0.0
                STATE.robot.update(joints=joints, gripper=gripper, connected=True)

                if tick % POSE_EVERY == 0:
                    pose = STATE.robot.base.GetMeasuredCartesianPose()
                    STATE.robot.set_pose({
                        "x": pose.x, "y": pose.y, "z": pose.z,
                        "theta_x": pose.theta_x,
                        "theta_y": pose.theta_y,
                        "theta_z": pose.theta_z,
                    })
            except Exception as exc:
                log.warning(f"Kinova poll: {exc}")
                STATE.robot.update(connected=False)
            tick += 1
            time.sleep(0.05)
    finally:
        STATE.status["kinova"] = False
        STATE.robot.base = None
        STATE.robot.cyclic = None
        STATE.robot.update(connected=False)
        try:
            if session_manager:
                session_manager.CloseSession()
            transport.disconnect()
        except Exception:
            pass
        log.info("Kinova disconnected")


# ── Control helpers ─────────────────────────────────────────────────────────
def _require():
    base = STATE.robot.base
    if base is None:
        raise RuntimeError("Kinova not connected")
    return base


def stop() -> None:
    _require().Stop()
    log.info("Robot stopped")


def _execute_named(names) -> bool:
    from kortex_api.autogen.messages import Base_pb2
    base = _require()
    req = Base_pb2.RequestedActionType()
    req.action_type = Base_pb2.REACH_JOINT_ANGLES
    for a in base.ReadAllActions(req).action_list:
        if a.name in names or a.name.lower() in names:
            base.ExecuteActionFromReference(a.handle)
            return True
    return False


def home() -> None:
    if not _execute_named({"Home"}):
        raise RuntimeError("Home action not found")
    log.info("Moving to Home")


def retract() -> None:
    if not _execute_named({"Retract"}):
        raise RuntimeError("Retract action not found")
    log.info("Moving to Retract")


def vertical() -> None:
    from kortex_api.autogen.messages import Base_pb2
    base = _require()
    if _execute_named({"vertical", "zero"}):
        log.info("Moving to Vertical")
        return
    action = Base_pb2.Action()
    action.name = "api_vertical"
    action.application_data = ""
    for i, deg in enumerate([0, -90, 0, -90, 0, 0, 0]):
        j = action.reach_joint_angles.joint_angles.joint_angles.add()
        j.joint_identifier = i
        j.value = deg
    base.ExecuteAction(action)
    log.info("Moving to Vertical (hardcoded)")


def move_joints(angles) -> None:
    from kortex_api.autogen.messages import Base_pb2
    base = _require()
    action = Base_pb2.Action()
    action.name = "api_joints"
    action.application_data = ""
    for i, v in enumerate(angles):
        j = action.reach_joint_angles.joint_angles.joint_angles.add()
        j.joint_identifier = i
        j.value = v
    base.ExecuteAction(action)
    log.info(f"Joint move: {[round(a, 1) for a in angles]}")


def move_pose(x, y, z, theta_x, theta_y, theta_z) -> None:
    from kortex_api.autogen.messages import Base_pb2
    base = _require()
    action = Base_pb2.Action()
    action.name = "api_pose"
    action.application_data = ""
    tp = action.reach_pose.target_pose
    tp.x, tp.y, tp.z = x, y, z
    tp.theta_x, tp.theta_y, tp.theta_z = theta_x, theta_y, theta_z
    base.ExecuteAction(action)
    log.info(f"Pose move: ({x:.3f}, {y:.3f}, {z:.3f})")


def set_gripper(position: float) -> None:
    from kortex_api.autogen.messages import Base_pb2
    base = _require()
    pos = max(0.0, min(1.0, position))
    gc = Base_pb2.GripperCommand()
    gc.mode = Base_pb2.GRIPPER_POSITION
    f = gc.gripper.finger.add()
    f.finger_identifier = 1
    f.value = pos
    base.SendGripperCommand(gc)
    log.info(f"Gripper → {int(pos * 100)}%")
