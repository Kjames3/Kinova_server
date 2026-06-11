"""Robot state + control routes. Thin wrappers over robot.kinova helpers."""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException
from fastapi.responses import JSONResponse

from .. import auth as _auth
from ..robot import kinova
from ..state import END_EFFECTORS, STATE
from .models import CartesianPose, EndEffectorReq, GripperCmd, JointAngles

router = APIRouter()


@router.get("/api/robot/state")
def robot_state(t: str = Depends(_auth.auth)):
    return JSONResponse(STATE.robot.snapshot())


# ── End-effector selection (PLAN GUI item 7) ────────────────────────────────
@router.get("/api/robot/end_effector")
def get_end_effector(t: str = Depends(_auth.auth)):
    return {"selected": STATE.end_effector, "options": END_EFFECTORS}


@router.post("/api/robot/end_effector")
def set_end_effector(body: EndEffectorReq, t: str = Depends(_auth.auth)):
    if body.name not in END_EFFECTORS:
        raise HTTPException(400, f"unknown end effector '{body.name}'")
    STATE.end_effector = body.name
    return {"selected": STATE.end_effector, "config": END_EFFECTORS[body.name]}


def _run(fn, *a):
    try:
        fn(*a)
        return {"ok": True}
    except RuntimeError as e:
        raise HTTPException(503, str(e))
    except Exception as e:
        raise HTTPException(500, str(e))


@router.post("/api/robot/stop")
def robot_stop(t: str = Depends(_auth.auth)):
    return _run(kinova.stop)


@router.post("/api/robot/home")
def robot_home(t: str = Depends(_auth.auth)):
    return _run(kinova.home)


@router.post("/api/robot/retract")
def robot_retract(t: str = Depends(_auth.auth)):
    return _run(kinova.retract)


@router.post("/api/robot/vertical")
def robot_vertical(t: str = Depends(_auth.auth)):
    return _run(kinova.vertical)


@router.post("/api/robot/joints")
def robot_joints(body: JointAngles, t: str = Depends(_auth.auth)):
    if len(body.angles) != 7:
        raise HTTPException(400, "Need 7 joint angles (degrees)")
    return _run(kinova.move_joints, body.angles)


@router.post("/api/robot/pose")
def robot_pose(body: CartesianPose, t: str = Depends(_auth.auth)):
    return _run(kinova.move_pose, body.x, body.y, body.z,
                body.theta_x, body.theta_y, body.theta_z)


@router.post("/api/robot/gripper")
def robot_gripper(body: GripperCmd, t: str = Depends(_auth.auth)):
    return _run(kinova.set_gripper, body.position)
