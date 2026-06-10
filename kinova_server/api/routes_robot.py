"""Robot state + control routes. Thin wrappers over robot.kinova helpers."""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException
from fastapi.responses import JSONResponse

from .. import auth as _auth
from ..robot import kinova
from ..state import STATE
from .models import CartesianPose, GripperCmd, JointAngles

router = APIRouter()


@router.get("/api/robot/state")
def robot_state(t: str = Depends(_auth.auth)):
    return JSONResponse(STATE.robot.snapshot())


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
