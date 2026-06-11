"""Shared Pydantic request models."""

from __future__ import annotations

from typing import List, Optional

from pydantic import BaseModel


class LoginReq(BaseModel):
    password: str


class JointAngles(BaseModel):
    angles: List[float]


class CartesianPose(BaseModel):
    x: float
    y: float
    z: float
    theta_x: float
    theta_y: float
    theta_z: float


class GripperCmd(BaseModel):
    position: float  # 0.0–1.0


class EndEffectorReq(BaseModel):
    name: str  # key in state.END_EFFECTORS


class ProcessReq(BaseModel):
    process: str  # "system" | "fusion" | "pcfusion" | "wrist"
    args: Optional[dict] = None  # validated launch overrides (see processes.LAUNCH_ARGS)


class InsertReq(BaseModel):
    target_x: float = 0.0
    target_y: float = 0.0
    hover_above_top: float = 0.03
    dry_run: bool = False
    skip_home_move: bool = False
    # If true and target_x/y left at 0, the server pulls the live fused centre.
    use_dynamic_tf: bool = True
