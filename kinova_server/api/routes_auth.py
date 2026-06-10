"""Auth + status routes."""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException
from fastapi.responses import JSONResponse

from .. import auth as _auth
from ..state import STATE
from .models import LoginReq

router = APIRouter()


@router.post("/api/auth/login", include_in_schema=False)
def login(body: LoginReq):
    if not _auth.check_password(body.password):
        raise HTTPException(401, "Invalid password")
    return {"token": _auth.new_token()}


@router.post("/api/auth/logout", include_in_schema=False)
def logout(t: str = Depends(_auth.auth)):
    _auth.revoke(t)
    return {"ok": True}


@router.get("/api/status")
def get_status(t: str = Depends(_auth.auth)):
    return JSONResponse(STATE.status.copy())


@router.get("/api/system")
def get_system(t: str = Depends(_auth.auth)):
    try:
        import psutil
        return {"cpu": psutil.cpu_percent(interval=0.1), "mem": psutil.virtual_memory().percent}
    except ImportError:
        return {"cpu": 0, "mem": 0}
