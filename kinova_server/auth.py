"""Token auth (PLAN 2.6).

* Hash comparison uses :func:`secrets.compare_digest` (constant-time).
* Timestamps use timezone-aware ``datetime.now(timezone.utc)``.
* WebSocket auth accepts the token from the ``Sec-WebSocket-Protocol`` header
  first (so it never lands in access logs) and falls back to the query string
  for the MJPEG ``<img>`` tags that cannot set headers.
"""

from __future__ import annotations

import hashlib
import secrets
from datetime import datetime, timedelta, timezone
from typing import Dict, Optional

from fastapi import HTTPException, Request, WebSocket

from .config import CFG

_sessions: Dict[str, datetime] = {}
_PWD_HASH = hashlib.sha256(CFG.password.encode()).hexdigest()


def check_password(candidate: str) -> bool:
    cand_hash = hashlib.sha256(candidate.encode()).hexdigest()
    return secrets.compare_digest(cand_hash, _PWD_HASH)


def new_token() -> str:
    tok = secrets.token_hex(32)
    _sessions[tok] = datetime.now(timezone.utc) + timedelta(hours=CFG.session_hours)
    return tok


def valid_token(tok: str) -> bool:
    exp = _sessions.get(tok)
    if not exp:
        return False
    if datetime.now(timezone.utc) > exp:
        del _sessions[tok]
        return False
    return True


def revoke(tok: str) -> None:
    _sessions.pop(tok, None)


def auth(request: Request, token: str = "") -> str:
    """FastAPI dependency for HTTP routes (Bearer header or ?token=)."""
    auth_header = request.headers.get("Authorization", "")
    t = token or (auth_header[7:] if auth_header.startswith("Bearer ") else "")
    if not valid_token(t):
        raise HTTPException(401, "Unauthorized — invalid or missing token")
    return t


def ws_token(ws: WebSocket, token: str = "") -> Optional[str]:
    """Extract a token from a WebSocket handshake.

    Prefers the ``Sec-WebSocket-Protocol`` header (kept out of logs); falls
    back to the ``?token=`` query parameter. Returns the token if valid, else
    ``None``.
    """
    proto = ws.headers.get("sec-websocket-protocol", "")
    candidate = proto.split(",")[0].strip() if proto else ""
    t = candidate or token
    return t if valid_token(t) else None
