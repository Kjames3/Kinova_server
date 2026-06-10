"""Central configuration — every environment variable read in one place.

Import :data:`CFG` anywhere; nothing else in the package should call
``os.environ`` directly.
"""

from __future__ import annotations

import logging
import os
import secrets
from dataclasses import dataclass, field

log = logging.getLogger("kinova-server")


def _resolve_workspace_setup() -> str:
    """Locate the kortex workspace overlay (``install/setup.bash``).

    Honours ``ROS2_WORKSPACE_SETUP`` if set; otherwise probes the conventional
    locations so the subprocess manager (PLAN 3.1.2) works without manual config.
    """
    explicit = os.environ.get("ROS2_WORKSPACE_SETUP")
    if explicit:
        return explicit
    candidates = [
        "~/ros2_kortex_ws/install/setup.bash",
        "~/workspace/ros2_kortex_ws/install/setup.bash",
    ]
    for c in candidates:
        p = os.path.expanduser(c)
        if os.path.exists(p):
            return p
    return os.path.expanduser(candidates[0])


def _resolve_password() -> str:
    """Return the dashboard password.

    Security (PLAN 2.6): do **not** silently default to ``"kinova"``. If
    ``SERVER_PASSWORD`` is unset we generate a random one and print it to the
    console so the operator can copy it, rather than shipping a known default.
    """
    pwd = os.environ.get("SERVER_PASSWORD")
    if pwd:
        return pwd
    generated = secrets.token_urlsafe(12)
    log.warning(
        "SERVER_PASSWORD not set — generated a random password for this run:\n"
        "    %s\n"
        "    Set SERVER_PASSWORD in the environment to choose your own.",
        generated,
    )
    return generated


@dataclass
class Config:
    # ── HTTP server ────────────────────────────────────────────────────────
    host: str = "0.0.0.0"
    port: int = field(default_factory=lambda: int(os.environ.get("PORT", 8080)))
    password: str = field(default_factory=_resolve_password)

    # ── Kinova arm (kortex API, direct TCP) ────────────────────────────────
    kinova_ip: str = field(default_factory=lambda: os.environ.get("KINOVA_IP", "192.168.1.10"))
    kinova_port: int = field(default_factory=lambda: int(os.environ.get("KINOVA_PORT", 10000)))
    kinova_user: str = field(default_factory=lambda: os.environ.get("KINOVA_USER", "admin"))
    kinova_pass: str = field(default_factory=lambda: os.environ.get("KINOVA_PASS", "admin"))

    # ── ROS2 (Part 3) ──────────────────────────────────────────────────────
    # IP the ros2_kortex driver should dial (passed to the launch file).
    robot_ip: str = field(default_factory=lambda: os.environ.get("ROBOT_IP", "192.168.1.10"))
    # Workspace overlay to source before launching ROS2 subprocesses. If unset,
    # we try the conventional location used in PLAN 3.0.1.
    ros2_workspace_setup: str = field(default_factory=_resolve_workspace_setup)
    ros2_distro_setup: str = field(
        default_factory=lambda: os.environ.get("ROS2_DISTRO_SETUP", "/opt/ros/humble/setup.bash")
    )
    session_hours: int = 24


CFG = Config()
