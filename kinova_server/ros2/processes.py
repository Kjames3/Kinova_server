"""ROS2 subprocess lifecycle manager (PLAN 3.1.2).

Manages the two long-running ROS2 commands the operator would otherwise start
by hand on the CLI:

* ``system`` — ``ros2 launch kortex_bringup gen3_complete_system.launch.py
  robot_ip:=<CFG.robot_ip>`` (brings up the arm driver + cameras)
* ``fusion`` — ``ros2 run kortex_bringup combine_cameras.py``

Each command is wrapped in ``bash -lc`` that sources the ROS distro and the
workspace overlay first, so it works regardless of how the server itself was
launched. stdout/stderr is merged, kept in a ring buffer, and fanned out to
``/ws/ros2/process/logs`` subscribers.
"""

from __future__ import annotations

import asyncio
import logging
import os
import signal
from collections import deque
from typing import Deque, Dict, Optional

from ..config import CFG
from ..state import Broadcaster

log = logging.getLogger("kinova-server")

# name → shell command (before sourcing wrapper)
COMMANDS: Dict[str, str] = {
    "system": f"ros2 launch kortex_bringup gen3_complete_system.launch.py robot_ip:={CFG.robot_ip}",
    "fusion": "ros2 run kortex_bringup combine_cameras.py",
    # PLAN GUI item 4 — point-cloud fusion node → publishes /fused_pointcloud
    "pcfusion": "ros2 launch kinova_gen3_7dof_robotiq_2f_140_moveit_config fusion.launch.py",
    # PLAN GUI item 6 — Kinova wrist (bracelet) vision → /camera/color/image_raw
    "wrist": f"ros2 launch kinova_vision kinova_vision.launch.py device:={CFG.robot_ip}",
}


class _ManagedProcess:
    def __init__(self, name: str) -> None:
        self.name = name
        self.proc: Optional[asyncio.subprocess.Process] = None
        self.state = "idle"   # idle | running | crashed | stopped
        self.returncode: Optional[int] = None
        self.log_buffer: Deque[str] = deque(maxlen=500)
        self.broadcaster = Broadcaster()
        self._reader: Optional[asyncio.Task] = None

    @property
    def running(self) -> bool:
        return self.proc is not None and self.proc.returncode is None


class ProcessManager:
    def __init__(self) -> None:
        self._procs: Dict[str, _ManagedProcess] = {n: _ManagedProcess(n) for n in COMMANDS}

    def _wrap(self, command: str) -> str:
        parts = []
        if os.path.exists(CFG.ros2_distro_setup):
            parts.append(f"source {CFG.ros2_distro_setup}")
        if os.path.exists(CFG.ros2_workspace_setup):
            parts.append(f"source {CFG.ros2_workspace_setup}")
        parts.append(f"exec {command}")
        return " && ".join(parts)

    def get(self, name: str) -> Optional[_ManagedProcess]:
        return self._procs.get(name)

    async def start(self, name: str) -> Dict:
        if name not in COMMANDS:
            return {"ok": False, "error": f"unknown process '{name}'"}
        mp = self._procs[name]
        if mp.running:
            return {"ok": False, "error": f"{name} already running", "state": mp.state}

        wrapped = self._wrap(COMMANDS[name])
        mp.log_buffer.clear()
        try:
            mp.proc = await asyncio.create_subprocess_exec(
                "bash", "-lc", wrapped,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
                start_new_session=True,   # own process group → clean kill
            )
        except Exception as exc:
            mp.state = "crashed"
            return {"ok": False, "error": str(exc)}

        mp.state = "running"
        mp.returncode = None
        mp._reader = asyncio.create_task(self._pump_output(mp))
        log.info(f"ROS2 process '{name}' started (pid {mp.proc.pid})")
        return {"ok": True, "state": mp.state, "pid": mp.proc.pid}

    async def _pump_output(self, mp: _ManagedProcess) -> None:
        assert mp.proc and mp.proc.stdout
        try:
            async for raw in mp.proc.stdout:
                line = raw.decode(errors="replace").rstrip("\n")
                mp.log_buffer.append(line)
                mp.broadcaster.publish(line)
        except Exception:
            pass
        rc = await mp.proc.wait()
        mp.returncode = rc
        # rc 0 or negative (killed by us) → stopped; otherwise crashed
        if mp.state == "running":
            mp.state = "stopped" if rc in (0, -signal.SIGTERM, -signal.SIGINT) else "crashed"
        msg = f"[process '{mp.name}' exited rc={rc}, state={mp.state}]"
        mp.log_buffer.append(msg)
        mp.broadcaster.publish(msg)
        log.info(f"ROS2 {msg}")

    async def stop(self, name: str) -> Dict:
        mp = self._procs.get(name)
        if not mp or not mp.running:
            return {"ok": False, "error": f"{name} not running"}
        try:
            os.killpg(os.getpgid(mp.proc.pid), signal.SIGINT)
            try:
                await asyncio.wait_for(mp.proc.wait(), timeout=8.0)
            except asyncio.TimeoutError:
                os.killpg(os.getpgid(mp.proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        except Exception as exc:
            return {"ok": False, "error": str(exc)}
        mp.state = "stopped"
        log.info(f"ROS2 process '{name}' stopped")
        return {"ok": True, "state": mp.state}

    def status(self) -> Dict:
        return {
            name: {
                "state": mp.state,
                "running": mp.running,
                "pid": mp.proc.pid if mp.running else None,
                "returncode": mp.returncode,
                "command": COMMANDS[name],
            }
            for name, mp in self._procs.items()
        }

    async def shutdown(self) -> None:
        for name, mp in self._procs.items():
            if mp.running:
                await self.stop(name)


MANAGER = ProcessManager()
