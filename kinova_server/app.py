"""FastAPI application assembly + lifespan.

Starts the hardware capture threads and the async pumps, mounts the static
dashboard, and wires up every router. The ROS2 pieces (Part 3) are imported
lazily and degrade gracefully when ``rclpy`` is unavailable (e.g. on a Windows
dev box) so the rest of the server still runs.
"""

from __future__ import annotations

import asyncio
import os
import threading
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from .cameras.oakd import oakd_thread
from .cameras.realsense import realsense_thread
from .config import CFG
from .logs import log_pump, setup_logging
from .pumps import pc_pump_oakd, pc_pump_realsense, robot_pump
from .robot.kinova import kinova_thread
from .state import STATE

log = setup_logging()

_STATIC_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "static")


@asynccontextmanager
async def lifespan(app: FastAPI):
    STATE.bind_loop(asyncio.get_running_loop())

    # Async pumps
    tasks = [
        asyncio.create_task(log_pump()),
        asyncio.create_task(robot_pump()),
        asyncio.create_task(pc_pump_realsense()),
        asyncio.create_task(pc_pump_oakd()),
    ]

    # ROS2 bridge (optional — no-op if rclpy missing)
    try:
        from .ros2 import bridge as ros2_bridge
        ros2_bridge.start()
    except Exception as exc:  # pragma: no cover - depends on host
        log.warning(f"ROS2 bridge not started: {exc}")

    # Hardware capture threads
    threads = [
        threading.Thread(target=realsense_thread, daemon=True, name="realsense"),
        threading.Thread(target=oakd_thread, daemon=True, name="oakd"),
        threading.Thread(target=kinova_thread, daemon=True, name="kinova"),
    ]
    for t in threads:
        t.start()

    log.info(f"Server ready — http://0.0.0.0:{CFG.port}")
    try:
        yield
    finally:
        STATE.stop_event.set()
        for t in tasks:
            t.cancel()
        try:
            from .ros2.processes import MANAGER
            await MANAGER.shutdown()
        except Exception:
            pass
        try:
            from .ros2 import bridge as ros2_bridge
            ros2_bridge.stop()
        except Exception:
            pass


def create_app() -> FastAPI:
    app = FastAPI(title="Kinova Robot Server", version="2.1.0", lifespan=lifespan)
    app.add_middleware(
        CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"]
    )

    from .api import routes_auth, routes_cameras, routes_robot, ws
    app.include_router(routes_auth.router)
    app.include_router(routes_cameras.router)
    app.include_router(routes_robot.router)
    app.include_router(ws.router)

    # ROS2 routes (import-safe even without rclpy; endpoints report availability)
    try:
        from .api import routes_ros2
        app.include_router(routes_ros2.router)
    except Exception as exc:  # pragma: no cover
        log.warning(f"ROS2 routes unavailable: {exc}")

    # Dashboard
    app.mount("/static", StaticFiles(directory=_STATIC_DIR), name="static")

    @app.get("/", include_in_schema=False)
    def dashboard():
        return FileResponse(os.path.join(_STATIC_DIR, "index.html"))

    return app


app = create_app()
