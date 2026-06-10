#!/usr/bin/env python3
"""Kinova Gen3 7-DOF · RealSense D435i · OAK-D Pro W — Remote Server.

Thin entry point. All logic lives in the ``kinova_server`` package; this file
only exists so ``python server.py`` keeps working.

Run:  python server.py
Open: http://<host>:8080

Env vars:
  PORT, SERVER_PASSWORD, KINOVA_IP, KINOVA_PORT, KINOVA_USER, KINOVA_PASS,
  ROBOT_IP, ROS2_WORKSPACE_SETUP, ROS2_DISTRO_SETUP

ROS2 (Part 3) requires sourcing the ROS install + workspace overlay BEFORE
launching, so rclpy is importable:
  source /opt/ros/humble/setup.bash
  source ~/workspace/ros2_kortex_ws/install/setup.bash
  python server.py
"""

import uvicorn

from kinova_server.app import app
from kinova_server.config import CFG

if __name__ == "__main__":
    # uvloop ships with uvicorn[standard] on Linux and is selected automatically.
    uvicorn.run(app, host=CFG.host, port=CFG.port, log_level="info")
