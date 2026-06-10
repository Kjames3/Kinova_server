# Kinova_server

Remote control + vision dashboard for a Kinova Gen3 7-DOF arm, an Intel
RealSense D435i and a Luxonis OAK-D Pro W, with an in-process ROS2 Humble bridge.

## Run

```bash
# Source ROS2 first so rclpy is importable (Part 3). Skip if you only need
# the arm/cameras without the ROS2 panel.
source /opt/ros/humble/setup.bash
source ~/workspace/ros2_kortex_ws/install/setup.bash

source venv/bin/activate
export SERVER_PASSWORD='choose-a-password'      # required; a random one is
                                                # generated + printed if unset
python server.py
```

Open `http://<host>:8080` and log in.

## Layout

```
server.py              thin entry point (python server.py still works)
kinova_server/         the application package
  config.py            all env vars in one place
  auth.py              token auth (constant-time, WS token via Sec-WebSocket-Protocol)
  state.py             FrameStore / RobotState / Broadcaster — no module globals
  logs.py              WS log handler + broadcaster
  pumps.py             shared robot-state + point-cloud broadcast tasks
  pointcloud.py        RealSense / OAK-D point-cloud compute
  cameras/             realsense.py, oakd.py (subscriber-aware capture threads)
  robot/kinova.py      kortex driver thread + control helpers
  ros2/                bridge.py, processes.py, action_bridge.py (Part 3)
  api/                 routes_auth, routes_cameras, routes_robot, routes_ros2, ws
  app.py               FastAPI assembly + lifespan
static/                index.html, app.js, viewers.js, style.css (the dashboard)
```

## ROS2 panel (Part 3)

The dashboard's **Automation & ROS 2 Control** panel exposes:
- **Subprocess control** — start/stop `gen3_complete_system.launch.py` and
  `combine_cameras.py`, with live console logs.
- **Fusion diagnostics** — live `/fused_marker_square_center` centre + corner count.
- **Container insertion** — drives the `/insert_container` action with live
  phase/progress feedback and an emergency cancel.

The insertion action requires the `InsertContainer.action` interface to be built
into the workspace (see `install.sh` / PLAN 3.0.1). Until then the panel works
but `START CONTAINER INSERTION` returns "action interface not built".

See [PLAN.md](PLAN.md) for the full design rationale.
