# Kinova Server — Performance Refactor & ROS2 Humble Integration Plan

> Status: planned, not yet implemented. Written 2026-06-10.
> Target runtime host: the Linux Kinova desktop (Ubuntu 22.04, `kinova@10.12.140.145`).
> Dev machine: Windows 11 (browser dashboard works from Windows today; ROS2 work happens on the Linux host).

---

## Part 1 — Restructure the project (split the script)

Splitting **is** the right call here, but not primarily for speed — `server.py` mixes an 840-line
HTML/JS string with hardware drivers and routes, which makes every other improvement harder.
The split itself is performance-neutral; it unlocks the Part 2 fixes and the Part 3 ROS2 module.

### Target layout

```
Kinova_server/
├── server.py                  # thin entry point: `python server.py` still works
├── kinova_server/
│   ├── __init__.py
│   ├── config.py              # env vars (PORT, SERVER_PASSWORD, KINOVA_*) — one place
│   ├── auth.py                # tokens, login/logout, _auth dependency
│   ├── state.py               # FrameStore + RobotState classes (replaces module globals)
│   ├── logs.py                # WS log handler + broadcaster
│   ├── pointcloud.py          # _compute_pc_rs / _compute_pc_oak
│   ├── cameras/
│   │   ├── realsense.py       # _realsense_thread
│   │   └── oakd.py            # _oakd_thread
│   ├── robot/
│   │   └── kinova.py          # _kinova_thread + control helpers (home/retract/joints/...)
│   ├── ros2/                  # Part 3 (new)
│   │   └── bridge.py
│   └── api/
│       ├── routes_auth.py
│       ├── routes_cameras.py
│       ├── routes_robot.py
│       ├── routes_ros2.py     # Part 3 (new)
│       └── ws.py              # /ws/robot, /ws/logs, /ws/terminal, /ws/pointcloud/*
├── static/
│   ├── index.html             # the dashboard, extracted from the _DASHBOARD string
│   ├── app.js
│   ├── viewers.js             # FK arm viewer + point-cloud viewer
│   └── style.css
├── requirements.txt
└── install.sh
```

Key moves:
1. Extract `_DASHBOARD` → `static/` files, served with `StaticFiles` + `FileResponse("static/index.html")`
   at `/`. Browser can cache JS/CSS; editing the UI no longer means editing Python.
2. Replace module-level globals (`_rs_rgb_frame`, `_robot_state`, …) with two small classes in
   `state.py` passed into the threads and routers (or held on `app.state`). This removes the
   `global` statements and makes per-camera locking (below) natural.
3. Hardware threads keep the exact same logic — they just move files and receive a `FrameStore`
   instead of writing globals.
4. Routers register via `app.include_router(...)`; lifespan stays in `server.py`/`main.py`.

Acceptance check: `python server.py` behaves identically (login, streams, robot control, point cloud).

---

## Part 2 — Performance fixes (ordered by impact)

### 2.1 Stop JPEG-encoding frames nobody is watching  (biggest CPU win)
Both camera threads encode **two JPEGs per frame at 30 fps** (~120 encodes/s total) even with zero
viewers, plus a full BGR→RGB `.copy()` and a float32 depth conversion (~1.2 MB/frame) for point
clouds that may not be streaming.

Fix: maintain subscriber counts in `FrameStore` (incremented by MJPEG generators / PC websockets).
- `if store.jpeg_subscribers == 0: skip imencode` — keep only the latest raw frame.
- Defer `depth.astype(np.float32) * scale` and BGR→RGB into `_compute_pc_*` (they already run in
  an executor); camera threads then just store the raw arrays they already have.

### 2.2 Event-driven frame fan-out instead of per-client polling
`_mjpeg_stream` polls every 1/30 s per client and compares identity. With N viewers that's N wake-ups
× 30/s each, plus N threading-lock acquisitions inside the event loop.

Fix: one `asyncio.Event` (or Condition) per stream, set from the camera thread via
`loop.call_soon_threadsafe(event.set)`. Generators `await event.wait()` → yield → clear. Clients wake
exactly once per new frame, never spin when the camera stalls.

Same pattern for `/ws/robot`: serialize the state **once** per tick and broadcast to all connected
sockets, instead of each socket copying + serializing independently at 20 Hz.

### 2.3 Per-resource locks (or lock-free swaps)
One global `_frame_lock` serializes RealSense writes, OAK-D writes, MJPEG reads, and point-cloud
reads. Fix: a lock per camera inside `FrameStore` — or, since these are single-writer
reference swaps, publish an immutable tuple `(jpeg, raw_color, raw_depth, intrinsics, seq)` per
camera and swap the reference atomically (atomic under the GIL); readers grab the tuple once,
no lock at all.

### 2.4 Share point-cloud computation between clients
Each `/ws/pointcloud/*` client triggers its own `_compute_pc_*` every 250 ms. Fix: one background
task per source computes when ≥1 subscriber exists; all sockets send the same packed bytes.

### 2.5 Kinova poll loop
`RefreshFeedback()` + `GetMeasuredCartesianPose()` are two blocking TCP RPCs every 50 ms. The pose
RPC is the expensive one and changes slowly relative to joints:
- Poll feedback at 20 Hz, pose at 5 Hz (or derive pose from FK — the dashboard already has the DH table).
- Optional later: switch to the kortex cyclic UDP channel for high-rate feedback.

### 2.6 Small items
- `_broadcast_logs` sleeps 100 ms forever even with no subscribers → skip drain when
  `_log_subscribers` is empty, or block on the queue in an executor.
- `datetime.utcnow()` is deprecated → `datetime.now(timezone.utc)`.
- Password check: use `secrets.compare_digest` on the hashes; consider PBKDF2/argon2 and **not**
  defaulting `SERVER_PASSWORD` to `"kinova"` (refuse to start without it, or generate and print one).
- Token in query string leaks into logs; the WS endpoints can read it from `Sec-WebSocket-Protocol`
  or first message instead (HTTP routes already accept the Bearer header).
- MJPEG/snapshot routes are `def` (threadpool) returning async generators — make them `async def`
  for clarity; with 2.2 they do no blocking work.
- Run with `uvloop` on the Linux host (already pulled in by `uvicorn[standard]`); keep a single
  worker (shared in-process state requires it — this is fine, the workload is I/O + a few threads).

What **not** to do: multiprocessing/multiple uvicorn workers (breaks shared frame state for no
benefit), rewriting camera threads as asyncio (the SDKs are blocking; threads are correct here).

---

## Part 3 — ROS2 Humble integration (topics from robot + cameras)

ROS2 Humble targets Ubuntu 22.04 — it runs on the **Kinova desktop**, not Windows. Windows stays
the client: the existing browser dashboard (plus optionally Foxglove Studio) views everything.

### 3.0 Prerequisites on the Linux host
```bash
# Ubuntu 22.04
sudo apt install ros-humble-ros-base ros-humble-cv-bridge ros-humble-image-transport
# Drivers (choose what's needed):
sudo apt install ros-humble-realsense2-camera        # RealSense D435i ROS2 node
sudo apt install ros-humble-depthai-ros              # OAK-D ROS2 nodes
# Kinova arm driver (builds from source):
#   https://github.com/Kinovarobotics/ros2_kortex  (humble branch) — colcon build in a workspace
```
Note: `rclpy` must come from the ROS install (`source /opt/ros/humble/setup.bash` before starting
the server), not pip. Update `install.sh` and the systemd/launch instructions accordingly.

#### 3.0.1 Compile Workspace Action Interfaces
To enable the FastAPI backend to call the robot insertion script via ROS 2 actions, the `InsertContainer.action` interface must be built in the workspace:
1. Verify `kortex_bringup/action/InsertContainer.action` is present in the workspace.
2. Build the workspace using:
   ```bash
   colcon build --executor sequential --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_PARALLEL_LEVEL=2
   ```
3. Source the workspace environment before launching `server.py`:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/workspace/ros2_kortex_ws/install/setup.bash
   ```

### 3.1 In-server ROS2 bridge (`kinova_server/ros2/bridge.py`) — primary approach
A single `rclpy` node spinning in its own daemon thread (same pattern as the camera threads),
gracefully disabled when `rclpy` isn't importable (so the server still runs on Windows for dev):

- **Topic discovery**: `node.get_topic_names_and_types()` → `GET /api/ros2/topics`.
- **Generic topic echo**: dynamic subscription via
  `rosidl_runtime_py.utilities.get_message(type_str)` +
  `rosidl_runtime_py.message_to_ordereddict(msg)` → JSON over `WS /ws/ros2/topic?name=/joint_states`.
  Subscriptions are created on first subscriber and torn down on last disconnect (mirrors 2.4).
- **Image topics**: subscribe `sensor_msgs/Image` / `CompressedImage`, convert with `cv_bridge`,
  JPEG-encode into the same `FrameStore` → `GET /api/ros2/image/{topic}` reuses the existing
  MJPEG generator. Camera feeds from any ROS2 node (or the robot itself) appear like the local cameras.
- **Robot topics of interest** (from `ros2_kortex`): `/joint_states`, `/tf`,
  `/robotiq_gripper_controller/...`, controller state topics.
- Dashboard: new "ROS2" panel — topic list with type + Hz, click to echo (JSON view) or view (image).

#### 3.1.2 ROS 2 Subprocess Manager (`kinova_server/ros2/processes.py`)
To avoid manual CLI management, the server will handle the lifecycle of the ROS 2 camera launcher and fusion node:
- **State tracker**: Keep track of running subprocess handles (`asyncio.subprocess.Process`).
- **Endpoints**:
  - `POST /api/ros2/process/start` with body `{"process": "system" | "fusion"}`
  - `POST /api/ros2/process/stop` with body `{"process": "system" | "fusion"}`
  - `GET /api/ros2/process/status` returning running/crashed/idle states.
- **Log streaming**: WebSocket route `/ws/ros2/process/logs?name=system` to stream stdout/stderr of the `ros2 launch` or `ros2 run` command directly to a dashboard terminal view.
- **Commands managed**:
  - **System**: `ros2 launch kortex_bringup gen3_complete_system.launch.py robot_ip:=192.168.1.10` (runs `oak_camera_node.py` as a subprocess node).
  - **Fusion**: `ros2 run kortex_bringup combine_cameras.py`

#### 3.1.3 Action Client & Telemetry Bridge (`kinova_server/ros2/action_bridge.py`)
- **Action Client (`/insert_container`)**:
  - Expose `POST /api/ros2/insert/start` to trigger the insertion task with configurable options (`hover_above_top`, `approach_clearance`, `dry_run`, `use_dynamic_tf`).
  - Expose `POST /api/ros2/insert/cancel` to issue a cancel request to the active action goal.
  - Expose WebSocket `/ws/ros2/insert/feedback` to stream the current task phase (e.g., `Phase 0 — Approach`, `Phase 1 — Descend`) and completion progress percentage.
- **Sensor Fusion Subscription**:
  - Subscribe to `/fused_marker_square_center` (PoseStamped) and `/fused_corners` (PoseArray).
  - Extract the raw X, Y, Z coordinates and orientation.
  - Stream these values over `WS /ws/ros2/fusion` to power a live sensor diagnostics panel.

### 3.2 Standard bridges (low-code alternatives / complements)
- **rosbridge_suite**: `ros-humble-rosbridge-server` exposes ws://host:9090; the dashboard could use
  `roslibjs` directly. Cheap to add, but duplicates auth (rosbridge has none by default — bind to
  localhost and proxy through FastAPI, or firewall it).
- **foxglove_bridge** (`ros-humble-foxglove-bridge`): best-in-class visualization (3D, plots, images)
  from Windows via Foxglove Studio with zero custom UI work. Recommended as a *complement* for
  debugging even if 3.1 is built.

### 3.3 Publishing (later, optional)
Mirror server state *into* ROS2: publish camera frames and a `sensor_msgs/JointState` from the
kortex feedback so this server can feed an existing ROS2 stack (MoveIt 2, etc.) without ros2_kortex.

### 3.4 Windows dev story
- Browser dashboard + Foxglove Studio (native Windows app) cover viewing.
- For native ROS2 CLI tooling on Windows, use **WSL2 Ubuntu 22.04** + Humble; DDS discovery across
  the LAN from WSL2 needs either `ROS_DOMAIN_ID` + mirrored networking (Win11 supports
  `networkingMode=mirrored` in `.wslconfig`) or simply rely on the bridges above.

### 3.5 Dashboard Frontend UI Additions
Introduce a new **"Automation & ROS 2 Control"** panel in the dashboard UI:
1. **Subprocess Controllers**: Toggle buttons to start/stop the Complete System and Camera Fusion, showing status badges (Idle, Running, Error) and a collapsable live command line console log.
2. **Camera Fusion Diagnostics**:
   - Live displays for **Fused Center** coordinates: `X: +0.299 m`, `Y: -0.192 m`, `Z: +0.056 m`.
   - Corner detection metrics (e.g. `4/4 (Perfect)`, `3/4 (Reconstructed)`, or `0/4 (Awaiting Markers)`).
   - Feed statuses showing which camera feeds are active (`RealSense: Online`, `OAK-D: Online`, `Kinova Wrist: Offline`).
3. **Container Insertion Controller**:
   - Inputs for clearance, hover distance, and a `Dry Run` toggle.
   - A large **"START CONTAINER INSERTION"** execution button and a red **"EMERGENCY CANCEL"** button.
   - A live progress bar showing the active movement phases.

---

## Suggested execution order

| Step | What | Effort | Risk |
|------|------|--------|------|
| 1 | Extract dashboard → `static/`, split into package (Part 1) | ~half day | Low — mechanical |
| 2 | FrameStore + subscriber-aware encoding (2.1, 2.3) | small | Low |
| 3 | Event-driven MJPEG + shared robot-WS broadcast (2.2) | small | Low |
| 4 | Shared point-cloud task (2.4) + Kinova poll split (2.5) | small | Low |
| 5 | Security/cleanup items (2.6) | small | Low |
| 6 | ROS2 bridge module + topics API + image relay (3.1) | ~1 day on the Linux box | Medium — needs hardware |
| 7 | Build workspace actions and implement subprocess manager (`processes.py`) (3.0.1, 3.1.2) | small | Low — process lifecycle |
| 8 | Implement action client interface for `insert_to_container` and subscribe to fusion telemetry (3.1.3) | ~1 day | Medium — needs hardware |
| 9 | Add dashboard ROS2 panel including process managers, diagnostics, and insertion progress controls (3.5) | ~half day | Low |
| 10 | foxglove_bridge / rosbridge install & standard bridges (3.2) | small | Low |

Steps 1–5 can be done and tested on Windows (threads degrade gracefully when SDKs are absent —
they already do). Steps 6–10 require the Ubuntu 22.04 host with ROS2 Humble sourced.
