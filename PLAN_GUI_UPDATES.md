# Kinova Server — Dashboard/Viewer Update Plan

> Requested 2026-06-10. Follow-on to [PLAN.md](PLAN.md) after the package split + ROS2 work
> shipped. Grounded in the actual `ros2_kortex_ws` contents on the dev laptop / desktop.
> Items 6 and parts of 3/7 need the arm online (expected back the next day).

| # | Item | Effort | Risk | Needs arm? |
|---|------|--------|------|-----------|
| 1 | Dark-grey theme | small | low | no |
| 2 | Rotate viewer table 90° (Y) | small | low | no |
| 3 | URDF arm model in 3D viewer | ~1–1.5 day | medium | no (visual only) |
| 4 | Fused point cloud (`/fused_pointcloud`) | ~half day | medium | cameras only |
| 5 | Scroll overflowing panels | small | low | no |
| 6 | Wrist (bracelet) camera feed | ~half day | low | **yes** |
| 7 | End-effector selector (assembly tip / 2F-140) | ~1 day | medium | partly |

---

## 1. Dark-grey theme (vs near-black)

Single-file change in [static/style.css](static/style.css) (+ two WebGL clear colors in
[static/viewers.js](static/viewers.js)).

- Introduce CSS custom properties at `:root` so the palette lives in one place:
  `--bg`, `--panel`, `--panel-2`, `--inset`, `--border`, `--accent (#4fc3f7)`.
- Remap the current near-black scale → dark grey:
  - `body #111 → #1e1e1e`, header `#0c0c0c → #262626`, panels `#181818 → #2a2a2a`,
    insets/log/canvas `#0a0a0a/#080808/#060606 → #1a1a1a`, camera body `#000 → #141414`.
- Update `armRenderer.setClearColor(0x0a0a0a)` and `pcRenderer.setClearColor(0x080808)`
  → `0x1e1e1e` so the 3D canvases match.
- Keep accent/status colors; only the greyscale shifts. Acceptance: nothing pure-black,
  text contrast still AA.

## 2. Rotate the viewer table 90° about Y

In [static/viewers.js](static/viewers.js) `initArmViewer()`. The table is a square
`BoxGeometry(0.9, 0.015, 0.9)` parented to `armGroup` (which carries `rotation.x = -π/2`
to map the DH Z-up frame to Three.js Y-up), positioned at `z = -0.008` in the DH frame.

- Apply the requested **+90° about world Y** to the table (and the `GridHelper`) and
  re-seat it as the ground plane directly under the base cylinder so the arm sits *on* it,
  not *in* it. Because the table is square, a pure Y-rotation is visually ambiguous — so
  the real fix is to verify against a screenshot and adjust rotation **and** offset until
  the base rests on the surface. Likely also nudge `table.position` to sit at the base Z.
- Acceptance: base cylinder sits on the table top; no link passes through the slab.

## 3. URDF arm model (replace cylinders/spheres)

Assets already in the workspace:
- Arm visual meshes: `kortex_description/arms/gen3/7dof/meshes/*.dae`
  (`base_link`, `shoulder_link`, `half_arm_1/2_link`, `forearm_link`,
  `spherical_wrist_1/2_link`, `bracelet_with_vision_link`).
- Gripper meshes: `robotiq_description/meshes/visual/2f_140/*.stl` and the assembly tip
  `kortex_description/grippers/thesis_ee/*.STL` (see item 7).

Steps:
1. **Generate a flat URDF** on the desktop: `xacro gen3_robotiq_2f_140.xacro > gen3.urdf`
   (one per selectable EE). Commit the generated URDFs + referenced meshes into
   `static/models/` (or serve via a read-only `/api/models/{path}` route that maps the
   description packages — avoids copying large trees).
2. **Browser loader**: add `urdf-loader` + `ColladaLoader` (.dae) + `STLLoader` (.stl)
   (three r128-compatible builds). Parse the URDF, build the link tree, attach meshes.
3. **Drive it** from the existing `/ws/robot` joint stream — set each revolute joint angle
   each tick (replaces `updateArm()` cylinder math; keep the old renderer behind a
   "simple/URDF" toggle as a fallback if a mesh fails to load).
4. Coordinate-frame + scale sanity (meshes are in metres, Z-up like the DH frame).

Risk: loader/CDN availability for r128, .dae material handling, scale. Mitigation:
pre-convert `.dae/.stl → .glb` offline and use `GLTFLoader` (most robust in three).

## 4. Fused point cloud from both cameras

Source confirmed: `kortex_moveit_config/.../launch/fusion.launch.py` runs
`~/workspace/ros2_kortex_ws/pointcloud_fusion.py`, publishing **`/fused_pointcloud`**
(`sensor_msgs/PointCloud2`, frame `world`, 10 Hz).

Steps:
1. **Copy `pointcloud_fusion.py`** to the desktop ws root (it sits beside the scripts
   already copied) — prerequisite for running the fusion node.
2. **Server**: add a `PointCloud2 → packed XYZRGB` decoder in `kinova_server/ros2/`
   (read `x,y,z` + the packed `rgb` field, downsample to `MAX_PC = 12000`). A shared task
   (mirrors the existing per-camera PC task) subscribes to `/fused_pointcloud` only when a
   client selects it, and emits the **same binary frame format** the viewer already parses
   (`<I count><xyz f32><rgb u8>`), so `viewers.js` needs no new parser.
3. **UI**: add a third option `Fused (ROS2)` to the point-cloud source `<select>`; route
   `/ws/pointcloud/fused` to the new task. Add a "Point-cloud Fusion" button to the ROS2
   subprocess panel to launch `fusion.launch.py`.

Risk: PointCloud2 RGB packing variants + decimation cost. Decode in an executor thread.

## 5. Scroll overflowing panels

In [static/style.css](static/style.css) / [static/index.html](static/index.html). The
right sidebar (`.sr`) already has `overflow-y:auto`, but the `flex` grow sections
(log/terminal) squeeze the new ROS2 panel, and expanded process consoles get clipped.

- Make `.sr` a clean scroll container: fixed sections keep natural height; cap the
  log/terminal panes with `max-height` instead of unbounded `flex:1`; ensure
  `min-height:0` on flex children so they don't overflow their parent.
- Give the center `.cams` column its own `overflow-y:auto`.
- Acceptance: launch both subprocesses, open both consoles, expand the insertion panel —
  everything reachable by scrolling, nothing clipped off-screen.

## 6. Wrist (bracelet) camera  — needs arm

The 7-DOF arm carries the vision module (`bracelet_with_vision_link`).
`ros2_kortex_vision/launch/kinova_vision.launch.py` pulls `rtsp://<robot_ip>` and
publishes `/camera/color/image_raw` (+ `depth/image_raw`, compressed variants).

Steps:
1. Reuse the **existing ROS2 image relay**: `GET /api/ros2/image?name=/camera/color/image_raw`
   already MJPEG-relays any ROS2 image topic via cv_bridge.
2. Add a **"KINOVA WRIST"** camera panel (third panel) whose `<img>` points at that relay.
3. Add a subprocess-panel button to launch `kinova_vision.launch.py` (with `robot_ip`).
4. Update the "Kinova Wrist" feed-status line in the fusion-diagnostics block to reflect it.

Alternative if ROS vision is flaky: an OpenCV RTSP capture thread reading
`rtsp://<robot_ip>/color` directly into a `FrameStore` (no ROS dependency).

## 7. End-effector selector (assembly tip ↔ Robotiq 2F-140)

Two EEs available:
- **Robotiq 2F-140** — `robotiq_description` URDF + `meshes/visual/2f_140`.
- **Assembly tip** — `kortex_description/grippers/thesis_ee/Full_Assembly.STL` (the
  insertion tip; TCP offset referenced by `scripts/test_pen_tip_offset.py` and the insert
  scripts).

Steps:
1. **State**: `GET/POST /api/robot/end_effector` holding the active EE + its TCP offset.
2. **3D viewer (item 3)**: load the matching gripper subtree/mesh on the wrist.
3. **Tool frame**: apply the selected TCP offset to the displayed pose and pass the chosen
   tool to the insertion action (`insert_to_container.py` expects a tool/pen-tip offset).
4. **Gripper UI**: enable the gripper slider only for the 2F-140; for the assembly tip show
   it as a rigid tool.
5. **UI**: a dropdown in the left ROBOT ARM panel; selection persists in `localStorage`.

Risk: correct TCP offsets per tool and keeping them consistent with the insertion scripts.

---

## Suggested order & dependencies

1. **Now, no hardware:** 1 (theme) → 5 (scroll) → 2 (table). Quick visual wins.
2. **Shared loader work:** 3 (URDF) lays the mesh/loader groundwork that 7 reuses.
3. **ROS2 reuse:** 4 (fused cloud) reuses the bridge + subprocess + PC-stream patterns.
4. **After arm is back:** 6 (wrist cam), then 7 (EE select + tool offsets), validated live.

Items 3, 4, 6 each add one subprocess-panel launch button, so fold those into the existing
ROS2 process manager rather than new plumbing.
