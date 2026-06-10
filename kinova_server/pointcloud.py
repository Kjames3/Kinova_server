"""Point-cloud generation (PLAN 2.1, 2.4).

These run in an executor thread. The colour BGR→RGB conversion and the depth
``float32`` scaling are done **here**, not in the camera threads, so the camera
threads only store the raw arrays they already have and pay nothing when no one
is streaming a cloud.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np

from .state import STATE

_EMPTY: Tuple[np.ndarray, np.ndarray] = (
    np.zeros((0, 3), np.float32),
    np.zeros((0, 3), np.uint8),
)


def _deproject(z, ys, xs, intr, color_rgb, cmap=None):
    x = (xs - intr["ppx"]) * z / intr["fx"]
    y = (ys - intr["ppy"]) * z / intr["fy"]
    pts = np.column_stack([x, y, z]).astype(np.float32)
    if cmap is None:
        colors = color_rgb[ys, xs].astype(np.uint8)
    else:
        cy, cx = cmap
        colors = color_rgb[cy, cx].astype(np.uint8)
    return pts, colors


def compute_pc_rs(max_pts: int = 8000):
    """RealSense: raw_depth is z16 uint16; intrinsics carry ``depth_scale``."""
    color_bgr, depth_raw, intr = STATE.realsense.pc_data()
    if depth_raw is None or color_bgr is None or intr is None:
        return _EMPTY

    depth = depth_raw.astype(np.float32) * intr.get("depth_scale", 0.001)  # → metres
    color_rgb = color_bgr[:, :, ::-1]                                       # BGR→RGB (view)

    h, w = depth.shape
    step = max(1, int(np.sqrt(h * w / max_pts)))
    ys, xs = np.mgrid[0:h:step, 0:w:step]
    ys, xs = ys.ravel(), xs.ravel()
    z = depth[ys, xs]

    valid = (z > 0.15) & (z < 6.0)
    ys, xs, z = ys[valid], xs[valid], z[valid]
    return _deproject(z, ys, xs, intr, color_rgb)


def compute_pc_oak(max_pts: int = 8000):
    """OAK-D: raw_depth is uint16 mm at 640×400; colour is 640×480 BGR."""
    color_bgr, depth_mm, intr = STATE.oakd.pc_data()
    if depth_mm is None or color_bgr is None or intr is None:
        return _EMPTY

    color_rgb = color_bgr[:, :, ::-1]
    dh, dw = depth_mm.shape          # 640×400
    ch, cw = color_rgb.shape[:2]     # 640×480

    step = max(1, int(np.sqrt(dh * dw / max_pts)))
    ys, xs = np.mgrid[0:dh:step, 0:dw:step]
    ys, xs = ys.ravel(), xs.ravel()
    z = depth_mm[ys, xs].astype(np.float32) / 1000.0   # mm → m

    valid = (z > 0.15) & (z < 6.0)
    ys, xs, z = ys[valid], xs[valid], z[valid]

    cx = (xs * cw / dw).astype(int).clip(0, cw - 1)
    cy = (ys * ch / dh).astype(int).clip(0, ch - 1)
    return _deproject(z, ys, xs, intr, color_rgb, cmap=(cy, cx))
