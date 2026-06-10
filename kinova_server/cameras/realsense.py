"""RealSense D435i capture thread.

Same capture logic as the original monolith, but subscriber-aware (PLAN 2.1):
each per-frame cost — colour JPEG, depth colorize+JPEG, the raw copy for point
clouds — is paid only when something is actually watching that output.
"""

from __future__ import annotations

import logging

import cv2
import numpy as np

from ..state import STATE

log = logging.getLogger("kinova-server")

_JPEG = [cv2.IMWRITE_JPEG_QUALITY, 80]


def realsense_thread() -> None:
    store = STATE.realsense
    try:
        import pyrealsense2 as rs
    except ImportError:
        log.warning("pyrealsense2 not installed — RealSense disabled")
        return

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    try:
        profile = pipeline.start(cfg)
        STATE.status["realsense"] = True
        log.info("RealSense D435i started")
    except Exception as exc:
        log.error(f"RealSense start failed: {exc}")
        return

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    align = rs.align(rs.stream.color)
    colorizer = rs.colorizer()

    try:
        while not STATE.stop_event.is_set():
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            aligned = align.process(frames)
            c = aligned.get_color_frame()
            d = aligned.get_depth_frame()
            if not c or not d:
                continue

            color_img = np.asanyarray(c.get_data())   # BGR view into SDK buffer

            if store.rgb.subscribers > 0:
                ok, cb = cv2.imencode(".jpg", color_img, _JPEG)
                if ok:
                    store.rgb.publish(cb.tobytes())

            if store.depth.subscribers > 0:
                depth_vis = np.asanyarray(colorizer.colorize(d).get_data())
                ok, db = cv2.imencode(".jpg", depth_vis, _JPEG)
                if ok:
                    store.depth.publish(db.tobytes())

            if store.pc_wanted:
                intr = c.profile.as_video_stream_profile().get_intrinsics()
                store.publish_pc(
                    color_img.copy(),                       # BGR, copy out of buffer
                    np.asanyarray(d.get_data()).copy(),     # z16 uint16
                    {
                        "fx": intr.fx, "fy": intr.fy,
                        "ppx": intr.ppx, "ppy": intr.ppy,
                        "depth_scale": depth_scale,
                    },
                )
    except Exception as exc:
        log.error(f"RealSense error: {exc}")
    finally:
        pipeline.stop()
        STATE.status["realsense"] = False
        log.info("RealSense stopped")
