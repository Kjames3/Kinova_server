"""OAK-D Pro W capture thread (subscriber-aware, PLAN 2.1)."""

from __future__ import annotations

import logging
import time

import cv2
import numpy as np

from ..state import STATE

log = logging.getLogger("kinova-server")

_JPEG = [cv2.IMWRITE_JPEG_QUALITY, 80]


def _build_pipeline(dai):
    p = dai.Pipeline()

    cam = p.create(dai.node.ColorCamera)
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam.setFps(30)

    manip = p.create(dai.node.ImageManip)
    manip.initialConfig.setResize(640, 480)
    manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
    manip.setMaxOutputFrameSize(640 * 480 * 3)
    cam.isp.link(manip.inputImage)

    xrgb = p.create(dai.node.XLinkOut)
    xrgb.setStreamName("rgb")
    manip.out.link(xrgb.input)

    ml = p.create(dai.node.MonoCamera)
    ml.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    ml.setBoardSocket(dai.CameraBoardSocket.CAM_B)

    mr = p.create(dai.node.MonoCamera)
    mr.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mr.setBoardSocket(dai.CameraBoardSocket.CAM_C)

    stereo = p.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetType.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(False)
    ml.out.link(stereo.left)
    mr.out.link(stereo.right)

    xdepth = p.create(dai.node.XLinkOut)
    xdepth.setStreamName("depth")
    stereo.depth.link(xdepth.input)
    return p


def oakd_thread() -> None:
    store = STATE.oakd
    try:
        import depthai as dai
    except ImportError:
        log.warning("depthai not installed — OAK-D disabled")
        return

    # latest raw arrays kept so the point cloud can pair colour + depth
    latest_color = None
    intr = store.pc_data()[2]   # default intrinsics seeded in AppState

    try:
        with dai.Device(_build_pipeline(dai)) as device:
            cams = [c.name for c in device.getConnectedCameras()]
            log.info(f"OAK-D Pro W started — cameras: {cams}")
            STATE.status["oakd"] = True

            q_rgb = device.getOutputQueue("rgb", maxSize=4, blocking=False)
            q_depth = device.getOutputQueue("depth", maxSize=4, blocking=False)

            try:
                calib = device.readCalibration()
                m = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, 640, 400)
                intr = {"fx": m[0][0], "fy": m[1][1], "ppx": m[0][2], "ppy": m[1][2]}
                log.info(f"OAK-D intrinsics: fx={m[0][0]:.1f} fy={m[1][1]:.1f}")
            except Exception as exc:
                log.warning(f"OAK-D calibration read failed (using defaults): {exc}")

            time.sleep(1.5)  # warm up

            while not STATE.stop_event.is_set():
                pkt_rgb = q_rgb.tryGet()
                pkt_depth = q_depth.tryGet()

                if pkt_rgb is not None:
                    frame = pkt_rgb.getCvFrame()   # BGR
                    if store.rgb.subscribers > 0:
                        ok, buf = cv2.imencode(".jpg", frame, _JPEG)
                        if ok:
                            store.rgb.publish(buf.tobytes())
                    if store.pc_wanted:
                        latest_color = frame.copy()

                if pkt_depth is not None:
                    raw = pkt_depth.getFrame()    # uint16 mm, 640×400
                    if store.depth.subscribers > 0:
                        norm = cv2.normalize(raw, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
                        color = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
                        ok, buf = cv2.imencode(".jpg", color, _JPEG)
                        if ok:
                            store.depth.publish(buf.tobytes())
                    if store.pc_wanted and latest_color is not None:
                        store.publish_pc(latest_color, raw.copy(), intr)

                if pkt_rgb is None and pkt_depth is None:
                    time.sleep(0.005)

    except Exception as exc:
        log.error(f"OAK-D error: {exc}")
    finally:
        STATE.status["oakd"] = False
        log.info("OAK-D stopped")
