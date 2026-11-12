"""Full-colour and depth capture for the RealSense D435i.

One process owns the camera and writes both frames to the runtime directory;
the console only reads them. Single ownership avoids two processes fighting
over the USB device, and the depth image is what acupoint recognition needs.

Platform notes
--------------
Linux (the NUC deployment target)
    No root, no shim. Install pyrealsense2 and the udev rules in deploy/udev,
    run as the service user via deploy/systemd.
macOS (the older Mac field machine)
    The colour sensor is only reachable from a root process, so deploy/launchd
    runs this as root with hid_stub.dylib loaded.
"""

import ctypes
import json
import os
import sys
import time

import heacon.config as config

# macOS only: load the AVFoundation shim before the RealSense is opened.
if sys.platform == "darwin":
    for _stub in (os.environ.get("HEACON_HID_STUB", ""), "/usr/local/heacon/hid_stub.dylib"):
        if _stub and os.path.isfile(_stub):
            try:
                ctypes.CDLL(_stub, mode=ctypes.RTLD_GLOBAL)
            except OSError:
                pass
            break

OUT = config.COLOR_LIVE
DEPTH = config.DEPTH_LIVE
CALIB = config.COLOR_CALIB_LIVE
STATUS = config.CAMERA_STATUS
HANDEYE = config.HANDEYE


def _calib(profile):
    import numpy as np
    import pyrealsense2 as rs

    color = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color.get_intrinsics()
    infrared = None
    for sensor in profile.get_device().query_sensors():
        for stream in sensor.get_stream_profiles():
            if stream.stream_type() != rs.stream.infrared or stream.stream_index() != 1:
                continue
            video = stream.as_video_stream_profile()
            if video.width() == 1280 and video.height() == 720:
                infrared = video
                break
        if infrared is not None:
            break
    payload = {
        "fx": intr.fx,
        "fy": intr.fy,
        "cx": intr.ppx,
        "cy": intr.ppy,
        "dist": list(intr.coeffs),
        "width": intr.width,
        "height": intr.height,
    }
    other = None
    try:
        other = profile.get_stream(rs.stream.depth)
    except Exception:
        other = infrared
    if other is None:
        other = infrared
    if other is not None and HANDEYE.exists():
        extr = color.get_extrinsics_to(other)
        rotation = np.array(extr.rotation, float).reshape(3, 3)
        translation = np.array(extr.translation, float) * 1000.0
        color_to_cam = np.eye(4)
        color_to_cam[:3, :3] = rotation
        color_to_cam[:3, 3] = translation
        ir_to_link = np.array(json.loads(HANDEYE.read_text())["camera_to_link_matrix"], float)
        payload["color_to_link"] = (ir_to_link @ color_to_cam).round(6).tolist()
    CALIB.write_text(json.dumps(payload))
    print("calib", "linked" if "color_to_link" in payload else "intrinsics-only", flush=True)


def _start_pipeline(rs):
    plans = (
        ((1280, 720, 15), (848, 480, 15)),
        ((1280, 720, 15), (1280, 720, 15)),
        ((1280, 720, 30), (848, 480, 30)),
        ((1280, 720, 15), (640, 480, 15)),
        ((1280, 720, 30), (1280, 720, 30)),
        ((1280, 720, 15), None),
        ((1280, 720, 30), None),
        ((640, 480, 30), (640, 480, 30)),
        ((640, 480, 30), None),
    )
    last = None
    for color, depth in plans:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, color[0], color[1], rs.format.bgr8, color[2])
        if depth:
            config.enable_stream(rs.stream.depth, depth[0], depth[1], rs.format.z16, depth[2])
        print("try %s+%s" % (color, depth), flush=True)
        try:
            profile = pipeline.start(config)
        except Exception as exc:
            last = exc
            print("skip %s+%s %s" % (color, depth, exc), flush=True)
            try:
                pipeline.stop()
            except Exception:
                pass
            continue
        # 有些机器开了深度流后 wait_for_frames 一直等不到帧。
        # 这里真的收一帧确认，收不到就降级到下一个更简单的组合。
        try:
            probe = pipeline.wait_for_frames(4000)
            got_color = probe.get_color_frame() is not None if depth is None else True
            if not got_color:
                raise RuntimeError("no color frame")
        except Exception as exc:
            print("no frames %s+%s %s" % (color, depth, exc), flush=True)
            last = exc
            try:
                pipeline.stop()
            except Exception:
                pass
            continue
        print("opened color %sx%s@%s depth %s" % (color[0], color[1], color[2], depth), flush=True)
        return pipeline, profile
    raise last


def _reset_device(rs):
    try:
        ctx = rs.context()
        devices = list(ctx.query_devices())
        print("devices %s" % len(devices), flush=True)
        for device in devices:
            print("reset " + device.get_info(rs.camera_info.name), flush=True)
            device.hardware_reset()
        if devices:
            time.sleep(3)
    except Exception as exc:
        print("reset " + str(exc), flush=True)


def _write_status(**fields):
    payload = {"ts": time.time()}
    payload.update(fields)
    try:
        STATUS.write_text(json.dumps(payload))
    except OSError:
        pass


def _stop_pipeline(pipeline):
    if pipeline is None:
        return
    try:
        pipeline.stop()
    except Exception:
        pass


def _wait_for_camera(rs):
    """Stay alive until the D435i shows up on USB instead of exiting for launchd."""
    waited = 0
    while True:
        try:
            devices = list(rs.context().query_devices())
        except Exception as exc:
            print("query " + str(exc), flush=True)
            devices = []
        if devices:
            for device in devices:
                try:
                    name = device.get_info(rs.camera_info.name)
                except Exception:
                    name = "camera"
                print("found " + name, flush=True)
            _write_status(ok=False, usb=True, message="opening")
            if waited:
                time.sleep(2.0)
            return
        _write_status(ok=False, usb=False, message="waiting for camera usb")
        if waited % 5 == 0:
            print("waiting for camera usb %ss" % waited, flush=True)
        time.sleep(1.0)
        waited += 1


def _capture_loop(rs, pipeline, profile):
    import cv2
    import numpy as np

    print("started " + profile.get_device().get_info(rs.camera_info.name), flush=True)
    try:
        _calib(profile)
    except Exception as exc:
        print("calib " + str(exc), flush=True)
    try:
        depth_sensor = profile.get_device().first_depth_sensor()
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled, 1)
            print("emitter on", flush=True)
    except Exception as exc:
        print("emitter " + str(exc), flush=True)
    align = rs.align(rs.stream.color)
    misses = 0
    while True:
        try:
            frames = align.process(pipeline.wait_for_frames(5000))
        except RuntimeError as exc:
            misses += 1
            print("wait " + str(exc), flush=True)
            _write_status(ok=False, usb=True, message="frame wait " + str(exc))
            if misses >= 4:
                return
            continue
        misses = 0
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if not color:
            continue
        image = np.asanyarray(color.get_data())
        ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ok:
            continue
        temporary = OUT.with_suffix(".part")
        temporary.write_bytes(encoded.tobytes())
        temporary.replace(OUT)
        if depth:
            depth_image = np.asanyarray(depth.get_data())
            depth_part = DEPTH.with_suffix(".part.png")
            if cv2.imwrite(str(depth_part), depth_image):
                depth_part.replace(DEPTH)
        _write_status(ok=True, usb=True, message="live")
        time.sleep(0.03)


def main():
    import pyrealsense2 as rs

    print("start", flush=True)
    while True:
        pipeline = None
        try:
            _wait_for_camera(rs)
            try:
                pipeline, profile = _start_pipeline(rs)
            except Exception as exc:
                print("open failed " + str(exc), flush=True)
                _reset_device(rs)
                time.sleep(2.0)
                continue
            _capture_loop(rs, pipeline, profile)
            print("pipeline stalled, reopening", flush=True)
        except Exception:
            import traceback

            traceback.print_exc()
        _stop_pipeline(pipeline)
        _write_status(ok=False, usb=False, message="reopening")
        time.sleep(1.5)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback

        traceback.print_exc()
        time.sleep(2.0)
        os._exit(1)
