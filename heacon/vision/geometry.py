"""Arm camera frames, and pixels on the calibration plane as robot coordinates."""

import json
import os
import subprocess
import sys
import threading
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

import heacon.config as config

HANDEYE_PATH = config.HANDEYE

_lock = threading.Lock()
_jpeg = None
_camera_error = ""
_started = False
_handeye = None
_frame_size = (1280, 720)


def load_handeye():
    global _handeye
    data = json.loads(HANDEYE_PATH.read_text())
    camera = np.array(data["camera_to_link_matrix"], float)
    intr = data["intrinsics"]
    matrix = np.array(
        [[intr["fx"], 0.0, intr["cx"]], [0.0, intr["fy"], intr["cy"]], [0.0, 0.0, 1.0]],
        float,
    )
    plane = data["board_plane"]
    _handeye = {
        "camera": camera,
        "matrix": matrix,
        "dist": np.array(intr["dist"], float),
        "plane_point": np.array(plane["point_mm"], float),
        "plane_normal": np.array(plane["normal"], float),
        "width": float(intr.get("width") or 1280),
        "height": float(intr.get("height") or 720),
    }
    return _handeye


def _rotation(rx, ry, rz):
    def about(axis, angle):
        c, s = np.cos(angle), np.sin(angle)
        if axis == "x":
            return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        if axis == "y":
            return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    return about("z", rz) @ about("y", ry) @ about("x", rx)


def link_pose(pose, q6):
    rotation = _rotation(
        np.deg2rad(pose["rx"]), np.deg2rad(pose["ry"]), np.deg2rad(pose["rz"])
    )
    flange = np.eye(4)
    flange[:3, :3] = rotation
    flange[:3, 3] = [pose["x"], pose["y"], pose["z"]]
    spin = np.eye(4)
    spin[:3, :3] = _rotation(0, 0, np.deg2rad(-float(q6)))
    return flange @ spin


def up_vector():
    """Direction from the table toward the robot. The saved normal points downward."""
    model = _handeye or load_handeye()
    normal = model["plane_normal"]
    up = -normal if normal[2] < 0 else normal
    return up / np.linalg.norm(up)


def height_above_table(point):
    model = _handeye or load_handeye()
    offset = np.array(point, float)[:3] - model["plane_point"]
    return float(np.dot(offset, up_vector()))


def note_frame_size(width, height=None):
    global _frame_size
    if height is None and hasattr(width, "shape"):
        height, width = width.shape[:2]
    try:
        width = int(width)
        height = int(height)
    except (TypeError, ValueError):
        return _frame_size
    if width >= 8 and height >= 8:
        _frame_size = (width, height)
    return _frame_size


def raw_pixel(u, v, width=None, height=None):
    """Map pixels from the current frame onto the active camera calibration."""
    model = _active_model()
    calib_w = float(model.get("width") or 1280)
    calib_h = float(model.get("height") or 720)
    frame_w = float(width if width not in (None, 0) else _frame_size[0])
    frame_h = float(height if height not in (None, 0) else _frame_size[1])
    if frame_w < 8 or frame_h < 8:
        return float(u), float(v)
    if abs(frame_w - calib_w) < 1 and abs(frame_h - calib_h) < 1:
        return float(u), float(v)
    return float(u) * calib_w / frame_w, float(v) * calib_h / frame_h


def _read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return None


def _color_payload():
    """Color-to-link from the live RealSense process, else the last saved copy."""
    live = _read_json(COLOR_CALIB)
    if live and live.get("color_to_link"):
        saved = _read_json(COLOR_CALIB_SAVED)
        if saved != live:
            try:
                COLOR_CALIB_SAVED.write_text(json.dumps(live))
            except OSError:
                pass
        return live
    saved = _read_json(COLOR_CALIB_SAVED)
    if saved and saved.get("color_to_link"):
        return saved
    return None


def _active_model():
    """Use the color camera for on-screen pixels. Handeye on disk is infrared."""
    model = _handeye or load_handeye()
    data = _color_payload()
    if not data:
        return model
    link = data.get("color_to_link")
    if not link:
        return model
    merged = dict(model)
    merged["matrix"] = np.array(
        [[data["fx"], 0.0, data["cx"]], [0.0, data["fy"], data["cy"]], [0.0, 0.0, 1.0]],
        float,
    )
    merged["dist"] = np.array(data["dist"], float)
    merged["camera"] = np.array(link, float)
    merged["width"] = float(data.get("width") or merged.get("width") or 1280)
    merged["height"] = float(data.get("height") or merged.get("height") or 720)
    return merged


def camera_model():
    return _active_model()


def focal_length():
    return float(_active_model()["matrix"][0, 0])


def ray(joints, pose, u, v, width=None, height=None):
    model = _active_model()
    u, v = raw_pixel(u, v, width, height)
    camera = link_pose(pose, joints[5]) @ model["camera"]
    undistorted = cv2.undistortPoints(
        np.array([[[float(u), float(v)]]], np.float32),
        model["matrix"],
        model["dist"],
    ).reshape(2)
    direction = camera[:3, :3] @ np.array([undistorted[0], undistorted[1], 1.0])
    direction = direction / np.linalg.norm(direction)
    return camera[:3, 3], direction


def _depth_at(depth, x, y, span=4):
    height, width = depth.shape[:2]
    samples = []
    for row in range(max(0, y - span), min(height, y + span + 1)):
        for col in range(max(0, x - span), min(width, x + span + 1)):
            value = float(depth[row, col])
            if 200.0 < value < 1800.0:
                samples.append(value)
    if not samples:
        return None
    return float(np.median(samples))


def _depth_point(joints, pose, u, v, width=None, height=None):
    """3D point on the skin, from aligned color depth."""
    depth = depth_mm()
    if depth is None:
        return None
    model = _active_model()
    u, v = raw_pixel(u, v, width, height)
    frame_w = float(width if width not in (None, 0) else model.get("width") or _frame_size[0])
    frame_h = float(height if height not in (None, 0) else model.get("height") or _frame_size[1])
    depth_h, depth_w = depth.shape[:2]
    if frame_w < 8 or frame_h < 8:
        frame_w, frame_h = float(depth_w), float(depth_h)
    col = int(round(float(u) * depth_w / frame_w))
    row = int(round(float(v) * depth_h / frame_h))
    z = _depth_at(depth, col, row)
    if z is None:
        return None
    undistorted = cv2.undistortPoints(
        np.array([[[float(u), float(v)]]], np.float32),
        model["matrix"],
        model["dist"],
    ).reshape(2)
    camera = link_pose(pose, joints[5]) @ model["camera"]
    point = camera[:3, 3] + camera[:3, :3] @ np.array(
        [undistorted[0] * z, undistorted[1] * z, z],
        float,
    )
    return point


def _plane_point(joints, pose, u, v, lift_mm, width=None, height=None):
    model = _handeye or load_handeye()
    origin, direction = ray(joints, pose, u, v, width, height)
    normal = model["plane_normal"]
    plane_point = model["plane_point"] + up_vector() * float(lift_mm)
    denom = float(np.dot(direction, normal))
    if abs(denom) < 1e-4:
        raise ValueError("这个画面点对不上人体所在的平面")
    distance = float(np.dot(plane_point - origin, normal) / denom)
    if distance < 50 or distance > 1200:
        raise ValueError("这个画面点落在机械臂够不到的地方")
    return origin + distance * direction


def project(joints, pose, u, v, lift_mm, width=None, height=None):
    """Pixel to flange XYZ. Depth gives the mark on the body; height stays planned."""
    skin = _depth_point(joints, pose, u, v, width, height)
    if skin is not None:
        point = skin + up_vector() * (float(lift_mm) - height_above_table(skin))
    else:
        point = _plane_point(joints, pose, u, v, lift_mm, width, height)
    return {
        "x": round(float(point[0]), 1),
        "y": round(float(point[1]), 1),
        "z": round(float(point[2]), 1),
    }


def locate(joints, pose, u, v):
    return project(joints, pose, u, v, 0)


def camera_basis(joints, pose):
    """Camera origin and image axes in the robot base. The third axis looks at the scene."""
    model = _active_model()
    camera = link_pose(pose, joints[5]) @ model["camera"]
    axes = camera[:3, :3].astype(float)
    for column in range(3):
        axes[:, column] /= np.linalg.norm(axes[:, column])
    return camera[:3, 3], axes[:, 0], axes[:, 1], axes[:, 2]


def camera_bgr():
    frame = cv2.imdecode(np.frombuffer(camera_jpeg(), np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError("摄像头画面无法读取")
    note_frame_size(frame)
    return frame


import heacon.config as config

COLOR_LIVE = config.COLOR_LIVE
DEPTH_LIVE = config.DEPTH_LIVE
COLOR_CALIB = config.COLOR_CALIB_LIVE
COLOR_CALIB_SAVED = config.COLOR_CALIB


def depth_mm():
    """Aligned depth in millimetres, same size as the live color frame."""
    try:
        if time.time() - DEPTH_LIVE.stat().st_mtime > 1.8:
            return None
        frame = cv2.imread(str(DEPTH_LIVE), cv2.IMREAD_UNCHANGED)
    except OSError:
        return None
    if frame is None or frame.size == 0:
        return None
    return frame


def _color_jpeg(max_age=1.5):
    """Full-color frames written by the root RealSense process, when it is running."""
    try:
        age = time.time() - COLOR_LIVE.stat().st_mtime
        if age > float(max_age):
            return None
        data = COLOR_LIVE.read_bytes()
    except OSError:
        return None
    if len(data) < 1000:
        return None
    frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        return None
    note_frame_size(frame)
    ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    if not ok:
        return None
    return encoded.tobytes()


def _preview_jpeg(plain=True):
    """Color frames from the USB RealSense preview, if that process is already open."""
    urls = []
    if plain:
        urls.append("http://%s:%d/raw.jpg" % (config.CAMERA_SERVER_HOST, config.CAMERA_SERVER_PORT))
    urls.append("http://%s:%d/frame.jpg" % (config.CAMERA_SERVER_HOST, config.CAMERA_SERVER_PORT))
    for url in urls:
        try:
            with urllib.request.urlopen(url, timeout=0.6) as resp:
                captured = resp.headers.get("X-Captured-At")
                if captured:
                    try:
                        if time.time() - float(captured) > 1.5:
                            continue
                    except ValueError:
                        pass
                data = resp.read()
        except OSError:
            continue
        if len(data) < 1000:
            continue
        probe = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if probe is not None:
            note_frame_size(probe)
        if plain and url.endswith("/frame.jpg"):
            cleaned = _plain_jpeg(data)
            return cleaned or data
        return data
    return None


def _plain_jpeg(data):
    """Drop the aiming plus so confirm and recognition see the real back."""
    frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        return None
    height, width = frame.shape[:2]
    cx, cy = width // 2, height // 2
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hue = hsv[:, :, 0]
    sat = hsv[:, :, 1]
    val = hsv[:, :, 2]
    band = np.zeros((height, width), np.uint8)
    band[max(0, cy - 3) : cy + 4, :] = 255
    band[:, max(0, cx - 3) : cx + 4] = 255
    green = (hue > 35) & (hue < 95) & (sat > 60) & (val > 40)
    black = val < 35
    mark = ((green | black) & (band > 0)).astype(np.uint8) * 255
    if int(np.count_nonzero(mark)) < 80:
        return data
    cleaned = cv2.inpaint(frame, mark, 3, cv2.INPAINT_TELEA)
    ok, encoded = cv2.imencode(".jpg", cleaned, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    return encoded.tobytes() if ok else data


def camera_jpeg(plain=True):
    color = _color_jpeg()
    if color is not None:
        return color
    preview = _preview_jpeg(plain)
    if preview is not None:
        return preview
    ensure_camera()
    deadline = time.time() + 1.0
    while time.time() < deadline:
        color = _color_jpeg()
        if color is not None:
            return color
        preview = _preview_jpeg(plain)
        if preview is not None:
            return preview
        with _lock:
            frame = _jpeg
            error = _camera_error
        if frame is not None:
            return frame
        if error:
            raise RuntimeError(error)
        time.sleep(0.05)
    raise RuntimeError("摄像头还没有画面")


def _placeholder_jpeg():
    image = np.zeros((720, 1280, 3), np.uint8)
    image[:] = (32, 30, 28)
    cv2.putText(
        image,
        "camera",
        (80, 380),
        cv2.FONT_HERSHEY_SIMPLEX,
        2.4,
        (230, 230, 230),
        3,
        cv2.LINE_AA,
    )
    ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
    return encoded.tobytes() if ok else b""


def camera_view_jpeg():
    """Always a JPEG for the operator page. Recognition still uses camera_jpeg()."""
    color = _color_jpeg(max_age=2.0)
    if color is not None:
        return color, "live"
    preview = _preview_jpeg(True)
    if preview is not None:
        return preview, "live"
    stale = _color_jpeg(max_age=24 * 3600)
    if stale is not None:
        return stale, "stale"
    placeholder = _placeholder_jpeg()
    if placeholder:
        return placeholder, "missing"
    raise RuntimeError("摄像头还没有画面")


def ensure_camera():
    """Color frames come from the root RealSense process, not OpenCV."""
    return


def _ensure_usb_preview():
    """Infrared preview steals the RealSense from the color+depth process."""
    return


def _arm_camera_index():
    """OpenCV sorts cameras by uniqueID, not by the order the system lists them.

    Asking for the RealSense's place in that sorted list opens only that camera.
    The phone and the laptop camera are never opened.
    """
    script = """import AVFoundation
let video = AVCaptureDevice.devices(for: .video)
let muxed = AVCaptureDevice.devices(for: .muxed)
let devices = (video + muxed).sorted { $0.uniqueID.compare($1.uniqueID) == .orderedAscending }
for (index, device) in devices.enumerated() {
    let name = device.localizedName.lowercased()
    if name.contains("iphone") || name.contains("ipad") { continue }
    if name.contains("realsense") {
        print(index)
        exit(0)
    }
}
exit(1)
"""
    try:
        completed = subprocess.run(
            ["swift", "-e", script],
            capture_output=True,
            text=True,
            timeout=8,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if completed.returncode != 0:
        return None
    line = completed.stdout.strip().splitlines()
    if not line:
        return None
    try:
        return int(line[-1])
    except ValueError:
        return None


def _capture_loop():
    global _jpeg, _camera_error, _started
    try:
        _capture_forever()
    finally:
        with _lock:
            _started = False
            _jpeg = None
            if not _camera_error:
                _camera_error = "摄像头采图中断，正在重试"


def _capture_forever():
    global _jpeg, _camera_error
    while True:
        if _color_jpeg() is not None:
            time.sleep(0.4)
            continue
        index = _arm_camera_index()
        if index is None:
            with _lock:
                _jpeg = None
                _camera_error = "机械臂摄像头没有接到这台电脑"
            time.sleep(2)
            continue
        with _lock:
            _camera_error = ""
        capture = cv2.VideoCapture(index, cv2.CAP_V4L2)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        misses = 0
        while misses < 20:
            if _color_jpeg() is not None:
                capture.release()
                break
            ok, frame = capture.read()
            if not ok or frame is None:
                misses += 1
                time.sleep(0.2)
                continue
            misses = 0
            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not ok:
                continue
            with _lock:
                _jpeg = encoded.tobytes()
                _camera_error = ""
            time.sleep(0.05)
        capture.release()
        with _lock:
            _jpeg = None
            _camera_error = "没有读到机械臂摄像头"
        time.sleep(1)
