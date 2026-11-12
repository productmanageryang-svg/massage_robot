#!/usr/bin/env python3
"""Check that this machine can actually drive the arm and camera.

Run this after moving the RealSense to a new computer, before trying a service:

    python scripts/check_hardware.py

It is read-only. It never moves the arm and never opens force control; it only
asks the controller and the camera to identify themselves.

Why this exists: the camera and arm are USB/TCP devices that belong to one
machine at a time. Moving the camera from the NUC to a laptop changes the USB
port, the kernel device numbers, and possibly the available bandwidth. The
code does not hard-code any of that, but the operating system does need the
device to be visible and permitted. This script separates "the code is wrong"
from "the device is not there yet".
"""

from __future__ import annotations

import argparse
import socket
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import heacon.config as config

OK = "PASS"
FAIL = "FAIL"
WARN = "WARN"

_results: list[tuple[str, str, str]] = []


def record(name: str, status: str, detail: str = "") -> None:
    _results.append((name, status, detail))
    line = "[%s] %-28s %s" % (status, name, detail)
    print(line, flush=True)


def section(title: str) -> None:
    print("\n" + "=" * 68)
    print(title)
    print("=" * 68)


# --- 1. network ------------------------------------------------------------


def check_network() -> bool:
    section("1. 机械臂网络")
    host, port = config.ROBOT_HOST, config.ROBOT_PORT
    record("目标", OK, "%s:%d" % (host, port))

    # Same subnet? The usual failure after moving machines is a different
    # network, not a broken cable.
    local = _local_addresses()
    record("本机地址", OK, ", ".join(local) or "无 IPv4")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(4)
    try:
        sock.connect((host, port))
    except OSError as exc:
        record("TCP 连接", FAIL, "%s（检查网线 / WiFi / 网段）" % exc)
        return False
    finally:
        sock.close()
    record("TCP 连接", OK, "可达")
    return True


def _local_addresses() -> list[str]:
    out = []
    try:
        import subprocess

        text = subprocess.run(
            ["hostname", "-I"] if sys.platform.startswith("linux") else ["ifconfig"],
            capture_output=True,
            text=True,
            timeout=5,
        ).stdout
        import re

        out = sorted(set(re.findall(r"\b(\d{1,3}(?:\.\d{1,3}){3})\b", text)))
        out = [ip for ip in out if not ip.startswith("127.")]
    except Exception:  # noqa: BLE001
        pass
    return out[:6]


# --- 2. arm ----------------------------------------------------------------


def check_arm() -> bool:
    section("2. 机械臂控制器")
    from heacon.robot import open_robot
    from heacon.robot.client import RobotError

    robot = open_robot()
    try:
        state = robot.state()
    except RobotError as exc:
        record("连接", FAIL, str(exc))
        return False
    if not state.get("connected"):
        record("连接", FAIL, state.get("message") or "连不上")
        return False

    record("连接", OK, "model=%s" % state.get("model"))
    fsm = state.get("fsm")
    if fsm == 33:
        record("状态", OK, "FSM 33 待机")
    elif fsm in (25, 27):
        record("状态", WARN, "FSM %s 运动中；启动服务前应回待机" % fsm)
    elif fsm in (21, 22):
        record("状态", FAIL, "FSM %s 故障；先复位/解锁" % fsm)
    else:
        record("状态", WARN, "FSM %s" % fsm)

    pose = state.get("pose") or {}
    if pose:
        record("位姿", OK, " ".join("%s=%.1f" % (k, float(v)) for k, v in pose.items()))
    try:
        record("力传感器", OK, "ReadFTControlState=%s" % robot.read_ft_state())
    except RobotError as exc:
        record("力传感器", FAIL, str(exc))
    return True


# --- 3. camera -------------------------------------------------------------


def check_camera() -> None:
    section("3. RealSense 相机")
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        record("pyrealsense2", FAIL, "未安装：%s" % exc)
        print("     安装：pip install -r requirements.txt")
        return
    record("pyrealsense2", OK, "已安装")

    try:
        devices = list(rs.context().query_devices())
    except Exception as exc:  # noqa: BLE001
        record("设备枚举", FAIL, str(exc))
        return

    if not devices:
        record("设备枚举", FAIL, "0 个设备")
        print(
            "     相机没被系统看到。依次检查：\n"
            "       1) 线插在 USB3 口（蓝色口），不要走无供电 hub\n"
            "       2) 换一根线试，RealSense 对线材敏感\n"
            "       3) Linux: lsusb | grep -i intel  应能看到 8086:0b3a\n"
            "       4) Linux: udev 规则装了吗  sudo cp deploy/udev/*.rules /etc/udev/rules.d/\n"
            "       5) 用户是否在 video 组   groups | grep video"
        )
        return

    for dev in devices:
        info = {}
        for key in ("name", "serial_number", "firmware_version", "usb_type"):
            try:
                info[key] = dev.get_info(getattr(rs.camera_info, key))
            except Exception:  # noqa: BLE001
                info[key] = "?"
        record(
            "设备",
            OK,
            "%s serial=%s fw=%s usb=%s"
            % (info["name"], info["serial_number"], info["firmware_version"], info["usb_type"]),
        )
        if str(info.get("usb_type", "")).lower().find("2") >= 0:
            record(
                "USB 带宽",
                WARN,
                "usb_type 显示 USB2：彩色 1280x720 可能开不起来，换 USB3 口",
            )

    # Can it actually stream? Opening the pipeline is the only real test, and
    # it is what fails first when the USB link is marginal.
    pipeline = rs.pipeline()
    cfg = rs.config()
    plans = (
        ((1280, 720, 15), (848, 480, 15)),
        ((1280, 720, 15), None),
        ((640, 480, 30), None),
    )
    opened = None
    for color, depth in plans:
        pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, color[0], color[1], rs.format.bgr8, color[2])
        if depth:
            cfg.enable_stream(rs.stream.depth, depth[0], depth[1], rs.format.z16, depth[2])
        try:
            pipeline.start(cfg)
            opened = (color, depth)
            break
        except Exception as exc:  # noqa: BLE001
            print("     尝试 %s+%s 失败: %s" % (color, depth, exc))
            try:
                pipeline.stop()
            except Exception:  # noqa: BLE001
                pass
    if opened is None:
        record("取流", FAIL, "所有分辨率都打不开")
        return

    record("取流", OK, "color=%s depth=%s" % opened)
    try:
        frames = pipeline.wait_for_frames(5000)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        import numpy as np

        if color_frame:
            img = np.asanyarray(color_frame.get_data())
            record("彩色帧", OK, "%dx%d" % (img.shape[1], img.shape[0]))
        else:
            record("彩色帧", FAIL, "没有彩色帧")
        if depth_frame:
            d = np.asanyarray(depth_frame.get_data())
            record("深度帧", OK, "%dx%d（识别必需）" % (d.shape[1], d.shape[0]))
        else:
            record("深度帧", FAIL, "没有深度帧，穴位识别无法工作")
    except Exception as exc:  # noqa: BLE001
        record("取帧", FAIL, str(exc))
    finally:
        try:
            pipeline.stop()
        except Exception:  # noqa: BLE001
            pass


# --- 4. calibration --------------------------------------------------------


def check_calibration() -> None:
    section("4. 标定文件")
    for label, path in (
        ("handeye", config.HANDEYE),
        ("color_calib", config.COLOR_CALIB),
        ("work_home", config.WORK_HOME),
    ):
        if not path.is_file():
            record(label, FAIL, "缺失 %s" % path)
        elif path.name.endswith(".example.json"):
            record(
                label,
                WARN,
                "用的是仓库示例 %s —— 换机后必须按本机重新标定" % path.name,
            )
        else:
            record(label, OK, str(path))


def check_daemon() -> None:
    section("5. 控制台")
    import urllib.error
    import urllib.request

    url = "http://127.0.0.1:%d/api/state" % config.HTTP_PORT
    try:
        with urllib.request.urlopen(url, timeout=3) as resp:
            import json

            data = json.loads(resp.read())
        record("8765", OK, "phase=%s fsm=%s" % (data.get("phase"), data.get("fsm")))
    except (urllib.error.URLError, OSError):
        record("8765", WARN, "没在跑（python main.py console）")


def main() -> int:
    parser = argparse.ArgumentParser(description="检查本机能否驱动机械臂和相机")
    parser.add_argument("--skip-camera", action="store_true", help="不打开相机")
    args = parser.parse_args()

    print("HEACON 硬件自检（只读，不会移动机械臂）")
    print("配置:", config.describe())

    network_ok = check_network()
    if network_ok:
        check_arm()
    else:
        record("机械臂", FAIL, "网络不通，跳过")
    if not args.skip_camera:
        check_camera()
    check_calibration()
    check_daemon()

    print("\n" + "=" * 68)
    fails = [r for r in _results if r[1] == FAIL]
    warns = [r for r in _results if r[1] == WARN]
    if fails:
        print("结论：%d 项未通过，还不能开始服务。" % len(fails))
        for name, _s, detail in fails:
            print("  - %s: %s" % (name, detail))
        return 1
    if warns:
        print("结论：可以启动，但有 %d 项需要确认。" % len(warns))
        for name, _s, detail in warns:
            print("  - %s: %s" % (name, detail))
        return 0
    print("结论：机械臂、相机、标定都正常，可以启动服务。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
