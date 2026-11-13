#!/usr/bin/env python3
"""Check the camera's depth stream specifically.

    python scripts/check_depth.py

The acupoint chart is built from the depth image; the colour image is only for
display. If depth is missing the chart cannot be correct, but the page will
still show a live picture, so the failure is silent. This isolates that.

It opens the RealSense itself, prints which stream combinations actually work,
and reports whether the depth file the console reads is being written.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import heacon.config as config


def main() -> int:
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        print("pyrealsense2 不可用:", exc)
        return 1

    print("HEACON 深度流检查")
    print("=" * 60)

    devices = list(rs.context().query_devices())
    print("设备数:", len(devices))
    for dev in devices:
        for key in ("name", "serial_number", "firmware_version", "usb_type"):
            try:
                print("  %-18s %s" % (key, getattr(rs.camera_info, key) and dev.get_info(getattr(rs.camera_info, key))))
            except Exception:  # noqa: BLE001
                pass

    if not devices:
        print("\n相机没有被系统看到。先解决 USB 连接。")
        return 1

    dev = devices[0]
    try:
        usb = str(dev.get_info(rs.camera_info.usb_type))
        print("\nUSB 类型:", usb)
        if "2" in usb and "3" not in usb:
            print("  !! 这是 USB2 连接。深度流在 USB2 上通常带宽不足。")
            print("     换到 USB3 口（蓝色或标 SS）再试。")
    except Exception:  # noqa: BLE001
        pass

    # Does this sensor even offer depth profiles?
    print("\n深度传感器支持的流:")
    try:
        for sensor in dev.query_sensors():
            name = sensor.get_info(rs.camera_info.name)
            if "depth" not in name.lower():
                continue
            for prof in sensor.get_stream_profiles():
                if prof.stream_type() == rs.stream.depth:
                    v = prof.as_video_stream_profile()
                    print("  %s: %dx%d @ %d fps %s" % (name, v.width(), v.height(), v.fps(), prof.format()))
                    break
    except Exception as exc:  # noqa: BLE001
        print("  查询失败:", exc)

    # Try each combination the console would try, in order.
    print("\n逐个尝试（与 heacon/camera_daemon.py 相同的顺序）:")
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
    winner = None
    for color, depth in plans:
        pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, color[0], color[1], rs.format.bgr8, color[2])
        if depth:
            cfg.enable_stream(rs.stream.depth, depth[0], depth[1], rs.format.z16, depth[2])
        label = "color %s + depth %s" % (color, depth)
        try:
            pipeline.start(cfg)
        except Exception as exc:  # noqa: BLE001
            print("  ✗ %s  打不开: %s" % (label, str(exc)[:80]))
            try:
                pipeline.stop()
            except Exception:  # noqa: BLE001
                pass
            continue

        try:
            frames = pipeline.wait_for_frames(4000)
            has_color = frames.get_color_frame() is not None
            has_depth = frames.get_depth_frame() is not None
            print("  ✓ %s  彩色=%s 深度=%s" % (label, has_color, has_depth))
            if has_depth:
                import numpy as np

                arr = np.asanyarray(frames.get_depth_frame().get_data())
                valid = arr[arr > 0]
                print(
                    "      深度图 %dx%d, 有效像素 %.1f%%, 范围 %s..%s mm"
                    % (
                        arr.shape[1],
                        arr.shape[0],
                        100.0 * valid.size / arr.size,
                        valid.min() if valid.size else "?",
                        valid.max() if valid.size else "?",
                    )
                )
                if winner is None:
                    winner = (color, depth)
            else:
                print("      !! 这一档只出了彩色，没有深度")
        except Exception as exc:  # noqa: BLE001
            print("  ✗ %s  收帧失败: %s" % (label, str(exc)[:80]))
        finally:
            try:
                pipeline.stop()
            except Exception:  # noqa: BLE001
                pass

        if winner:
            break

    print()
    if winner:
        print("结论：可用组合 color=%s depth=%s" % winner)
        print("      深度流能开。如果控制台仍报没有深度，是 heacon-camera 服务的问题。")
    else:
        print("结论：没有任何组合能同时拿到彩色和深度。")
        print("      这就是穴位错位的原因——识别只能用深度，没有深度就只能乱猜。")
        print("      先解决 USB：换 USB3 口、换线、不要用 hub。")

    # Is the console's depth file being refreshed?
    print()
    live = config.DEPTH_LIVE
    if live.is_file():
        age = time.time() - live.stat().st_mtime
        print("控制台读的深度文件: %s" % live)
        print("  存在，%d 字节，%.1f 秒前更新" % (live.stat().st_size, age))
        if age > 3:
            print("  !! 太旧了。heacon-camera 可能没在写。")
    else:
        print("控制台读的深度文件不存在: %s" % live)
        print("  heacon-camera 没有写出深度图。")

    color = config.COLOR_LIVE
    if color.is_file():
        age = time.time() - color.stat().st_mtime
        print("控制台读的彩色文件: 存在，%.1f 秒前更新" % age)

    return 0 if winner else 1


if __name__ == "__main__":
    raise SystemExit(main())
