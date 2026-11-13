#!/usr/bin/env python3
"""One command that produces a single file to send back when something is wrong.

    python scripts/collect_report.py

Everything goes into ``heacon-report-<timestamp>.txt`` in the current directory,
plus a few JPEGs referenced by name. Send the whole folder, or at minimum the
.txt and overlay.jpg.

What it does, in order:

1. Records the environment: OS, Python, package versions, git commit. This is
   what tells us whether the machine matches the one the code was tuned on.
2. Runs the acupoint recogniser on a live frame and prints every decision:
   frame size, depth sanity, the three votes that pick the head end, and each
   acupoint. It saves overlay.jpg with the acupoints drawn on the untouched
   photo, which is what settles "is it right or wrong" by eye.
3. Dumps the raw frame and depth so we can re-run recognition offline.
4. Optionally captures what the page is served (via /api/camera.jpg) to catch
   any difference between the recognition frame and the displayed one.
5. If a console is running, records its status and the last log lines.

It is read-only with respect to the robot: it never connects to the arm and
never opens force control. Opening the camera is the only hardware access.
"""

from __future__ import annotations

import platform
import sys
import time
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

STAMP = time.strftime("%Y%m%d-%H%M%S")
OUTDIR = Path.cwd() / ("heacon-report-" + STAMP)
REPORT = OUTDIR / ("report-" + STAMP + ".txt")

_lines: list[str] = []


def say(text: str = "") -> None:
    _lines.append(text)
    print(text, flush=True)


def head(title: str) -> None:
    say()
    say("=" * 70)
    say(title)
    say("=" * 70)


def kv(key: str, value) -> None:
    say("  %-26s %s" % (key, value))


# --- 1. environment -------------------------------------------------------


def environment() -> None:
    head("1. 环境")
    kv("时间", time.strftime("%Y-%m-%d %H:%M:%S"))
    kv("系统", platform.platform())
    kv("发行版", _distro())
    kv("内核", platform.release())
    kv("Python", sys.version.split()[0])
    kv("可执行文件", sys.executable)
    kv("工作目录", Path.cwd())

    try:
        import subprocess

        sha = subprocess.run(
            ["git", "log", "-1", "--format=%h %s"], capture_output=True, text=True, timeout=10
        ).stdout.strip()
        kv("git 提交", sha or "（不是 git 仓库）")
        dirty = subprocess.run(
            ["git", "status", "--porcelain"], capture_output=True, text=True, timeout=10
        ).stdout.strip()
        kv("工作区改动", "有（见下）" if dirty else "无")
        if dirty:
            for line in dirty.splitlines()[:15]:
                say("      " + line)
    except Exception as exc:  # noqa: BLE001
        kv("git", "读取失败: %s" % exc)

    say()
    say("  Python 包版本：")
    for name in ("numpy", "cv2", "mediapipe", "pyrealsense2", "scipy", "vosk"):
        try:
            mod = __import__(name)
            version = getattr(mod, "__version__", "?")
            kv(name, version)
        except Exception as exc:  # noqa: BLE001
            kv(name, "缺失 (%s)" % type(exc).__name__)

    # The recogniser was tuned with these; a different mediapipe can change
    # landmark behaviour, so print the pin it declares.
    try:
        import mediapipe

        got = getattr(mediapipe, "__version__", "")
        if got and not got.startswith("0.10.18"):
            say()
            say("  !! mediapipe 版本 %s 与验证过的 0.10.18 不一致。" % got)
            say("     识别结果可能与现场不同，请把这一行一并反馈。")
    except Exception:  # noqa: BLE001
        pass

    import heacon.config as config

    say()
    say("  配置解析：")
    for key, value in config.describe().items():
        kv(key, value)


def _distro() -> str:
    try:
        text = Path("/etc/os-release").read_text()
        for line in text.splitlines():
            if line.startswith("PRETTY_NAME="):
                return line.split("=", 1)[1].strip().strip('"')
    except Exception:  # noqa: BLE001
        pass
    return "unknown"


# --- 2. calibration -------------------------------------------------------


def calibration() -> None:
    head("2. 标定文件")
    import json

    import heacon.config as config

    for label, path in (
        ("handeye", config.HANDEYE),
        ("color_calib", config.COLOR_CALIB),
        ("work_home", config.WORK_HOME),
    ):
        if not path.is_file():
            kv(label, "!! 缺失 %s" % path)
            continue
        import hashlib

        digest = hashlib.md5(path.read_bytes()).hexdigest()
        flag = "（仓库示例，未在本机标定）" if path.name.endswith(".example.json") else ""
        kv(label, "%s  md5=%s %s" % (path.name, digest[:12], flag))
        if label == "color_calib":
            try:
                data = json.loads(path.read_text())
                kv(
                    "  intr",
                    "fx=%.2f fy=%.2f cx=%.2f cy=%.2f %dx%d"
                    % (
                        data.get("fx", 0),
                        data.get("fy", 0),
                        data.get("cx", 0),
                        data.get("cy", 0),
                        data.get("width", 0),
                        data.get("height", 0),
                    ),
                )
                kv("  color_to_link", "有" if data.get("color_to_link") else "无")
            except Exception as exc:  # noqa: BLE001
                kv("  ", "解析失败: %s" % exc)

    # The runtime calibration the camera service writes can override the file.
    live = config.COLOR_CALIB_LIVE
    if live.is_file():
        try:
            data = json.loads(live.read_text())
            kv(
                "运行时 color-calib",
                "width=%s height=%s color_to_link=%s"
                % (data.get("width"), data.get("height"), "有" if data.get("color_to_link") else "无"),
            )
        except Exception as exc:  # noqa: BLE001
            kv("运行时 color-calib", "解析失败: %s" % exc)
    else:
        kv("运行时 color-calib", "不存在（%s）" % live)


# --- 3. recognition -------------------------------------------------------


def recognition() -> None:
    head("3. 穴位识别（关键）")
    import numpy as np

    import heacon.config as config
    from heacon.vision import geometry as vision
    from heacon.vision import hybrid_geom as hg

    kv("pose 模型", Path(hg.POSE_MODEL).is_file())
    kv("hand 模型", Path(hg.HAND_MODEL).is_file())
    say()

    # --- the frame the recogniser actually gets
    try:
        frame = vision.camera_bgr()
    except Exception as exc:  # noqa: BLE001
        kv("取画面", "失败: %s" % exc)
        say()
        say("  没有画面就无法诊断识别问题。检查：")
        say("    1) 相机插在 USB3 口")
        say("    2) python scripts/check_hardware.py")
        say("    3) systemctl status heacon-camera")
        return

    h, w = frame.shape[:2]
    kv("彩色帧", "%dx%d" % (w, h))
    if (w, h) != (1280, 720):
        say()
        say("  !! 分辨率不是 1280x720。识别按这个尺寸调参，其他尺寸依赖等比缩放。")
        say("     这不是错误，但请把这一行连同结果一起反馈。")
    _save(OUTDIR / "frame.jpg", frame)

    # --- what the page would be shown
    try:
        shown, state = vision.camera_view_jpeg()
        import cv2

        shown_img = cv2.imdecode(np.frombuffer(shown, np.uint8), cv2.IMREAD_COLOR)
        if shown_img is not None:
            kv("页面收到的帧", "%dx%d (%s)" % (shown_img.shape[1], shown_img.shape[0], state))
            if (shown_img.shape[1], shown_img.shape[0]) != (w, h):
                say()
                say("  !! 页面尺寸与识别尺寸不同。坐标换算可能因此出错。")
        _save(OUTDIR / "served.jpg", shown_img)
    except Exception as exc:  # noqa: BLE001
        kv("页面收到的帧", "读取失败: %s" % exc)

    # --- depth
    try:
        depth = vision.depth_mm()
    except Exception as exc:  # noqa: BLE001
        depth = None
        kv("深度图", "读取失败: %s" % exc)
    if depth is None:
        kv("深度图", "!! 没有")
        say()
        say("  深度图是背部识别的唯一依据，没有它一定识别不出穴位。")
        say("  检查 heacon-camera 是否在跑，以及 %s 是否在更新。" % config.DEPTH_LIVE)
        return

    dh, dw = depth.shape[:2]
    kv("深度图", "%dx%d dtype=%s" % (dw, dh, depth.dtype))
    valid = (depth > 200) & (depth < 1800)
    kv("有效深度像素", "%.1f%%" % (100.0 * valid.sum() / depth.size))
    if valid.sum():
        vals = depth[valid]
        kv(
            "深度范围",
            "p5=%.0f p50=%.0f p95=%.0f mm"
            % (
                np.percentile(vals, 5),
                np.percentile(vals, 50),
                np.percentile(vals, 95),
            ),
        )
    import cv2

    depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    _save(OUTDIR / "depth.jpg", depth_vis)

    if (dw, dh) != (w, h):
        kv("注意", "深度与彩色尺寸不同，识别会先缩放深度")

    # --- blob
    say()
    say("  --- 人体分割 ---")
    try:
        blob = hg.depth_blob(depth, frame.shape)
    except Exception:  # noqa: BLE001
        say("  depth_blob 抛异常：")
        for line in traceback.format_exc().splitlines():
            say("    " + line)
        return
    if blob is None:
        say("  没分出人体。可能：画面里没有人 / 桌面与人体深度差太小 / 深度有空洞。")
        return

    kv("质心", "(%.0f, %.0f)" % tuple(blob["center"]))
    kv("长度", "%.0f px" % blob["length"])
    kv("厚度", "%.0f px" % blob["thickness"])
    kv("轴", "(%.3f, %.3f)" % tuple(blob["axis"]))
    kv("蒙版占比", "%.1f%%" % (100.0 * blob["area"] / (w * h)))
    _save(OUTDIR / "mask.png", (blob["mask"] * 255).astype(np.uint8))

    # --- the vote that decides orientation
    say()
    say("  --- 头端投票（决定穴位图朝向）---")
    span = float(blob["along_range"][1] - blob["along_range"][0])
    try:
        votes = _votes(blob, frame, span, hg)
        kv("窄端 lo / hi", "%.1f / %.1f  -> %+d" % (votes["narrow_neg"], votes["narrow_pos"], votes["vote_narrow"]))
        kv("亮度 lo / hi", "%.1f / %.1f  -> %+d" % (votes["bright_neg"], votes["bright_pos"], votes["vote_hair"]))
        kv("宽度 lo22 / hi22", "%.1f / %.1f  -> %+d" % (votes["width_lo22"], votes["width_hi22"], votes["vote_shoulder"]))
        kv("合计", "%+d -> head_sign=%+d" % (votes["total"], votes["head_sign"]))
        if abs(votes["total"]) <= 1:
            say()
            say("  !! 票数接近零，方向判断不稳。这是穴位图转错方向的典型信号。")
    except Exception:  # noqa: BLE001
        say("  投票计算失败：")
        for line in traceback.format_exc().splitlines():
            say("    " + line)

    # --- final result
    say()
    say("  --- 识别结果 ---")
    try:
        body = hg.recognize(frame, depth)
    except Exception:  # noqa: BLE001
        say("  recognize 抛异常：")
        for line in traceback.format_exc().splitlines():
            say("    " + line)
        return
    if not body:
        say("  recognize 返回空。")
        return

    kv("来源", body.get("source"))
    kv("neck", "(%.1f, %.1f)" % tuple(body["neck"]))
    kv("hip", "(%.1f, %.1f)" % tuple(body["hip"]))
    kv("宽度 / 长度", "%.1f / %.1f" % (body["width"], body["length"]))
    acu = body.get("acupoints") or {}
    kv("穴位数", len(acu))
    say()
    for name, (u, v) in acu.items():
        say("      %-8s (%7.1f, %7.1f)" % (name, u, v))

    overlay = OUTDIR / "overlay.jpg"
    _overlay(frame, body, overlay)
    say()
    say("  overlay.jpg 已保存：穴位画在原始照片上。这一张最能说明问题。")


def _votes(blob, image, span, hg):
    """Recompute head_at_which_end's three votes and return each one."""
    import numpy as np

    lo, hi = blob["along_range"]

    def max_width_near(pos, inward):
        best = 0.0
        for k in range(8):
            wv, _a, _b = hg._span_at(blob, pos + inward * (span * 0.015 * k))
            if wv > best:
                best = wv
        return best

    narrow_neg = max_width_near(lo + span * 0.02, 1.0)
    narrow_pos = max_width_near(hi - span * 0.02, -1.0)
    v1 = 0
    if narrow_neg > 0 and narrow_pos > 0:
        if narrow_pos < narrow_neg * 0.90:
            v1 = 2
        elif narrow_neg < narrow_pos * 0.90:
            v1 = -2

    gray = np.average(image, axis=2)

    def brightness_at(pos):
        c = blob["center"] + blob["axis"] * pos
        x = int(np.clip(c[0], 8, gray.shape[1] - 9))
        y = int(np.clip(c[1], 8, gray.shape[0] - 9))
        patch = gray[y - 8 : y + 9, x - 8 : x + 9]
        return float(patch.mean()) if patch.size else 255.0

    b_pos = brightness_at(hi - span * 0.06)
    b_neg = brightness_at(lo + span * 0.06)
    v2 = 0
    if b_pos + 15 < b_neg:
        v2 = 1
    elif b_neg + 15 < b_pos:
        v2 = -1

    w_neg, _a1, _b1 = hg._span_at(blob, lo + span * 0.22)
    w_pos, _a2, _b2 = hg._span_at(blob, hi - span * 0.22)
    v3 = 0
    if w_pos > 0 and w_neg > 0:
        if w_pos > w_neg * 1.10:
            v3 = 1
        elif w_neg > w_pos * 1.10:
            v3 = -1

    return {
        "narrow_neg": narrow_neg,
        "narrow_pos": narrow_pos,
        "vote_narrow": v1,
        "bright_neg": b_neg,
        "bright_pos": b_pos,
        "vote_hair": v2,
        "width_lo22": w_neg,
        "width_hi22": w_pos,
        "vote_shoulder": v3,
        "total": v1 + v2 + v3,
        "head_sign": 1 if (v1 + v2 + v3) > 0 else -1,
    }


def _save(path: Path, image) -> None:
    import cv2

    if image is None:
        return
    ok, buf = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    if ok:
        path.write_bytes(buf.tobytes())


def _overlay(image, body, out: Path) -> None:
    import cv2

    canvas = image.copy()
    for name in ("neck", "hip", "head", "center"):
        value = body.get(name)
        if value is None:
            continue
        p = (int(round(value[0])), int(round(value[1])))
        cv2.circle(canvas, p, 7, (0, 255, 0), -1)
        cv2.putText(canvas, name, (p[0] + 9, p[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    neck = body.get("neck")
    hip = body.get("hip")
    if neck and hip:
        cv2.line(
            canvas,
            (int(neck[0]), int(neck[1])),
            (int(hip[0]), int(hip[1])),
            (0, 255, 0),
            2,
        )
    for name, (u, v) in (body.get("acupoints") or {}).items():
        p = (int(round(u)), int(round(v)))
        cv2.circle(canvas, p, 6, (0, 128, 255), -1)
        cv2.putText(canvas, name, (p[0] + 7, p[1] - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1)
    _save(out, canvas)


# --- 4. console -----------------------------------------------------------


def console() -> None:
    head("4. 控制台状态")
    import json
    import urllib.error
    import urllib.request

    import heacon.config as config

    base = "http://127.0.0.1:%d" % config.HTTP_PORT
    try:
        with urllib.request.urlopen(base + "/api/state", timeout=4) as resp:
            data = json.loads(resp.read())
        kv("8765", "在跑")
        for key in ("connected", "phase", "fsm", "ready"):
            kv("  " + key, data.get(key))
    except (urllib.error.URLError, OSError) as exc:
        kv("8765", "没在跑 (%s)" % exc)

    try:
        with urllib.request.urlopen(base + "/api/service/status", timeout=4) as resp:
            data = json.loads(resp.read())
        kv("服务", "%s / %s" % (data.get("running"), data.get("phase")))
    except Exception:  # noqa: BLE001
        pass


# --- 5. logs --------------------------------------------------------------


def logs() -> None:
    head("5. 相机日志（抓取分辨率）")
    import subprocess

    import heacon.config as config

    for cmd in (
        ["journalctl", "-u", "heacon-camera", "-n", "60", "--no-pager"],
        ["tail", "-n", "60", str(config.CAMERA_LOG)],
    ):
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=15).stdout
        except Exception:  # noqa: BLE001
            continue
        if not out.strip():
            continue
        kv("来源", " ".join(cmd))
        # The line that matters most.
        for line in out.splitlines():
            if "opened color" in line or "skip " in line or "try " in line:
                say("      " + line.strip())
        say("      ...")
        for line in out.splitlines()[-8:]:
            say("      " + line.strip())
        say()
        return
    kv("日志", "没找到（服务可能不是 systemd 起的）")


def main() -> int:
    OUTDIR.mkdir(parents=True, exist_ok=True)
    say("HEACON 诊断报告")
    say("生成于 " + STAMP)

    for step in (environment, calibration, recognition, console, logs):
        try:
            step()
        except Exception:  # noqa: BLE001
            say()
            say("!! %s 阶段出错：" % step.__name__)
            for line in traceback.format_exc().splitlines():
                say("    " + line)

    head("完成")
    say("报告：%s" % REPORT)
    say()
    say("同目录下的图片：")
    for name, why in (
        ("overlay.jpg", "穴位画在原始照片上 —— 最能说明问题的一张"),
        ("frame.jpg", "识别用的原始画面"),
        ("depth.jpg", "深度图"),
        ("mask.png", "人体蒙版"),
        ("served.jpg", "页面实际收到的画面"),
    ):
        p = OUTDIR / name
        if p.is_file():
            say("  %-12s %8d bytes   %s" % (name, p.stat().st_size, why))

    REPORT.write_text("\n".join(_lines))
    say()
    say("把整个 %s 文件夹发回来。" % OUTDIR.name)
    say("如果只能发一个文件，先发 report 和 overlay.jpg。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
