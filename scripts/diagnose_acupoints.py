#!/usr/bin/env python3
"""Dump what the recogniser actually decided, for one live frame.

Run this on the machine that shows wrong acupoints:

    python scripts/diagnose_acupoints.py

It captures one frame, runs the same recognition the console runs, and prints
every intermediate decision: the body blob, which end was called the head, the
three votes that decided it, and where each acupoint landed. It also saves the
frame and an overlay JPEG so the numbers can be checked against the picture.

Read-only: it opens the camera through the same path the console uses and does
not touch the arm.

Why this exists: acupoint placement depends on which end of the blob is the
head. That is a majority vote of three cues (narrow end, hair darkness, shoulder
width), and a single flipped vote rotates the whole chart by 180 degrees in
image space — which looks like "rotated 90 degrees" once the page rotates the
picture. Printing the votes turns a guess into a number.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import heacon.config as config


def _save(path: Path, image) -> None:
    import cv2

    ok, buf = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    if ok:
        path.write_bytes(buf.tobytes())
        print("  已保存", path)


def _overlay(image, body, out: Path) -> None:
    """Draw the detected axis and every acupoint, so numbers can be eyeballed."""
    import cv2

    canvas = image.copy()
    h, w = canvas.shape[:2]

    def pt(name):
        value = body.get(name)
        if value is None:
            return None
        return int(round(value[0])), int(round(value[1]))

    for name in ("neck", "hip", "head", "center"):
        p = pt(name)
        if p:
            cv2.circle(canvas, p, 6, (0, 255, 0), -1)
            cv2.putText(
                canvas, name, (p[0] + 8, p[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
            )

    neck, hip = pt("neck"), pt("hip")
    if neck and hip:
        cv2.line(canvas, neck, hip, (0, 255, 0), 2)

    for name, (u, v) in (body.get("acupoints") or {}).items():
        p = (int(round(u)), int(round(v)))
        cv2.circle(canvas, p, 5, (0, 128, 255), -1)
        cv2.putText(canvas, name, (p[0] + 6, p[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    _save(out, canvas)


def _head_votes(blob, image, span):
    """Recompute the three votes exactly as head_at_which_end does, and report each."""
    from heacon.vision import hybrid_geom as hg

    lo, hi = blob["along_range"]
    detail = {"lo": float(lo), "hi": float(hi), "span": float(span)}

    def max_width_near(pos, inward):
        best = 0.0
        for k in range(8):
            wv, _a, _b = hg._span_at(blob, pos + inward * (span * 0.015 * k))
            if wv > best:
                best = wv
        return best

    narrow_neg = max_width_near(lo + span * 0.02, 1.0)
    narrow_pos = max_width_near(hi - span * 0.02, -1.0)
    detail["narrow_neg"] = round(narrow_neg, 1)
    detail["narrow_pos"] = round(narrow_pos, 1)
    v1 = 0
    if narrow_neg > 0 and narrow_pos > 0:
        if narrow_pos < narrow_neg * 0.90:
            v1 = 2
        elif narrow_neg < narrow_pos * 0.90:
            v1 = -2
    detail["vote_narrow"] = v1

    gray = np.average(image, axis=2)

    def brightness_at(pos):
        c = blob["center"] + blob["axis"] * pos
        x = int(np.clip(c[0], 8, gray.shape[1] - 9))
        y = int(np.clip(c[1], 8, gray.shape[0] - 9))
        patch = gray[y - 8 : y + 9, x - 8 : x + 9]
        return float(patch.mean()) if patch.size else 255.0

    b_pos = brightness_at(hi - span * 0.06)
    b_neg = brightness_at(lo + span * 0.06)
    detail["bright_pos"] = round(b_pos, 1)
    detail["bright_neg"] = round(b_neg, 1)
    v2 = 0
    if b_pos + 15 < b_neg:
        v2 = 1
    elif b_neg + 15 < b_pos:
        v2 = -1
    detail["vote_hair"] = v2

    w_neg, _a1, _b1 = hg._span_at(blob, lo + span * 0.22)
    w_pos, _a2, _b2 = hg._span_at(blob, hi - span * 0.22)
    detail["width_lo22"] = round(w_neg, 1)
    detail["width_hi22"] = round(w_pos, 1)
    v3 = 0
    if w_pos > 0 and w_neg > 0:
        if w_pos > w_neg * 1.10:
            v3 = 1
        elif w_neg > w_pos * 1.10:
            v3 = -1
    detail["vote_shoulder"] = v3

    detail["total"] = v1 + v2 + v3
    detail["head_sign"] = 1 if detail["total"] > 0 else -1
    return detail


def main() -> int:
    parser = argparse.ArgumentParser(description="导出穴位识别的中间结果")
    parser.add_argument("--out", default="acupoint-debug", help="输出目录")
    args = parser.parse_args()

    from heacon.vision import geometry as vision
    from heacon.vision import hybrid_geom as hg

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    print("HEACON 穴位识别诊断")
    print("配置:", config.describe())
    print("模型:", hg.POSE_MODEL)
    print()

    print("== 取一帧 ==")
    try:
        frame = vision.camera_bgr()
    except Exception as exc:  # noqa: BLE001
        print("  取不到画面:", exc)
        print("  先确认相机在: python scripts/check_hardware.py")
        return 1
    height, width = frame.shape[:2]
    print("  彩色帧 %dx%d" % (width, height))
    _save(out / "frame.jpg", frame)

    try:
        depth = vision.depth_mm()
    except Exception as exc:  # noqa: BLE001
        print("  读深度失败:", exc)
        depth = None
    if depth is None:
        print("  !! 没有深度图。背部识别一定失败，穴位不可能准。")
        print("     检查相机服务: systemctl status heacon-camera")
        print("     深度文件:", config.DEPTH_LIVE)
    else:
        print("  深度图 %dx%d dtype=%s" % (depth.shape[1], depth.shape[0], depth.dtype))
        valid = (depth > 200) & (depth < 1800)
        print("  有效深度像素 %.1f%%" % (100.0 * valid.sum() / depth.size))
        import cv2

        depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
        _save(out / "depth.jpg", depth_vis.astype(np.uint8))

    print()
    print("== 识别 ==")
    blob = hg.depth_blob(depth, frame.shape) if depth is not None else None
    if blob is None:
        print("  深度没分出人体。可能原因：画面里没有人体、或桌面/人体深度差太小。")
        body = None
    else:
        span = float(blob["along_range"][1] - blob["along_range"][0])
        print(
            "  blob: center=(%.0f,%.0f) length=%.0f thickness=%.0f"
            % (blob["center"][0], blob["center"][1], blob["length"], blob["thickness"])
        )
        print("  axis=(%.3f,%.3f) perp=(%.3f,%.3f)" % (*blob["axis"], *blob["perp"]))
        print()
        votes = _head_votes(blob, frame, span)
        print("  头端投票:")
        print("    窄端   lo=%.1f hi=%.1f  -> %+d" % (votes["narrow_neg"], votes["narrow_pos"], votes["vote_narrow"]))
        print("    头发   lo=%.1f hi=%.1f  -> %+d" % (votes["bright_neg"], votes["bright_pos"], votes["vote_hair"]))
        print("    肩宽   lo=%.1f hi=%.1f  -> %+d" % (votes["width_lo22"], votes["width_hi22"], votes["vote_shoulder"]))
        print("    合计 %+d -> head_sign=%+d" % (votes["total"], votes["head_sign"]))
        if abs(votes["total"]) <= 1:
            print("    !! 票数接近零，方向判断不稳，换台机器/换分辨率就可能翻。")
        body = hg.recognize(frame, depth)

    print()
    if not body:
        print("  识别结果：无")
        print(json.dumps({"frame": [width, height], "body": None}, ensure_ascii=False, indent=2))
        return 1

    print("  识别结果：有")
    print("    neck=(%.0f,%.0f) hip=(%.0f,%.0f)" % (*body["neck"], *body["hip"]))
    print("    width=%.0f length=%.0f" % (body["width"], body["length"]))
    acu = body.get("acupoints") or {}
    print("    穴位 %d 个:" % len(acu))
    for name, (u, v) in acu.items():
        print("      %-6s (%6.1f, %6.1f)" % (name, u, v))

    _overlay(frame, body, out / "overlay.jpg")

    report = {
        "frame": [width, height],
        "depth_present": depth is not None,
        "camera_backend": config.camera_backend(),
        "body": {
            "neck": body["neck"],
            "hip": body["hip"],
            "width": body["width"],
            "length": body["length"],
        },
        "acupoints": {k: list(v) for k, v in acu.items()},
    }
    (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print()
    print("把整个 %s 目录发回来。" % out)
    print("其中 overlay.jpg 能直接看出穴位落在人体的哪里。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
