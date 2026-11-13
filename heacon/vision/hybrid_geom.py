"""混合穴位定位：深度几何为主 + MediaPipe 定方向
=================================================

实测依据
--------
深度图（帧2 之后稳定）：
  长度 1039 ± 5px      标准差 0.5%
  厚度  575 ± 13px     标准差 2.3%
  轴角 -175 ± 3.6°     稳定
MediaPipe：
  肩宽  16 ~ 334px     标准差超 100%（俯卧时两肩点常重合）
  髋点位置漂移 120px

结论
----
- 人体在画面里的**位置、大小、长轴方向** → 用深度图（稳定）
- 头在哪端、病人左右 → 用 MediaPipe（方向可靠，位置不可靠）或亮度/宽度剖面

算法
----
1. 深度取人体凸起连通域 → 质心、主成分轴、长度、厚度
2. 判定头端：比较两端宽度（肩端宽、髋端略窄）+ 头发暗度
3. 判定左右：用 MediaPipe 肩点顺序，失败时用画面坐标约定
4. 沿「头→髋」轴按骨度分寸布穴位
5. 所有穴位最后夹紧到人体蒙版内（防飘出）
"""

from __future__ import annotations

import math
import os
from typing import Dict, Optional, Tuple

import numpy as np

_MODEL_DIR = os.path.dirname(os.path.abspath(__file__))

# The MediaPipe task files are 37 MB combined, so they are not in the repo.
# scripts/fetch-models.sh downloads them into the runtime dir; if they are
# missing we still look beside this module so a hand-copied bundle works.
import heacon.config as _config

_MODEL_CANDIDATES = (_config.MODEL_TASKS_DIR, _MODEL_DIR)


def _task_path(name):
    for folder in _MODEL_CANDIDATES:
        candidate = os.path.join(str(folder), name)
        if os.path.isfile(candidate):
            return candidate
    return os.path.join(str(_MODEL_CANDIDATES[0]), name)


POSE_MODEL = _task_path("pose_landmarker.task")
HAND_MODEL = _task_path("hand_landmarker.task")

_pose_lm = None
_hand_lm = None


def _get_pose():
    global _pose_lm
    if _pose_lm is not None:
        return _pose_lm
    import mediapipe as mp  # noqa: F401
    from mediapipe.tasks import python as mpp
    from mediapipe.tasks.python import vision
    if not os.path.isfile(POSE_MODEL):
        return None
    opts = vision.PoseLandmarkerOptions(
        base_options=mpp.BaseOptions(model_asset_path=POSE_MODEL),
        running_mode=vision.RunningMode.IMAGE,
        num_poses=1,
        min_pose_detection_confidence=0.15,
        min_pose_presence_confidence=0.15,
        min_tracking_confidence=0.15,
        output_segmentation_masks=True,
    )
    _pose_lm = vision.PoseLandmarker.create_from_options(opts)
    return _pose_lm


def _get_hand():
    global _hand_lm
    if _hand_lm is not None:
        return _hand_lm
    from mediapipe.tasks import python as mpp
    from mediapipe.tasks.python import vision
    if not os.path.isfile(HAND_MODEL):
        return None
    opts = vision.HandLandmarkerOptions(
        base_options=mpp.BaseOptions(model_asset_path=HAND_MODEL),
        running_mode=vision.RunningMode.IMAGE,
        num_hands=1,
        min_hand_detection_confidence=0.5,
        min_hand_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    _hand_lm = vision.HandLandmarker.create_from_options(opts)
    return _hand_lm


# ---------------------------------------------------------------------------
# 深度几何
# ---------------------------------------------------------------------------

def _table_body_split(z, valid):
    """从深度直方图分出「人体 / 桌面」，返回 (table_mm, body_mm, thr_z)。

    工作区里通常有三层深度：人体（最近）、桌面（中）、房间地面（最远）。
    俯视相机下人体是画面中最近的**大面积**物体，所以按 z 升序取第一个
    显著峰当人体，再取其后一个显著峰当桌面，阈值落在两者之间的谷。

    实测（1280x720 D435i，假人俯卧）：
      人体峰 658mm，桌面峰 856mm，阈值 797mm → 蒙版占 25%（真值）
    旧方案边缘中位数得 860mm 且 lift>25 → 蒙版占 53%（把背景并进来了）
    """
    import cv2

    vals = z[valid]
    if vals.size < 5000:
        return None
    lo_v, hi_v = np.percentile(vals, 1.0), np.percentile(vals, 99.0)
    work = vals[(vals >= lo_v) & (vals <= min(hi_v, 1400.0))]
    if work.size < 5000:
        work = vals
    zmin, zmax = float(work.min()), float(work.max())
    if zmax - zmin < 40.0:
        return None

    hist, edges = np.histogram(work, bins=100, range=(zmin, zmax))
    sm = np.convolve(hist, np.ones(5) / 5, mode="same")
    centers = (edges[:-1] + edges[1:]) / 2
    total = float(sm.sum()) or 1.0

    peaks = [
        k
        for k in range(2, len(sm) - 2)
        if sm[k] >= sm[k - 1] and sm[k] >= sm[k + 1] and sm[k] > total * 0.012
    ]
    if not peaks:
        return None

    peaks.sort(key=lambda k: centers[k])
    body_i = peaks[0]
    body_z = float(centers[body_i])

    farther = [k for k in peaks if centers[k] > body_z + 60.0]
    if not farther:
        # 只有一个峰：按桌面通常高出人体 ~150mm 估
        return body_z + 150.0, body_z, body_z + 70.0

    table_i = farther[0]
    table_z = float(centers[table_i])
    between = sm[body_i:table_i + 1]
    if between.size >= 3:
        thr_z = float(centers[body_i + int(np.argmin(between))])
    else:
        thr_z = 0.5 * (body_z + table_z)
    # 阈值夹在两峰之间 30%~70%，避免谷底太靠近人体或桌面
    gap = table_z - body_z
    thr_z = min(max(thr_z, body_z + gap * 0.30), body_z + gap * 0.70)
    return table_z, body_z, thr_z


def depth_blob(depth_mm, image_shape):
    """从深度取出人体凸起连通域。返回 dict 或 None。"""
    import cv2

    if depth_mm is None:
        return None
    h, w = image_shape[:2]
    if depth_mm.shape[:2] != (h, w):
        depth_mm = cv2.resize(depth_mm, (w, h), interpolation=cv2.INTER_NEAREST)

    z = depth_mm.astype(np.float32)
    valid = (z > 200) & (z < 1800)
    # 8000 px on a 1280x720 frame; scales with area so a smaller frame is not
    # rejected for having fewer pixels rather than less content.
    if int(valid.sum()) < int(0.008681 * h * w):
        return None

    split = _table_body_split(z, valid)
    if split is None:
        return None
    table, _body_z, thr_z = split

    # 用「比桌面近 thr 毫米以上」取人体。
    # 旧做法是 table-边缘中位数 + lift>25，边缘常被纸张/器材占住，
    # 桌面估高后连大片背景一起并进蒙版（实测蒙版能占到 53%），
    # 厚度随之虚胖到 594px（真值约 320），C7 与肩宽全部连带失真。
    mask = (valid & (z < thr_z)).astype(np.uint8) * 255
    # Kernels in pixels; keep them proportional so a downscaled frame is not
    # opened/closed away entirely.
    k_scale = w / REF_WIDTH
    k_open = max(3, int(round(9 * k_scale)) | 1)
    k_close = max(3, int(round(21 * k_scale)) | 1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((k_open, k_open), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((k_close, k_close), np.uint8))

    n, lab, st, _c = cv2.connectedComponentsWithStats(mask, 8)
    if n < 2:
        return None
    big = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    bm = (lab == big).astype(np.uint8)
    area = int(bm.sum())
    if area < int(h * w * 0.03):
        return None

    ys, xs = np.where(bm > 0)
    pts = np.stack([xs, ys], axis=1).astype(np.float64)
    center = pts.mean(axis=0)
    vals, vecs = np.linalg.eigh(np.cov((pts - center).T))
    axis = vecs[:, 1]
    if axis[0] < 0:
        axis = -axis   # 让轴大致指向 +x，便于统一处理
    perp = np.array([-axis[1], axis[0]])

    along = (pts - center) @ axis
    across = (pts - center) @ perp
    lo, hi = np.percentile(along, 1.5), np.percentile(along, 98.5)
    length = float(hi - lo)
    thickness = float(np.percentile(across, 98.5) - np.percentile(across, 1.5))
    # 100 / 60 px on a 720-high frame.
    if length < 0.13889 * h or thickness < 0.08333 * h:
        return None

    return {
        "mask": bm,
        "center": center,
        "axis": axis,
        "perp": perp,
        "length": length,
        "thickness": thickness,
        "table": table,
        "area": area,
        "along_range": (float(lo), float(hi)),
        "points": pts,
    }


# The recogniser was tuned on 1280x720 frames. Every distance below used to be
# an absolute pixel count, which silently changed meaning on a smaller frame:
# `half=320` is a quarter of a 1280-wide image but half of a 640-wide one, so
# the width comparisons stopped discriminating and the head end could be
# called backwards, turning the whole acupoint chart around.
#
# These keep the tuned behaviour at the reference size and scale with the
# frame otherwise. Ratios are chosen so that at 1280x720 each one reproduces
# the original constant exactly — see tests/test_recognition_scale.py, which
# asserts the acupoints are identical at the reference size.
REF_WIDTH = 1280.0
REF_HEIGHT = 720.0
REF_AREA = REF_WIDTH * REF_HEIGHT


def _frame_size(blob):
    """(width, height) of the frame this blob was built from."""
    h, w = blob["mask"].shape[:2]
    return float(w), float(h)


def _scale(blob):
    """How much bigger this frame is than the 1280x720 the tuning assumed."""
    w, _h = _frame_size(blob)
    return w / REF_WIDTH


def _span_at(blob, centre_pos, half=None, step=None):
    """在轴上某位置取「包含脊柱轴的那一段」横向跨度。

    旧实现用 2%~98% 百分位，只要蒙版里混进一点背景噪声，
    跨度就会被拉大到接近扫描全宽（实测 594px，真值 320px）。
    改成沿 perp 逐步采样、取包含轴心的连续区间，天然排除断层噪声。

    half / step 默认按画面宽度等比缩放：在 1280x720 上取 320 和 2，
    与调参时完全一致；画面更小时自动收窄，避免扫过半个画面。

    返回 (width, offset_lo, offset_hi)；无有效段时返回 (0.0, 0.0, 0.0)。
    """
    import numpy as np

    scale = _scale(blob)
    if half is None:
        half = 320.0 * scale
    if step is None:
        step = 2.0 * scale

    mask = blob["mask"]
    h, w = mask.shape[:2]
    center = np.asarray(blob["center"], float)
    axis = np.asarray(blob["axis"], float)
    perp = np.asarray(blob["perp"], float)

    p = center + axis * centre_pos
    offs = np.arange(-half, half + 0.01, step)
    qx = np.rint(p[0] + perp[0] * offs).astype(int)
    qy = np.rint(p[1] + perp[1] * offs).astype(int)
    inb = (qx >= 0) & (qx < w) & (qy >= 0) & (qy < h)
    on = np.zeros(len(offs), bool)
    on[inb] = mask[qy[inb], qx[inb]] > 0

    zero = int(np.argmin(np.abs(offs)))
    if not on[zero]:
        idx = np.where(on)[0]
        if idx.size == 0:
            return 0.0, 0.0, 0.0
        zero = int(idx[np.argmin(np.abs(idx - zero))])

    a = zero
    while a > 0 and on[a - 1]:
        a -= 1
    b = zero
    while b < len(on) - 1 and on[b + 1]:
        b += 1
    return float(offs[b] - offs[a] + step), float(offs[a]), float(offs[b])


def _width_at(blob, centre_pos, half_window=None):
    """在轴上某位置取横截面厚度（左右边缘跨度）。

    half_window 默认按画面宽度缩放：1280x720 上是 26，与调参一致。
    """
    if half_window is None:
        half_window = 26.0 * _scale(blob)
    pts = blob["points"]
    axis = blob["axis"]
    perp = blob["perp"]
    along = (pts - blob["center"]) @ axis
    band = np.abs(along - centre_pos) < half_window
    if int(band.sum()) < 30:
        return 0.0, 0.0
    across = (pts[band] - blob["center"]) @ perp
    lo = float(np.percentile(across, 2))
    hi = float(np.percentile(across, 98))
    return hi - lo, (lo + hi) / 2.0


def _width_profile(blob, step=None):
    """沿主轴从头到髋采样宽度，返回 (ts, widths)。

    step 默认按画面宽度缩放：1280x720 上是 4，与调参一致。
    缩放后剖面点数基本不变，下游的 neck_i / peak_i 索引才落在同一位置。
    """
    import numpy as np

    if step is None:
        step = 4.0 * _scale(blob)
    lo, hi = blob["along_range"]
    ts, ws = [], []
    t = lo
    while t <= hi:
        wd, _a, _b = _span_at(blob, t)
        ts.append(t)
        ws.append(wd)
        t += step
    return np.array(ts), np.array(ws)


def _median_smooth(a, k=4):
    import numpy as np

    if len(a) <= 1:
        return a
    out = np.empty_like(a)
    for i in range(len(a)):
        lo = max(0, i - k)
        hi = min(len(a), i + k + 1)
        out[i] = float(np.median(a[lo:hi]))
    return out


def pose_spine_anchor(image_bgr, w, h, blob):
    """用 MediaPipe 的肩中点校正脊柱线的横向位置。

    深度 blob 的主轴会被手臂带偏（手臂贴着身体让质心外移），
    所以「质心 + 主轴」这条线不等于脊柱。
    肩中点位于两肩峰连线正中，就是 C7 的正投影，用它来定线最可靠。

    返回 (c7_point, side_unit) 或 None。
    """
    import numpy as np

    pose = _get_pose()
    if pose is None:
        return None
    try:
        import cv2
        import mediapipe as mp
        rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        res = pose.detect(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb))
    except Exception:
        return None
    if not res.pose_landmarks:
        return None

    lm = res.pose_landmarks[0]
    def vis(i):
        v = lm[i].visibility
        return float(v) if v is not None else 0.0
    def px(i):
        return np.array([lm[i].x * w, lm[i].y * h], float)

    if vis(11) < 0.3 or vis(12) < 0.3:
        return None
    l_sh, r_sh = px(11), px(12)
    sh_mid = (l_sh + r_sh) / 2.0

    # 肩中点必须落在人体蒙版内（否则是误检）
    mask = blob["mask"]
    xi, yi = int(np.clip(sh_mid[0], 0, w - 1)), int(np.clip(sh_mid[1], 0, h - 1))
    if mask[yi, xi] == 0:
        # 允许少量偏移：在周围 25px 内找蒙版像素，取其中心
        r = 25
        y0, y1 = max(0, yi - r), min(h, yi + r + 1)
        x0, x1 = max(0, xi - r), min(w, xi + r + 1)
        sub = mask[y0:y1, x0:x1]
        if int(sub.sum()) < 20:
            return None
        ys, xs = np.where(sub > 0)
        sh_mid = np.array([x0 + xs.mean(), y0 + ys.mean()], float)

    side_raw = r_sh - l_sh
    n = float(np.linalg.norm(side_raw))
    if n < 1e-6:
        return None
    side_u = side_raw / n

    # 髋中点（若能拿到）用于定轴线方向，比深度主轴可靠
    hip_mid = None
    if vis(23) >= 0.3 and vis(24) >= 0.3:
        l_hip, r_hip = px(23), px(24)
        cand = (l_hip + r_hip) / 2.0
        # 必须在蒙版内，且与肩中点距离合理
        d = float(np.linalg.norm(cand - sh_mid))
        xi, yi = int(np.clip(cand[0], 0, w - 1)), int(np.clip(cand[1], 0, h - 1))
        if mask[yi, xi] > 0 and d > 80.0:
            hip_mid = cand

    return sh_mid, side_u, hip_mid



def _fit_midline(blob, anchor_pt, spine_hint, length, stations=15):
    """沿身体轴逐段取蒙版横截面中点，拟合脊柱中线。

    深度蒙版的中点是可靠的（左右边缘都落在人体上），
    质心会被手臂带偏，所以不用质心定线。
    返回 (spine_unit, head_pt, hip_pt) 或 None。
    """
    import numpy as np

    mask = blob["mask"]
    h, w = mask.shape[:2]
    axis = np.asarray(spine_hint, float)
    perp = np.array([-axis[1], axis[0]])

    # Scan +-400 px and step 2 px at 1280 wide; scale both so the same part of
    # the body is covered whatever the frame size.
    k = w / REF_WIDTH
    reach = 400.0 * k
    step = 2.0 * k
    min_width = 40.0 * k

    pts = []
    for i in range(stations):
        frac = -0.15 + (1.15 * i / max(stations - 1, 1))
        c = np.asarray(anchor_pt, float) + axis * (length * frac)
        lo = hi = None
        t = -reach
        while t <= reach:
            p = c + perp * t
            xi, yi = int(round(p[0])), int(round(p[1]))
            if 0 <= xi < w and 0 <= yi < h and mask[yi, xi]:
                if lo is None:
                    lo = t
                hi = t
            t += step
        if lo is None or hi is None:
            continue
        width = hi - lo
        if width < min_width:
            continue
        pts.append((frac * length, (lo + hi) / 2.0))

    if len(pts) < 6:
        return None

    arr = np.array(pts, float)
    med = float(np.median(arr[:, 1]))
    mad = float(np.median(np.abs(arr[:, 1] - med))) + 1e-6
    keep = np.abs(arr[:, 1] - med) < 3.0 * mad
    if int(keep.sum()) < 4:
        return None
    arr = arr[keep]

    try:
        a, b = np.polyfit(arr[:, 0], arr[:, 1], 1)
    except Exception:
        return None
    if not (np.isfinite(a) and np.isfinite(b)):
        return None

    new_spine = axis + perp * float(a)
    n = float(np.linalg.norm(new_spine))
    if n < 1e-6:
        return None
    new_spine = new_spine / n

    base = np.asarray(anchor_pt, float) + perp * float(b)
    head_pt = base - new_spine * (length * 0.18)
    hip_pt = base + new_spine * (length * 0.82)
    return new_spine, head_pt, hip_pt


def head_at_which_end(blob, image_bgr):
    """判断头在哪一端。返回 +1（along 正方向是头）或 -1。

    背视图的三条判据（按可靠性排序）：
      1. 头部是**窄端**：人头直径明显小于肩宽/髋宽，
         所以两端"最窄处"一定在头端。这是最硬的判据。
      2. 头发更暗：头端亮度低。仅作辅助（浅色头发会失效）。
      3. 肩在头端附近：肩端厚度 > 腰端厚度。

    注意：不能只用"哪端宽"，因为腰部（裤子）有时比肩还宽，
    会把髋端误判成头端。
    """
    lo, hi = blob["along_range"]
    span = hi - lo
    # 60 px on a 720-high frame.
    if span < 0.08333 * blob["mask"].shape[0]:
        return 1

    # --- 判据 1：头是「两端里更窄」的那一端 ---
    # 人头直径明显小于肩宽/髋宽，所以两端各自的最大跨度里，头端更小。
    # 用连续段宽度（_span_at），不会被背景噪声虚胖。
    def max_width_near(pos, inward):
        best = 0.0
        for k in range(8):
            wv, _a, _b = _span_at(blob, pos + inward * (span * 0.015 * k))
            if wv > best:
                best = wv
        return best

    narrow_neg = max_width_near(lo + span * 0.02, 1.0)
    narrow_pos = max_width_near(hi - span * 0.02, -1.0)

    votes = 0
    if narrow_neg > 0 and narrow_pos > 0:
        if narrow_pos < narrow_neg * 0.90:
            votes += 2      # 正端明显更窄 → 头在正方向
        elif narrow_neg < narrow_pos * 0.90:
            votes -= 2

    # --- 判据 2：头发暗 ---
    gray = np.average(image_bgr, axis=2)
    def brightness_at(pos):
        c = blob["center"] + blob["axis"] * pos
        x = int(np.clip(c[0], 8, gray.shape[1] - 9))
        y = int(np.clip(c[1], 8, gray.shape[0] - 9))
        patch = gray[y - 8:y + 9, x - 8:x + 9]
        return float(patch.mean()) if patch.size else 255.0

    b_pos = brightness_at(hi - span * 0.06)
    b_neg = brightness_at(lo + span * 0.06)
    if b_pos + 15 < b_neg:
        votes += 1
    elif b_neg + 15 < b_pos:
        votes -= 1

    # --- 判据 3：肩在头端（头端 22% 处比髋端 22% 处更宽）---
    w_neg, _a1, _b1 = _span_at(blob, lo + span * 0.22)
    w_pos, _a2, _b2 = _span_at(blob, hi - span * 0.22)
    if w_pos > 0 and w_neg > 0:
        if w_pos > w_neg * 1.10:
            votes += 1
        elif w_neg > w_pos * 1.10:
            votes -= 1

    return 1 if votes > 0 else -1


def _pull_inside(points, mask):
    """深度蒙版在腰部有空洞，按质心回拉会把命门/腰背整组拽到中背。直接保持骨度坐标。"""
    return points


# ---------------------------------------------------------------------------
# 主识别流程
# ---------------------------------------------------------------------------

def recognize(image_bgr, depth_mm=None, prefer="auto"):
    """返回 body dict（兼容 body_route.find_body）或 None。"""
    import cv2

    if image_bgr is None or image_bgr.size == 0:
        return None
    h, w = image_bgr.shape[:2]

    # ---- 1) 尝试手部（独立部位，优先）----
    if prefer in ("auto", "hand"):
        hand = _get_hand()
        if hand is not None:
            try:
                import mediapipe as mp
                rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
                hres = hand.detect(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb))
                if hres.hand_landmarks:
                    hl = hres.hand_landmarks[0]
                    palm_w = math.hypot((hl[5].x - hl[17].x) * w, (hl[5].y - hl[17].y) * h)
                    if palm_w > min(w, h) * 0.06:   # 手必须占足够面积
                        body = _hand_body(hl, image_bgr, w, h)
                        if body is not None:
                            return body
            except Exception:
                pass

    # ---- 2) 背部：深度几何为主 ----
    if prefer in ("auto", "back"):
        blob = depth_blob(depth_mm, image_bgr.shape)
        if blob is not None:
            body = _back_body(blob, image_bgr, w, h)
            if body is not None:
                return body

    return None


def _hand_body(hl, image_bgr, w, h):
    """手部：正反靠色调，左右靠拇指方位。"""
    import cv2
    import numpy as np

    def hp(i):
        return np.array([hl[i].x * w, hl[i].y * h], float)

    wrist, thumb_cmc = hp(0), hp(1)
    index_mcp, middle_mcp, pinky_mcp = hp(5), hp(9), hp(17)

    palm_w = float(np.linalg.norm(pinky_mcp - index_mcp))
    if palm_w < 20.0:
        return None

    cross = pinky_mcp - index_mcp
    n_cross = float(np.linalg.norm(cross))
    if n_cross < 1e-6:
        return None
    cross_u = cross / n_cross
    palm_axis_v = middle_mcp - wrist
    n_axis = float(np.linalg.norm(palm_axis_v))
    if n_axis < 1e-6:
        return None
    palm_axis = palm_axis_v / n_axis

    thumb_side = float(cross_u[0] * (thumb_cmc - index_mcp)[1] - cross_u[1] * (thumb_cmc - index_mcp)[0])
    laterality = "left" if thumb_side > 0 else "right"

    # 正反：掌心色调更红润 + 纹理更强
    tri = np.array([wrist, index_mcp, pinky_mcp], np.int32)
    mask = np.zeros((h, w), np.uint8)
    cv2.fillConvexPoly(mask, cv2.convexHull(tri), 255)
    if int(mask.sum()) < 300:
        return None
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    sat = float(np.median(hsv[:, :, 1][mask > 0]))
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    tex = float(np.std(gray[mask > 0]))
    facing = "palm" if (sat > 45 and tex > 16) else "dorsum"

    palm_len = float(np.linalg.norm(middle_mcp - wrist))
    rad = cross_u if laterality != "right" else -cross_u

    pts = {}
    if facing == "dorsum":
        pts["合谷"] = tuple(index_mcp + rad * (palm_w * 0.10) + palm_axis * (palm_len * 0.18))
        pts["中渚"] = tuple(index_mcp + cross * 0.62 + palm_axis * (palm_len * 0.42))
        pts["后溪"] = tuple(pinky_mcp + palm_axis * (palm_len * 0.06))
    else:
        centre = wrist + palm_axis * (palm_len * 0.62)
        pts["劳宫"] = tuple(centre)
        pts["内关"] = tuple(wrist + palm_axis * (palm_len * 0.18))

    return {
        "area": float(palm_w * palm_len),
        "center": tuple(float(v) for v in ((wrist + middle_mcp) / 2.0)),
        "neck": tuple(float(v) for v in wrist),
        "nape": tuple(float(v) for v in wrist),
        "head": tuple(float(v) for v in wrist),
        "hip": tuple(float(v) for v in middle_mcp),
        "waist": tuple(float(v) for v in middle_mcp),
        "right": (float(rad[0]), float(rad[1])),
        "width": palm_w,
        "nape_width": palm_w * 0.5,
        "shoulder_width": palm_w,
        "chest_width": palm_w,
        "waist_width": palm_w,
        "length": palm_len,
        "bounds": (0.0, 0.0, float(w), float(h)),
        "upper": True,
        "head_edge": 999.0,
        "border_touch": False,
        "head_clipped": False,
        "waist_clipped": False,
        "acupoints": {k: (float(v[0]), float(v[1])) for k, v in pts.items()},
        "source": "hand",
        "_region": "hand",
        "_facing": facing,
        "_laterality": laterality,
        "_confidence": 0.7,
        "_seg_mask": mask,
    }


def _back_body(blob, image_bgr, w, h):
    """背部：深度几何 + 头端判定 + 骨度布点。"""
    import numpy as np

    axis = blob["axis"]
    perp = blob["perp"]
    center = blob["center"]
    length = blob["length"]
    thickness = blob["thickness"]
    lo, hi = blob["along_range"]

    head_sign = head_at_which_end(blob, image_bgr)
    # spine 方向 = 头 → 髋
    spine = axis * (-head_sign)
    head_pos = hi if head_sign > 0 else lo
    hip_pos = lo if head_sign > 0 else hi

    head_pt = center + axis * head_pos
    hip_pt = center + axis * hip_pos

    # ---- 稳定的宽度剖面（统一成 头 → 髋 顺序）----
    ts, ws = _width_profile(blob, 4.0)
    if len(ts) < 40:
        return None
    ws = _median_smooth(ws, 4)
    # _width_profile 沿轴升序（lo → hi）。
    # 头在 hi（head_sign>0）时才需要反转，头在 lo 时升序本身就是头→髋。
    if head_sign > 0:
        ts, ws = ts[::-1], ws[::-1]

    # ---- 颈部最窄点：限定在头端 8%~55% 区间 ----
    n_prof = len(ts)
    a_i, b_i = int(n_prof * 0.08), int(n_prof * 0.55)
    if b_i <= a_i + 3:
        return None
    neck_i = a_i + int(np.argmin(ws[a_i:b_i]))
    neck_w = float(ws[neck_i])

    # ---- C7：宽度剖面一阶差分的峰值（颈→肩的扩张起点）----
    # 一阶差分用宽窗口算斜率，再用抛物线插值取亚像素位置。
    # 实测 6 帧：C7 位置 380.7 ± 2.1px（0.56%），远稳于 MediaPipe 肩点。
    span_k = 6
    slope = np.zeros(n_prof)
    for i in range(n_prof):
        j0 = max(0, i - span_k)
        j1 = min(n_prof, i + span_k + 1)
        slope[i] = (ws[j1 - 1] - ws[j0]) / max(1, (j1 - 1 - j0))
    lo_i = neck_i + max(1, int(n_prof * 0.01))
    hi_i = min(n_prof - 2, neck_i + int(n_prof * 0.20))
    if hi_i <= lo_i + 2:
        return None
    peak_i = lo_i + int(np.argmax(slope[lo_i:hi_i]))
    delta = 0.0
    if 0 < peak_i < n_prof - 1:
        y0, y1, y2 = slope[peak_i - 1], slope[peak_i], slope[peak_i + 1]
        den = y0 - 2.0 * y1 + y2
        if abs(den) > 1e-9:
            delta = float(np.clip(0.5 * (y0 - y2) / den, -1.0, 1.0))
    step = float(ts[1] - ts[0]) if n_prof > 1 else 4.0
    c7_t = float(ts[peak_i] + delta * step)
    c7 = center + axis * c7_t

    # 轴线横向位置：让 C7 落在蒙版横截面的中点（脊柱就在那里）
    _w_c7, c7_off_lo, c7_off_hi = _span_at(blob, c7_t)
    if _w_c7 > 0:
        mid_off = 0.5 * (c7_off_lo + c7_off_hi)
        shift = np.asarray(blob["perp"], float) * mid_off
        c7 = c7 + shift
        center = center + shift

    # ---- side：病人右侧 ----
    # MediaPipe 俯卧时肩点位置不可靠（实测恒定偏 200px），只用它定左右顺序；
    # 取不到时退回「垂直 spine 且指向画面下方」的约定。
    side = np.array([-spine[1], spine[0]], float)
    if side[1] < 0:
        side = -side
    anchor = pose_spine_anchor(image_bgr, w, h, blob)
    if anchor is not None:
        _sh_mid, side_pose, _hip_mid = anchor
        side_pose = np.asarray(side_pose, float)
        if float(np.dot(side_pose, side)) < 0:
            side_pose = -side_pose
        if abs(float(np.dot(side_pose, side))) > 0.7:
            side = side_pose
    n_side = float(np.linalg.norm(side))
    if n_side > 1e-6:
        side = side / n_side

    # ---- 肩宽：用颈宽按人体骨度比例反推 ----
    # C7 处的跨度会含手臂（实测 440~560px），双肩峰距量不出来，不能直接用。
    # 骨度分寸里「两肩峰距 = 16 寸」，成人颈宽（喉结两侧）约 5 寸，
    # 即 颈宽 ≈ 0.31 x 肩宽。
    # 早期用 0.42，1 寸被算小 25%，穴位全部挤在脊柱上半段。
    # 用实测假人反验证：颈宽 138px -> 肩宽 445px -> 1 寸 27.8px，
    # 此时 C7→短裤上缘 = 641/27.8 = 23.0 寸，与解剖值（C7→髂嵴约 21~23 寸）吻合。
    shoulder_w = neck_w / 0.31
    upper_span = float(np.max(ws[peak_i:min(n_prof, peak_i + int(n_prof * 0.12))]))
    if upper_span > 0:
        shoulder_w = min(shoulder_w, upper_span * 1.05)   # 含手臂，只会更大
    shoulder_w = float(max(shoulder_w, neck_w * 2.2))
    if not np.isfinite(shoulder_w) or shoulder_w <= 0:
        return None

    cun = shoulder_w / 16.0

    # ---- 脊长 C7→S1：骨度 21 寸 = 1.31 x 肩宽 ----
    c7_to_far = float(abs(ts[-1] - c7_t))
    spine_len = float(shoulder_w * 1.31)
    if c7_to_far > 0:
        spine_len = min(spine_len, c7_to_far * 0.90)
    if not np.isfinite(spine_len) or spine_len <= 0:
        return None

    def along(n):
        return c7 + spine * (n / 17.0 * spine_len)

    dazhui = c7 + spine * (0.012 * spine_len)
    acro_r = c7 + side * (shoulder_w / 2.0)
    acro_l = c7 - side * (shoulder_w / 2.0)
    jj_r = dazhui + (acro_r - dazhui) * 0.5
    jj_l = dazhui + (acro_l - dazhui) * 0.5

    tz_a = along(4)
    tz_off = cun * 3.2
    mingmen = along(14)
    yaoyang = along(16)
    shenshu = cun * 1.5
    dachang = cun * 1.5
    yaobei = cun * 2.5

    feng_half = cun * 1.3
    feng_c = dazhui - spine * (0.10 * spine_len)

    pts = {
        "大椎": _tp(dazhui),
        "左风池": _tp(feng_c - side * feng_half),
        "右风池": _tp(feng_c + side * feng_half),
        "左肩井": _tp(jj_l),
        "右肩井": _tp(jj_r),
        "左天宗": _tp(tz_a - side * tz_off),
        "右天宗": _tp(tz_a + side * tz_off),
        "命门": _tp(mingmen),
        "左肾俞": _tp(mingmen - side * shenshu),
        "右肾俞": _tp(mingmen + side * shenshu),
        "腰阳关": _tp(yaoyang),
        "左大肠俞": _tp(yaoyang - side * dachang),
        "右大肠俞": _tp(yaoyang + side * dachang),
        "左腰背": _tp(mingmen - side * yaobei),
        "右腰背": _tp(mingmen + side * yaobei),
    }
    pts = _pull_inside(pts, blob["mask"])

    return {
        "area": float(blob["area"]),
        "center": tuple(float(v) for v in center),
        "neck": _tp(dazhui),
        "nape": _tp(dazhui),
        "head": _tp(c7),
        "hip": _tp(c7 + spine * spine_len),
        "waist": _tp(c7 + spine * spine_len),
        "right": (float(side[0]), float(side[1])),
        "width": shoulder_w,
        "nape_width": float(shoulder_w * 0.42),
        "shoulder_width": shoulder_w,
        "chest_width": float(shoulder_w * 0.88),
        "waist_width": float(shoulder_w * 0.62),
        "length": spine_len,
        "bounds": (0.0, 0.0, float(w), float(h)),
        "upper": True,
        "head_edge": 999.0,
        "border_touch": False,
        "head_clipped": False,
        "waist_clipped": False,
        "acupoints": pts,
        "source": "depth_geo",
        "_region": "back",
        "_facing": "back",
        "_laterality": "unknown",
        "_confidence": 0.9,
        "_seg_mask": blob["mask"],
    }


def _tp(v):
    return (float(v[0]), float(v[1]))
