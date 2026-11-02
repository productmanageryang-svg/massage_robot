"""Place a plan's acupoints on the person or mannequin seen by the arm camera.

Overhead color frames are turned into named back landmarks first — nape, C7,
shoulders, scapula, L2, L4 — then every plan reads those same points.
"""

import cv2
import numpy as np

# The body drawing used by the page. Neck is the small Y, the low back is larger.
NECK_Y = 210.0
HIP_Y = 560.0
LEFT_X = 140.0
RIGHT_X = 400.0


def find_body(image):
    """Return the upper-body axis in pixels, or None when the picture has no clear body.

    Depth is preferred: the table is flat and the person is the raised blob.
    A clothed silhouette is next — skin color is not required.
    The beige-plate fallback is only for a bare mannequin when GrabCut fails.
    """
    if image is None or image.size == 0:
        return None
    depth = _live_depth(image)
    if depth is not None:
        raised = _depth_overhead(image, depth)
        if raised is not None:
            return raised
    if image.ndim == 3:
        clean = _strip_overlay(image)
        silhouette = _silhouette_overhead(clean)
        if silhouette is not None:
            return silhouette
        shape = _shape_overhead(clean)
        if shape is not None:
            return shape
        color = _color_overhead(clean)
        if color is not None:
            return color
        image = clean
    upper = person_line(image)
    if upper is not None and not upper.get("head_clipped"):
        _ensure_landmarks(upper)
        return upper
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    classic = _classic_body(gray)
    plate = _plate_body(gray)
    body = _prefer(classic, plate, gray.shape)
    if body is not None:
        _ensure_landmarks(body)
    return body


def _live_depth(image):
    try:
        import vision
    except ImportError:
        return None
    try:
        depth = vision.depth_mm()
    except Exception:
        return None
    if depth is None:
        return None
    if depth.shape[:2] != image.shape[:2]:
        depth = cv2.resize(depth, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)
    return depth


def _depth_overhead(image, depth):
    """Find the raised back from aligned depth. Lighting and paint color do not matter."""
    height, width = depth.shape[:2]
    z = depth.astype(np.float32)
    valid = (z > 200.0) & (z < 1800.0)
    if int(np.count_nonzero(valid)) < 8000:
        return None
    border = np.zeros((height, width), bool)
    rim = 28
    border[:rim, :] = True
    border[-rim:, :] = True
    border[:, :rim] = True
    border[:, -rim:] = True
    sample = z[valid & border]
    if sample.size < 300:
        sample = z[valid]
    # The 80th percentile of the rim sees the room through the table holes
    # (~1.6 m) and then the dummy is "too tall" and gets thrown away.
    work = sample[(sample > 400.0) & (sample < 1200.0)]
    if work.size < 200:
        work = sample
    table = float(np.median(work))
    lift = table - z
    mask = (valid & (lift > 22.0) & (lift < 350.0)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((17, 17), np.uint8))
    part = _largest(mask, int(height * width * 0.04))
    if part is None:
        return None
    body_mask, torso, area = part
    if area > height * width * 0.70:
        return None
    ys, xs = np.where(body_mask)
    if len(xs) < 400:
        return None
    points = np.stack([xs.astype(np.float64), ys.astype(np.float64)], axis=1)
    center = points.mean(axis=0)
    if not (width * 0.12 < center[0] < width * 0.88 and height * 0.10 < center[1] < height * 0.90):
        return None
    spread = points - center
    cov = np.cov(spread.T)
    if not np.all(np.isfinite(cov)):
        return None
    _vals, vecs = np.linalg.eigh(cov)
    axis = vecs[:, 1]
    nrm = float(np.linalg.norm(axis))
    if nrm < 1e-6 or not np.isfinite(nrm):
        return None
    axis = axis / nrm
    if not np.all(np.isfinite(axis)):
        return None
    along = spread @ axis
    if not np.all(np.isfinite(along)):
        return None
    lo, hi = np.percentile(along, 6), np.percentile(along, 94)
    if hi - lo < min(width, height) * 0.28:
        return None
    end_a = center + axis * lo
    end_b = center + axis * hi
    head, _far = _depth_head_hip(body_mask, end_a, end_b, axis, image)
    # Same waist/nape as the clothed silhouette path. The raised blob includes
    # pants, so the far PCA end is the seat, not L4.
    _center, direction = _plate_axis(points, head)
    hip = _plate_hip(points, _center, direction, station_pct=80)
    side = np.array([-direction[1], direction[0]], dtype=np.float64)
    if np.dot(side, np.array([0.0, -1.0])) < 0:
        side = -side
    length = float(np.linalg.norm(hip - np.asarray(head, dtype=np.float64)))
    if length < min(width, height) * 0.16:
        return None
    hair = _depth_hair(image, head, np.asarray(head, dtype=np.float64) + direction * (length * 0.12)) if image.ndim == 3 else None
    if hair is not None:
        head_area = float(np.count_nonzero(hair))
    else:
        head_along = np.dot(points - np.asarray(head, dtype=np.float64), direction)
        head_area = float(np.count_nonzero(head_along < max(48.0, length * 0.18)))
    radius = max(40.0, min(160.0, (head_area / np.pi) ** 0.5))
    nape_along = min(max(radius * 0.90 + 14.0, length * 0.10), length * 0.26)
    nape = np.asarray(head, dtype=np.float64) + direction * nape_along
    if float(np.linalg.norm(hip - nape)) < min(width, height) * 0.16:
        return None
    filled = cv2.morphologyEx(body_mask.astype(np.uint8) * 255, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))
    rows = _spine_profile(filled, nape, hip, side)
    nape_width, shoulder_width, chest_width, waist_width, shoulder_t = _robust_widths(rows)
    shoulder_width = min(400.0, max(shoulder_width, chest_width * 1.22))
    atlas = _landmarks_from_profile(
        hair,
        head,
        nape,
        hip,
        side,
        rows,
        nape_width,
        shoulder_width,
        chest_width,
        waist_width,
        shoulder_t,
    )
    edge = min(float(head[0]), float(head[1]), width - 1.0 - float(head[0]), height - 1.0 - float(head[1]))
    body = {
        "area": float(area),
        "center": (float(torso[0]), float(torso[1])),
        "neck": (float(nape[0]), float(nape[1])),
        "nape": (float(nape[0]), float(nape[1])),
        "hip": (float(hip[0]), float(hip[1])),
        "head": (float(head[0]), float(head[1])),
        "waist": (float(hip[0]), float(hip[1])),
        "right": (float(side[0]), float(side[1])),
        "width": shoulder_width,
        "nape_width": nape_width,
        "shoulder_width": shoulder_width,
        "chest_width": chest_width,
        "waist_width": waist_width,
        "length": float(np.linalg.norm(hip - nape)),
        "bounds": (
            float(np.percentile(xs, 4)),
            float(np.percentile(ys, 4)),
            float(np.percentile(xs, 96)),
            float(np.percentile(ys, 96)),
        ),
        "upper": True,
        "head_edge": float(edge),
        "border_touch": edge < 36,
        "head_clipped": edge < 36,
        "waist_clipped": _near_border(hip, width, height, 36),
        "acupoints": atlas,
        "source": "depth",
    }
    body.update(atlas)
    return body


def _depth_head_hip(mask, end_a, end_b, axis, image):
    """The head is the narrower raised end. Darker hair/head is only a tie-break."""
    a = _end_width(mask, end_a, axis)
    b = _end_width(mask, end_b, axis)
    if a + 18 < b:
        head, hip = end_a, end_b
    elif b + 18 < a:
        head, hip = end_b, end_a
    else:
        dark_a = _end_darkness(image, end_a)
        dark_b = _end_darkness(image, end_b)
        head, hip = (end_a, end_b) if dark_a >= dark_b else (end_b, end_a)
    return np.array(head, float), np.array(hip, float)


def _end_width(mask, point, axis, span=70):
    height, width = mask.shape[:2]
    side = np.array([-axis[1], axis[0]])
    widths = []
    for step in range(-8, 9):
        center = np.array(point, float) + axis * float(step * 4)
        if not (4 <= center[0] < width - 4 and 4 <= center[1] < height - 4):
            continue
        filled = mask.astype(np.uint8)
        if filled.max() == 1:
            filled = filled * 255
        _pos, _neg, run = _axis_run(filled, center, side)
        if run > 8:
            widths.append(run)
    return float(np.median(widths)) if widths else 999.0


def _end_darkness(image, point):
    if image is None or image.ndim != 3:
        return 0.0
    x, y = int(point[0]), int(point[1])
    h, w = image.shape[:2]
    x0, x1 = max(0, x - 28), min(w, x + 28)
    y0, y1 = max(0, y - 28), min(h, y + 28)
    patch = image[y0:y1, x0:x1]
    if patch.size == 0:
        return 0.0
    return 255.0 - float(np.median(cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)))


def _depth_nape(rows, head, hip):
    early = [row for row in rows if row["t"] <= 0.32 and row["width"] > 16]
    if not early:
        return np.array(head, float) + (np.array(hip, float) - np.array(head, float)) * 0.12
    pinch = min(early, key=lambda row: row["width"])
    return np.array(pinch["center"], float)


def _depth_hair(image, head, nape):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    val = hsv[:, :, 2]
    sat = hsv[:, :, 1]
    axis = np.array(nape, float) - np.array(head, float)
    length = float(np.linalg.norm(axis)) or 1.0
    axis = axis / length
    ys, xs = np.indices(val.shape)
    along = (xs - head[0]) * axis[0] + (ys - head[1]) * axis[1]
    side = -(xs - head[0]) * axis[1] + (ys - head[1]) * axis[0]
    hair = (
        (along > -30)
        & (along < max(36.0, length * 0.22))
        & (np.abs(side) < 90)
        & (val < 110)
        & (sat > 20)
    ).astype(np.uint8) * 255
    if int(np.count_nonzero(hair)) < 80:
        return None
    return hair


def _compact_blob(mask, minimum, max_area=None):
    """Largest filled oval-like blob. Drops cables, table strips, and ragged patches."""
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    best = None
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < minimum:
            continue
        if max_area is not None and area > max_area:
            continue
        box_w = max(int(stats[label, cv2.CC_STAT_WIDTH]), 1)
        box_h = max(int(stats[label, cv2.CC_STAT_HEIGHT]), 1)
        fill = area / float(box_w * box_h)
        aspect = max(box_w, box_h) / float(min(box_w, box_h))
        if fill < 0.42 or aspect > 2.6:
            continue
        score = area * fill / aspect
        if best is None or score > best[0]:
            blob = labels == label
            center = np.array(centroids[label], float)
            best = (score, blob, center, area)
    if best is None:
        return None
    return best[1], best[2], best[3]


def _plate_axis(points, head):
    """Spine is head to the back-plate center. A rectangular plate's PCA can tilt and is ignored."""
    center = np.asarray(points.mean(axis=0), dtype=np.float64)
    primary = center - np.asarray(head, dtype=np.float64)
    norm = float(np.linalg.norm(primary))
    if norm < 40 or not np.isfinite(norm):
        return center, np.array([1.0, 0.0], dtype=np.float64)
    return center, primary / norm


def _plate_hip(points, center, axis, station_pct=90):
    """Far end of the plate on the midline, so a sleeve or pant hem cannot drag the waist."""
    axis = np.asarray(axis, dtype=np.float64)
    center = np.asarray(center, dtype=np.float64)
    if not np.all(np.isfinite(axis)) or float(np.linalg.norm(axis)) < 1e-6:
        axis = np.array([1.0, 0.0], dtype=np.float64)
    along = np.dot(points - center, axis)
    side = np.array([-axis[1], axis[0]], dtype=np.float64)
    offset = np.dot(points - center, side)
    finite = np.isfinite(along) & np.isfinite(offset)
    if int(np.count_nonzero(finite)) < 80:
        return center + axis * 220.0
    along = along[finite]
    offset = offset[finite]
    pts = points[finite]
    half = max(36.0, float(np.percentile(np.abs(offset), 32)))
    mid = np.abs(offset) <= half
    if int(np.count_nonzero(mid)) < 80:
        mid = np.ones(len(pts), dtype=bool)
    station = float(np.percentile(along[mid], station_pct))
    band = mid & (np.abs(along - station) < 30.0)
    if int(np.count_nonzero(band)) >= 20:
        picked = pts[band]
        return np.array([float(np.median(picked[:, 0])), float(np.median(picked[:, 1]))])
    return center + axis * station


def _learned_person_mask(image):
    """Optional YOLO-seg person mask. Missing weights must not break the scan."""
    try:
        from ultralytics import YOLO
    except ImportError:
        return None
    try:
        model = getattr(_learned_person_mask, "_model", None)
        if model is None:
            model = YOLO("yolov8n-seg.pt")
            _learned_person_mask._model = model
        result = model.predict(image, verbose=False, conf=0.25, classes=[0])[0]
        if result.masks is None or len(result.masks.data) == 0:
            return None
        mask = result.masks.data[0].cpu().numpy()
        mask = cv2.resize(mask, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)
        return (mask > 0.5).astype(np.uint8) * 255
    except Exception:
        return None


def _grabcut_person_mask(image):
    """Person versus table. Clothes stay in the blob; the pegboard does not."""
    height, width = image.shape[:2]
    scale = 2 if width >= 900 else 1
    small = image if scale == 1 else cv2.resize(image, (width // scale, height // scale), interpolation=cv2.INTER_AREA)
    sh, sw = small.shape[:2]
    mask = np.full((sh, sw), cv2.GC_PR_BGD, np.uint8)
    rim = max(8, min(sw, sh) // 40)
    border = max(6, rim - 4)
    mask[rim : sh - rim, rim : sw - rim] = cv2.GC_PR_FGD
    mask[:border, :] = cv2.GC_BGD
    mask[-border:, :] = cv2.GC_BGD
    mask[:, :border] = cv2.GC_BGD
    mask[:, -border:] = cv2.GC_BGD
    background = np.zeros((1, 65), np.float64)
    foreground = np.zeros((1, 65), np.float64)
    try:
        cv2.grabCut(small, mask, None, background, foreground, 1, cv2.GC_INIT_WITH_MASK)
    except cv2.error:
        return None
    blob = np.where((mask == cv2.GC_FGD) | (mask == cv2.GC_PR_FGD), 255, 0).astype(np.uint8)
    if scale > 1:
        blob = cv2.resize(blob, (width, height), interpolation=cv2.INTER_NEAREST)
    blob = cv2.morphologyEx(blob, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    blob = cv2.morphologyEx(blob, cv2.MORPH_CLOSE, np.ones((17, 17), np.uint8))
    part = _largest(blob, 12000)
    if part is None:
        return None
    person, _center, area = part
    if area > height * width * 0.72:
        return None
    return person.astype(np.uint8) * 255


def _person_mask(image):
    learned = _learned_person_mask(image)
    if learned is not None:
        return learned
    return _grabcut_person_mask(image)


def _silhouette_hair(image, person, head, direction, length):
    """Darker patch at the head end — hair, a cap, or the mannequin skull."""
    if image.ndim != 3:
        return None
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ys, xs = np.where(person > 0)
    if len(xs) < 80:
        return None
    points = np.stack([xs.astype(np.float64), ys.astype(np.float64)], axis=1)
    along = np.dot(points - np.asarray(head, dtype=np.float64), direction)
    near = along < max(48.0, length * 0.22)
    if int(np.count_nonzero(near)) < 40:
        return None
    vals = gray[ys[near], xs[near]]
    dark = vals < float(np.percentile(vals, 55))
    hair = np.zeros(person.shape, np.uint8)
    hair[ys[near][dark], xs[near][dark]] = 255
    hair = cv2.morphologyEx(hair, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    if int(np.count_nonzero(hair)) < 40:
        return None
    return hair


def _silhouette_overhead(image):
    """Clothed body on the table. Hue and bare skin are not required."""
    if image is None or image.ndim != 3:
        return None
    person = _person_mask(image)
    if person is None:
        return None
    height, width = person.shape[:2]
    ys, xs = np.where(person > 0)
    if len(xs) < 400:
        return None
    points = np.stack([xs.astype(np.float64), ys.astype(np.float64)], axis=1)
    center = points.mean(axis=0)
    spread = points - center
    cov = np.cov(spread.T)
    if cov.shape != (2, 2) or not np.all(np.isfinite(cov)):
        return None
    _vals, vecs = np.linalg.eigh(cov)
    axis = vecs[:, 1]
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-6:
        return None
    axis = axis / axis_norm
    along = np.dot(spread, axis)
    lo, hi = np.percentile(along, 6), np.percentile(along, 94)
    if hi - lo < min(width, height) * 0.18:
        return None
    end_a = center + axis * lo
    end_b = center + axis * hi
    head, _far = _depth_head_hip(person > 0, end_a, end_b, axis, image)
    _center, direction = _plate_axis(points, head)
    hip = _plate_hip(points, _center, direction, station_pct=80)
    side = np.array([-direction[1], direction[0]], dtype=np.float64)
    if np.dot(side, np.array([0.0, -1.0])) < 0:
        side = -side
    length = float(np.linalg.norm(hip - np.asarray(head, dtype=np.float64)))
    if length < min(width, height) * 0.16:
        return None
    hair = _silhouette_hair(image, person, head, direction, length)
    if hair is not None:
        head_area = float(np.count_nonzero(hair))
    else:
        head_along = np.dot(points - np.asarray(head, dtype=np.float64), direction)
        head_area = float(np.count_nonzero(head_along < max(48.0, length * 0.18)))
    radius = max(40.0, min(160.0, (head_area / np.pi) ** 0.5))
    nape_along = min(max(radius * 0.90 + 14.0, length * 0.10), length * 0.26)
    nape = np.asarray(head, dtype=np.float64) + direction * nape_along
    if float(np.linalg.norm(hip - nape)) < min(width, height) * 0.16:
        return None
    filled = cv2.morphologyEx(person, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))
    rows = _spine_profile(filled, nape, hip, side)
    nape_width, shoulder_width, chest_width, waist_width, shoulder_t = _robust_widths(rows)
    shoulder_width = min(400.0, max(shoulder_width, chest_width * 1.22))
    atlas = _landmarks_from_profile(
        hair,
        head,
        nape,
        hip,
        side,
        rows,
        nape_width,
        shoulder_width,
        chest_width,
        waist_width,
        shoulder_t,
    )
    torso = center
    area = float(len(xs))
    edge = min(float(head[0]), float(head[1]), width - 1.0 - float(head[0]), height - 1.0 - float(head[1]))
    body = {
        "area": area,
        "center": (float(torso[0]), float(torso[1])),
        "neck": (float(nape[0]), float(nape[1])),
        "nape": (float(nape[0]), float(nape[1])),
        "hip": (float(hip[0]), float(hip[1])),
        "head": (float(head[0]), float(head[1])),
        "waist": (float(hip[0]), float(hip[1])),
        "right": (float(side[0]), float(side[1])),
        "width": shoulder_width,
        "nape_width": nape_width,
        "shoulder_width": shoulder_width,
        "chest_width": chest_width,
        "waist_width": waist_width,
        "length": float(np.linalg.norm(hip - nape)),
        "bounds": (
            float(np.percentile(xs, 4)),
            float(np.percentile(ys, 4)),
            float(np.percentile(xs, 96)),
            float(np.percentile(ys, 96)),
        ),
        "upper": True,
        "head_edge": float(edge),
        "border_touch": edge < 36,
        "head_clipped": edge < 36,
        "waist_clipped": _near_border(hip, width, height, 36),
        "acupoints": atlas,
        "source": "silhouette",
    }
    body.update(atlas)
    return body


def _neck_pinch(mask, head, hip, side):
    """Nape is the narrowest slice after the head, before the back plate widens."""
    rows = _spine_profile(mask, head, hip, side)
    early = [row for row in rows if 0.10 <= row["t"] <= 0.34 and row["width"] > 18]
    if not early:
        return np.array(head, float) + (np.array(hip, float) - np.array(head, float)) * 0.16
    pinch = min(early, key=lambda row: row["width"])
    return np.array(pinch["center"], float)


def _sane_body(body):
    if not body:
        return False
    for key in ("head", "nape", "hip"):
        point = body.get(key)
        if point is None or not np.all(np.isfinite(point)):
            return False
    length = float(np.linalg.norm(np.array(body["hip"], float) - np.array(body["nape"], float)))
    width = float(body.get("width") or 0)
    return 80.0 < length < 1400.0 and 80.0 < width < 520.0


def median_body(bodies):
    """One body from several frames so a blink of the mask does not move every acupoint."""
    bodies = [body for body in bodies if _sane_body(body)]
    if not bodies:
        return None
    if len(bodies) >= 3:
        napes = np.stack([np.array(body["nape"], float) for body in bodies])
        hips = np.stack([np.array(body["hip"], float) for body in bodies])
        nape_mid = np.median(napes, axis=0)
        hip_mid = np.median(hips, axis=0)
        kept = []
        for body, nape, hip in zip(bodies, napes, hips):
            if float(np.linalg.norm(nape - nape_mid)) < 70.0 and float(np.linalg.norm(hip - hip_mid)) < 90.0:
                kept.append(body)
        if len(kept) >= 2:
            bodies = kept
    if not bodies:
        return None
    if len(bodies) == 1:
        return bodies[0]
    out = dict(bodies[len(bodies) // 2])

    def mid_point(key):
        pts = [np.array(body[key], float) for body in bodies if body.get(key) is not None]
        if not pts:
            return out.get(key)
        arr = np.median(np.stack(pts, axis=0), axis=0)
        return (float(arr[0]), float(arr[1]))

    for key in ("head", "nape", "neck", "hip", "waist", "center"):
        if any(body.get(key) is not None for body in bodies):
            out[key] = mid_point(key)
    if out.get("nape") is not None:
        out["neck"] = out["nape"]
    for key in ("width", "nape_width", "shoulder_width", "chest_width", "waist_width", "length"):
        vals = [float(body[key]) for body in bodies if body.get(key) is not None]
        if vals:
            out[key] = float(np.median(vals))
    sides = [np.array(body["right"], float) for body in bodies if body.get("right") is not None]
    if sides:
        side = np.median(np.stack(sides, axis=0), axis=0)
        norm = float(np.linalg.norm(side)) or 1.0
        side = side / norm
        if np.dot(side, np.array([0.0, -1.0])) < 0:
            side = -side
        out["right"] = (float(side[0]), float(side[1]))
    names = set()
    for body in bodies:
        names.update((body.get("acupoints") or {}).keys())
    atlas = {}
    for name in names:
        pts = [
            np.array(body["acupoints"][name], float)
            for body in bodies
            if name in (body.get("acupoints") or {})
        ]
        if not pts:
            continue
        arr = np.median(np.stack(pts, axis=0), axis=0)
        atlas[name] = (float(arr[0]), float(arr[1]))
    out["acupoints"] = atlas
    out.update(atlas)
    return out


def _shape_overhead(image):
    """Smooth raised plate plus a darker oval head. Hue is ignored."""
    if image is None or image.ndim != 3:
        return None
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    sat = hsv[:, :, 1].astype(np.float32)
    val = hsv[:, :, 2].astype(np.float32)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    soft = cv2.GaussianBlur(gray, (0, 0), 2)
    texture = cv2.GaussianBlur((gray.astype(np.float32) - soft) ** 2, (0, 0), 4)
    height, width = gray.shape[:2]
    back = ((texture < 16) & (val > 72) & (val < 220) & (sat < 135)).astype(np.uint8) * 255
    back = cv2.morphologyEx(back, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    back = cv2.morphologyEx(back, cv2.MORPH_CLOSE, np.ones((13, 13), np.uint8))
    part = _largest(back, 16000)
    if part is None:
        return None
    back_mask, torso, area = part
    if area > height * width * 0.58:
        return None
    head_mask = ((texture < 22) & (val > 16) & (val < 95) & (sat < 170)).astype(np.uint8) * 255
    head_mask = cv2.morphologyEx(head_mask, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    head_mask = cv2.morphologyEx(head_mask, cv2.MORPH_CLOSE, np.ones((13, 13), np.uint8))
    head_mask[back_mask] = 0
    head_part = _compact_blob(head_mask, 2500, max_area=int(area * 0.85))
    if head_part is None:
        head_part = _largest(head_mask, 900)
    if head_part is None:
        return None
    hair, head, head_area = head_part
    if head_area > area * 0.9:
        return None
    ys, xs = np.where(back_mask)
    if len(xs) < 400:
        return None
    points = np.stack([xs.astype(np.float64), ys.astype(np.float64)], axis=1)
    _center, direction = _plate_axis(points, head)
    hip = _plate_hip(points, _center, direction)
    side = np.array([-direction[1], direction[0]])
    if np.dot(side, np.array([0.0, -1.0])) < 0:
        side = -side
    span = float(np.linalg.norm(hip - np.asarray(head, dtype=np.float64)))
    radius = max(40.0, min(160.0, (float(head_area) / np.pi) ** 0.5))
    nape_along = min(max(radius * 0.90 + 14.0, span * 0.10), span * 0.26)
    nape = np.asarray(head, dtype=np.float64) + direction * nape_along
    if float(np.linalg.norm(hip - nape)) < min(width, height) * 0.16:
        return None
    filled = cv2.morphologyEx(back_mask.astype(np.uint8) * 255, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))
    rows = _spine_profile(filled, nape, hip, side)
    nape_width, shoulder_width, chest_width, waist_width, shoulder_t = _robust_widths(rows)
    shoulder_width = min(360.0, max(shoulder_width, chest_width * 1.28))
    atlas = _landmarks_from_profile(
        hair,
        head,
        nape,
        hip,
        side,
        rows,
        nape_width,
        shoulder_width,
        chest_width,
        waist_width,
        shoulder_t,
    )
    edge = min(float(head[0]), float(head[1]), width - 1.0 - float(head[0]), height - 1.0 - float(head[1]))
    body = {
        "area": float(area),
        "center": (float(torso[0]), float(torso[1])),
        "neck": (float(nape[0]), float(nape[1])),
        "nape": (float(nape[0]), float(nape[1])),
        "hip": (float(hip[0]), float(hip[1])),
        "head": (float(head[0]), float(head[1])),
        "waist": (float(hip[0]), float(hip[1])),
        "right": (float(side[0]), float(side[1])),
        "width": shoulder_width,
        "nape_width": nape_width,
        "shoulder_width": shoulder_width,
        "chest_width": chest_width,
        "waist_width": waist_width,
        "length": float(np.linalg.norm(hip - nape)),
        "bounds": (
            float(np.percentile(xs, 4)),
            float(np.percentile(ys, 4)),
            float(np.percentile(xs, 96)),
            float(np.percentile(ys, 96)),
        ),
        "upper": True,
        "head_edge": float(edge),
        "border_touch": edge < 36,
        "head_clipped": edge < 36,
        "waist_clipped": _near_border(hip, width, height, 36),
        "acupoints": atlas,
        "source": "shape",
    }
    body.update(atlas)
    return body


def _strip_overlay(image):
    """The aiming preview paints a green-black plus through the frame center."""
    if image is None or image.ndim != 3:
        return image
    height, width = image.shape[:2]
    cx, cy = width // 2, height // 2
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
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
        return image
    return cv2.inpaint(image, mark, 3, cv2.INPAINT_TELEA)


def _color_overhead(image):
    """Beige back and brown head in the overhead RealSense color frame.

    The raw frame is landscape: head on one long side, waist on the other.
    Width is the torso plate, not the hands. Named acupoints are placed on
    nape, C7, shoulder, scapula, L2, and L4 landmarks from that plate.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue = hsv[:, :, 0].astype(np.int16)
    sat = hsv[:, :, 1].astype(np.float32)
    val = hsv[:, :, 2].astype(np.float32)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    soft = cv2.GaussianBlur(gray, (0, 0), 2)
    texture = cv2.GaussianBlur((gray.astype(np.float32) - soft) ** 2, (0, 0), 4)
    height, width = gray.shape[:2]
    back = (
        (hue > 5)
        & (hue < 40)
        & (val > 100)
        & (val < 250)
        & (sat < 90)
    ).astype(np.uint8) * 255
    back = cv2.morphologyEx(back, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    back = cv2.morphologyEx(back, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))
    part = _largest(back, 18000)
    if part is None:
        return None
    back_mask, torso, area = part
    if area > height * width * 0.62:
        return None
    head_mask = (
        (val > 35)
        & (val < 125)
        & (sat > 45)
        & (sat < 150)
        & (hue < 28)
        & (texture < 50)
    ).astype(np.uint8) * 255
    head_mask = cv2.morphologyEx(head_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    head_mask = cv2.morphologyEx(head_mask, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))
    head_part = _largest(head_mask, 1200)
    if head_part is None:
        return None
    hair, head, head_area = head_part
    if head_area > area * 0.85:
        return None
    direction = torso - head
    norm = float(np.linalg.norm(direction))
    if norm < 80:
        return None
    direction = direction / norm
    ys, xs = np.where(back_mask)
    if len(xs) < 400:
        return None
    points = np.stack([xs.astype(np.float64), ys.astype(np.float64)], axis=1)
    along = np.dot(points - head, direction)
    side = np.array([-direction[1], direction[0]])
    # Person's right is the top of this landscape frame when the head is on the left.
    if np.dot(side, np.array([0.0, -1.0])) < 0:
        side = -side
    hairline = np.array(head) + direction * float(np.min(along))
    nape = hairline + direction * 10.0
    offset = np.dot(points - nape, side)
    corridor = np.abs(offset) < max(90.0, float(np.percentile(np.abs(offset), 42)))
    if int(np.count_nonzero(corridor)) < 400:
        corridor = np.ones(len(xs), dtype=bool)
    far = along[corridor] >= np.percentile(along[corridor], 88)
    hip = np.array(
        [float(np.median(xs[corridor][far])), float(np.median(ys[corridor][far]))]
    )
    pants = (
        (hue > 40)
        & (hue < 95)
        & (val > 35)
        & (val < 170)
        & (sat > 18)
        & (texture > 18)
    ).astype(np.uint8) * 255
    pants = cv2.morphologyEx(pants, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    pants_part = _largest(pants, 4000)
    if pants_part is not None:
        _pants_mask, _pants_c, _pants_area = pants_part
        pys, pxs = np.where(_pants_mask)
        if len(pxs) > 80:
            pants_pts = np.stack([pxs.astype(np.float64), pys.astype(np.float64)], axis=1)
            pants_along = np.dot(pants_pts - nape, direction)
            toward = pants_along > 50.0
            if int(np.count_nonzero(toward)) > 80:
                hip = nape + direction * float(np.percentile(pants_along[toward], 10))
    if float(np.linalg.norm(hip - nape)) < min(width, height) * 0.16:
        return None
    axis = hip - nape
    axis_len = float(np.linalg.norm(axis))
    filled = cv2.morphologyEx(back_mask.astype(np.uint8) * 255, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))
    rows = _spine_profile(filled, nape, hip, side)
    nape_width, shoulder_width, chest_width, waist_width, shoulder_t = _robust_widths(rows)
    atlas = _landmarks_from_profile(
        hair,
        head,
        nape,
        hip,
        side,
        rows,
        nape_width,
        shoulder_width,
        chest_width,
        waist_width,
        shoulder_t,
    )
    edge = min(float(head[0]), float(head[1]), width - 1.0 - float(head[0]), height - 1.0 - float(head[1]))
    body = {
        "area": float(area),
        "center": (float(torso[0]), float(torso[1])),
        "neck": (float(nape[0]), float(nape[1])),
        "nape": (float(nape[0]), float(nape[1])),
        "hip": (float(hip[0]), float(hip[1])),
        "head": (float(head[0]), float(head[1])),
        "waist": (float(hip[0]), float(hip[1])),
        "right": (float(side[0]), float(side[1])),
        "width": shoulder_width,
        "nape_width": nape_width,
        "shoulder_width": shoulder_width,
        "chest_width": chest_width,
        "waist_width": waist_width,
        "length": axis_len,
        "bounds": (
            float(np.percentile(xs, 4)),
            float(np.percentile(ys, 4)),
            float(np.percentile(xs, 96)),
            float(np.percentile(ys, 96)),
        ),
        "upper": True,
        "head_edge": float(edge),
        "border_touch": edge < 36,
        "head_clipped": edge < 36,
        "waist_clipped": _near_border(hip, width, height, 36),
        "acupoints": atlas,
    }
    body.update(atlas)
    return body


def _spine_profile(mask, nape, hip, side, steps=41):
    axis = np.array(hip, float) - np.array(nape, float)
    rows = []
    for index in range(steps):
        along = index / float(steps - 1)
        center = np.array(nape, float) + axis * along
        pos, neg, width = _axis_run(mask, center, side)
        rows.append({"t": along, "center": center, "pos": pos, "neg": neg, "width": width})
    return rows


def _axis_run(mask, center, side, max_half=320):
    """Contiguous mask run through the spine, so a thin overlay gap does not stop the walk."""
    height, width = mask.shape[:2]
    occupancy = []
    offsets = []
    for dist in range(-max_half, max_half + 1):
        point = center + side * float(dist)
        x = int(round(point[0]))
        y = int(round(point[1]))
        inside = 0 <= x < width and 0 <= y < height and mask[y, x] > 0
        occupancy.append(inside)
        offsets.append(dist)
    mid = max_half
    if not occupancy[mid]:
        nearest = None
        for index, filled in enumerate(occupancy):
            if not filled:
                continue
            if nearest is None or abs(offsets[index]) < abs(offsets[nearest]):
                nearest = index
        if nearest is None:
            return 0.0, 0.0, 0.0
        mid = nearest
    left = right = mid
    while left > 0 and occupancy[left - 1]:
        left -= 1
    while right < len(occupancy) - 1 and occupancy[right + 1]:
        right += 1
    pos = float(offsets[right])
    neg = float(offsets[left])
    return pos, neg, pos - neg


def _robust_widths(rows):
    """Neck is measured where arms cannot reach. Later widths that jump to the hands are discarded."""
    early = [row["width"] for row in rows if row["t"] <= 0.10 and row["width"] > 24]
    nape_width = float(np.median(early)) if early else 110.0
    nape_width = min(160.0, max(70.0, nape_width))
    later = [row["width"] for row in rows if 0.20 <= row["t"] <= 0.72]
    plate = [width for width in later if 1.35 * nape_width <= width <= 2.9 * nape_width]
    if len(plate) >= 3:
        chest_width = float(np.median(plate))
    else:
        chest_width = min(340.0, max(180.0, 2.05 * nape_width))
    shoulder = nape_width
    shoulder_t = 0.14
    for row in rows:
        if not (0.07 <= row["t"] <= 0.28):
            continue
        width = min(row["width"], chest_width * 1.18)
        if width >= shoulder:
            shoulder = width
            shoulder_t = row["t"]
    shoulder_width = min(chest_width * 1.38, max(nape_width * 1.35, shoulder))
    waist_vals = [
        min(row["width"], chest_width * 1.08)
        for row in rows
        if 0.74 <= row["t"] <= 0.96 and row["width"] > 24
    ]
    waist_width = float(np.median(waist_vals)) if waist_vals else chest_width * 0.94
    waist_width = min(chest_width * 1.05, max(chest_width * 0.86, waist_width))
    return nape_width, shoulder_width, chest_width, waist_width, shoulder_t


def _landmarks_from_profile(
    hair,
    head,
    nape,
    hip,
    side,
    rows,
    nape_width,
    shoulder_width,
    chest_width,
    waist_width,
    shoulder_t,
):
    nape = np.array(nape, float)
    hip = np.array(hip, float)
    side = np.array(side, float)
    axis = hip - nape
    c7 = nape + axis * 0.028
    fengchi_l, fengchi_r = _fengchi(hair, nape, axis, side, nape_width)
    row = min(rows, key=lambda item: abs(item["t"] - shoulder_t)) if rows else None
    half_sh = shoulder_width / 2.0
    if row is not None:
        pos = row["pos"] if 0.35 * half_sh < row["pos"] < 1.25 * half_sh else half_sh
        neg = abs(row["neg"]) if 0.35 * half_sh < abs(row["neg"]) < 1.25 * half_sh else half_sh
        acromion_r = row["center"] + side * pos
        acromion_l = row["center"] - side * neg
    else:
        acromion_r = nape + axis * 0.14 + side * half_sh
        acromion_l = nape + axis * 0.14 - side * half_sh
    jianjing_r = c7 + (acromion_r - c7) * 0.64
    jianjing_l = c7 + (acromion_l - c7) * 0.64
    half_ch = chest_width / 2.0
    tianzong_r = nape + axis * 0.36 + side * (half_ch * 0.68)
    tianzong_l = nape + axis * 0.36 - side * (half_ch * 0.68)
    mingmen = nape + axis * 0.80
    yaoyangguan = nape + axis * 0.945
    # Lumbar laterality uses the back plate, not a hole-punched waist slice.
    half_w = max(waist_width, chest_width * 0.88) / 2.0
    shenshu = half_w * 0.42
    dachang = half_w * 0.45
    yaobei = half_w * 0.58
    return {
        "大椎": (float(c7[0]), float(c7[1])),
        "左风池": (float(fengchi_l[0]), float(fengchi_l[1])),
        "右风池": (float(fengchi_r[0]), float(fengchi_r[1])),
        "左肩井": (float(jianjing_l[0]), float(jianjing_l[1])),
        "右肩井": (float(jianjing_r[0]), float(jianjing_r[1])),
        "左天宗": (float(tianzong_l[0]), float(tianzong_l[1])),
        "右天宗": (float(tianzong_r[0]), float(tianzong_r[1])),
        "命门": (float(mingmen[0]), float(mingmen[1])),
        "左肾俞": (float(mingmen[0] - side[0] * shenshu), float(mingmen[1] - side[1] * shenshu)),
        "右肾俞": (float(mingmen[0] + side[0] * shenshu), float(mingmen[1] + side[1] * shenshu)),
        "腰阳关": (float(yaoyangguan[0]), float(yaoyangguan[1])),
        "左大肠俞": (
            float(yaoyangguan[0] - side[0] * dachang),
            float(yaoyangguan[1] - side[1] * dachang),
        ),
        "右大肠俞": (
            float(yaoyangguan[0] + side[0] * dachang),
            float(yaoyangguan[1] + side[1] * dachang),
        ),
        "左腰背": (float(mingmen[0] - side[0] * yaobei), float(mingmen[1] - side[1] * yaobei)),
        "右腰背": (float(mingmen[0] + side[0] * yaobei), float(mingmen[1] + side[1] * yaobei)),
    }


def _fengchi(hair, nape, axis, side, nape_width):
    """Occiput-nape depressions, not the crown or the ears."""
    nape = np.array(nape, float)
    axis_len = float(np.linalg.norm(axis))
    axis_u = axis / axis_len if axis_len > 1e-6 else np.array([1.0, 0.0])
    along = -20.0
    half = max(36.0, nape_width * 0.50)
    if hair is not None:
        ys, xs = np.where(hair)
        if len(xs) >= 40:
            points = np.stack([xs.astype(np.float64), ys.astype(np.float64)], axis=1)
            hair_along = np.dot(points - nape, axis_u)
            hair_side = np.dot(points - nape, side)
            band = (hair_along > -44.0) & (hair_along < -6.0)
            if int(np.count_nonzero(band)) >= 30:
                along = float(np.median(hair_along[band]))
                half = max(half, float(np.percentile(np.abs(hair_side[band]), 70)))
            else:
                along = float(np.percentile(hair_along, 20))
            along = min(-8.0, max(-46.0, along))
            half = min(78.0, max(36.0, half))
    return nape + axis_u * along - side * half, nape + axis_u * along + side * half


def _ensure_landmarks(body):
    if body is None or body.get("acupoints"):
        return body
    nape = np.array(body.get("nape") or body["neck"], float)
    hip = np.array(body["hip"], float)
    side = np.array(body["right"], float)
    width = float(body.get("width") or 220.0)
    nape_width = float(body.get("nape_width") or min(150.0, max(80.0, width * 0.42)))
    shoulder_width = float(body.get("shoulder_width") or width)
    chest_width = float(body.get("chest_width") or width * 0.92)
    waist_width = float(body.get("waist_width") or width * 0.82)
    body.setdefault("nape", (float(nape[0]), float(nape[1])))
    body.setdefault("nape_width", nape_width)
    body.setdefault("shoulder_width", shoulder_width)
    body.setdefault("chest_width", chest_width)
    body.setdefault("waist_width", waist_width)
    atlas = _landmarks_from_profile(
        None,
        body.get("head") or nape,
        nape,
        hip,
        side,
        [],
        nape_width,
        shoulder_width,
        chest_width,
        waist_width,
        0.14,
    )
    body["acupoints"] = atlas
    return body


def person_line(image):
    """Head, torso center, and waist. Those three sit on one line."""
    if image is None or image.size == 0:
        return None
    if image.ndim == 2:
        color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    else:
        color = image
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    height, width = gray.shape[:2]
    soft = cv2.GaussianBlur(gray, (0, 0), 3)
    texture = cv2.GaussianBlur((gray.astype(np.float32) - soft) ** 2, (0, 0), 5)
    saturation = hsv[:, :, 1].astype(np.float32)
    value = hsv[:, :, 2].astype(np.float32)
    if float(np.median(saturation)) < 15:
        return _upper_body_gray(gray)
    back = (
        (texture < 16) & (value > 90) & (value < 190) & (saturation > 25) & (saturation < 100)
    ).astype(np.uint8) * 255
    back = cv2.morphologyEx(back, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    back = cv2.morphologyEx(back, cv2.MORPH_CLOSE, np.ones((25, 25), np.uint8))
    part = _largest(back, 12000)
    if part is None:
        return None
    back_mask, torso, area = part
    if area > height * width * 0.55:
        return None
    ys, xs = np.where(back_mask)
    if len(xs) < 400:
        return None
    back_val = float(np.median(value[back_mask]))
    dark = (
        (value < back_val - 20)
        & (value > 25)
        & (saturation > 40)
        & (texture < 25)
    ).astype(np.uint8) * 255
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    dark[back_mask] = 0
    outside = (~back_mask).astype(np.uint8)
    distance = cv2.distanceTransform(outside, cv2.DIST_L2, 5)
    near = distance < 200
    heads = cv2.connectedComponentsWithStats(dark, 8)
    chosen = None
    for index in range(1, heads[0]):
        blob = heads[1] == index
        if not np.any(blob & near):
            continue
        blob_area = int(heads[2][index, cv2.CC_STAT_AREA])
        if blob_area < 2500 or blob_area > 80000:
            continue
        box_w = int(heads[2][index, cv2.CC_STAT_WIDTH])
        box_h = int(heads[2][index, cv2.CC_STAT_HEIGHT])
        aspect = max(box_w, box_h) / max(min(box_w, box_h), 1)
        if aspect > 3.4:
            continue
        point = np.array([float(heads[3][index][0]), float(heads[3][index][1])])
        tex = float(np.median(texture[blob]))
        if tex >= 8:
            continue
        sat = float(np.median(saturation[blob]))
        if sat > 110:
            continue
        border = min(point[0], point[1], width - 1.0 - point[0], height - 1.0 - point[1])
        if chosen is None or border < chosen[0]:
            chosen = (border, point, index)
    points = np.stack([xs.astype(np.float64), ys.astype(np.float64)], axis=1)
    hair_edge = None
    if chosen is None:
        core = cv2.erode(
            back_mask.astype(np.uint8) * 255, np.ones((35, 35), np.uint8)
        )
        cy, cx = np.where(core > 0)
        if len(cx) < 400:
            cy, cx = ys, xs
        mid_x = float(np.median(cx))
        mid_y = float(np.median(cy))
        across = cx[np.abs(cy - mid_y) < 50]
        upright = cy[np.abs(cx - mid_x) < 50]
        span_x = float(np.percentile(across, 92) - np.percentile(across, 8)) if len(across) > 50 else 0.0
        span_y = float(np.percentile(upright, 92) - np.percentile(upright, 8)) if len(upright) > 50 else 0.0
        if span_x < 80 and span_y < 80:
            return None
        if span_x >= span_y:
            left_x = float(np.percentile(across, 8))
            right_x = float(np.percentile(across, 92))
            left = np.array([left_x, mid_y])
            right = np.array([right_x, mid_y])
            left_band = np.abs(xs - left_x) < 36
            right_band = np.abs(xs - right_x) < 36
            left_w = float(np.percentile(ys[left_band], 90) - np.percentile(ys[left_band], 10)) if np.any(left_band) else 1e9
            right_w = float(np.percentile(ys[right_band], 90) - np.percentile(ys[right_band], 10)) if np.any(right_band) else 1e9
            head, waist = (left, right) if left_w <= right_w else (right, left)
        else:
            top_y = float(np.percentile(upright, 8))
            bot_y = float(np.percentile(upright, 92))
            top = np.array([mid_x, top_y])
            bot = np.array([mid_x, bot_y])
            top_band = np.abs(ys - top_y) < 36
            bot_band = np.abs(ys - bot_y) < 36
            top_w = float(np.percentile(xs[top_band], 90) - np.percentile(xs[top_band], 10)) if np.any(top_band) else 1e9
            bot_w = float(np.percentile(xs[bot_band], 90) - np.percentile(xs[bot_band], 10)) if np.any(bot_band) else 1e9
            head, waist = (top, bot) if top_w <= bot_w else (bot, top)
        direction = waist - head
    else:
        head = chosen[1]
        direction = torso - head
        box = heads[2][chosen[2]]
        hair_left = float(box[cv2.CC_STAT_LEFT])
        hair_top = float(box[cv2.CC_STAT_TOP])
        hair_right = hair_left + float(box[cv2.CC_STAT_WIDTH])
        hair_bottom = hair_top + float(box[cv2.CC_STAT_HEIGHT])
        hair_edge = min(hair_left, hair_top, width - hair_right, height - hair_bottom)
    norm = float(np.linalg.norm(direction))
    if norm < 40:
        return None
    direction = direction / norm
    along = np.dot(points - torso, direction)
    far = along >= np.percentile(along, 88)
    waist = np.array([float(np.median(xs[far])), float(np.median(ys[far]))])
    length = float(np.linalg.norm(waist - head))
    if length < min(width, height) * 0.12:
        return None
    side = np.array([-direction[1], direction[0]])
    if side[1] < 0:
        side = -side
    if hair_edge is None:
        hair_edge = min(head[0], head[1], width - 1.0 - head[0], height - 1.0 - head[1])
    # Hair box (or the head point) must sit inside the page, not on the cut edge.
    head_clipped = hair_edge < 70.0
    waist_clipped = _near_border(waist, width, height, 48)
    return {
        "area": float(area),
        "center": (float(torso[0]), float(torso[1])),
        "neck": (float(head[0]), float(head[1])),
        "hip": (float(waist[0]), float(waist[1])),
        "head": (float(head[0]), float(head[1])),
        "waist": (float(waist[0]), float(waist[1])),
        "right": (float(side[0]), float(side[1])),
        "width": float(np.percentile(xs, 96) - np.percentile(xs, 4)),
        "length": length,
        "bounds": (
            float(np.percentile(xs, 4)),
            float(np.percentile(ys, 4)),
            float(np.percentile(xs, 96)),
            float(np.percentile(ys, 96)),
        ),
        "upper": True,
        "head_edge": float(hair_edge),
        "border_touch": head_clipped,
        "head_clipped": head_clipped,
        "waist_clipped": waist_clipped,
    }


def _gap_to_box(point, box):
    left, top, box_w, box_h = [float(value) for value in box]
    right = left + box_w
    bottom = top + box_h
    dx = 0.0 if left <= point[0] <= right else min(abs(point[0] - left), abs(point[0] - right))
    dy = 0.0 if top <= point[1] <= bottom else min(abs(point[1] - top), abs(point[1] - bottom))
    return float((dx * dx + dy * dy) ** 0.5)


def _strip_has_body(color, side):
    """True when the mannequin or hair is sitting on that picture edge."""
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    soft = cv2.GaussianBlur(gray, (0, 0), 3)
    texture = cv2.GaussianBlur((gray.astype(np.float32) - soft) ** 2, (0, 0), 5)
    sat = hsv[:, :, 1].astype(np.float32)
    val = hsv[:, :, 2].astype(np.float32)
    skin = (texture < 22) & (val > 70) & (val < 200) & (sat > 12) & (sat < 110)
    if side == "left":
        strip = skin[:, :18]
    elif side == "right":
        strip = skin[:, -18:]
    elif side == "top":
        strip = skin[:18, :]
    else:
        strip = skin[-18:, :]
    return float(np.mean(strip)) > 0.06


def _near_border(point, width, height, margin):
    return bool(
        point[0] < margin
        or point[1] < margin
        or point[0] > width - margin
        or point[1] > height - margin
    )


def _classic_body(gray):
    blur = cv2.GaussianBlur(gray, (31, 31), 0)
    _level, bright = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    dark = cv2.bitwise_not(bright)
    best = None
    for mask in (bright, dark):
        body = _body_from_mask(mask)
        if body is None:
            continue
        if best is None or body["area"] > best["area"]:
            best = body
    return best


def _plate_body(gray):
    """The smooth back filling an overhead view, with the neck toward the head."""
    blur = cv2.GaussianBlur(gray, (21, 21), 0)
    height, width = blur.shape[:2]
    patch = blur[height // 3 : height * 2 // 3, width // 3 : width * 2 // 3]
    if patch.size == 0:
        return None
    mid = float(np.median(patch))
    mask = ((blur >= mid - 22) & (blur <= mid + 22)).astype(np.uint8) * 255
    kernel = np.ones((11, 11), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    body = _body_from_mask(mask, minimum=0.08, maximum=0.70)
    if body is None:
        return None
    return _turn_toward_head(body, blur, mid)


def _turn_toward_head(body, blur, mid):
    """Point the neck at a compact head beside the torso. Otherwise keep the wider end."""
    height, width = blur.shape[:2]
    dark = (blur < mid - 18).astype(np.uint8) * 255
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    count, _labels, stats, centroids = cv2.connectedComponentsWithStats(dark, 8)
    image_area = float(height * width)
    neck = np.array(body["neck"], float)
    hip = np.array(body["hip"], float)
    nearest = None
    for label in range(1, count):
        area = float(stats[label, cv2.CC_STAT_AREA])
        if area < image_area * 0.015 or area > image_area * 0.12:
            continue
        box_w = float(stats[label, cv2.CC_STAT_WIDTH])
        box_h = float(stats[label, cv2.CC_STAT_HEIGHT])
        if max(box_w, box_h) > min(box_w, box_h) * 2.2:
            continue
        center = np.array(centroids[label], float)
        to_neck = float(np.linalg.norm(center - neck))
        to_hip = float(np.linalg.norm(center - hip))
        distance = min(to_neck, to_hip)
        if nearest is None or distance < nearest[0]:
            nearest = (distance, to_neck, to_hip)
    if nearest is not None and nearest[2] + 40 < nearest[1]:
        body["neck"] = (float(hip[0]), float(hip[1]))
        body["hip"] = (float(neck[0]), float(neck[1]))
    return body


def _prefer(classic, plate, shape):
    if plate is None:
        return classic
    if classic is None:
        return plate
    if _central(plate, shape) >= _central(classic, shape):
        return plate
    return classic


def _central(body, shape):
    height, width = shape[:2]
    center = body["center"]
    dx = abs(float(center[0]) - width / 2.0) / max(width / 2.0, 1.0)
    dy = abs(float(center[1]) - height / 2.0) / max(height / 2.0, 1.0)
    return float(body["area"]) / (0.35 + (dx * dx + dy * dy) ** 0.5)


def place_points(points, body, width, height):
    """Place named acupoints from back landmarks. Unknown names keep the old layout map."""
    _ensure_landmarks(body)
    atlas = body.get("acupoints") or {}
    placed = []
    nape = np.array(body.get("nape") or body["neck"], float)
    hip = np.array(body["hip"], float)
    right = np.array(body["right"], float)
    half = float(body.get("shoulder_width") or body["width"]) / 2.0
    for name, x, y in points:
        pixel = atlas.get(name)
        if pixel is None:
            span_x = RIGHT_X - LEFT_X
            span_y = HIP_Y - NECK_Y
            across = min(1.0, max(0.0, (float(x) - LEFT_X) / span_x))
            along = min(1.0, max(0.0, (float(y) - NECK_Y) / span_y))
            point = nape * (1.0 - along) + hip * along + right * ((across - 0.5) * 2.0 * half)
            pixel = (float(point[0]), float(point[1]))
        u = min(width - 2.0, max(1.0, float(pixel[0])))
        v = min(height - 2.0, max(1.0, float(pixel[1])))
        placed.append((u, v))
    return placed


def _body_from_mask(mask, minimum=0.06, maximum=0.55):
    height, width = mask.shape[:2]
    image_area = float(height * width)
    kernel = np.ones((9, 9), np.uint8)
    cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)
    count, labels, stats, _centroids = cv2.connectedComponentsWithStats(cleaned, 8)
    chosen = None
    for label in range(1, count):
        area = float(stats[label, cv2.CC_STAT_AREA])
        if area < image_area * minimum or area > image_area * maximum:
            continue
        touches = (
            stats[label, cv2.CC_STAT_LEFT] <= 1
            and stats[label, cv2.CC_STAT_TOP] <= 1
            and stats[label, cv2.CC_STAT_LEFT] + stats[label, cv2.CC_STAT_WIDTH] >= width - 2
            and stats[label, cv2.CC_STAT_TOP] + stats[label, cv2.CC_STAT_HEIGHT] >= height - 2
        )
        if touches:
            continue
        if chosen is None or area > chosen[0]:
            chosen = (area, label)
    if chosen is None:
        return None
    area, label = chosen
    ys, xs = np.where(labels == label)
    if len(xs) < 200:
        return None
    coords = np.column_stack([xs, ys]).astype(np.float64)
    center = coords.mean(axis=0)
    spread = coords - center
    _values, vectors = np.linalg.eigh(np.cov(spread.T))
    long_axis = vectors[:, 1]
    long_axis = long_axis / np.linalg.norm(long_axis)
    side = np.array([-long_axis[1], long_axis[0]])
    along = spread[:, 0] * long_axis[0] + spread[:, 1] * long_axis[1]
    length = float(np.percentile(along, 96) - np.percentile(along, 4))
    if not np.isfinite(length) or length < min(width, height) * 0.18:
        return None
    wide_pos = _band_width(spread, long_axis, side, length, 0.32)
    wide_neg = _band_width(spread, long_axis, side, length, -0.32)
    if max(wide_pos, wide_neg) < 20:
        return None
    aspect = length / max(wide_pos, wide_neg, 1.0)
    if aspect < 0.85 or aspect > 6.0:
        return None
    neck_dir = long_axis if wide_pos >= wide_neg else -long_axis
    reach = length * 0.42
    neck = center + neck_dir * reach
    hip = center - neck_dir * reach
    right = side if side[0] >= 0 else -side
    width_px = max(wide_pos, wide_neg)
    return {
        "area": area,
        "center": (float(center[0]), float(center[1])),
        "neck": (float(neck[0]), float(neck[1])),
        "hip": (float(hip[0]), float(hip[1])),
        "right": (float(right[0]), float(right[1])),
        "width": float(width_px),
        "length": float(length),
        "bounds": (
            float(np.percentile(xs, 4)),
            float(np.percentile(ys, 4)),
            float(np.percentile(xs, 96)),
            float(np.percentile(ys, 96)),
        ),
    }


def _band_width(spread, long_axis, side, length, where):
    along = spread[:, 0] * long_axis[0] + spread[:, 1] * long_axis[1]
    across = spread[:, 0] * side[0] + spread[:, 1] * side[1]
    band = np.abs(along - where * length) < length * 0.1
    if int(band.sum()) < 25:
        return 0.0
    return float(np.percentile(across[band], 95) - np.percentile(across[band], 5))



def _gap_to_mask(point, xs, ys):
    """Distance from a point to the bounding box of a mask. Zero when inside."""
    dx = 0.0
    dy = 0.0
    if point[0] < xs.min():
        dx = float(xs.min() - point[0])
    elif point[0] > xs.max():
        dx = float(point[0] - xs.max())
    if point[1] < ys.min():
        dy = float(ys.min() - point[1])
    elif point[1] > ys.max():
        dy = float(point[1] - ys.max())
    return (dx * dx + dy * dy) ** 0.5


def _largest(mask, minimum):
    count, labels, stats, _centroids = cv2.connectedComponentsWithStats(mask, 8)
    if count < 2:
        return None
    label = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = int(stats[label, cv2.CC_STAT_AREA])
    if area < minimum:
        return None
    ys, xs = np.where(labels == label)
    return labels == label, np.array([xs.mean(), ys.mean()], float), area


def _upper_body_gray(gray):
    """Head-to-waist on the infrared picture, where the back is a bright even plate."""
    height, width = gray.shape[:2]
    blur = cv2.medianBlur(gray, 5)
    mask = ((blur > 135) & (blur < 185)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((25, 25), np.uint8))
    part = _largest(mask, 8000)
    if part is None:
        return None
    back_mask, center, _area = part
    ys, xs = np.where(back_mask)
    top = int(ys.min())
    bottom = int(ys.max())
    left = int(xs.min())
    right = int(xs.max())
    if bottom - top < height * 0.28:
        return None
    head = np.array([float(center[0]), float(min(height - 1, top + 24))])
    waist = np.array([float(center[0]), float(bottom)])
    head_clipped = top < 48
    waist_clipped = bottom >= height - 12 and top > 240
    side_clipped = (left < 16 and float(center[0]) < width * 0.32) or (
        right > width - 16 and float(center[0]) > width * 0.68
    )
    waist_clipped = waist_clipped or side_clipped
    length = float(np.linalg.norm(head - waist))
    return {
        "area": float(len(xs)),
        "center": (float(center[0]), float(center[1])),
        "neck": (float(head[0]), float(head[1])),
        "hip": (float(waist[0]), float(waist[1])),
        "right": (1.0, 0.0),
        "width": float(right - left),
        "length": length,
        "bounds": (float(left), float(top), float(right), float(bottom)),
        "upper": True,
        "head_edge": float(top),
        "border_touch": head_clipped,
        "head_clipped": head_clipped,
        "waist_clipped": waist_clipped,
    }


def _upper_body(image):
    """Head-to-waist axis of the bare back, not the long axis of the trousers.

    The back plate is smooth and bright. Patterned cloth is the legs. The head
    is the far end of the back, away from that cloth.
    """
    if image is None or image.ndim != 3:
        return None
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    soft = cv2.GaussianBlur(gray, (0, 0), 2)
    texture = cv2.GaussianBlur((gray.astype(np.float32) - soft) ** 2, (0, 0), 4)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]
    height, width = gray.shape[:2]
    if float(np.median(saturation)) < 15:
        return _upper_body_gray(gray)
    # The back plate stays smooth with moderate saturation. The perforated table
    # is either darker or much less saturated, and the trousers are rough.
    back = ((texture < 14) & (value > 85) & (saturation >= 35) & (saturation <= 90)).astype(np.uint8) * 255
    back = cv2.morphologyEx(back, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    back = cv2.morphologyEx(back, cv2.MORPH_CLOSE, np.ones((17, 17), np.uint8))
    back_part = _largest(back, 3000)
    if back_part is None:
        return None
    back_mask, back_center, _area = back_part
    hue = hsv[:, :, 0].astype(np.int16)
    back_hue = int(np.median(hue[back_mask]))
    back_sat = int(np.median(saturation[back_mask]))
    hue_distance = np.minimum(np.abs(hue - back_hue), 180 - np.abs(hue - back_hue))
    ys, xs = np.where(back_mask)
    # Hair is the same hue as the back, but darker and more saturated, and it
    # sits against the shoulders. Patterned cloth is only a fallback for the legs.
    hair = (
        (hue_distance <= 12)
        & (saturation >= back_sat + 20)
        & (value >= 40)
        & (value <= 140)
        & (texture < 80)
    ).astype(np.uint8) * 255
    hair = cv2.morphologyEx(hair, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    hair = cv2.morphologyEx(hair, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))
    hair_part = _largest(hair, 4000)
    cloth = (
        (texture > 100)
        & (hue_distance > 20)
        & (value > 30)
        & (value < 130)
    ).astype(np.uint8) * 255
    cloth = cv2.morphologyEx(cloth, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cloth = cv2.morphologyEx(cloth, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    cloth_part = _largest(cloth, 8000)
    head_from_hair = None
    if hair_part is not None and _gap_to_mask(hair_part[1], xs, ys) < 220:
        head_from_hair = hair_part[1]
        direction = head_from_hair - back_center
    elif cloth_part is not None:
        direction = back_center - cloth_part[1]
    else:
        direction = np.array([0.0, -1.0])
    norm = float(np.linalg.norm(direction))
    if head_from_hair is None and cloth_part is not None and norm < 30:
        return None
    if norm < 1e-3:
        return None
    direction = direction / norm if norm >= 30 else np.array([0.0, -1.0])
    along = xs * direction[0] + ys * direction[1]
    if head_from_hair is not None:
        head = np.array([float(head_from_hair[0]), float(head_from_hair[1])])
        # A few pixels of the crown may touch the border while the head is still
        # in the picture. Only treat the head as missing when its center is.
        head_clipped = bool(
            head[0] < 36
            or head[1] < 36
            or head[0] > width - 36
            or head[1] > height - 36
        )
        # The live picture is turned so the spine runs down the screen.
        # The waist is whatever part of the back sits below the head.
        band = 170
        x0 = max(0, int(head[0]) - band)
        x1 = min(width, int(head[0]) + band)
        column = back_mask[:, x0:x1]
        col_ys, _col_xs = np.where(column)
        if len(col_ys) < 400:
            waist = np.array([float(head[0]), float(min(height - 1, head[1] + 240))])
            waist_clipped = True
        else:
            bottom = col_ys >= np.percentile(col_ys, 75)
            waist = np.array([float(head[0]), float(np.median(col_ys[bottom]))])
            waist_clipped = bool(int(col_ys.max()) >= height - 20)
        side_clipped = bool((x0 <= 16 and column[:, :12].any()) or (x1 >= width - 16 and column[:, -12:].any()))
        waist_clipped = waist_clipped or side_clipped
        direction = np.array([0.0, 1.0])
    else:
        low = along <= np.percentile(along, 8)
        high = along >= np.percentile(along, 94)
        head = np.array([float(xs[high].mean()), float(ys[high].mean())])
        waist = np.array([float(xs[low].mean()), float(ys[low].mean())])
        head_clipped = bool(
            xs[high].min() < 20
            or ys[high].min() < 20
            or xs[high].max() > width - 20
            or ys[high].max() > height - 20
        )
        waist_clipped = bool(
            xs[low].min() < 20
            or ys[low].min() < 20
            or xs[low].max() > width - 20
            or ys[low].max() > height - 20
        )
    length = float(np.linalg.norm(head - waist))
    if length < min(width, height) * 0.08:
        return None
    side = np.array([-direction[1], direction[0]])
    if side[0] < 0:
        side = -side
    edge = min(float(head[0]), float(head[1]), width - 1.0 - float(head[0]), height - 1.0 - float(head[1]))
    return {
        "area": float(len(xs)),
        "center": (float(back_center[0]), float(back_center[1])),
        "neck": (float(head[0]), float(head[1])),
        "hip": (float(waist[0]), float(waist[1])),
        "right": (float(side[0]), float(side[1])),
        "width": float(np.percentile(xs, 96) - np.percentile(xs, 4)),
        "length": length,
        "bounds": (
            float(np.percentile(xs, 4)),
            float(np.percentile(ys, 4)),
            float(np.percentile(xs, 96)),
            float(np.percentile(ys, 96)),
        ),
        "upper": True,
        "head_edge": float(edge),
        "border_touch": head_clipped,
        "head_clipped": head_clipped,
        "waist_clipped": waist_clipped,
    }


def coverage(image):
    """Whether the head, shoulders, back, and waist fit in the picture.

    Legs may leave the frame. The head end of the back has to sit inside the
    picture, not on the border.
    """
    if image is None or image.size == 0:
        return None
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    height, width = gray.shape[:2]
    floor = float(np.median(gray[: min(40, height), :]))

    def edge(strip):
        return float(np.mean(np.abs(strip.astype(np.float32) - floor) > 28.0))

    left = edge(gray[:, :28])
    right = edge(gray[:, -28:])
    top = edge(gray[:28, :])
    bottom = edge(gray[-28:, :])
    body = find_body(image)
    if body is None or not body.get("upper"):
        cropped = True
        too_close = False
        center = (width / 2.0, height / 2.0)
        aim = center
        length = 0.0
        bounds = (0.0, 0.0, float(width), float(height))
        head_edge = 0.0
        line_angle = 0.0
        off_horizontal = 90.0
        square = False
    else:
        neck = body["neck"]
        hip = body["hip"]
        length = float(body["length"])
        bounds = body["bounds"]
        head_clipped = bool(body.get("head_clipped") or body.get("border_touch"))
        waist_clipped = bool(body.get("waist_clipped"))
        if "waist_clipped" not in body:
            waist_clipped = not (48.0 <= hip[0] <= width - 48.0 and 48.0 <= hip[1] <= height - 48.0)
        too_close = head_clipped and waist_clipped
        cropped = head_clipped or waist_clipped
        center = ((neck[0] + hip[0]) / 2.0, (neck[1] + hip[1]) / 2.0)
        head_edge = float(body.get("head_edge", 0.0))
        line_angle = float(np.degrees(np.arctan2(hip[1] - neck[1], hip[0] - neck[0])))
        folded = abs(line_angle) % 180.0
        off_horizontal = float(min(folded, 180.0 - folded))
        square = (
            off_horizontal <= 8.0
            and abs(center[1] - height / 2.0) <= 40.0
            and abs(center[0] - width / 2.0) <= 90.0
        )
        head_visible = (not head_clipped) and head_edge >= 70.0
        placed = (not cropped) and square and head_visible
        # Slide toward whichever end left the picture. Push past that end so
        # one step is enough to start bringing it back in.
        if waist_clipped and not head_clipped:
            target = hip
        elif head_clipped and not waist_clipped:
            target = neck
        else:
            target = center
        if cropped:
            aim = (
                float(target[0] + (target[0] - width / 2.0) * 0.45),
                float(target[1] + (target[1] - height / 2.0) * 0.45),
            )
        else:
            aim = (float(center[0]), float(center[1]))
    if body is None or not body.get("upper"):
        placed = False
        head_visible = False
        extra_u = 0.0
    else:
        extra_u = 0.0
        if head_clipped:
            extra_u = -110.0 if neck[0] < width / 2.0 else 110.0
    error_u = float(center[0] - width / 2.0 + extra_u)
    error_v = float(center[1] - height / 2.0)
    return {
        "complete": bool(placed),
        "placed": bool(placed),
        "head_visible": bool(head_visible),
        "square": square,
        "center": (float(center[0]), float(center[1])),
        "aim": (float(aim[0]), float(aim[1])),
        "too_close": too_close,
        "length": length,
        "bounds": bounds,
        "left": left,
        "right": right,
        "top": top,
        "bottom": bottom,
        "width": width,
        "height": height,
        "head_edge": head_edge,
        "line_angle": line_angle,
        "off_horizontal": off_horizontal,
        "cropped": cropped,
        "error_u": error_u,
        "error_v": error_v,
        "head": (float(neck[0]), float(neck[1])) if body is not None and body.get("upper") else None,
        "waist": (float(hip[0]), float(hip[1])) if body is not None and body.get("upper") else None,
    }
