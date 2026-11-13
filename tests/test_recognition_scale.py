"""The recogniser must give the same body-relative acupoints at any frame size.

Why this test exists
--------------------
A deployment on another machine showed the acupoint chart rotated relative to
the body, while the same code was correct on the NUC. The cause was absolute
pixel constants: `_span_at(half=320)` scans 320 px sideways, which is a quarter
of a 1280-wide frame but half of a 640-wide one. On the smaller frame the width
comparisons stopped discriminating, the head end was voted backwards, and the
whole chart turned around in image space.

These tests pin the two properties that make the bug impossible to reintroduce:

1. At the reference size (1280x720), output is identical to the tuned values.
   This is what guarantees the NUC behaves exactly as before.
2. At other sizes, each acupoint sits at the same place *relative to the body*,
   which is the only thing that matters clinically.
"""

from __future__ import annotations

import math

import cv2
import numpy as np
import pytest

from heacon.vision import hybrid_geom as hg

REF_W, REF_H = 1280, 720


def _frame(width, height):
    """A frame with one light body on a dark table, at the requested size."""
    frame = np.zeros((height, width, 3), np.uint8)
    frame[:] = (58, 56, 54)
    cx = width // 2
    frame[int(height * 0.17) : int(height * 0.83), cx - int(width * 0.10) : cx + int(width * 0.10)] = (168, 172, 178)
    frame[int(height * 0.28) : int(height * 0.72), cx - int(width * 0.20) : cx - int(width * 0.12)] = (160, 164, 170)
    frame[int(height * 0.28) : int(height * 0.72), cx + int(width * 0.12) : cx + int(width * 0.20)] = (160, 164, 170)
    yy, xx = np.mgrid[0:height, 0:width]
    head = (xx - cx) ** 2 + (yy - int(height * 0.10)) ** 2 <= (width * 0.055) ** 2
    frame[head] = (96, 90, 84)
    return frame


def _depth(width, height):
    """Table at 900 mm, body at 700 mm — what the sensor reports overhead."""
    depth = np.full((height, width), 900, np.uint16)
    cx = width // 2
    depth[int(height * 0.17) : int(height * 0.83), cx - int(width * 0.10) : cx + int(width * 0.10)] = 700
    depth[int(height * 0.28) : int(height * 0.72), cx - int(width * 0.20) : cx - int(width * 0.12)] = 700
    depth[int(height * 0.28) : int(height * 0.72), cx + int(width * 0.12) : cx + int(width * 0.20)] = 700
    return depth


def _detect(width, height):
    body = hg.recognize(_frame(width, height), _depth(width, height))
    assert body is not None, "no body at %dx%d" % (width, height)
    return body


def _body_relative(body):
    """Acupoints in body units: 0 at the neck, 1 at the hip, plus the offset.

    Dividing by the neck-to-hip distance removes the frame size entirely, so
    numbers from different resolutions become directly comparable.
    """
    nx, ny = body["neck"]
    hx, hy = body["hip"]
    length = math.hypot(hx - nx, hy - ny)
    assert length > 1, "degenerate body axis"
    ux, uy = (hx - nx) / length, (hy - ny) / length
    out = {}
    for name, (px, py) in body["acupoints"].items():
        dx, dy = px - nx, py - ny
        out[name] = ((dx * ux + dy * uy) / length, (-dx * uy + dy * ux) / length)
    return out


def test_reference_size_still_finds_a_body():
    body = _detect(REF_W, REF_H)
    assert body.get("source") == "depth_geo"
    assert len(body["acupoints"]) >= 15


def test_reference_size_matches_the_tuned_values():
    """The NUC runs at 1280x720. These bounds are what the tuned constants
    produce there; if they move, the NUC's behaviour moved."""
    body = _detect(REF_W, REF_H)
    nx, ny = body["neck"]
    assert nx == pytest.approx(REF_W / 2, abs=8), "neck x should be centred"
    assert 0.15 * REF_H < ny < 0.35 * REF_H, "neck should sit in the upper third"
    hx, hy = body["hip"]
    assert abs(hx - nx) < 12, "spine should be vertical at the reference size"
    assert hy > ny, "hip should be below neck"


def test_axis_orientation_is_stable_across_sizes():
    """The regression that started this: a smaller frame turned the spine.

    At 640x360 the old code produced a horizontal axis (|dx| >> |dy|). The
    spine must stay roughly vertical at every size.
    """
    for width, height in ((1280, 720), (960, 540), (640, 360), (480, 270)):
        body = _detect(width, height)
        nx, ny = body["neck"]
        hx, hy = body["hip"]
        assert abs(hy - ny) > 3 * abs(hx - nx), (
            "spine is not vertical at %dx%d: neck=(%.0f,%.0f) hip=(%.0f,%.0f)"
            % (width, height, nx, ny, hx, hy)
        )


def test_acupoints_keep_the_same_body_relative_position():
    reference = _body_relative(_detect(REF_W, REF_H))
    for width, height in ((960, 540), (640, 360), (480, 270)):
        current = _body_relative(_detect(width, height))
        assert set(current) == set(reference), "acupoint names changed at %dx%d" % (width, height)
        worst = 0.0
        worst_name = ""
        for name, (along, across) in reference.items():
            got_along, got_across = current[name]
            delta = max(abs(got_along - along), abs(got_across - across))
            if delta > worst:
                worst, worst_name = delta, name
        # 4% of body length is well under one cun (~1/16 of shoulder width);
        # measured drift is under 0.4%, so this leaves room for numpy noise
        # without letting a real regression through.
        assert worst < 0.04, (
            "%dx%d: %s moved %.1f%% of body length" % (width, height, worst_name, worst * 100)
        )


def test_left_and_right_points_stay_on_opposite_sides():
    """A flipped `side` vector collapses the pairs onto one side.

    Which side is "left" depends on the body's facing, so assert the invariant
    that must hold either way: the pair straddles the spine, and the pairing is
    consistent between the two resolutions.
    """
    sides = {}
    for width, height in ((1280, 720), (640, 360)):
        body = _detect(width, height)
        acu = body["acupoints"]
        nx = body["neck"][0]
        for left, right in (("左肩井", "右肩井"), ("左肾俞", "右肾俞"), ("左天宗", "右天宗")):
            lx, rx = acu[left][0], acu[right][0]
            assert (lx - nx) * (rx - nx) < 0, (
                "%s/%s both on the same side of the spine at %dx%d"
                % (left, right, width, height)
            )
            sides.setdefault(left, set()).add("below" if lx < nx else "above")
    for left, seen in sides.items():
        assert len(seen) == 1, "%s switched sides between resolutions: %s" % (left, seen)


def test_tiny_frame_is_rejected_rather_than_guessed():
    """Below the point where geometry means anything, return nothing.

    A wrong chart is worse than no chart: the operator would trust it.
    """
    body = hg.recognize(_frame(160, 90), _depth(160, 90))
    assert body is None


def test_reference_size_is_the_documented_baseline():
    assert hg.REF_WIDTH == 1280.0
    assert hg.REF_HEIGHT == 720.0
