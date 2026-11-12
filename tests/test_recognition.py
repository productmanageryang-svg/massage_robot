"""Acupoint recognition: the NUC build's distinguishing feature.

The NUC replaced the old contour/skin fallback with depth geometry. Two things
must hold or the machine puts the tool in the wrong place:

1. Only the depth path is allowed (no stale fallback drawing a wrong map).
2. A depth map is required: colour alone cannot separate a back from the table.
"""

from __future__ import annotations

import numpy as np
import pytest

from heacon.vision import hybrid_geom
from heacon.vision.synthetic import synthetic_depth, synthetic_frame


def test_recognize_returns_none_without_depth():
    # This is the behaviour that made the first Linux deployment look broken:
    # a colour frame alone yields no body, so no service can start.
    frame = synthetic_frame()
    assert hybrid_geom.recognize(frame, None) is None


def test_recognize_finds_a_body_from_depth():
    frame = synthetic_frame()
    depth = synthetic_depth(frame.shape)
    body = hybrid_geom.recognize(frame, depth)
    assert body is not None, "depth geometry did not find the synthetic body"
    assert body.get("source") == "depth_geo"


def test_recognized_body_has_the_fields_the_route_expects():
    frame = synthetic_frame()
    body = hybrid_geom.recognize(frame, synthetic_depth(frame.shape))
    assert body is not None
    for key in ("neck", "hip", "width", "length"):
        assert key in body, key
    assert body["width"] > 0
    assert body["length"] > 0


def test_acupoints_are_inside_the_frame():
    frame = synthetic_frame()
    height, width = frame.shape[:2]
    body = hybrid_geom.recognize(frame, synthetic_depth(frame.shape))
    assert body is not None
    points = body.get("acupoints") or {}
    assert points, "no acupoints produced"
    for name, (u, v) in points.items():
        assert 0 <= u <= width, (name, u)
        assert 0 <= v <= height, (name, v)


def test_acupoint_names_cover_what_the_plans_need():
    frame = synthetic_frame()
    body = hybrid_geom.recognize(frame, synthetic_depth(frame.shape))
    assert body is not None
    found = set(body.get("acupoints") or {})
    # Every point any plan can ask for must be produced by recognition.
    from heacon.motion.plans import PLANS

    needed = {name for plan in PLANS.values() for name, _u, _v in plan["points"]}
    missing = needed - found
    assert not missing, "recognition does not produce: %s" % sorted(missing)


def test_find_body_does_not_fall_back_to_old_detectors():
    """body.find_body must route only through hybrid_geom.

    An old contour/skin fallback would happily return a wrong body on a bad
    frame, and the route would then treat a wrong acupoint map as correct.
    """
    import inspect

    from heacon.vision import body

    source = inspect.getsource(body.find_body)
    assert "hybrid_geom" in source
    for legacy in ("_classic_body", "_plate_body", "_shape_overhead", "_color_overhead"):
        assert legacy not in source, "find_body still calls %s" % legacy


def test_depth_below_the_valid_window_is_rejected():
    # All-zeros depth means the sensor returned nothing usable.
    frame = synthetic_frame()
    empty = np.zeros(frame.shape[:2], np.uint16)
    assert hybrid_geom.recognize(frame, empty) is None


def test_flat_depth_has_no_body():
    # A uniform table with nothing on it must not produce a body.
    frame = synthetic_frame()
    flat = np.full(frame.shape[:2], 900, np.uint16)
    assert hybrid_geom.recognize(frame, flat) is None
