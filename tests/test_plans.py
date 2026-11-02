"""The plan tables are product content. Check they stay coherent."""

from __future__ import annotations

import pytest

from heacon.motion.plans import PLANS, POINT_TECH, TRAVEL_TECH

PLAN_IDS = ("p1", "p2", "p3", "p4")


def test_every_plan_has_a_name_and_points():
    for plan_id in PLAN_IDS:
        plan = PLANS[plan_id]
        assert plan["name"]
        assert plan["points"], plan_id


def test_point_names_in_a_plan_are_unique():
    for plan_id in PLAN_IDS:
        names = [name for name, _u, _v in PLANS[plan_id]["points"]]
        assert len(names) == len(set(names)), plan_id


def test_point_pixels_are_inside_the_page_canvas():
    for plan_id in PLAN_IDS:
        for name, u, v in PLANS[plan_id]["points"]:
            assert 0 <= u <= 1280, (plan_id, name, u)
            assert 0 <= v <= 720, (plan_id, name, v)


def test_every_technique_named_for_a_point_exists_in_the_page():
    known = {
        "柔顺", "悬停灸", "回旋灸", "雀啄灸", "往复灸",
        "点按", "揉", "推", "拨", "滚压", "揉捏", "冲击",
    }
    for plan_id, sides in POINT_TECH.items():
        for direction, table in sides.items():
            for point, technique in table.items():
                assert technique in known, (plan_id, direction, point, technique)


def test_massage_plans_do_not_use_moxa_techniques():
    moxa = {"悬停灸", "回旋灸", "雀啄灸", "往复灸", "柔顺"}
    for plan_id in ("p3", "p4"):
        for table in POINT_TECH[plan_id].values():
            assert not (set(table.values()) & moxa), plan_id


def test_moxa_plans_are_marked_as_such():
    assert TRAVEL_TECH["p1"] == "循经灸"
    assert TRAVEL_TECH["p2"] == "循经灸"


def test_plan_ids_are_the_four_the_page_offers():
    assert set(PLANS) == set(PLAN_IDS)
