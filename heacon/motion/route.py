"""Move the tool through one service plan, above the person in the camera.

Acupoints are the plan's layout, placed on the body found in the arm camera.
Massage travels the route back and forth. Ordinary points get one short press.
Heavier points get a press, a knead, and a push, with a small tilt.
Once at work height, force stays on: Z constant force on the point, XY
admittance around the moving path, Rx/Ry a small spring back to camera-down,
Rz locked. Between points the tool lifts, travels, and settles with the same
spring around x_d(t). Moxibustion holds 30 mm above every acupoint
with Cartesian admittance and does not run massage gestures.
The bare flange still has no massage or moxibustion head.
"""

import math
import threading
import time

import numpy as np

from heacon.robot.client import RobotError
from heacon.vision import body as body_route
from heacon.motion.constants import *  # noqa: F401,F403 - tuned values
from heacon.motion.plans import PLANS, POINT_TECH, TRAVEL_TECH, _EMPHASIS
from heacon.vision import geometry as vision

# Tip height above the calibrated table. Approved on site as 40 mm from skin.
# Massage and moxibustion both use this as the work height, then offset by technique.
# Allow the 16 mm press / peck dip under the 40 mm standoff, plus a little noise.
# Pose for the 就位 step, taught on site 2026-10-29. The camera looks down at
# the prone person from here. The arm returns to this pose on that step.
# Same pose. A cropped view returns here instead of backing up in small steps.
# Flange angles whose camera looks straight down. Tilting away from these faults a joint.
# Semantic gaps from skin. The live flange pose at WORK_LIFT_MM is 40 mm.
# Flange at WORK_LIFT_MM is 40 mm from skin. Moxa sits 30 mm above the acupoint.
# Visible technique size. The low point is the approved work height.

# Heavier massage points get the full press, knead, and push group.

# Same drawing coordinates as pointSets in the page.

_lock = threading.Lock()
_stop = threading.Event()
_thread = None
_running = False
_starting = False
_message = "连上机械臂和摄像头后，开始服务会按画面里的人体走穴位，并下降到假人上方。"
_plan_id = ""
_origin = None
_joints = None
_targets = []
_index = 0
_cmd_id = 1
_segments = []
_spent = 0.0
_mark = None
_route_floor = WORK_FLOOR_MM
_level_grade = 2
_technique_show = None
_gesture = ""
_emphasis = False
_round = 1
_visited = []
_speed = 1
_service_seconds = 35 * 60
_step_until = 0.0
_phase = ""
_direction = 1
_scan_origin = None
_image_size = (1280, 720)
_scan_joints = None
_positioning = False
_position_ready = False
_position_message = "机械臂还没有就位"
_position_thread = None
_active_robot = None
_hold_moves = threading.Event()
_hybrid_ok = False
_hybrid_mode = "work"
_hybrid_z = 0.0
_hybrid_bias = None
_hybrid_ft = False
_hybrid_ft_drive = False
_hybrid_damp_set = None
_hybrid_force_read = None
_hybrid_lateral_read = None
_hybrid_home = None
_hybrid_moxa_prev = None
_hybrid_moxa_prev_t = 0.0
_hybrid_moxa_log_t = 0.0
_hybrid_xd = None
_hybrid_massage_air = False
_hybrid_massage_frozen = False
_hybrid_massage_prev = None
_hybrid_massage_prev_t = 0.0
_hybrid_massage_log_t = 0.0
_hybrid_torque_bias = {"mx": 0.0, "my": 0.0, "mz": 0.0}
# Z search stays slow; XY yield uses the same linear cap.
# Skip SetForceZero if a hand is already on the tool (zero-under-load inverts F).
# Protocol max. Default FT protection is a tight band that faults on a shove.
# Massage: strategy 0, Z tracks force, XY/Rx/Ry are a soft spring around the path.
# Moxa: strategy 1 as admittance around the commanded acupoint (yield, then return).
# Huayan 柔顺 has no pose spring (goal distance unused). Spring is F_goal = K * error.


def status():
    with _lock:
        plan = PLANS.get(_plan_id, {})
        point = ""
        if _targets and 0 <= _index < len(_targets):
            point = _targets[_index]["name"]
        remain = None
        if _phase == "手法" and _step_until:
            remain = max(0, int(round(_step_until - time.monotonic())))
        massage = _plan_id in ("p3", "p4")
        extra = 0.0 if _mark is None else time.monotonic() - _mark
        remain_s = None
        if _running:
            remain_s = max(0, int(round(_service_seconds - (_spent + extra))))
        return {
            "running": _running,
            "plan": _plan_id,
            "planName": plan.get("name", ""),
            "technique": _gesture or plan.get("technique", ""),
            "gesture": _gesture,
            "point": point,
            "index": _index,
            "message": _message,
            "massage": massage,
            "emphasis": _emphasis,
            "phase": _phase or "走线",
            "working": _phase == "手法",
            "round": _round,
            "direction": _direction,
            "visited": list(_visited),
            "pointRemain": remain,
            "level": _level_grade,
            "speed": _speed,
            "positioning": _positioning,
            "positionReady": _position_ready,
            "positionMessage": _position_message,
            "remainSeconds": remain_s,
            "hybrid": (
                "" if not _hybrid_ft
                else "spring" if _plan_id in ("p1", "p2")
                else "path" if _hybrid_massage_air
                else "force-xyr"
            ),
            "forceN": _hybrid_force_read,
            "forceLateral": _hybrid_lateral_read,
            "leadPreview": False,
        }


def _plan_is_moxa():
    with _lock:
        return _plan_id in ("p1", "p2")


def _plan_is_massage():
    with _lock:
        return _plan_id in ("p3", "p4")


def _hybrid_reset():
    global _hybrid_ok, _hybrid_mode, _hybrid_z, _hybrid_bias
    global _hybrid_ft, _hybrid_ft_drive, _hybrid_damp_set, _hybrid_force_read
    global _hybrid_lateral_read, _hybrid_home
    global _hybrid_moxa_prev, _hybrid_moxa_prev_t, _hybrid_moxa_log_t
    global _hybrid_xd, _hybrid_massage_air, _hybrid_massage_frozen
    global _hybrid_massage_prev, _hybrid_massage_prev_t, _hybrid_massage_log_t
    global _hybrid_torque_bias
    with _lock:
        _hybrid_ok = False
        _hybrid_mode = "work"
        _hybrid_z = 0.0
        _hybrid_bias = None
        _hybrid_ft = False
        _hybrid_ft_drive = False
        _hybrid_damp_set = None
        _hybrid_force_read = None
        _hybrid_lateral_read = None
        _hybrid_home = None
        _hybrid_moxa_prev = None
        _hybrid_moxa_prev_t = 0.0
        _hybrid_moxa_log_t = 0.0
        _hybrid_xd = None
        _hybrid_massage_air = False
        _hybrid_massage_frozen = False
        _hybrid_massage_prev = None
        _hybrid_massage_prev_t = 0.0
        _hybrid_massage_log_t = 0.0
        _hybrid_torque_bias = {"mx": 0.0, "my": 0.0, "mz": 0.0}


def _hybrid_warm(robot):
    """Force stays off until the tool is at work height."""
    global _hybrid_ok, _hybrid_bias
    _hybrid_shutdown(robot)
    with _lock:
        _hybrid_ok = False
        _hybrid_bias = None
        _hybrid_mode = "work"


def _hybrid_leave_freedrive(robot):
    global _hybrid_ft_drive, _hybrid_damp_set
    try:
        robot.set_ft_free_drive(False)
    except RobotError:
        try:
            robot.command("SetFTFreeDriveState,0,0,;")
        except RobotError:
            pass
    with _lock:
        _hybrid_ft_drive = False
        _hybrid_damp_set = None


def _hybrid_ft_idle(robot):
    _hybrid_contact_off(robot)


def _hybrid_force_goal():
    with _lock:
        grade = _parse_grade(_level_grade)
    return max(4.0, min(12.0, 5.0 + 1.5 * grade))


def _hybrid_read_wrench(robot):
    if robot is None:
        return None
    try:
        return robot.read_force_calibrated()
    except RobotError:
        try:
            return robot.read_force()
        except RobotError:
            return None


def _hybrid_wrench_n(wrench):
    if not wrench:
        return 0.0
    return math.sqrt(
        float(wrench.get("fx") or 0.0) ** 2
        + float(wrench.get("fy") or 0.0) ** 2
        + float(wrench.get("fz") or 0.0) ** 2
    )


def _hybrid_note_force(robot):
    global _hybrid_force_read, _hybrid_lateral_read
    wrench = _hybrid_read_wrench(robot)
    if wrench is None:
        return None
    fx = float(wrench.get("fx") or 0.0)
    fy = float(wrench.get("fy") or 0.0)
    with _lock:
        _hybrid_force_read = float(wrench.get("fz") or 0.0)
        _hybrid_lateral_read = math.hypot(fx, fy)
    return wrench


def _hybrid_capture_torque_bias(wrench):
    """Remember residual Mx/My/Mz so 恒力 does not keep twisting when the hand is gone."""
    global _hybrid_torque_bias
    if not wrench:
        return
    with _lock:
        _hybrid_torque_bias = {
            "mx": float(wrench.get("mx") or 0.0),
            "my": float(wrench.get("my") or 0.0),
            "mz": float(wrench.get("mz") or 0.0),
        }


def _hybrid_torque_goal(key):
    with _lock:
        bias = dict(_hybrid_torque_bias)
    return float(bias.get(key) or 0.0)


def _hybrid_try(robot, label, fn):
    try:
        fn()
        return True
    except RobotError as exc:
        print("[heacon] hybrid %s:" % label, exc, flush=True)
        return False


def _hybrid_apply_params(robot):
    moxa = _plan_is_moxa()
    _hybrid_try(robot, "base frame", lambda: robot.set_ft_tool_frame(False))
    _hybrid_try(robot, "no free axes", lambda: robot.set_ft_motion_freedom(0, 0, 0, 0, 0, 0))
    _hybrid_try(
        robot,
        "mass",
        lambda: robot.set_ft_mass(HYBRID_XY_MASS, HYBRID_XY_MASS, HYBRID_Z_MASS, 10, 10, 10),
    )
    if moxa:
        _hybrid_try(robot, "strategy", lambda: robot.set_ft_strategy(0))
        _hybrid_try(robot, "search XYZ", lambda: robot.set_ft_search_freedom(1, 1, 1, 0, 0, 0))
        _hybrid_try(
            robot,
            "search speed",
            lambda: robot.set_ft_search_speed(HYBRID_MOXA_SEARCH_MM_S, HYBRID_SEARCH_DEG_S),
        )
        _hybrid_try(
            robot,
            "mass",
            lambda: robot.set_ft_mass(
                HYBRID_MOXA_MASS, HYBRID_MOXA_MASS, HYBRID_MOXA_MASS, 10, 10, 10
            ),
        )
        _hybrid_try(
            robot,
            "damp",
            lambda: robot.set_ft_damp(
                HYBRID_MOXA_XY_DAMP, HYBRID_MOXA_XY_DAMP, HYBRID_MOXA_Z_DAMP, 40, 40, 40
            ),
        )
        _hybrid_try(
            robot,
            "stiff",
            lambda: robot.set_ft_stiff(
                HYBRID_MOXA_XY_STIFF, HYBRID_MOXA_XY_STIFF, HYBRID_MOXA_Z_STIFF,
                HYBRID_MOXA_ROT_STIFF, HYBRID_MOXA_ROT_STIFF, HYBRID_MOXA_ROT_STIFF,
            ),
        )
        _hybrid_try(robot, "goal", lambda: robot.set_ft_goal())
        return True
    _hybrid_try(robot, "strategy", lambda: robot.set_ft_strategy(0))
    # Contact + Rx/Ry search lets the 8 N press twist the flange; lock attitude.
    _hybrid_try(robot, "search XYZ", lambda: robot.set_ft_search_freedom(1, 1, 1, 0, 0, 0))
    _hybrid_try(
        robot,
        "search speed",
        lambda: robot.set_ft_search_speed(HYBRID_MASSAGE_SEARCH_MM_S, HYBRID_MASSAGE_SEARCH_DEG_S),
    )
    _hybrid_try(
        robot,
        "mass",
        lambda: robot.set_ft_mass(HYBRID_MOXA_MASS, HYBRID_MOXA_MASS, HYBRID_MOXA_MASS, 8, 8, 8),
    )
    _hybrid_try(
        robot,
        "damp",
        lambda: robot.set_ft_damp(90, 90, 90, 25, 25, 40),
    )
    _hybrid_try(
        robot,
        "stiff",
        lambda: robot.set_ft_stiff(80, 80, 80, 40, 40, 80),
    )
    _hybrid_try(robot, "goal", lambda: robot.set_ft_goal())
    return True


def _hybrid_apply_goal(robot):
    if robot is None or not _hybrid_ft:
        return False
    if _plan_is_moxa():
        return _hybrid_moxa_spring(robot)
    return _hybrid_massage_spring(robot)


def _hybrid_set_collide(robot, level):
    _hybrid_try(
        robot, "collide %s" % level, lambda: robot.command("SetCollideLevel,0,%d,;" % int(level))
    )


def _hybrid_arm_protect(robot):
    before = _hybrid_read_wrench(robot)
    loaded = _hybrid_wrench_n(before)
    if loaded >= HYBRID_MOXA_ZERO_MAX_N:
        print(
            "[heacon] skip zero, loaded F=%.2f M=(%.3f,%.3f,%.3f)"
            % (
                loaded,
                0.0 if before is None else (before.get("mx") or 0),
                0.0 if before is None else (before.get("my") or 0),
                0.0 if before is None else (before.get("mz") or 0),
            ),
            flush=True,
        )
    else:
        _hybrid_try(robot, "zero", robot.set_force_zero)
        time.sleep(0.25)
        after = _hybrid_read_wrench(robot)
        _hybrid_capture_torque_bias(after)
        print(
            "[heacon] ft zero before=%s after F=(%.2f,%.2f,%.2f) M=(%.3f,%.3f,%.3f)"
            % (
                None if before is None else round(loaded, 2),
                0.0 if after is None else (after.get("fx") or 0),
                0.0 if after is None else (after.get("fy") or 0),
                0.0 if after is None else (after.get("fz") or 0),
                0.0 if after is None else (after.get("mx") or 0),
                0.0 if after is None else (after.get("my") or 0),
                0.0 if after is None else (after.get("mz") or 0),
            ),
            flush=True,
        )
    _hybrid_try(
        robot,
        "ft limit",
        lambda: robot.set_ft_data_limit(
            HYBRID_MOXA_FORCE_LIMIT_N, HYBRID_MOXA_TORQUE_LIMIT_NM
        ),
    )
    _hybrid_set_collide(robot, HYBRID_MOXA_COLLIDE)


def _hybrid_contact_on(robot):
    """Moxa: 30 mm spring. Massage: Z force on the point, path spring in the air."""
    global _hybrid_ft, _hybrid_mode, _hybrid_home
    global _hybrid_moxa_prev, _hybrid_moxa_prev_t
    global _hybrid_xd, _hybrid_massage_air, _hybrid_massage_frozen
    global _hybrid_massage_prev, _hybrid_massage_prev_t
    if robot is None:
        return False
    if _hybrid_ft:
        _hybrid_apply_goal(robot)
        return True
    _hybrid_leave_freedrive(robot)
    try:
        fsm = robot.read_fsm()
    except RobotError as exc:
        print("[heacon] hybrid fsm:", exc, flush=True)
        return False
    if fsm != 33:
        if fsm in (25, 27):
            if not _wait_fsm(robot, {33}, 8):
                print("[heacon] hybrid skip: still moving", fsm, flush=True)
                return False
        elif not _hybrid_unlock_limit(robot):
            print("[heacon] hybrid skip: not standby", fsm, flush=True)
            return False
    moxa = _plan_is_moxa()
    _hybrid_arm_protect(robot)
    if not _hybrid_try(robot, "force on", robot.force_control_on):
        return False
    _hybrid_apply_params(robot)
    with _lock:
        _hybrid_ft = True
        _hybrid_mode = "spring" if moxa else "force"
        if moxa:
            _hybrid_moxa_prev = None
            _hybrid_moxa_prev_t = 0.0
        else:
            _hybrid_massage_air = False
            _hybrid_massage_frozen = False
            _hybrid_massage_prev = None
            _hybrid_massage_prev_t = 0.0
    try:
        here = robot.read_pose()
    except RobotError:
        here = None
    pose = here.get("pose") if here else None
    if moxa:
        if _hybrid_home is None and pose:
            _hybrid_home = dict(pose)
        _hybrid_moxa_spring(robot)
        print("[heacon] hybrid admittance on, moxa 30mm spring", flush=True)
    else:
        if pose:
            _hybrid_xd = _lock_down(pose)
        _hybrid_massage_spring(robot)
        print(
            "[heacon] hybrid massage on Z=%.1fN path-spring down lock" % _hybrid_force_goal(),
            flush=True,
        )
    return True


def _hybrid_contact_off(robot):
    if robot is None:
        return
    was = _hybrid_ft
    _hybrid_shutdown(robot)
    if was:
        _wait_fsm(robot, {33, 24}, 6, hold=True)
        try:
            if robot.read_fsm() == 24:
                robot.command("GrpEnable,0,;")
                _wait_fsm(robot, {33}, 6, hold=True)
        except RobotError:
            pass
        print("[heacon] hybrid force off", flush=True)


def _hybrid_shutdown(robot):
    global _hybrid_ft, _hybrid_force_read, _hybrid_lateral_read, _hybrid_home
    global _hybrid_moxa_prev, _hybrid_moxa_prev_t
    global _hybrid_xd, _hybrid_massage_air, _hybrid_massage_frozen
    global _hybrid_massage_prev, _hybrid_massage_prev_t
    global _hybrid_torque_bias
    if robot is None:
        return
    _hybrid_leave_freedrive(robot)
    try:
        robot.force_control_off()
    except RobotError:
        pass
    _hybrid_set_collide(robot, 3)
    with _lock:
        _hybrid_ft = False
        _hybrid_force_read = None
        _hybrid_lateral_read = None
        _hybrid_home = None
        _hybrid_moxa_prev = None
        _hybrid_moxa_prev_t = 0.0
        _hybrid_xd = None
        _hybrid_massage_air = False
        _hybrid_massage_frozen = False
        _hybrid_massage_prev = None
        _hybrid_massage_prev_t = 0.0
        _hybrid_torque_bias = {"mx": 0.0, "my": 0.0, "mz": 0.0}


def _hybrid_blocked(exc):
    text = str(exc)
    return "20018" in text or "40004" in text or "40010" in text


def _hybrid_unreachable(exc):
    text = str(exc)
    return "20031" in text or "10022" in text or "Point calculation failed" in text


def _hybrid_unlock_limit(robot):
    """Joint or workspace edge: reset and stay movable."""
    try:
        fsm = robot.read_fsm()
    except RobotError:
        return False
    if fsm == 33:
        return True
    try:
        if fsm in (20, 21, 22):
            try:
                robot.command("GrpReset,0,;", timeout=1.0)
            except RobotError:
                pass
            time.sleep(0.2)
            try:
                fsm = robot.read_fsm()
            except RobotError:
                return False
            if fsm == 20:
                try:
                    robot.command("XToStandby,0,;", timeout=2.0)
                except RobotError:
                    return False
                return bool(_wait_fsm(robot, {33}, 8, hold=True))
        if fsm == 25:
            try:
                robot.command("GrpStop,0,;", timeout=1.0)
            except RobotError:
                pass
            fsm = robot.read_fsm()
        if fsm in (7, 21, 22, 24):
            robot.command("GrpEnable,0,;")
        return bool(_wait_fsm(robot, {33}, 6, hold=True))
    except RobotError:
        return False


def _hybrid_recover(robot):
    global _hybrid_ft, _hybrid_force_read, _hybrid_lateral_read
    global _hybrid_moxa_prev, _hybrid_moxa_prev_t
    global _hybrid_massage_prev, _hybrid_massage_prev_t, _hybrid_massage_frozen
    _hybrid_leave_freedrive(robot)
    try:
        robot.force_control_off()
    except RobotError:
        pass
    _hybrid_set_collide(robot, 3)
    with _lock:
        _hybrid_ft = False
        _hybrid_force_read = None
        _hybrid_lateral_read = None
        _hybrid_moxa_prev = None
        _hybrid_moxa_prev_t = 0.0
        _hybrid_massage_prev = None
        _hybrid_massage_prev_t = 0.0
        _hybrid_massage_frozen = False
    if _hybrid_unlock_limit(robot):
        return True
    try:
        fsm = robot.read_fsm()
    except RobotError:
        return False
    if fsm == 33:
        return True
    try:
        if fsm == 25:
            try:
                robot.stop()
            except RobotError:
                pass
            fsm = robot.read_fsm()
        if fsm in (21, 22):
            robot.command("GrpReset,0,;")
            time.sleep(0.15)
            fsm = robot.read_fsm()
        if fsm in (7, 21, 22, 24):
            robot.command("GrpEnable,0,;")
        return bool(_wait_fsm(robot, {33}, 6, hold=True))
    except RobotError:
        return False


def set_speed(value):
    global _speed
    try:
        speed = int(value)
    except (TypeError, ValueError):
        speed = 1
    with _lock:
        _speed = max(1, min(5, speed))
        return _speed


def set_level(grade):
    wanted = _parse_grade(grade)
    with _lock:
        current = _parse_grade(_level_grade)
    if wanted == current:
        return {"level": current, "changed": False, "spoken": "已完成"}
    return _commit_level(wanted)


def nudge_level(delta):
    try:
        step = int(delta)
    except (TypeError, ValueError):
        step = 0
    with _lock:
        current = _parse_grade(_level_grade)
    wanted = current + step
    if wanted > 4:
        return {"level": current, "changed": False, "spoken": "已经是最高了"}
    if wanted < 0:
        return {"level": current, "changed": False, "spoken": "已经是最低了"}
    if wanted == current:
        return {"level": current, "changed": False, "spoken": "已完成"}
    return _commit_level(wanted)


def _commit_level(wanted):
    global _level_grade
    with _lock:
        _level_grade = wanted
        robot = _active_robot
        running = _running
    if running and robot is not None:
        try:
            _apply_level_now(robot)
        except Exception as exc:
            print("[heacon] level apply failed:", exc, flush=True)
    return {"level": wanted, "changed": True, "spoken": "已完成"}


def _apply_level_now(robot):
    global _hybrid_home
    if _hybrid_ft and not _plan_is_moxa():
        _hybrid_apply_goal(robot)
        return
    if _hybrid_ft and _plan_is_moxa():
        want = _show_lift()
        home = _hybrid_home
        try:
            state = robot.state()
        except RobotError:
            return
        if not state.get("connected"):
            return
        if home is None:
            pose = _lift_to_floor(_pose_at_lift(state["pose"], want))
        else:
            pose = _lift_to_floor(_pose_at_lift(home, want))
        print("[heacon] moxa standoff", round(vision_height(state["pose"]), 1), "->", round(want, 1), flush=True)
        _hybrid_contact_off(robot)
        _send_move(robot, pose, kind="vertical", urgent=True)
        try:
            _wait_settled(robot, pose, hold=True, seconds=12)
        except RobotError as exc:
            print("[heacon] moxa level settle:", exc, flush=True)
        _hybrid_home = dict(pose)
        _remember_joints(robot)
        _hybrid_contact_on(robot)
        return
    _hold_moves.set()
    try:
        for _attempt in range(3):
            try:
                robot.stop()
            except RobotError as exc:
                print("[heacon] level stop:", exc, flush=True)
            if _wait_fsm(robot, {24, 33}, 3, hold=True):
                break
        fsm = robot.read_fsm()
        if fsm == 24:
            try:
                robot.command("GrpEnable,0,;")
            except RobotError as exc:
                print("[heacon] level enable:", exc, flush=True)
            _wait_fsm(robot, {33}, 8, hold=True)
        time.sleep(0.2)
        state = robot.state()
        if not state.get("connected"):
            print("[heacon] level skipped: arm not connected", flush=True)
            return
        want = _show_lift()
        have = vision_height(state["pose"])
        print("[heacon] level height", round(have, 1), "->", round(want, 1), flush=True)
        if abs(have - want) < 0.8:
            _remember_joints(robot)
            return
        pose = _lift_to_floor(_pose_at_lift(state["pose"], want))
        _remember_joints(robot)
        _send_move(robot, pose, kind="vertical", urgent=True)
        try:
            _wait_settled(robot, pose, hold=True, seconds=12)
        except RobotError as exc:
            print("[heacon] level settle:", exc, flush=True)
        _remember_joints(robot)
    finally:
        _hold_moves.clear()


def focus(index):
    """The arm chooses the point. A page update must not pull it off the route."""
    with _lock:
        if _running or not _targets:
            return False
        _set_index(index)
        return True


def preview(plan_id, points=None):
    """Pixel positions of this plan on the current camera frame. The arm does not move."""
    import importlib
    import time


    body_route = importlib.reload(body_route)
    importlib.reload(vision)
    plan = _plan(plan_id)
    frame = None
    bodies = []
    for _ in range(5):
        try:
            frame = _camera_frame()
        except RobotError:
            time.sleep(0.05)
            continue
        found = body_route.find_body(frame)
        if found is not None:
            bodies.append(found)
        time.sleep(0.04)
    body = body_route.median_body(bodies)
    if body is None:
        raise RobotError("摄像头里没有看清人或假人")
    raw = _raw_points(plan, points)
    height, width = frame.shape[:2]
    pixels = body_route.place_points(raw, body, width, height)
    _set_image_size(width, height)
    return {
        "width": int(width),
        "height": int(height),
        "body": {
            "neck": [round(float(body["neck"][0]), 2), round(float(body["neck"][1]), 2)],
            "hip": [round(float(body["hip"][0]), 2), round(float(body["hip"][1]), 2)],
            "right": [round(float(body["right"][0]), 2), round(float(body["right"][1]), 2)],
            "width": round(float(body["width"]), 2),
            "neckY": body_route.NECK_Y,
            "hipY": body_route.HIP_Y,
            "leftX": body_route.LEFT_X,
            "rightX": body_route.RIGHT_X,
        },
        "points": [
            {"name": name, "u": round(float(u), 1), "v": round(float(v), 1)}
            for (name, _x, _y), (u, v) in zip(raw, pixels)
        ],
    }


_framing = False
_align_sign = 1.0
_align_joint = None
_align_last_off = None
_slide_sign_u = 1.0
_slide_sign_v = 1.0
_slide_last = None
_frame_hold = list(POSITION_JOINTS)


def hold_frame(state):
    """Keep this camera pose. The position page must not walk the arm off it."""
    global _frame_hold
    joints = state.get("joints") if isinstance(state, dict) else None
    with _lock:
        _frame_hold = list(joints) if joints and len(joints) == 6 else None
    return _frame_hold is not None


def remember_scan_pose(state):
    """Camera pose used to turn dragged pixels into arm points."""
    global _scan_origin, _scan_joints
    pose = state.get("pose") if isinstance(state, dict) else None
    joints = state.get("joints") if isinstance(state, dict) else None
    if not pose or not joints or len(joints) != 6:
        return False
    with _lock:
        _scan_origin = dict(pose)
        _scan_joints = list(joints)
    return True


def _near_position(state):
    pose = state.get("pose") if isinstance(state, dict) else None
    joints = state.get("joints") if isinstance(state, dict) else None
    if pose and _same_pose(pose, POSITION_POSE):
        return True
    if joints and len(joints) == 6:
        return all(abs(float(a) - float(b)) < 2.0 for a, b in zip(joints, POSITION_JOINTS))
    return False


def position_status():
    with _lock:
        return {
            "positioning": _positioning,
            "ready": _position_ready,
            "message": _position_message,
        }


def go_position(robot):
    """Return to the remembered 就位 pose and show the live downward camera."""
    global _positioning, _position_ready, _position_message, _position_thread
    if status()["running"]:
        return False, "服务进行中，先停止再就位"
    service = _thread
    if service is not None and service.is_alive() and threading.current_thread() is not service:
        return True, "服务正在收尾，随后回到就位位置"
    with _lock:
        if _positioning:
            return True, _position_message
        _positioning = True
        _position_ready = False
        _position_message = "机械臂正在回到就位位置"
        _stop.clear()
    _position_thread = threading.Thread(
        target=_position_loop, args=(robot,), name="go-position", daemon=True
    )
    _position_thread.start()
    return True, "机械臂正在回到就位位置"


def _position_loop(robot):
    global _positioning, _position_ready, _position_message, _joints
    try:
        try:
            robot.command("SetFTFreeDriveState,0,0,;")
        except RobotError:
            pass
        _prepare(robot)
        state = robot.state()
        if not state.get("connected"):
            raise RobotError(state.get("message") or "没有连上机械臂")
        remember_scan_pose(state)
        if _near_position(state):
            hold_frame(state)
            with _lock:
                _position_ready = True
                _position_message = "机械臂已在就位位置，彩色画面已打开"
            return
        _set_floor()
        _apply_override(robot, "home")
        robot.command("SetCollideLevel,0,3,;")
        with _lock:
            _joints = list(state["joints"])
        goal = dict(POSITION_POSE)
        settled = False
        for _try in range(8):
            if _stop.is_set():
                break
            _remember_joints(robot)
            _send_move(robot, goal, kind="home")
            if _wait_settled(robot, goal, hold=True):
                settled = True
                break
        if not settled:
            with _lock:
                _position_message = "就位时已停止"
            return
        if _stop.is_set():
            with _lock:
                _position_message = "就位时已停止"
            return
        arrived = robot.state()
        if not arrived.get("connected"):
            raise RobotError(arrived.get("message") or "就位后没有读到机械臂")
        if not _near_position(arrived) and not _same_pose(arrived["pose"], goal):
            raise RobotError("机械臂没有到达就位位置")
        remember_scan_pose(arrived)
        hold_frame(arrived)
        with _lock:
            _position_ready = True
            _position_message = "机械臂已就位，彩色画面已打开"
    except RobotError as exc:
        with _lock:
            _position_ready = False
            _position_message = str(exc)
    except Exception as exc:
        with _lock:
            _position_ready = False
            _position_message = f"就位时出了问题：{exc}"
    finally:
        with _lock:
            _positioning = False


def holding_frame(state=None):
    with _lock:
        held = list(_frame_hold) if _frame_hold else None
    if held is None:
        return False
    if state is None:
        return True
    joints = state.get("joints") if isinstance(state, dict) else None
    if not joints or len(joints) != 6:
        return True
    return all(abs(float(a) - float(b)) < 1.8 for a, b in zip(joints, held))


def frame_body(robot):
    """Move the camera until the head, shoulders, back, and waist fit.

    The camera looks straight down. It slides toward the head, and drops a
    little when that direction is blocked.
    """
    global _framing, _align_sign, _align_joint, _align_last_off
    global _slide_sign_u, _slide_sign_v, _slide_last
    if holding_frame(robot.state()):
        return True, "上半身已经拍全，中线在画面中间"
    if status()["running"]:
        return True, "服务进行中，先不调整画面"
    with _lock:
        if _framing:
            return False, "正在调整画面"
        _framing = True
        _align_sign = 1.0
        _align_joint = None
        _align_last_off = None
        _slide_sign_u = 1.0
        _slide_sign_v = 1.0
        _slide_last = None
    _stop.clear()
    try:
        return _frame_loop(robot)
    finally:
        with _lock:
            _framing = False


def _frame_loop(robot):

    _prepare(robot)
    rises = 0
    level_blocked = False
    for _step in range(24):
        if _stop.is_set():
            return True, "调整画面时已停止"
        state = robot.state()
        if not state.get("connected"):
            raise RobotError(state.get("message") or "没有连上机械臂")
        off_vertical = _off_vertical(state)
        if not level_blocked and off_vertical > 12.0:
            _set(False, "镜头正在转到垂直向下")
            leveled = _level_toward_vertical(state, min(8.0, off_vertical), shift_mm=16.0)
            if leveled is not None and _frame_move(robot, state["joints"], leveled):
                continue
            level_blocked = True
        frame = vision.camera_bgr()
        view = body_route.coverage(frame)
        if _view_is_placed(view):
            _set(False, "上半身已经拍全，中线在画面中间")
            return True, "上半身已经拍全，中线在画面中间"
        if view is None or view["length"] < 80:
            if rises < 2 and vision_height(state["pose"]) < 520:
                _set(False, "没看清上半身，正在把镜头抬高一点")
                rises += 1
                if _frame_move(robot, state["joints"], _rise_pose(state["pose"], 36.0)):
                    continue
            _set(False, "画面里没有看清上半身")
            return True, "画面里没有看清上半身"
        if view.get("too_close") and rises < 2 and vision_height(state["pose"]) < 520:
            _set(False, "头和腰都贴着边缘，正在把镜头抬高一点")
            rises += 1
            if _frame_move(robot, state["joints"], _rise_pose(state["pose"], 36.0)):
                continue
        off = float(view.get("off_horizontal") or 90.0)
        head_ok = bool(view.get("head_visible"))
        head_edge = float(view.get("head_edge") or 0.0)
        head = view.get("head") or (0.0, 0.0)
        head_high = float(head[1]) < float(view.get("height") or 720) * 0.42
        if (not head_ok) or (off > 8.0 and (head_edge < 110.0 or head_high)):
            _set(False, "镜头正在把头留在画面里")
            if _nudge_margin(robot, view):
                continue
            if (not head_ok) and _place_step(robot, state, view):
                continue
            if head_ok and off > 8.0 and _align_step(robot, state, view, off):
                continue
        elif off > 8.0:
            _set(False, "镜头正在转到和上半身平行")
            if _align_step(robot, state, view, off):
                continue
        _set(False, "镜头正在把人移到画面中间")
        if _place_step(robot, state, view):
            continue
        _set(False, "这个方向再挪会超出范围，上半身还没摆正")
        return True, "这个方向再挪会超出范围，上半身还没摆正"
    _set(False, "还没把上半身摆进画面中间")
    return True, "还没把上半身摆进画面中间"


def _near_overview(pose):
    return (
        math.hypot(pose["x"] - OVERVIEW_POSE["x"], pose["y"] - OVERVIEW_POSE["y"]) < 45
        and abs(pose["z"] - OVERVIEW_POSE["z"]) < 45
    )


def _overview_goal(pose):
    goal = dict(pose)
    goal.update(POSITION_POSE)
    return goal


def _align_step(robot, state, view, off):
    """Roll the camera mount until the head-to-waist line is horizontal.

    The camera sits on the link before joint 6, so joint 4 turns the picture
    and joint 5 nods it. Twisting the flange (joint 6) does not move the
    camera. The live picture is never warped.
    """
    global _align_sign, _align_last_off

    if _nudge_roll(robot, off):
        return True
    rolled = _roll_to_line(state, view, _align_sign)
    if rolled is not None and _frame_move(robot, state["joints"], rolled):
        again = body_route.coverage(vision.camera_bgr())
        new_off = float((again or {}).get("off_horizontal") or 90.0)
        if _align_last_off is not None and new_off > _align_last_off + 1.5:
            _align_sign = -_align_sign
        _align_last_off = new_off
        if again and not again.get("head_visible"):
            _place_step(robot, robot.state(), again)
        return True
    return False


def _roll_to_line(state, view, sign=1.0):
    off = float(view.get("off_horizontal") or 0.0)
    if off < 3.0:
        return None
    return _roll_camera(state, float(sign) * min(6.0, off))


def _roll_camera(state, degrees):
    """Keep the camera origin. Roll it about its look so the raw picture rotates."""
    import numpy as np

    pose = state["pose"]
    model = vision.load_handeye()
    camera = vision.link_pose(pose, state["joints"][5]) @ model["camera"]
    rotation = camera[:3, :3].astype(float)
    origin = camera[:3, 3].astype(float)
    look = rotation[:, 2]
    look = look / np.linalg.norm(look)
    turned = np.eye(4)
    turned[:3, :3] = _rotate_about(look, float(degrees)) @ rotation
    turned[:3, 3] = origin
    link = turned @ np.linalg.inv(model["camera"])
    wrist = np.eye(4)
    wrist[:3, :3] = vision._rotation(0.0, 0.0, np.deg2rad(float(state["joints"][5])))
    flange = link @ wrist
    angles = _euler_near(flange[:3, :3], pose)
    goal = dict(pose)
    goal["x"] = float(flange[0, 3])
    goal["y"] = float(flange[1, 3])
    goal["z"] = float(flange[2, 3])
    for key, raw in zip(("rx", "ry", "rz"), angles):
        delta = _angle_delta(raw, pose[key])
        if abs(delta) > 16.0:
            return None
        value = float(pose[key]) + delta
        if value <= -179.0:
            value = -179.0
        if value >= 179.0:
            value = 179.0
        if pose[key] < -160.0 and value > 160.0:
            return None
        if pose[key] > 160.0 and value < -160.0:
            return None
        goal[key] = value
    travel = math.hypot(goal["x"] - pose["x"], goal["y"] - pose["y"])
    if travel > 80.0 or abs(goal["z"] - pose["z"]) > 40.0:
        return None
    if travel < 5.0:
        _origin, axis_u, _axis_v, _look = vision.camera_basis(state["joints"], pose)
        sideways = axis_u / max(float(np.linalg.norm(axis_u)), 1e-6) * 8.0
        goal["x"] = float(goal["x"] + sideways[0])
        goal["y"] = float(goal["y"] + sideways[1])
        goal["z"] = float(goal["z"] + sideways[2])
    return goal


def _nudge_roll(robot, off):
    """Turn joint 4 (camera-mount roll) 2°. Joint 6 is not used."""
    global _align_joint

    if _align_joint is not None:
        if not _jog_axis(robot, _align_joint[0], _align_joint[1]):
            _align_joint = None
            return False
        time.sleep(0.25)
        again = body_route.coverage(vision.camera_bgr())
        new_off = float((again or {}).get("off_horizontal") or 90.0)
        if (again is None) or (not again.get("head_visible")) or new_off > off + 1.0:
            _jog_axis(robot, _align_joint[0], 1 - _align_joint[1])
            _align_joint = None
            return False
        return True
    # Joint 4 rolls the bracket the camera is bolted to. Joint 5 only nods.
    for axis, direction in ((3, 0), (3, 1), (4, 1), (4, 0)):
        if not _jog_axis(robot, axis, direction):
            continue
        time.sleep(0.25)
        again = body_route.coverage(vision.camera_bgr())
        new_off = float((again or {}).get("off_horizontal") or 90.0)
        if again and again.get("head_visible") and new_off < off - 0.4:
            _align_joint = (axis, direction)
            return True
        _jog_axis(robot, axis, 1 - direction)
    return False


def _nudge_margin(robot, view):
    """Nod joint 5 so the head sits lower in the page. Joint 6 is not used."""

    before = float((view or {}).get("head_edge") or 0.0)
    for direction in (1, 0):
        if not _jog_axis(robot, 4, direction):
            continue
        time.sleep(0.25)
        again = body_route.coverage(vision.camera_bgr())
        edge = float((again or {}).get("head_edge") or 0.0)
        if again and again.get("head_visible") and edge > before + 8.0:
            return True
        _jog_axis(robot, 4, 1 - direction)
    return False


def _jog_axis(robot, axis, direction):
    try:
        robot.command(f"MoveRelJ,0,{int(axis)},{int(direction)},2,;", timeout=8)
    except RobotError as exc:
        text = str(exc)
        if "20031" in text or "20018" in text or "Point calculation failed" in text:
            return False
        raise
    deadline = time.time() + 8
    while time.time() < deadline:
        if _stop.is_set():
            return False
        fsm = robot.read_fsm()
        if fsm == 33:
            return True
        if fsm in (21, 22, 24):
            return False
        time.sleep(0.1)
    return True


def _view_is_placed(view):
    """Head in, line level, line on the page center. All three must hold."""
    return bool(view and view.get("placed") and view.get("head_visible"))


def _place_step(robot, state, view):
    """Slide the real camera so the body line sits on the page center."""
    global _slide_sign_u, _slide_sign_v, _slide_last

    error_u = float(view.get("error_u") or 0.0) * _slide_sign_u
    error_v = float(view.get("error_v") or 0.0) * _slide_sign_v
    goal = _slide_by_pixels(state, view, error_u, error_v)
    if goal is None or not _frame_move(robot, state["joints"], goal):
        return False
    again = body_route.coverage(vision.camera_bgr())
    new_u = abs(float((again or {}).get("error_u") or 999.0))
    new_v = abs(float((again or {}).get("error_v") or 999.0))
    if _slide_last is not None:
        if new_u > _slide_last[0] + 24:
            _slide_sign_u = -_slide_sign_u
        if new_v > _slide_last[1] + 24:
            _slide_sign_v = -_slide_sign_v
    _slide_last = (new_u, new_v)
    return True


def _slide_by_pixels(state, view, du, dv):
    """Move the camera in its image axes. The picture itself is not warped."""
    import numpy as np

    pose = state["pose"]
    _origin, axis_u, axis_v, _look = vision.camera_basis(state["joints"], pose)
    fx = vision.focal_length()
    length = float(view.get("length") or 0.0)
    distance = fx * ASSUMED_BACK_LENGTH_MM / max(length, 1.0) if length > 80 else 470.0
    distance = min(900.0, max(280.0, distance))
    shift_u = max(-40.0, min(40.0, du / fx * distance))
    shift_v = max(-40.0, min(40.0, dv / fx * distance))
    if math.hypot(shift_u, shift_v) < 10.0:
        return None
    delta = axis_u * shift_u + axis_v * shift_v
    up = vision.up_vector()
    downward = float(np.dot(delta, up))
    if downward < 0:
        delta = delta - up * downward
    if float(np.linalg.norm(delta)) < 10.0:
        return None
    goal = dict(pose)
    goal["x"] = float(pose["x"] + delta[0])
    goal["y"] = float(pose["y"] + delta[1])
    goal["z"] = float(pose["z"] + delta[2])
    if vision_height(goal) < vision_height(pose) - 2:
        return None
    if vision_height(goal) > 720:
        return None
    return goal


def _center_line_pose(state, view):
    """Slide so the already-level body sits inside the page, not on the edge."""
    width = float(view.get("width") or 1280)
    height = float(view.get("height") or 720)
    center = view.get("center") or (width / 2.0, height / 2.0)
    head_edge = float(view.get("head_edge") or 0.0)
    if (
        abs(center[0] - width / 2.0) < 100
        and abs(center[1] - height / 2.0) < 56
        and head_edge >= 70
    ):
        return None
    return _framing_pose(state, view)


def _framing_pose(state, view):
    import numpy as np

    pose = state["pose"]
    _origin, axis_u, axis_v, look = vision.camera_basis(state["joints"], pose)
    fx = vision.focal_length()
    length = view["length"] if view["length"] > 80 else 0.0
    aim = view.get("aim") or view["center"]
    if length == 0.0:
        retreat = 110.0
        shift_u = 0.0
        shift_v = 0.0
        distance = 400.0
    else:
        distance = fx * ASSUMED_BACK_LENGTH_MM / max(length, 1.0)
        distance = min(900.0, max(280.0, distance))
        center_u, center_v = vision.raw_pixel(view["width"] / 2.0, view["height"] / 2.0, view["width"], view["height"])
        aim_u, aim_v = vision.raw_pixel(float(aim[0]), float(aim[1]), view["width"], view["height"])
        shift_u = (aim_u - center_u) / fx * distance
        shift_v = (aim_v - center_v) / fx * distance
        shift_u = min(48.0, max(-48.0, shift_u))
        shift_v = min(36.0, max(-36.0, shift_v))
        # Slide toward the head first. Backing up is only for a torso that is already
        # in frame and still too large; from the overview, a retreat often cannot go farther.
        retreat = 0.0
    delta = -look * retreat + axis_u * shift_u + axis_v * shift_v
    up = vision.up_vector()
    downward = float(np.dot(delta, up))
    if downward < 0:
        delta = delta - up * downward
    if float(np.linalg.norm(delta)) < 12:
        return None
    goal = dict(pose)
    goal["x"] = float(pose["x"] + delta[0])
    goal["y"] = float(pose["y"] + delta[1])
    goal["z"] = float(pose["z"] + delta[2])
    if vision_height(goal) < vision_height(pose) - 2:
        return None
    if vision_height(goal) > 720:
        return None
    return goal


def _scaled_pose(pose, goal, scale):
    if goal is None:
        return None
    shorter = dict(pose)
    for key in ("x", "y", "z"):
        shorter[key] = float(pose[key] + scale * (goal[key] - pose[key]))
    return shorter


def _rise_pose(pose, millimeters):

    up = vision.up_vector()
    risen = dict(pose)
    risen["x"] = float(pose["x"] + up[0] * millimeters)
    risen["y"] = float(pose["y"] + up[1] * millimeters)
    risen["z"] = float(pose["z"] + up[2] * millimeters)
    return risen


def _drop_pose(pose, millimeters):

    up = vision.up_vector()
    dropped = dict(pose)
    dropped["x"] = float(pose["x"] - up[0] * millimeters)
    dropped["y"] = float(pose["y"] - up[1] * millimeters)
    dropped["z"] = float(pose["z"] - up[2] * millimeters)
    return dropped


def _off_vertical(state):
    import numpy as np

    _origin, _axis_u, _axis_v, look = vision.camera_basis(state["joints"], state["pose"])
    down = -vision.up_vector()
    down = down / np.linalg.norm(down)
    return float(np.degrees(np.arccos(np.clip(float(np.dot(look, down)), -1.0, 1.0))))


def _level_toward_vertical(state, degrees, shift_mm=0.0):
    """Turn the camera toward straight down.

    A pure turn has no path length, so the wrist snaps and a joint faults.
    The small sideways shift gives the turn a short, slow path.
    """
    import numpy as np

    pose = state["pose"]
    model = vision.load_handeye()
    camera = vision.link_pose(pose, state["joints"][5]) @ model["camera"]
    rotation = camera[:3, :3].astype(float)
    look = rotation[:, 2]
    look = look / np.linalg.norm(look)
    up = vision.up_vector()
    down = -up / np.linalg.norm(up)
    axis = np.cross(look, down)
    if float(np.linalg.norm(axis)) < 1e-4:
        return None
    axis = axis / np.linalg.norm(axis)
    turned = _rotate_about(axis, float(degrees)) @ rotation
    extrinsic = model["camera"][:3, :3]
    link = turned @ extrinsic.T
    wrist = vision._rotation(0.0, 0.0, np.deg2rad(float(state["joints"][5])))
    angles = _euler_near(link @ wrist, pose)
    goal = dict(pose)
    _origin, axis_u, _axis_v, _look = vision.camera_basis(state["joints"], pose)
    sideways = axis_u - up * float(np.dot(axis_u, up))
    norm = float(np.linalg.norm(sideways))
    if norm > 1e-4 and abs(shift_mm) > 1:
        sideways = sideways / norm * float(shift_mm)
        goal["x"] = float(pose["x"] + sideways[0])
        goal["y"] = float(pose["y"] + sideways[1])
        goal["z"] = float(pose["z"] + sideways[2])
    goal["rx"], goal["ry"], goal["rz"] = angles
    for key in ("rx", "ry", "rz"):
        delta = _angle_delta(goal[key], pose[key])
        if abs(delta) > 12:
            return None
        value = float(pose[key]) + delta
        if value <= -179.0:
            value = -179.0
        if value >= 179.0:
            value = 179.0
        goal[key] = value
    return goal


def _tilt_toward(state, view, home_angles):
    """Turn the camera a few degrees toward the head. Position stays put."""
    import numpy as np

    pose = state["pose"]
    aim = view.get("aim") or view["center"]
    aim_u, aim_v = vision.raw_pixel(float(aim[0]), float(aim[1]), view["width"], view["height"])
    center_u, center_v = vision.raw_pixel(view["width"] / 2.0, view["height"] / 2.0, view["width"], view["height"])
    du = aim_u - center_u
    dv = aim_v - center_v
    if abs(du) < 30 and abs(dv) < 30:
        return None
    _origin, axis_u, axis_v, look = vision.camera_basis(state["joints"], pose)
    toward = axis_u * du + axis_v * dv
    toward = toward / max(float(np.linalg.norm(toward)), 1e-6)
    axis = np.cross(look, toward)
    if float(np.linalg.norm(axis)) < 1e-4:
        return None
    axis = axis / np.linalg.norm(axis)
    rotation = vision._rotation(np.deg2rad(pose["rx"]), np.deg2rad(pose["ry"]), np.deg2rad(pose["rz"]))
    turned = _rotate_about(axis, 8.0) @ rotation
    angles = _euler_near(turned, pose)
    goal = dict(pose)
    goal["rx"], goal["ry"], goal["rz"] = angles
    for key in ("rx", "ry", "rz"):
        delta = _angle_delta(goal[key], pose[key])
        if abs(delta) > 12 or abs(_angle_delta(pose[key] + delta, home_angles[key])) > 26:
            return None
        value = float(pose[key]) + max(-8.0, min(8.0, delta))
        # Stay inside the controller's Euler range so the wrist does not wrap.
        if value <= -179.0:
            value = -179.0
        if value >= 179.0:
            value = 179.0
        goal[key] = value
    return goal


def _rotate_about(axis, degrees):
    import numpy as np

    x, y, z = (float(number) for number in axis)
    cosine = float(np.cos(np.deg2rad(degrees)))
    sine = float(np.sin(np.deg2rad(degrees)))
    complement = 1.0 - cosine
    return np.array(
        [
            [cosine + x * x * complement, x * y * complement - z * sine, x * z * complement + y * sine],
            [y * x * complement + z * sine, cosine + y * y * complement, y * z * complement - x * sine],
            [z * x * complement - y * sine, z * y * complement + x * sine, cosine + z * z * complement],
        ]
    )


def _euler_near(rotation, reference):
    import numpy as np

    horizontal = float(np.hypot(rotation[2, 1], rotation[2, 2]))
    ry = float(np.degrees(np.arctan2(-rotation[2, 0], horizontal)))
    rx = float(np.degrees(np.arctan2(rotation[2, 1], rotation[2, 2])))
    rz = float(np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0])))
    values = []
    for value, origin in ((rx, reference["rx"]), (ry, reference["ry"]), (rz, reference["rz"])):
        while value - origin > 180:
            value -= 360
        while origin - value > 180:
            value += 360
        values.append(value)
    return values


def _frame_move(robot, joints, goal):
    global _cmd_id
    with _lock:
        _cmd_id += 1
        cmd_id = _cmd_id
    command = waypoint_command(goal, joints, cmd_id)
    try:
        robot.command(command, timeout=12)
    except RobotError as exc:
        text = str(exc)
        if "20031" in text or "Point calculation failed" in text:
            return False
        if _stop.is_set() or "20018" not in text:
            raise
        if not _wait_fsm(robot, {33}, 8) or _stop.is_set():
            raise RobotError("调整画面时被停下了") from exc
        try:
            robot.command(command, timeout=12)
        except RobotError as again:
            if "20031" in str(again) or "Point calculation failed" in str(again):
                return False
            raise
    if not _wait_settled(robot, goal):
        raise RobotError("调整画面时被停下了")
    return True


def start(robot, plan_id, points=None, fresh=True, segments=None, level=2, duration=None, speed=1, width=None, height=None):
    global _thread, _starting, _running, _message, _plan_id, _origin, _joints, _targets, _index, _segments, _spent
    global _level_grade, _gesture, _emphasis, _round, _visited, _speed, _service_seconds, _phase, _direction
    global _active_robot, _step_until
    with _lock:
        if _running and _thread is not None and _thread.is_alive():
            return True, _message
        if _thread is not None and _thread.is_alive():
            return False, "机械臂正在停下"
        if _starting:
            return False, "机械臂正在准备动作"
        _starting = True
        _stop.clear()
    try:
        _set_image_size(width, height)
        plan = _plan(plan_id)
        same_route = (
            not fresh
            and _origin is not None
            and _joints is not None
            and _plan_id == plan_id
            and _targets
        )
        _active_robot = robot
        _hybrid_reset()
        _prepare(robot)
        _hybrid_warm(robot)
        _apply_override(robot, "travel")
        state = robot.state()
        _guard_state(state)
        if same_route:
            _guard_still_here(state)
        else:
            remember_scan_pose(state)
            origin = dict(state["pose"])
            joints = list(state["joints"])
            frame = None if _has_camera_pixels(plan, points) else _camera_frame()
            targets = build_targets(plan_id, origin, joints, frame, points)
            parsed = _parse_segments(segments, len(targets))
            with _lock:
                _plan_id = plan_id
                _origin = origin
                _joints = joints
                _targets = targets
                _index = 0
                _segments = parsed
                _spent = 0.0
                _mark = None
                _round = 1
                _visited = []
                _gesture = ""
                _emphasis = False
                _phase = "走线"
                _step_until = 0.0
                _direction = 1
                try:
                    _speed = max(1, min(5, int(speed or 1)))
                except (TypeError, ValueError):
                    _speed = 1
                if duration:
                    _service_seconds = max(60.0, float(duration))
                elif parsed:
                    _service_seconds = sum(parsed)
                else:
                    _service_seconds = 35 * 60
    except RobotError as exc:
        with _lock:
            _starting = False
        _hybrid_reset()
        _set(False, str(exc))
        return False, str(exc)
    except Exception:
        with _lock:
            _starting = False
        _hybrid_reset()
        _set(False, "准备动作时出了问题，机械臂没有动")
        return False, "准备动作时出了问题，机械臂没有动"
    with _lock:
        _starting = False
        _running = True
        _level_grade = _parse_grade(level)
        point_name = _targets[_index]["name"]
        _gesture = "靠近"
        _emphasis = _emphasis_name(_plan_id, point_name)
        _message = _progress_text(point_name, "靠近", _plan_id, _emphasis, _round)
        message = _message
        _thread = threading.Thread(target=_loop, args=(robot,), name="route-motion", daemon=True)
        _thread.start()
    return True, message


def stop(robot):
    _stop.set()
    _pause_clock()
    _set(False, "机械臂已停在原地")
    try:
        _hybrid_leave_freedrive(robot)
    except Exception:
        pass
    try:
        robot.force_control_off()
    except Exception:
        pass
    try:
        fsm = robot.read_fsm()
    except RobotError:
        fsm = None
    try:
        if fsm == 20:
            _hybrid_unlock_limit(robot)
        elif fsm in (25, 27):
            robot.command("GrpStop,0,;", timeout=1.0)
    except RobotError:
        try:
            _hybrid_unlock_limit(robot)
        except Exception:
            pass
    _hybrid_reset()
    return True, "机械臂已停在原地"


def start_lead_preview(robot):
    return False, "跟手已经取消。请在平板上确认路线并开始"


def route_height(surface_lift, arm_lift, plan_id=None):
    """Approved 40 mm skin standoff. Camera height is only for 就位, not work."""
    kind = "moxa" if (plan_id or "") in ("p1", "p2") else "massage"
    return work_height(kind)


def work_height(kind="massage"):
    """Flange height above the table for a treatment kind, from the 40 mm pose."""
    if kind == "moxa":
        return float(WORK_LIFT_MM + (MOXA_STANDOFF_MM - SKIN_STANDOFF_MM))
    return float(WORK_LIFT_MM)


def _grade_lift():
    with _lock:
        grade = _parse_grade(_level_grade)
    return (2 - grade) * GRADE_STEP_MM


def _show_lift():
    with _lock:
        plan_id = _plan_id
        extra = _hybrid_z
    kind = "moxa" if plan_id in ("p1", "p2") else "massage"
    return work_height(kind) + _grade_lift() + extra


def _bake_show():
    if _technique_show is None:
        return _show_lift()
    return _technique_show


def _pause_for_level():
    while _hold_moves.is_set() and not _stop.is_set() and not _time_is_up():
        time.sleep(0.05)
    return not _stop.is_set() and not _time_is_up()


def _set_image_size(width, height=None):
    global _image_size
    if height is None and hasattr(width, "shape"):
        height, width = width.shape[:2]
    try:
        width = int(width)
        height = int(height)
    except (TypeError, ValueError):
        return _image_size
    if width >= 8 and height >= 8:
        _image_size = (width, height)
    return _image_size


def _set_floor(surface_lift=None, pose=None):
    global _route_floor
    with _lock:
        _route_floor = WORK_FLOOR_MM


def _floor():
    if _hybrid_ft and _plan_is_moxa():
        band = HYBRID_MOXA_FLOOR_MM
    elif _hybrid_ft:
        band = HYBRID_FORCE_FLOOR_MM
    else:
        band = 22.0
    return _show_lift() - band


def build_targets(plan_id, origin, joints, image, points=None):
    """Poses for one plan, from the body in this camera frame."""
    import importlib

    importlib.reload(vision)

    plan = _plan(plan_id)
    placed = _direct_pixels(plan, points)
    if placed and _is_plan_layout(plan, placed):
        placed = None
    surface = None
    if placed is None:
        if image is None:
            raise RobotError("摄像头里没有看清人或假人，机械臂先不动")
        body = body_route.find_body(image)
        if body is None:
            raise RobotError("摄像头里没有看清人或假人，机械臂先不动")
        raw = _raw_points(plan, points)
        height, width = image.shape[:2]
        pixels = body_route.place_points(raw, body, width, height)
        placed = [(name, u, v) for (name, _x, _y), (u, v) in zip(raw, pixels)]
        surface = _surface_lift(joints, origin, body)
        _set_image_size(width, height)
    arm_lift = vision_height(origin)
    lift = route_height(surface, arm_lift, plan_id)
    _set_floor(surface, origin)
    targets = []
    pixel_w, pixel_h = _image_size
    for name, u, v in placed:
        try:
            point = vision.project(joints, origin, u, v, lift, pixel_w, pixel_h)
        except ValueError as exc:
            raise RobotError(str(exc)) from exc
        pose = {
            "name": name,
            "x": point["x"],
            "y": point["y"],
            "z": point["z"],
            "rx": origin["rx"],
            "ry": origin["ry"],
            "rz": origin["rz"],
        }
        _reject_if_unsafe(origin, pose, name)
        targets.append(pose)
    return targets


def _surface_lift(joints, pose, body):
    """Height of the mannequin surface, from how large the back appears."""

    length = float(body["length"])
    if length < 80:
        return None
    distance = float(vision.focal_length()) * ASSUMED_BACK_LENGTH_MM / length
    origin, direction = vision.ray(joints, pose, body["center"][0], body["center"][1])
    up = vision.up_vector()
    if float(direction @ up) > -0.25:
        return None
    point = origin + direction * distance
    lift = vision.height_above_table(point)
    if lift < 40 or lift > 450:
        return None
    return lift


def waypoint_command(pose, joints, cmd_id, speed=SPEED_MM_S, acc=ACC_MM_S2):
    """MoveL via WayPoint. Matches HRIF_MoveL in the Python SDK."""
    if len(joints) != 6:
        raise RobotError("关节参考无效")
    numbers = [
        pose["x"], pose["y"], pose["z"], pose["rx"], pose["ry"], pose["rz"],
        *joints,
    ]
    fields = ["WayPoint", "0"]
    fields.extend(f"{float(number):.3f}" for number in numbers)
    fields.extend(["TCP", "Base", str(int(speed)), str(int(acc)), "0", "1", "0", "0", "0", "0", str(int(cmd_id))])
    return ",".join(fields) + ",;"


def _loop(robot):
    try:
        _resume_clock()
        _show_run(robot)
        if _stop.is_set():
            _set(False, "服务已结束，机械臂正在回到就位位置")
        elif _time_is_up():
            robot.stop()
            _set(False, "服务时间已到，机械臂正在回到就位位置")
        else:
            robot.stop()
            _set(False, "这次路线已经走完，机械臂正在回到就位位置")
    except RobotError as exc:
        _set(False, str(exc))
        try:
            robot.stop()
        except RobotError:
            pass
    except Exception as exc:
        print("[heacon] service:", exc, flush=True)
        _set(False, "服务中断，机械臂正在回到就位位置")
        try:
            robot.stop()
        except RobotError:
            pass
    finally:
        _pause_clock()
        _hybrid_shutdown(robot)
        _hybrid_reset()
        _home_after_service(robot)


def _show_run(robot):
    with _lock:
        count = len(_targets)
        direction = _direction
    if count == 0:
        return
    index = _current_index()
    while not _stop.is_set() and not _time_is_up():
        _set_index(index)
        _set_direction(direction)
        _clear_step()
        _approach(robot, index)
        if _stop.is_set() or _time_is_up():
            break
        gesture = _point_technique(index, direction)
        _begin_step(HOLD_SECONDS)
        _do_named(robot, index, gesture)
        if _stop.is_set():
            break
        _note_visited(index)
        if _time_is_up():
            break
        _leave_point(robot, index)
        if _stop.is_set() or _time_is_up():
            break
        index, direction, wrapped = _advance(index, direction, count)
        if wrapped:
            _next_round()


def _set_direction(direction):
    global _direction
    with _lock:
        _direction = 1 if direction >= 0 else -1


def _point_technique(index, direction):
    with _lock:
        plan_id = _plan_id
        name = _targets[index]["name"] if _targets and 0 <= index < len(_targets) else ""
    if plan_id in ("p1", "p2"):
        return "柔顺"
    table = POINT_TECH.get(plan_id, {})
    side = table.get(1 if direction >= 0 else -1, {})
    return side.get(name) or table.get(1, {}).get(name) or TRAVEL_TECH.get(plan_id, "循经灸")


def _do_named(robot, index, gesture):
    global _hybrid_massage_air
    if _plan_is_moxa():
        gesture = "柔顺"
    try:
        _hybrid_contact_on(robot)
        if _plan_is_massage():
            _hybrid_massage_air = False
        _do_named_body(robot, index, gesture)
    finally:
        if _plan_is_moxa():
            try:
                _hybrid_ft_idle(robot)
            except Exception as exc:
                print("[heacon] ft idle:", exc, flush=True)


def _do_named_body(robot, index, gesture):
    actions = {
        "柔顺": _do_hover,
        "悬停灸": _do_hover,
        "回旋灸": _do_circle,
        "雀啄灸": _do_peck,
        "往复灸": _do_scan,
        "点按": _do_tap,
        "揉": _do_knead,
        "推": _do_push,
        "拨": _do_pluck,
        "滚压": _do_roll,
        "揉捏": _do_pinch,
        "冲击": _do_shock,
    }
    action = actions.get(gesture, _do_hover)
    global _technique_show
    while not _halted(index):
        if not _pause_for_level():
            break
        _technique_show = _show_lift()
        if not action(robot, index):
            break


def _begin_step(seconds):
    global _step_until, _phase
    with _lock:
        pace = max(1, int(_speed))
        _phase = "手法"
    _step_until = time.monotonic() + max(0.4, float(seconds) / pace)


def _clear_step():
    global _step_until
    _step_until = 0.0


def _step_over():
    return _step_until > 0 and time.monotonic() >= _step_until


def _approach(robot, index):
    global _phase, _hybrid_massage_air, _hybrid_xd
    target = _target(index)
    state = robot.state()
    _guard_state(state)
    with _lock:
        travel = TRAVEL_TECH.get(_plan_id, "循经灸")
        _phase = "走线"
    _clear_step()
    held = _hold_angles(target)
    work = _pose_at_lift(held, _show_lift())
    hover = _pose_at_lift(held, _hover_lift())
    if _plan_is_massage() and _hybrid_ft:
        if not _hybrid_massage_follow(
            robot, hover, target["name"], travel, kind="travel", air=True
        ):
            return
        if _stop.is_set() or _time_is_up():
            return
        if not _hybrid_massage_follow(
            robot, work, target["name"], travel, kind="vertical", air=True
        ):
            return
        _hybrid_massage_air = False
        _hybrid_xd = _clone_pose(_lock_down(work))
        _hybrid_massage_spring(robot)
        return
    if vision_height(state["pose"]) > _hover_lift() + HIGH_ABOVE_HOVER_MM:
        _hybrid_contact_off(robot)
        _move_pose(robot, work, target["name"], travel, gesture=False, kind="vertical")
        if not _stop.is_set() and not _time_is_up():
            _hybrid_contact_on(robot)
        return
    _hybrid_contact_off(robot)
    _move_pose(robot, hover, target["name"], travel, gesture=False, kind="travel")
    if _stop.is_set() or _time_is_up():
        return
    _move_pose(robot, work, target["name"], travel, gesture=False, kind="vertical")
    if _stop.is_set() or _time_is_up():
        return
    _hybrid_contact_on(robot)


def _leave_point(robot, index):
    """Lift off the point at the fixed 3x-of-1x speed before the next travel."""
    if _stop.is_set() or _time_is_up():
        return False
    global _phase
    with _lock:
        travel = TRAVEL_TECH.get(_plan_id, "循经灸")
        _phase = "走线"
    _clear_step()
    target = _target(index)
    hover = _pose_at_lift(_hold_angles(target), _hover_lift())
    if _plan_is_massage():
        return _hybrid_massage_follow(
            robot, hover, target["name"], travel, kind="vertical", air=True
        )
    _hybrid_contact_off(robot)
    _move_pose(robot, hover, target["name"], travel, gesture=False, kind="vertical")
    return not _stop.is_set()


def _do_tap(robot, index, brief=False):
    global _phase
    with _lock:
        _phase = "手法"
    reps, down_s, up_s = _tempo()
    if brief:
        reps, down_s, up_s = 1, min(down_s, 0.45), min(up_s, 0.45)
    base = _target(index)
    show = _bake_show()
    high = _gesture_pose(base, lift=show)
    low = _gesture_pose(base, ry=TILT_DEG, lift=show - DIP_MM)
    for _rep in range(reps):
        if not _go(robot, index, high, "点按"):
            return False
        if not _go(robot, index, low, "点按"):
            return False
        if not _rest(down_s, index):
            return False
        if not _go(robot, index, high, "点按"):
            return False
        if not _rest(up_s, index):
            return False
    return True


def _do_knead(robot, index):
    return _do_orbit(robot, index, "揉")


def _do_circle(robot, index):
    return _do_orbit(robot, index, "回旋灸")


def _do_orbit(robot, index, gesture):
    global _phase
    with _lock:
        _phase = "手法"
    reps, _down_s, _up_s = _tempo()
    base = _target(index)
    show = _bake_show()
    _up, along, side = _axes(index)
    for _rep in range(reps):
        for step in range(8):
            angle = math.tau * step / 8.0
            vector = (math.cos(angle) * along + math.sin(angle) * side) * CIRCLE_MM
            twist = TWIST_DEG if step % 2 == 0 else -TWIST_DEG
            pose = _gesture_pose(_offset(base, vector), ry=TILT_DEG, rz=twist, lift=show)
            if not _go(robot, index, pose, gesture):
                return False
    return True


def _do_push(robot, index):
    return _do_sweep(robot, index, "推")


def _do_scan(robot, index):
    return _do_sweep(robot, index, "往复灸")


def _do_sweep(robot, index, gesture):
    global _phase
    with _lock:
        _phase = "手法"
    reps, _down_s, _up_s = _tempo()
    base = _target(index)
    show = _bake_show()
    _up, along, _side = _axes(index)
    center = _gesture_pose(base, lift=show)
    forth = _gesture_pose(_offset(base, along * SWEEP_MM), ry=TILT_DEG, rz=TWIST_DEG, lift=show)
    back = _gesture_pose(_offset(base, -along * SWEEP_MM), ry=-TILT_DEG, rz=-TWIST_DEG, lift=show)
    for _rep in range(reps):
        for pose in (forth, center, back, center):
            if not _go(robot, index, pose, gesture):
                return False
    return True


def _do_peck(robot, index):
    global _phase
    with _lock:
        _phase = "手法"
    reps, down_s, up_s = _tempo()
    base = _target(index)
    show = _bake_show()
    low = _gesture_pose(base, ry=TILT_DEG, rz=TWIST_DEG, lift=show - DIP_MM)
    high = _gesture_pose(base, rz=-TWIST_DEG, lift=show)
    for _rep in range(reps):
        if not _go(robot, index, low, "雀啄"):
            return False
        if not _rest(down_s, index):
            return False
        if not _go(robot, index, high, "雀啄"):
            return False
        if not _rest(up_s, index):
            return False
    return True


def _do_hover(robot, index):
    global _phase, _hybrid_home
    with _lock:
        _phase = "手法"
    base = _target(index)
    show = _bake_show()
    seconds = max(1.2, (_step_until - time.monotonic()) if _step_until else 2.0)
    pose = _gesture_pose(base, lift=show)
    name = "柔顺" if _plan_is_moxa() else "悬停灸"
    if _hybrid_ft and _plan_is_moxa():
        _hybrid_home = dict(_lift_to_floor(pose))
        _mark_progress(_target(index)["name"], name)
        return _rest(seconds, index, wall=True)
    if not _go(robot, index, pose, name):
        return False
    return _rest(seconds, index, wall=True)


def _do_pluck(robot, index):
    global _phase
    with _lock:
        _phase = "手法"
    base = _target(index)
    show = _bake_show()
    _up, _along, side = _axes(index)
    left = _gesture_pose(_offset(base, side * (SWEEP_MM * 0.6)), ry=TILT_DEG, lift=show)
    right = _gesture_pose(_offset(base, -side * (SWEEP_MM * 0.6)), ry=-TILT_DEG, lift=show)
    for pose in (left, right, left):
        if not _go(robot, index, pose, "拨"):
            return False
    return True


def _do_roll(robot, index):
    if not _do_sweep(robot, index, "滚压"):
        return False
    if not _hybrid_ft:
        _spin_tool(robot, 40)
    return not _halted(index)


def _do_pinch(robot, index):
    global _phase
    with _lock:
        _phase = "手法"
    base = _target(index)
    show = _bake_show()
    _up, along, side = _axes(index)
    for step in range(8):
        angle = math.tau * step / 8.0
        vector = math.sin(angle) * along * CIRCLE_MM + math.sin(2.0 * angle) * side * (CIRCLE_MM * 0.55)
        bob = 6.0 * math.sin(2.0 * angle)
        pose = _gesture_pose(_offset(base, vector), ry=TILT_DEG, rz=TWIST_DEG if step % 2 == 0 else -TWIST_DEG, lift=show + bob)
        if not _go(robot, index, pose, "揉捏"):
            return False
    return True


def _do_shock(robot, index):
    global _phase
    with _lock:
        _phase = "手法"
    base = _target(index)
    show = _bake_show()
    high = _gesture_pose(base, lift=show)
    low = _gesture_pose(base, lift=show - 10.0)
    for _rep in range(4):
        if not _go(robot, index, low, "冲击"):
            return False
        if not _rest(0.12, index):
            return False
        if not _go(robot, index, high, "冲击"):
            return False
        if not _rest(0.12, index):
            return False
    return True


def _spin_tool(robot, degrees=35):
    if _stop.is_set():
        return False
    direction = 1 if degrees >= 0 else 0
    try:
        robot.command(f"MoveRelJ,0,5,{direction},{abs(int(degrees))},;", timeout=8)
    except RobotError:
        return False
    _wait_fsm(robot, {33}, 6)
    _remember_joints(robot)
    return True


def _go(robot, index, pose, gesture):
    while True:
        if _halted(index):
            return False
        if not _pause_for_level():
            return False
        if _halted(index):
            return False
        aimed = _retarget_grade(pose)
        if _hybrid_ft and _plan_is_massage():
            try:
                _hybrid_massage_follow(
                    robot,
                    aimed,
                    _target(index)["name"],
                    gesture,
                    kind="gesture",
                    air=False,
                    gesture=True,
                    index=index,
                )
            except Exception as exc:
                print("[heacon] massage go:", exc, flush=True)
        else:
            _move_pose(robot, aimed, _target(index)["name"], gesture, gesture=True, kind="gesture")
        if _hold_moves.is_set() and not _stop.is_set() and not _time_is_up():
            continue
        return not _stop.is_set()


def _retarget_grade(pose):
    if _technique_show is None:
        return pose
    delta = _show_lift() - _technique_show
    if abs(delta) < 0.4:
        return pose
    return _pose_at_lift(pose, vision_height(pose) + delta)


def _nudge_grade(robot, index):
    global _technique_show
    if _hybrid_ft and not _plan_is_moxa():
        _hybrid_apply_goal(robot)
        return True
    if _technique_show is None or robot is None:
        return True
    if _hold_moves.is_set():
        return True
    if abs(_show_lift() - _technique_show) < 0.8:
        return True
    try:
        state = robot.state()
    except RobotError:
        return True
    have = vision_height(state["pose"])
    if not (_hybrid_ft and _plan_is_moxa()) and abs(have - _technique_show) > 8:
        return True
    want = _show_lift()
    if abs(have - want) < 2:
        return True
    if _hybrid_ft and _plan_is_moxa():
        pose = _gesture_pose(_target(index), lift=want)
        _hybrid_contact_off(robot)
        _move_pose(robot, pose, _target(index)["name"], "调档", gesture=True, kind="vertical")
        _technique_show = want
        _hybrid_contact_on(robot)
        return not _stop.is_set()
    pose = _pose_at_lift(state["pose"], want)
    _move_pose(robot, pose, _target(index)["name"], "调档", gesture=True, kind="vertical")
    _technique_show = want
    return not _stop.is_set()


def _rest(seconds, index, wall=False):
    with _lock:
        pace = 1 if wall else max(1, int(_speed))
    deadline = time.monotonic() + max(0.0, float(seconds) / pace)
    while time.monotonic() < deadline:
        if _halted(index):
            return False
        if not _nudge_grade(_active_robot, index):
            return False
        if _hybrid_ft:
            if _plan_is_moxa():
                if not _hybrid_moxa_tick(_active_robot):
                    return False
            elif _plan_is_massage():
                if not _hybrid_massage_tick(_active_robot):
                    return False
            else:
                _hybrid_note_force(_active_robot)
        time.sleep(0.05)
    return True


def _halted(index):
    if _stop.is_set() or _time_is_up() or _step_over():
        return True
    return False


def _dwell_over(index):
    with _lock:
        if _plan_id not in ("p1", "p2"):
            return False
        segments = list(_segments)
    if not segments or index < 0 or index >= len(segments):
        return False
    return _running_seconds() >= sum(segments[: index + 1])


def _hybrid_setpoint(index, pose=None):
    """Commanded moxa pose: acupoint at the 30 mm standoff, or the last gesture."""
    if pose is not None:
        return dict(pose)
    if _hybrid_home is not None:
        return dict(_hybrid_home)
    return _gesture_pose(_target(index), lift=_show_lift())


def _hybrid_moxa_spring(robot):
    """恒力目标 = K * (穴位 - 当前). That is the missing stiffness term."""
    global _hybrid_moxa_prev, _hybrid_moxa_prev_t, _hybrid_moxa_log_t
    if robot is None or not _hybrid_ft:
        return False
    home = _hybrid_home
    if home is None:
        return False
    try:
        here = robot.read_pose()
    except RobotError:
        return False
    live = here.get("pose") if here else None
    if not live:
        return False
    now = time.monotonic()
    vx = vy = vz = 0.0
    if _hybrid_moxa_prev is not None and now - _hybrid_moxa_prev_t > 0.015:
        dt = min(0.2, now - _hybrid_moxa_prev_t)
        if dt > 0.015:
            vx = (float(live["x"]) - _hybrid_moxa_prev["x"]) / dt
            vy = (float(live["y"]) - _hybrid_moxa_prev["y"]) / dt
            vz = (float(live["z"]) - _hybrid_moxa_prev["z"]) / dt
    _hybrid_moxa_prev = {
        "x": float(live["x"]),
        "y": float(live["y"]),
        "z": float(live["z"]),
    }
    _hybrid_moxa_prev_t = now
    ex = float(home["x"]) - float(live["x"])
    ey = float(home["y"]) - float(live["y"])
    ez = float(home["z"]) - float(live["z"])
    k = HYBRID_MOXA_K_N_PER_MM
    d = HYBRID_MOXA_D_N_PER_MM_S
    cap = HYBRID_MOXA_F_MAX_N
    fx = max(-cap, min(cap, k * ex - d * vx))
    fy = max(-cap, min(cap, k * ey - d * vy))
    fz = max(-cap, min(cap, k * ez - d * vz))
    try:
        robot.set_ft_goal(fx=fx, fy=fy, fz=fz)
    except RobotError as exc:
        print("[heacon] moxa spring goal:", exc, flush=True)
        return False
    dist = math.sqrt(ex * ex + ey * ey + ez * ez)
    if dist > 12 and now - _hybrid_moxa_log_t > 1.2:
        _hybrid_moxa_log_t = now
        print(
            "[heacon] moxa spring dxyz=(%.0f,%.0f,%.0f) F=(%.1f,%.1f,%.1f)"
            % (ex, ey, ez, fx, fy, fz),
            flush=True,
        )
    _hybrid_note_force(robot)
    return True


def _hybrid_moxa_tick(robot):
    """Keep the virtual spring up. A shove must not sit in collision/fault."""
    if robot is None:
        return True
    _hybrid_moxa_spring(robot)
    try:
        fsm = robot.read_fsm()
    except RobotError:
        return True
    if fsm not in (20, 21, 22, 24):
        return True
    print("[heacon] moxa recover fsm", fsm, flush=True)
    if not (_hybrid_unlock_limit(robot) or _hybrid_recover(robot)):
        return True
    _hybrid_contact_on(robot)
    return not _stop.is_set()


def _clone_pose(pose):
    src = pose or {}
    return {key: float(src[key]) for key in ("x", "y", "z", "rx", "ry", "rz")}


def _lock_rz(pose):
    held = _hold_angles(pose)
    locked = dict(pose)
    locked["rz"] = held["rz"]
    return locked


def _lock_down(pose):
    """Keep XYZ, restore the camera-down flange angles."""
    held = _hold_angles(pose)
    locked = dict(pose)
    locked["rx"] = held["rx"]
    locked["ry"] = held["ry"]
    locked["rz"] = held["rz"]
    return locked


def _pose_advance(src, dst, step_mm):
    dx = float(dst["x"]) - float(src["x"])
    dy = float(dst["y"]) - float(src["y"])
    dz = float(dst["z"]) - float(src["z"])
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    out = dict(src)
    frac = 1.0 if dist < 1e-6 else min(1.0, float(step_mm) / dist)
    out["x"] = float(src["x"]) + dx * frac
    out["y"] = float(src["y"]) + dy * frac
    out["z"] = float(src["z"]) + dz * frac
    out["rx"] = float(dst["rx"])
    out["ry"] = float(dst["ry"])
    out["rz"] = float(dst["rz"])
    return out, dist


def _hybrid_massage_spring(robot):
    """Path spring around x_d(t). On the point, Fz is the grade force, downward."""
    global _hybrid_massage_prev, _hybrid_massage_prev_t, _hybrid_massage_log_t
    if robot is None or not _hybrid_ft or _plan_is_moxa():
        return False
    home = _hybrid_xd
    if home is None:
        return False
    try:
        here = robot.read_pose()
    except Exception:
        return False
    live = here.get("pose") if here else None
    if not live:
        return False
    try:
        now = time.monotonic()
        vx = vy = vz = 0.0
        if _hybrid_massage_prev is not None and now - _hybrid_massage_prev_t > 0.015:
            dt = min(0.2, now - _hybrid_massage_prev_t)
            if dt > 0.015:
                vx = (float(live["x"]) - _hybrid_massage_prev["x"]) / dt
                vy = (float(live["y"]) - _hybrid_massage_prev["y"]) / dt
                vz = (float(live["z"]) - _hybrid_massage_prev["z"]) / dt
        _hybrid_massage_prev = {
            "x": float(live["x"]),
            "y": float(live["y"]),
            "z": float(live["z"]),
        }
        _hybrid_massage_prev_t = now
        ex = float(home["x"]) - float(live["x"])
        ey = float(home["y"]) - float(live["y"])
        ez = float(home["z"]) - float(live["z"])
        drx = _angle_delta(home["rx"], live["rx"])
        dry = _angle_delta(home["ry"], live["ry"])
        k = HYBRID_MASSAGE_K_N_PER_MM
        d = HYBRID_MASSAGE_D_N_PER_MM_S
        cap = HYBRID_MASSAGE_F_MAX_N
        fx = max(-cap, min(cap, k * ex - d * vx))
        fy = max(-cap, min(cap, k * ey - d * vy))
        if _hybrid_massage_air:
            fz = max(-cap, min(cap, k * ez - d * vz))
            err = math.sqrt(ex * ex + ey * ey + ez * ez)
        else:
            fz = -abs(_hybrid_force_goal())
            err = math.hypot(ex, ey)
        mx = _hybrid_torque_goal("mx")
        my = _hybrid_torque_goal("my")
        mz = _hybrid_torque_goal("mz")
        robot.set_ft_goal(fx=fx, fy=fy, fz=fz, mx=mx, my=my, mz=mz)
        wrench = _hybrid_note_force(robot)
        if (err > 20 or abs(drx) > 3 or abs(dry) > 3) and now - _hybrid_massage_log_t > 1.2:
            _hybrid_massage_log_t = now
            smx = 0.0 if wrench is None else (wrench.get("mx") or 0)
            smy = 0.0 if wrench is None else (wrench.get("my") or 0)
            print(
                "[heacon] massage %s dxy=(%.0f,%.0f,%.0f) att=(%.1f,%.1f) F=(%.1f,%.1f,%.1f) M=(%.2f,%.2f) senseM=(%.2f,%.2f)"
                % (
                    "air" if _hybrid_massage_air else "work",
                    ex, ey, ez, drx, dry, fx, fy, fz, mx, my, smx, smy,
                ),
                flush=True,
            )
        return True
    except Exception as exc:
        print("[heacon] massage spring:", exc, flush=True)
        return False


def _hybrid_massage_tick(robot):
    if robot is None:
        return True
    try:
        _hybrid_massage_spring(robot)
        fsm = robot.read_fsm()
    except Exception:
        return True
    if fsm not in (20, 21, 22, 24):
        return True
    print("[heacon] massage recover fsm", fsm, flush=True)
    try:
        if _hybrid_unlock_limit(robot) or _hybrid_recover(robot):
            _hybrid_contact_on(robot)
    except Exception as exc:
        print("[heacon] massage recover:", exc, flush=True)
    return not _stop.is_set()


def _hybrid_massage_follow(robot, goal, point_name="", action="", kind="travel", air=True, gesture=False, index=None):
    """Move the spring center along the path. A shove must not end the service."""
    global _hybrid_xd, _hybrid_massage_air
    if robot is None:
        return False
    try:
        if not _hybrid_ft:
            _hybrid_contact_on(robot)
        pose = _lift_to_floor(dict(goal))
        pose = _lock_down(pose)
        _hybrid_massage_air = bool(air)
        if point_name:
            _mark_progress(point_name, action)
        if _hybrid_xd is None:
            try:
                here = robot.read_pose()
            except RobotError:
                here = None
            live = here.get("pose") if here else None
            _hybrid_xd = _clone_pose(live or pose)
        for key in ("rx", "ry", "rz"):
            _hybrid_xd[key] = float(pose[key])
        speed = max(20.0, float(_move_speed(kind)))
        dt = 0.05
        started = time.monotonic()
        timeout = 40.0
        while not _stop.is_set() and not _time_is_up():
            if index is not None and _halted(index):
                return False
            if not _pause_for_level():
                return False
            if not _hybrid_massage_tick(robot):
                return False
            arrive_z = HYBRID_MASSAGE_ARRIVE_MM if air else 80.0
            xy = math.hypot(pose["x"] - _hybrid_xd["x"], pose["y"] - _hybrid_xd["y"])
            z_off = abs(pose["z"] - _hybrid_xd["z"])
            if xy < HYBRID_MASSAGE_ARRIVE_MM and z_off < arrive_z:
                _hybrid_xd = _clone_pose(pose)
                _hybrid_massage_spring(robot)
                return True
            _hybrid_xd, _remain = _pose_advance(_hybrid_xd, pose, speed * dt)
            _hybrid_xd = _lock_down(_hybrid_xd)
            if time.monotonic() - started > timeout:
                print("[heacon] massage follow timeout", action, flush=True)
                return not _stop.is_set()
            time.sleep(dt)
        return False
    except Exception as exc:
        print("[heacon] massage follow:", exc, flush=True)
        try:
            _hybrid_unlock_limit(robot) or _hybrid_recover(robot)
            _hybrid_contact_on(robot)
        except Exception:
            pass
        return not _stop.is_set()


def _move_pose(robot, pose, point_name, action, gesture=False, kind="travel"):
    global _hybrid_home
    if _hybrid_ft and _plan_is_massage():
        _hybrid_massage_follow(
            robot, pose, point_name, action, kind=kind, air=_hybrid_massage_air, gesture=gesture
        )
        return
    state = robot.state()
    _guard_state(state)
    _guard_still_here(state)
    pose = _lift_to_floor(pose)
    if not gesture:
        pose = _hold_angles(pose)
    if _hybrid_ft and not _plan_is_moxa():
        live = (state.get("pose") or {})
        if "z" in live:
            pose = dict(pose)
            pose["z"] = live["z"]
    if _hybrid_ft and _plan_is_moxa():
        _hybrid_home = dict(pose)
    _guard_pose(pose, gesture=gesture, kind=kind)
    if gesture:
        _mark_progress(point_name, action)
    else:
        _mark_progress(point_name, action)
    current = state["pose"]
    if _same_pose(current, pose):
        return
    for _ in range(8):
        if _stop.is_set() or _time_is_up():
            return
        try:
            now = robot.state()
            current = now.get("pose") if now else None
        except RobotError:
            current = None
        if current and _same_pose(current, pose):
            _remember_joints(robot)
            return
        try:
            _send_move(robot, pose, kind=kind)
            settled = _wait_settled(robot, pose)
        except RobotError as exc:
            if _stop.is_set():
                raise
            if _hybrid_blocked(exc) or _hybrid_unreachable(exc):
                if _hybrid_unlock_limit(robot) or _hybrid_recover(robot):
                    continue
            raise
        if settled is True:
            try:
                arrived = robot.state()
            except RobotError:
                arrived = None
            if arrived and arrived.get("pose") and _same_pose(arrived["pose"], pose):
                _remember_joints(robot)
                return
            if _hybrid_ft and not _plan_is_moxa():
                _remember_joints(robot)
                return
            continue
        if not settled:
            continue
        return


def _send_move(robot, pose, kind="travel", urgent=False):
    global _cmd_id, _hybrid_ft
    if not urgent:
        while _hold_moves.is_set() and not _stop.is_set():
            time.sleep(0.05)
    with _lock:
        joints = list(_joints)
        _cmd_id += 1
        cmd_id = _cmd_id
    command = waypoint_command(pose, joints, cmd_id, speed=_move_speed(kind))
    try:
        _apply_override(robot, kind)
        robot.command(command, timeout=12)
    except RobotError as exc:
        if _stop.is_set():
            raise
        if not (_hybrid_blocked(exc) or _hybrid_unreachable(exc)):
            raise
        if _hybrid_ft:
            print("[heacon] hybrid move:", exc, flush=True)
            time.sleep(0.35)
            try:
                robot.command(command, timeout=12)
                return
            except RobotError as retry_exc:
                print("[heacon] hybrid move retry:", retry_exc, flush=True)
                return
        _hybrid_leave_freedrive(robot)
        try:
            robot.force_control_off()
        except RobotError:
            pass
        with _lock:
            _hybrid_ft = False
        if not _wait_fsm(robot, {33}, 3):
            if _stop.is_set():
                return
            if not (_hybrid_unlock_limit(robot) or _hybrid_recover(robot)):
                if _plan_is_massage():
                    print("[heacon] massage move continue:", exc, flush=True)
                    return
                raise
        state = robot.state()
        if state.get("connected") and _same_pose(state["pose"], pose):
            return
        try:
            robot.command(command, timeout=12)
        except RobotError as again:
            if _plan_is_massage():
                print("[heacon] massage move continue:", again, flush=True)
                return
            raise


def _prepare(robot):
    state = robot.state()
    if not state.get("connected"):
        raise RobotError(state.get("message") or "没有连上机械臂")
    try:
        if robot.read_ft_state() not in (0, None):
            robot.force_control_off()
    except RobotError:
        pass
    fsm = robot.read_fsm()
    if fsm in (20, 21, 22):
        if fsm == 20:
            _hybrid_unlock_limit(robot)
        else:
            robot.command("GrpReset,0,;")
        if not _wait_fsm(robot, {24, 33}, 8):
            raise RobotError("机械臂还停在故障状态，没有开始动作")
        fsm = robot.read_fsm()
    if fsm in (7, 24):
        robot.command("GrpEnable,0,;")
        if not _wait_fsm(robot, {33}, 8):
            raise RobotError("机械臂没有进入保持状态，没有开始动作")
    elif fsm in (25, 27):
        if not _wait_fsm(robot, {33}, 8):
            raise RobotError("机械臂正在移动，请先点停止")
        fsm = robot.read_fsm()
        if fsm != 33:
            raise RobotError(f"机械臂当前不能动作（状态 {fsm}）")
    elif fsm != 33:
        raise RobotError(f"机械臂当前不能动作（状态 {fsm}）")
    robot.command(f"SetOverride,0,{OVERRIDE:.2f},;")
    robot.command("SetCollideLevel,0,3,;")


def _guard_state(state):
    if not state.get("connected"):
        raise RobotError(state.get("message") or "没有连上机械臂")
    if _plan_is_massage():
        return
    if vision_height(state["pose"]) < _floor() - 12:
        raise RobotError("机械臂离床面太近，已停止")


def _guard_still_here(state):
    with _lock:
        origin = dict(_origin) if _origin else None
    if origin is None:
        raise RobotError("还没有记下开始时的位置")
    pose = state["pose"]
    if _plan_is_massage():
        return
    if vision_height(pose) < _floor() - 12:
        raise RobotError("机械臂离床面太近，已停止")
    if not (_hybrid_ft and (_plan_is_moxa() or _plan_is_massage())):
        if float(pose["z"]) > origin["z"] + 80:
            raise RobotError("机械臂高度异常，已停止")
        if math.hypot(float(pose["x"]) - origin["x"], float(pose["y"]) - origin["y"]) > MAX_TRAVEL_MM + SWEEP_MM + HYBRID_YIELD_XY + 20:
            raise RobotError("机械臂偏离了这次路线，已停止")
    if _hybrid_ft and not _plan_is_moxa() and not _plan_is_massage():
        if abs(_angle_delta(pose["rz"], origin["rz"])) > 8.0:
            raise RobotError("末端转得太多，已停止")
        tilt = max(
            abs(_angle_delta(pose["rx"], origin["rx"])),
            abs(_angle_delta(pose["ry"], origin["ry"])),
        )
        if tilt > GESTURE_LIMIT_DEG + HYBRID_TILT_YIELD_DEG:
            raise RobotError("末端姿态偏得太多，已停止")


def _guard_pose(pose, gesture=False, kind="travel"):
    if _plan_is_massage():
        return
    with _lock:
        origin = dict(_origin)
    show = _show_lift()
    hover = _hover_lift()
    if vision_height(pose) < _floor() - 0.5:
        raise RobotError("这次会低于确认过的工作高度，已停止")
    ceiling = show + 28 if gesture else hover + 12
    if vision_height(pose) > ceiling:
        raise RobotError("这次手法抬得太高，已停止")
    limit = GESTURE_LIMIT_DEG if gesture else 0.5
    for key in ("rx", "ry", "rz"):
        if abs(_angle_delta(pose[key], origin[key])) > limit + 0.05:
            if gesture:
                raise RobotError("这次手法的转动超出了安全范围，已停止")
            raise RobotError("点与点之间要保持末端姿态，已停止")
    if math.hypot(pose["x"] - origin["x"], pose["y"] - origin["y"]) > MAX_TRAVEL_MM + SWEEP_MM + 5:
        raise RobotError("这次路线超出了安全范围，已停止")


def _wait_settled(robot, goal, hold=False, seconds=150):
    # The drop from the overview camera pose to the work height is about 60 cm.
    # At the 0.20 speed limit that takes about a minute, so this wait covers it.
    deadline = time.time() + max(2.0, float(seconds))
    began = time.time()
    seen_moving = False
    last_remain = None
    last_change = time.time()
    xy = z_off = 1e9
    moxa = False
    while time.time() < deadline:
        if _stop.is_set():
            return False
        if _hold_moves.is_set() and not hold:
            return False
        fsm = robot.read_fsm()
        if fsm == 25:
            seen_moving = True
        moxa = _hybrid_ft and _plan_is_moxa()
        try:
            here = robot.read_pose()
        except RobotError:
            here = None
        pose = here.get("pose") if here else None
        if pose:
            xy = math.hypot(pose["x"] - goal["x"], pose["y"] - goal["y"])
            if _hybrid_ft and not moxa:
                z_off = 0.0
            else:
                z_off = abs(pose["z"] - goal["z"])
            remain = math.sqrt(xy * xy + z_off * z_off)
            if last_remain is None or abs(remain - last_remain) > 1.5:
                last_remain = remain
                last_change = time.time()
            elif time.time() - last_change > 8:
                if moxa and xy < HYBRID_MOXA_XY_DONE_MM and z_off < HYBRID_MOXA_Z_DONE_MM:
                    _hybrid_note_force(robot)
                    return True
                if _hybrid_ft and not moxa and xy < HYBRID_YIELD_XY:
                    _hybrid_note_force(robot)
                    return True
                return False
            if moxa:
                xy_ok, z_ok = HYBRID_MOXA_XY_DONE_MM, HYBRID_MOXA_Z_DONE_MM
            elif _hybrid_ft:
                xy_ok, z_ok = HYBRID_XY_DONE_MM, 80.0
            else:
                xy_ok, z_ok = 3.0, 3.0
            if xy < xy_ok and z_off < z_ok and time.time() - began > 0.3:
                if _hybrid_ft or seen_moving or time.time() - began > 1.0:
                    if _hybrid_ft:
                        _hybrid_note_force(robot)
                    return True
        if fsm == 33 and seen_moving:
            if moxa:
                xy_ok, z_ok = HYBRID_MOXA_XY_DONE_MM, HYBRID_MOXA_Z_DONE_MM
            elif _hybrid_ft:
                xy_ok, z_ok = HYBRID_XY_DONE_MM, 80.0
            else:
                xy_ok, z_ok = 3.0, 3.0
            if pose and xy < xy_ok and z_off < z_ok:
                return True
        elif fsm == 33 and time.time() - began > 2:
            state = robot.state()
            if state.get("connected") and _same_pose(state["pose"], goal):
                return True
        elif fsm in (21, 22):
            if moxa or _plan_is_massage():
                print("[heacon] settle recover fsm", fsm, flush=True)
                _hybrid_unlock_limit(robot) or _hybrid_recover(robot)
                return False
            if _hybrid_recover(robot):
                return False
            raise RobotError("机械臂停住了，请查看现场后再开始")
        elif fsm == 24 and not hold:
            if moxa or _plan_is_massage():
                _hybrid_unlock_limit(robot) or _hybrid_recover(robot)
                return False
            if _hybrid_recover(robot):
                return False
            raise RobotError("机械臂停住了，请查看现场后再开始")
        time.sleep(0.08)
    if moxa or _plan_is_massage():
        print("[heacon] settle timeout continue", flush=True)
        return False
    raise RobotError("机械臂这次移动没有完成")


def _wait_fsm(robot, wanted, seconds, hold=False):
    deadline = time.time() + seconds
    while time.time() < deadline:
        if not hold and _stop.is_set():
            return False
        if robot.read_fsm() in wanted:
            return True
        time.sleep(0.2)
    return False


def vision_height(pose):

    return vision.height_above_table((pose["x"], pose["y"], pose["z"]))


def _lift_to_floor(pose):

    height = vision_height(pose)
    if height >= _floor():
        return pose
    up = vision.up_vector()
    delta = (_floor() - height) * up
    lifted = dict(pose)
    lifted["x"] = pose["x"] + float(delta[0])
    lifted["y"] = pose["y"] + float(delta[1])
    lifted["z"] = pose["z"] + float(delta[2])
    return lifted


def _pose_at_lift(pose, lift_mm):

    height = vision_height(pose)
    up = vision.up_vector()
    delta = (float(lift_mm) - height) * up
    lifted = dict(pose)
    lifted["x"] = pose["x"] + float(delta[0])
    lifted["y"] = pose["y"] + float(delta[1])
    lifted["z"] = pose["z"] + float(delta[2])
    return lifted


def _reject_if_unsafe(origin, pose, name=""):
    if vision_height(pose) < _floor() - 0.5:
        raise RobotError("这次会碰到假人，已停止")
    travel = math.hypot(pose["x"] - origin["x"], pose["y"] - origin["y"])
    if travel > MAX_TRAVEL_MM:
        print(
            "[heacon] travel reject %s xy=%.0f from (%.0f,%.0f) to (%.0f,%.0f)"
            % (name, travel, origin["x"], origin["y"], pose["x"], pose["y"]),
            flush=True,
        )
        raise RobotError("看到的穴位离当前末端太远，先不移动")


def _camera_frame():

    vision.ensure_camera()
    deadline = time.time() + 4.0
    saw_frame = False
    last = "机械臂摄像头没有接到这台电脑"
    while time.time() < deadline:
        try:
            frame = vision.camera_bgr()
        except RuntimeError as exc:
            last = str(exc)
            time.sleep(0.25)
            continue
        saw_frame = True
        if body_route.find_body(frame) is not None:
            return frame
        time.sleep(0.2)
    if saw_frame:
        raise RobotError("摄像头里没有看清人或假人，机械臂先不动")
    if "摄像头" not in last:
        last = "摄像头里还没有画面，机械臂先不动"
    raise RobotError(last)


def _parse_segments(segments, count):
    if not isinstance(segments, list) or len(segments) != count:
        return []
    parsed = []
    for item in segments:
        try:
            seconds = float(item)
        except (TypeError, ValueError):
            return []
        if not math.isfinite(seconds) or seconds <= 0 or seconds > 3600:
            return []
        parsed.append(seconds)
    if sum(parsed) > 3 * 3600:
        return []
    return parsed


def _running_seconds():
    with _lock:
        extra = 0.0 if _mark is None else time.monotonic() - _mark
        return _spent + extra


def _pause_clock():
    global _spent, _mark
    with _lock:
        if _mark is not None:
            _spent += time.monotonic() - _mark
        _mark = None


def _resume_clock():
    global _mark
    with _lock:
        if _mark is None:
            _mark = time.monotonic()


def _time_is_up():
    with _lock:
        limit = _service_seconds
    if not limit:
        return False
    return _running_seconds() >= limit


def _move_speed(kind="travel"):
    if kind == "hybrid":
        return HYBRID_SPEED_MM_S
    if kind in ("home", "vertical", "gesture"):
        return SPEED_MM_S
    with _lock:
        pace = max(1, int(_speed))
    return min(SPEED_CAP_MM_S, SPEED_MM_S * pace)


def _override_ratio(kind="travel"):
    if kind == "hybrid":
        return 0.70
    if kind in ("home", "vertical"):
        return min(0.70, OVERRIDE * VERTICAL_PACE)
    with _lock:
        pace = max(1, int(_speed))
    if kind == "gesture":
        return min(1.0, OVERRIDE * pace)
    return min(OVERRIDE_CAP, OVERRIDE * pace)


def _apply_override(robot, kind="travel"):
    robot.command(f"SetOverride,0,{_override_ratio(kind):.2f},;")


def _hover_lift():
    return _show_lift() + TRAVEL_CLEARANCE_MM


def _home_after_service(robot):
    _hybrid_shutdown(robot)
    try:
        fsm = robot.read_fsm()
    except RobotError:
        fsm = None
    if fsm in (20, 21, 22, 24, 7):
        _hybrid_unlock_limit(robot) if fsm == 20 else _hybrid_recover(robot)
    try:
        robot.stop()
    except RobotError:
        pass
    _wait_fsm(robot, {24, 33}, 8, hold=True)
    try:
        if robot.read_fsm() in (20, 21, 22, 24, 7):
            _hybrid_unlock_limit(robot) if robot.read_fsm() == 20 else _hybrid_recover(robot)
    except RobotError:
        pass
    try:
        go_position(robot)
    except RobotError as exc:
        print("[heacon] home after service:", exc, flush=True)


def _plan(plan_id):
    plan = PLANS.get(plan_id)
    if plan is None:
        raise RobotError("没有这个服务方案")
    return plan


def _has_camera_pixels(plan, points):
    return _direct_pixels(plan, points) is not None


def _is_plan_layout(plan, placed):
    """True when the tablet sent the drawing's 540-wide dots, not camera pixels."""
    layout = [(float(item[1]), float(item[2])) for item in plan["points"]]
    if len(placed) != len(layout):
        return False
    return all(
        abs(float(u) - x) <= 1.5 and abs(float(v) - y) <= 1.5
        for (_name, u, v), (x, y) in zip(placed, layout)
    )


def _direct_pixels(plan, points):
    expected = len(plan["points"])
    if not isinstance(points, list) or len(points) != expected:
        return None
    placed = []
    for item, fallback in zip(points, plan["points"]):
        if not isinstance(item, dict):
            return None
        try:
            u = float(item["u"])
            v = float(item["v"])
        except (KeyError, TypeError, ValueError):
            return None
        if not math.isfinite(u) or not math.isfinite(v):
            return None
        placed.append((str(item.get("name") or fallback[0]), u, v))
    return placed


def _raw_points(plan, points):
    expected = len(plan["points"])
    if isinstance(points, list) and len(points) == expected:
        raw = []
        for item, fallback in zip(points, plan["points"]):
            if not isinstance(item, dict):
                raise RobotError("穴位位置无效")
            name = str(item.get("name") or fallback[0])
            try:
                x = float(item["x"])
                y = float(item["y"])
            except (KeyError, TypeError, ValueError) as exc:
                raise RobotError("穴位位置无效") from exc
            if not math.isfinite(x) or not math.isfinite(y):
                raise RobotError("穴位位置无效")
            raw.append((name, x, y))
        return raw
    return list(plan["points"])


def _hold_angles(pose):
    with _lock:
        origin = _origin
    if origin is None:
        return pose
    held = dict(pose)
    held["rx"] = origin["rx"]
    held["ry"] = origin["ry"]
    held["rz"] = origin["rz"]
    return held


def _remember_joints(robot):
    global _joints
    state = robot.state()
    joints = state.get("joints") if state.get("connected") else None
    if not joints or len(joints) != 6:
        return
    with _lock:
        _joints = list(joints)


def _shifted(pose, dx, dy, dz=0.0):
    moved = dict(pose)
    moved["x"] = pose["x"] + dx
    moved["y"] = pose["y"] + dy
    moved["z"] = pose["z"] + dz
    return moved


def _copy_pose(pose, x, y, z):
    copied = dict(pose)
    copied["x"] = x
    copied["y"] = y
    copied["z"] = z
    return copied


def _close(current, pose):
    return (
        math.hypot(current["x"] - pose["x"], current["y"] - pose["y"]) < 1.5
        and abs(current["z"] - pose["z"]) < 1.5
    )


def _current_index():
    with _lock:
        return _index


def _set_index(index):
    global _index
    _index = max(0, min(len(_targets) - 1, int(index)))


def _parse_grade(level):
    try:
        grade = int(level)
    except (TypeError, ValueError):
        return 2
    return max(0, min(4, grade))


def _tempo():
    with _lock:
        grade = _parse_grade(_level_grade)
    idx = 0 if grade <= 1 else 2 if grade >= 3 else 1
    return (1, 2, 3)[idx], (0.4, 0.8, 1.3)[idx], (1.2, 0.6, 0.3)[idx]


def _emphasis_name(plan_id, name):
    return any(token in str(name) for token in _EMPHASIS.get(plan_id, ()))


def _point_emphasis(index):
    with _lock:
        plan_id = _plan_id
        name = _targets[index]["name"] if _targets and 0 <= index < len(_targets) else ""
    return _emphasis_name(plan_id, name)


def _progress_text(name, gesture, plan_id, emphasis, round_no):
    title = PLANS.get(plan_id, {}).get("name", "")
    travel = TRAVEL_TECH.get(plan_id, "循经灸")
    if gesture == travel:
        return f"{title} · 第{round_no}轮 · 前往{name} · {gesture}"
    return f"{title} · 第{round_no}轮 · {name} · {gesture}"


def _mark_progress(name, gesture):
    global _gesture, _emphasis, _message
    with _lock:
        emphasis = _emphasis_name(_plan_id, name)
        _gesture = gesture
        _emphasis = emphasis
        _message = _progress_text(name, gesture, _plan_id, emphasis, _round)


def _note_visited(index):
    global _visited
    with _lock:
        if index not in _visited:
            _visited.append(index)


def _next_round():
    global _round, _visited
    with _lock:
        _round += 1
        _visited = []


def _advance(index, direction, count):
    """Walk forward, then back through the middle, then start the next round."""
    if count <= 1:
        return 0, 1, True
    if direction > 0:
        nxt = index + 1
        if nxt < count:
            return nxt, 1, False
        back = count - 2
        if back >= 1:
            return back, -1, False
        return 0, 1, True
    nxt = index - 1
    if nxt >= 1:
        return nxt, -1, False
    return 0, 1, True


def _target(index):
    with _lock:
        return dict(_targets[index])


def _axes(index):

    up = np.array(vision.up_vector(), float)
    up = up / np.linalg.norm(up)
    with _lock:
        targets = [dict(item) for item in _targets]
    here = targets[index]
    if len(targets) < 2:
        raw = np.array([1.0, 0.0, 0.0])
    else:
        other = targets[index + 1] if index + 1 < len(targets) else targets[index - 1]
        raw = np.array(
            [other["x"] - here["x"], other["y"] - here["y"], other["z"] - here["z"]],
            float,
        )
    along = raw - up * float(np.dot(raw, up))
    if float(np.linalg.norm(along)) < 1.0:
        seed = np.array([1.0, 0.0, 0.0])
        along = seed - up * float(np.dot(seed, up))
    along = along / np.linalg.norm(along)
    side = np.cross(up, along)
    side = side / np.linalg.norm(side)
    return up, along, side


def _offset(pose, vector):
    moved = dict(pose)
    moved["x"] = float(pose["x"]) + float(vector[0])
    moved["y"] = float(pose["y"]) + float(vector[1])
    moved["z"] = float(pose["z"]) + float(vector[2])
    return moved


def _clamp_angle(origin, delta):
    delta = max(-GESTURE_LIMIT_DEG, min(GESTURE_LIMIT_DEG, float(delta)))
    target = float(origin) + delta
    if target > 179.0:
        target = 179.0
    elif target < -179.0:
        target = -179.0
    return target


def _nudge(pose, rx=0.0, ry=0.0, rz=0.0):
    moved = dict(pose)
    moved["rx"] = _clamp_angle(pose["rx"], rx)
    moved["ry"] = _clamp_angle(pose["ry"], ry)
    moved["rz"] = _clamp_angle(pose["rz"], rz)
    return moved


def _gesture_pose(base, rx=0.0, ry=0.0, rz=0.0, lift=None):
    pose = _hold_angles(base)
    if lift is not None:
        pose = _pose_at_lift(pose, lift)
    return _nudge(pose, rx, ry, rz)


def _angle_delta(value, origin):
    return (float(value) - float(origin) + 180.0) % 360.0 - 180.0


def _same_pose(current, pose):
    if not _hybrid_ft:
        xy_lim, z_lim, tilt_lim, yaw_lim = 1.5, 1.5, 1.0, 1.0
    elif _plan_is_moxa():
        xy_lim, z_lim, tilt_lim, yaw_lim = (
            HYBRID_MOXA_SETTLE_MM, HYBRID_MOXA_SETTLE_MM, 3.0, 3.0
        )
    else:
        xy_lim, z_lim, tilt_lim, yaw_lim = HYBRID_XY_SETTLE_MM, 80.0, 8.0, 3.0
    return (
        math.hypot(current["x"] - pose["x"], current["y"] - pose["y"]) < xy_lim
        and abs(current["z"] - pose["z"]) < z_lim
        and abs(_angle_delta(current["rx"], pose["rx"])) < tilt_lim
        and abs(_angle_delta(current["ry"], pose["ry"])) < tilt_lim
        and abs(_angle_delta(current["rz"], pose["rz"])) < yaw_lim
    )


def _set(running, message):
    global _running, _message
    with _lock:
        _running = running
        _message = message
