"""A slow side-to-side move at the current height, for a live demonstration."""

import threading
import time

from heacon.robot.client import RobotError

AXIS_Y = 1
SWING_MM = 50.0
MIN_Z_MM = 750.0

_lock = threading.Lock()
_stop = threading.Event()
_thread = None
_running = False
_starting = False
_message = "点「机械臂开始动作」后，机械臂会在当前高度左右缓慢移动，不会下降。"
_origin_y = None
_go_forward = True


def status():
    with _lock:
        return {"running": _running, "message": _message}


def start(robot):
    global _thread, _origin_y, _go_forward, _starting, _running, _message
    with _lock:
        if _running and _thread is not None and _thread.is_alive():
            return True, _message
        if _starting:
            return False, "机械臂正在准备动作"
        _starting = True
        _stop.clear()
    try:
        _prepare(robot)
        state = robot.state()
        _guard_height(state)
        origin = float(state["pose"]["y"])
        _command_toward(robot, origin + SWING_MM)
    except RobotError as exc:
        _set(False, str(exc))
        with _lock:
            _starting = False
        return False, str(exc)
    with _lock:
        _origin_y = origin
        _go_forward = False
        _starting = False
        _running = True
        _message = "机械臂正在当前高度左右缓慢移动"
        message = _message
        _thread = threading.Thread(target=_loop, args=(robot,), name="demo-motion", daemon=True)
        _thread.start()
    return True, message


def stop(robot):
    _stop.set()
    try:
        robot.stop()
    except RobotError as exc:
        _set(False, str(exc))
        return False, str(exc)
    _set(False, "机械臂已停在原地")
    return True, "机械臂已停在原地"


def _prepare(robot):
    state = robot.state()
    if not state.get("connected"):
        raise RobotError(state.get("message") or "没有连上机械臂")
    fsm = robot.read_fsm()
    if fsm in (21, 22):
        robot.command("GrpReset,0,;")
        if not _wait_fsm(robot, {24, 33}, 8):
            raise RobotError("机械臂还停在故障状态，没有开始动作")
        fsm = robot.read_fsm()
    if fsm == 24:
        robot.command("GrpEnable,0,;")
        if not _wait_fsm(robot, {33}, 8):
            raise RobotError("机械臂没有进入保持状态，没有开始动作")
    elif fsm == 25:
        raise RobotError("机械臂正在移动，请先点停止")
    elif fsm != 33:
        raise RobotError(f"机械臂当前不能动作（状态 {fsm}）")
    robot.command("SetOverride,0,0.10,;")
    robot.command("SetCollideLevel,0,3,;")


def _loop(robot):
    global _go_forward
    try:
        while not _stop.is_set():
            if not _wait_settled(robot):
                break
            if _stop.is_set():
                break
            with _lock:
                origin = _origin_y
                forward = _go_forward
                _go_forward = not _go_forward
            target = origin + (SWING_MM if forward else 0.0)
            _command_toward(robot, target)
    except RobotError as exc:
        _set(False, str(exc))
        return
    if _stop.is_set():
        _set(False, "机械臂已停在原地")


def _command_toward(robot, target_y):
    state = robot.state()
    if not state.get("connected"):
        raise RobotError(state.get("message") or "没有连上机械臂")
    _guard_height(state)
    delta = float(target_y) - float(state["pose"]["y"])
    if abs(delta) < 2:
        return
    if abs(delta) > 60:
        raise RobotError("机械臂偏离了演示位置，已停止动作")
    direction = 1 if delta > 0 else 0
    robot.move_horizontal(AXIS_Y, direction, abs(delta))


def _guard_height(state):
    if float(state["pose"]["z"]) < MIN_Z_MM:
        raise RobotError("机械臂高度太低，这次没有让它动")


def _wait_settled(robot):
    deadline = time.time() + 30
    began = time.time()
    seen_moving = False
    while time.time() < deadline:
        if _stop.is_set():
            return False
        fsm = robot.read_fsm()
        if fsm == 25:
            seen_moving = True
        elif fsm == 33 and (seen_moving or time.time() - began > 2):
            return True
        elif fsm in (21, 22, 24):
            raise RobotError("机械臂停住了，请查看现场后再开始")
        time.sleep(0.15)
    raise RobotError("机械臂这次移动没有完成")


def _wait_fsm(robot, wanted, seconds):
    deadline = time.time() + seconds
    while time.time() < deadline:
        if _stop.is_set():
            return False
        if robot.read_fsm() in wanted:
            return True
        time.sleep(0.2)
    return False


def _set(running, message):
    global _running, _message
    with _lock:
        _running = running
        _message = message
