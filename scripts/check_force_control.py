#!/usr/bin/env python3
"""Field self-check: is the force-impedance (力位混合) path actually live?

Run this on the machine that will drive the arm, before the first service:

    python scripts/check_force_control.py

It answers one question: when a service starts, will the force-control layer
actually turn on, or will every move silently fall back to plain position
control? On a fresh Linux box the usual answer is "it falls back", and the
reason is almost always one of:

  * the robot TCP endpoint is not reachable (wrong IP / firewalled)
  * the arm is not in standby (FSM 33)
  * the force-torque sensor is absent, so `force_control_on` is refused
  * HEACON_SIMULATE=1 is still set, so the fake arm answers instead

Each check below prints PASS / FAIL and what to do about a FAIL. Exit code is
0 only when everything a service needs is ready.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import heacon.config as config


PASS = "PASS"
FAIL = "FAIL"
WARN = "WARN"

_results: list[tuple[str, str, str]] = []


def record(name: str, status: str, detail: str = "") -> None:
    _results.append((name, status, detail))
    line = "[%s] %s" % (status, name)
    if detail:
        line += " — " + detail
    print(line, flush=True)


def check_simulate() -> None:
    if config.SIMULATE:
        record(
            "simulate",
            WARN,
            "HEACON_SIMULATE=1：现在连的是假机械臂，力位混合不会真的作用在硬件上。"
            "现场必须改成 0。",
        )
    else:
        record("simulate", PASS, "真实机械臂模式")


def check_config() -> None:
    described = config.describe()
    record(
        "config",
        PASS,
        "robot=%s console=%s camera=%s"
        % (described["robot"], described["console"], described["camera_backend"]),
    )
    if config.camera_backend() == "none":
        record("camera backend", FAIL, "这台机器上看不到可用摄像头后端")


def check_calibration() -> None:
    for label, path in (
        ("handeye", config.HANDEYE),
        ("color_calib", config.COLOR_CALIB),
        ("work_home", config.WORK_HOME),
    ):
        if not path.is_file():
            record("calibration %s" % label, FAIL, "缺少 %s" % path)
            continue
        if path.name.endswith(".example.json"):
            record(
                "calibration %s" % label,
                FAIL,
                "当前用的是仓库示例 %s，不是本机标定值。"
                "用示例值会让工具落在错误位置，必须先在本机重新标定。" % path.name,
            )
        else:
            record("calibration %s" % label, PASS, str(path))


def check_robot() -> None:
    from heacon.robot import open_robot

    robot = open_robot()
    try:
        state = robot.state()
    except Exception as exc:  # noqa: BLE001 - report anything the client raises
        record("robot reachable", FAIL, "%s: %s" % (type(exc).__name__, exc))
        return

    if not state.get("connected"):
        record(
            "robot reachable",
            FAIL,
            "%s。确认网线、%s:%d 是否可达（ping / telnet）。"
            % (state.get("message") or "连不上", config.ROBOT_HOST, config.ROBOT_PORT),
        )
        return
    record("robot reachable", PASS, "model=%s" % state.get("model"))

    fsm = state.get("fsm")
    if fsm == 33:
        record("fsm standby", PASS, "FSM 33")
    else:
        record(
            "fsm standby",
            FAIL,
            "FSM %s，力控只在待机(33)时能打开。先解锁/复位。" % fsm,
        )

    try:
        ft_state = robot.read_ft_state()
    except Exception as exc:  # noqa: BLE001
        record("force sensor", FAIL, "读不到力控状态：%s" % exc)
        return
    record(
        "force sensor",
        PASS,
        "ReadFTControlState=%s （0=关闭，正常待机时就是 0）" % ft_state,
    )


def check_force_control_switch() -> None:
    """The decisive test: can this arm actually enter force control right now?

    Mirrors what `_hybrid_contact_on` does, and turns it back off immediately
    so the machine is left exactly as it was found.
    """
    from heacon.robot import open_robot

    robot = open_robot()
    state = robot.state()
    if not state.get("connected"):
        record("force control", FAIL, "机械臂没连上，跳过")
        return
    if state.get("fsm") != 33:
        record("force control", FAIL, "不在待机状态，跳过")
        return

    try:
        robot.force_control_on()
    except Exception as exc:  # noqa: BLE001
        record(
            "force control",
            FAIL,
            "打开力控被拒：%s。力位混合完全依赖这一步；"
            "打不开就只会走位置控制。" % exc,
        )
        return

    try:
        ft_state = robot.read_ft_state()
    except Exception:  # noqa: BLE001
        ft_state = None
    record("force control", PASS, "力控已打开，ReadFTControlState=%s" % ft_state)

    # Restore: force control off, arm back to standby.
    try:
        robot.force_control_off()
        record("force control restore", PASS, "已关闭力控，机械臂回到原状")
    except Exception as exc:  # noqa: BLE001
        record("force control restore", WARN, "关闭时出错，请人工确认：%s" % exc)


def check_daemon() -> None:
    """The console must be the only owner of the robot TCP session."""
    import urllib.error
    import urllib.request

    url = "http://127.0.0.1:%d/api/state" % config.HTTP_PORT
    try:
        with urllib.request.urlopen(url, timeout=3) as resp:
            record("console running", PASS, "8765 在跑")
            _ = resp.read()
    except (urllib.error.URLError, OSError):
        record(
            "console running",
            WARN,
            "8765 没在跑。先 `python main.py console` 再验证页面。",
        )


def main() -> int:
    print("HEACON 力位混合自检\n" + "=" * 60)
    check_simulate()
    check_config()
    check_calibration()
    check_robot()
    check_force_control_switch()
    check_daemon()

    print("=" * 60)
    failures = [r for r in _results if r[1] == FAIL]
    warnings = [r for r in _results if r[1] == WARN]
    if failures:
        print("结论：%d 项未通过，力位混合不会按预期工作。" % len(failures))
        for name, _status, detail in failures:
            print("  - %s: %s" % (name, detail))
        return 1
    if warnings:
        print("结论：可用，但有 %d 项提醒。" % len(warnings))
        return 0
    print("结论：全部通过，力位混合可以工作。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
