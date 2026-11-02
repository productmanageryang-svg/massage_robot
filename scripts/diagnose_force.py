#!/usr/bin/env python3
"""只读诊断：力位混合为什么不回弹。

    python scripts/diagnose_force.py

在同事那台 Linux 上跑，把完整输出发回来即可。**不改动机械臂**：
不下发任何运动指令，不开力控超过两秒，结束时恢复原状。

它回答四个问题：
  1. 力控能不能打开？能打开多久？（华炎搜索模式会在空中自动退出）
  2. 工作高度算出来是多少？和示教的穴位高度差多少？
  3. 标定文件是本机实测的，还是仓库里的示例？
  4. 皮肤接触时，控制器报的力是多少？（判断传感器是否正常）

第 1 项是"推得动但不回弹"最直接的原因：力控根本没活着。
"""

from __future__ import annotations

import json
import math
import platform
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import heacon.config as config

SEP = "=" * 68
SUB = "-" * 68


def title(text: str) -> None:
    print("\n" + SEP)
    print(text)
    print(SEP)


def kv(key: str, value, note: str = "") -> None:
    line = "  %-34s %s" % (key, value)
    if note:
        line += "    # " + note
    print(line)


def section_environment() -> None:
    title("1. 环境")
    kv("系统", platform.platform())
    kv("Python", platform.python_version())
    kv("工作目录", Path.cwd())
    cfg = config.describe()
    for key in ("robot", "console", "runtime_dir", "config_dir", "web_dir"):
        kv(key, cfg[key])
    kv("HEACON_SIMULATE", cfg["simulate"], "必须为 False，否则连的是假机械臂")
    kv("camera_backend", cfg["camera_backend"])

    for label, pkg in (("numpy", "numpy"), ("opencv", "cv2"), ("pyrealsense2", "pyrealsense2")):
        try:
            mod = __import__(pkg)
            kv(label, getattr(mod, "__version__", "已安装"))
        except Exception as exc:  # noqa: BLE001
            kv(label, "缺失", str(exc)[:60])


def section_calibration() -> None:
    title("2. 标定文件")
    for label, path, key in (
        ("handeye", config.HANDEYE, None),
        ("color_calib", config.COLOR_CALIB, None),
        ("work_home", config.WORK_HOME, "pose"),
    ):
        exists = path.is_file()
        is_example = path.name.endswith(".example.json")
        flag = "示例文件" if is_example else "本机文件"
        kv(label, "%s (%s)" % (path.name, flag))
        if not exists:
            print("      !! 文件不存在")
            continue
        try:
            data = json.loads(path.read_text())
        except Exception as exc:  # noqa: BLE001
            print("      !! 无法解析：%s" % exc)
            continue
        if key and key in data:
            kv("  " + key, {k: round(float(v), 2) for k, v in data[key].items()})
        if label == "handeye":
            intr = data.get("intrinsics", {})
            kv("  fx/fy", "%s / %s" % (round(intr.get("fx", 0), 2), round(intr.get("fy", 0), 2)))
            plane = data.get("board_plane", {})
            kv("  plane point", [round(float(x), 2) for x in plane.get("point_mm", [])])
            kv("  plane normal", [round(float(x), 3) for x in plane.get("normal", [])])

    if config.HANDEYE.name.endswith(".example.json"):
        print("\n  => 现在用的是仓库示例，不是本机标定。")
        print("     这会让“工作高度”偏离真实皮肤，力控因此贴不到面而失效。")


def section_geometry() -> None:
    title("3. 工作高度推算")
    from heacon.motion import constants as C
    from heacon.motion import route as R

    moxa = R.work_height("moxa")
    massage = R.work_height("massage")
    kv("WORK_LIFT_MM", C.WORK_LIFT_MM)
    kv("SKIN_STANDOFF_MM", C.SKIN_STANDOFF_MM)
    kv("MOXA_STANDOFF_MM", C.MOXA_STANDOFF_MM)
    kv("艾灸工作高度", round(moxa, 2), "= WORK_LIFT + (MOXA - SKIN)")
    kv("按摩工作高度", round(massage, 2))
    kv("艾灸悬停高度", round(moxa + C.TRAVEL_CLEARANCE_MM, 2), "工作高度 + 50mm")
    kv("TRAVEL_CLEARANCE_MM", C.TRAVEL_CLEARANCE_MM)
    kv("K 刚度", "%s N/mm" % C.HYBRID_MOXA_K_N_PER_MM, "回弹靠这一项")
    kv("D 阻尼", "%s N/(mm/s)" % C.HYBRID_MOXA_D_N_PER_MM_S)
    kv("力上限", "%s N" % C.HYBRID_MOXA_F_MAX_N)


def section_robot() -> tuple[object, dict]:
    title("4. 机械臂连接与力控存活")
    from heacon.robot import open_robot

    robot = open_robot()
    state = robot.state()
    if not state.get("connected"):
        kv("连接", "失败", state.get("message") or "")
        print("\n  => 连不上机械臂，后面几项无法诊断。先解决网络。")
        return robot, state
    kv("连接", "成功")
    kv("型号", state.get("model"))
    kv("FSM", state.get("fsm"), "33=待机")
    pose = state.get("pose") or {}
    if pose:
        kv("当前位姿", {k: round(float(v), 2) for k, v in pose.items()})
    try:
        from heacon.vision import geometry as vision
        if pose:
            h = vision.height_above_table((pose["x"], pose["y"], pose["z"]))
            kv("距台面高度", round(float(h), 2), "工作高度应接近 -30/-40")
    except Exception as exc:  # noqa: BLE001
        kv("高度计算", "失败", str(exc)[:60])

    if state.get("fsm") != 33:
        print("\n  => 不在待机，跳过力控测试。")
        return robot, state

    # The decisive measurement: how long does force control stay on?
    print(SUB)
    print("  力控存活测试（最关键的一项）")
    print(SUB)
    try:
        robot.force_control_on()
    except Exception as exc:  # noqa: BLE001
        kv("  force_control_on", "被拒", str(exc)[:80])
        print("\n  => 力控打不开。力位混合不可能工作。")
        return robot, state

    timeline = []
    for i in range(16):
        time.sleep(0.25)
        try:
            fsm, ft = robot.read_fsm(), robot.read_ft_state()
        except Exception:  # noqa: BLE001
            break
        timeline.append((round(i * 0.25, 2), fsm, ft))
    for t, fsm, ft in timeline:
        mark = "力控在线" if ft in (1, 2, 3) else "力控已关闭"
        print("    t=%5.2fs  fsm=%-3s ft=%-2s %s" % (t, fsm, ft, mark))

    alive = [t for t, _f, ft in timeline if ft in (1, 2, 3)]
    if alive:
        kv("  力控存活时长", "%.2f 秒" % (max(alive) + 0.25), "空中会自己退出是正常现象")
    else:
        kv("  力控存活时长", "0 秒")

    # Contact force while pressed, if a hand is on the tool.
    try:
        w = robot.read_force_calibrated()
        kv("  当前力读数", {k: round(float(v), 2) for k, v in w.items()})
    except Exception as exc:  # noqa: BLE001
        kv("  力读数", "读取失败", str(exc)[:60])

    try:
        robot.set_ft_goal()
        robot.force_control_off()
        time.sleep(0.4)
        robot.wake()
        kv("  已恢复", "力控关闭，机械臂回待机")
    except Exception as exc:  # noqa: BLE001
        kv("  恢复", "出错，请人工确认", str(exc)[:80])

    return robot, state


def section_airborne_verdict(timeline_alive: bool) -> None:
    title("5. 结论")
    if timeline_alive:
        print("  力控在待机姿态下能被打开。")
    print("""
  “推得动但不回弹”有两种可能，请对照上面的数字：

  A. 力控根本没活着
     判断：力控存活测试里 ft 很快变 0，且服务过程中日志看不到
           [heacon] moxa spring dxyz=... 这样的行。
     原因：华炎力控是搜索模式，空中找不到接触面就自动退出。
           第三版靠“降到工作高度时正好贴到皮肤”赶在这个窗口内建立接触。
           换机器后工作高度偏差几毫米，就会错过这个窗口。

  B. 力控活着但没有刚度
     判断：日志里能看到 moxa spring 行，但 F= 一直是 0.00。
     原因：平衡点和当前位置重合，误差为 0，所以目标力为 0。
           这通常意味着标定把穴位算到了工具当前位置，而不是皮肤上。

  请把“工作高度推算”里算出来的数值，和实际用卷尺量的
  “法兰到皮肤”距离对比。差得多，就是标定问题。
""")


def main() -> int:
    print("HEACON 力位混合诊断（只读，不移动机械臂）")
    print("时间:", time.strftime("%Y-%m-%d %H:%M:%S"))
    section_environment()
    section_calibration()
    section_geometry()
    robot, state = section_robot()
    alive = bool(state.get("connected") and state.get("fsm") == 33)
    section_airborne_verdict(alive)
    print(SEP)
    print("诊断结束。请把以上全部输出发回。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
