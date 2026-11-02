"""The fake arm must answer exactly like the real client, or nothing else works."""

from __future__ import annotations

import time

import pytest

from heacon.robot.simulator import FSM_MOVING, FSM_STANDBY, SimulatedRobot


def test_state_shape_matches_the_real_client(robot):
    state = robot.state()
    assert state["connected"] is True
    assert state["ready"] is True
    assert state["fsm"] == FSM_STANDBY
    assert set(state["pose"]) == {"x", "y", "z", "rx", "ry", "rz"}
    assert len(state["joints"]) == 6


def test_wake_returns_ok_and_message_like_the_real_client(robot):
    # The console unpacks this as (ok, message). Returning a bare string here
    # is what broke the keep-awake thread during the restructure.
    ok, message = robot.wake()
    assert ok is True
    assert isinstance(message, str) and message


def test_lock_returns_ok_and_message(robot):
    ok, message = robot.lock()
    assert ok is True
    assert isinstance(message, str) and message


def test_waypoint_moves_the_pose_then_returns_to_standby(robot):
    before = robot.state()["pose"]
    robot.command(
        "WayPoint,0,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,0,0,0,0,0,0,0,1"
        % (before["x"] + 30, before["y"], before["z"], before["rx"], before["ry"], before["rz"])
    )
    assert robot.read_fsm() == FSM_MOVING

    deadline = time.time() + 3
    while time.time() < deadline and robot.read_fsm() == FSM_MOVING:
        time.sleep(0.05)

    after = robot.state()["pose"]
    assert robot.read_fsm() == FSM_STANDBY
    assert after["x"] == pytest.approx(before["x"] + 30, abs=0.5)


def test_force_goal_shows_up_on_the_sensor(robot):
    robot.force_control_on()
    robot.set_ft_goal(fz=-8.0)
    assert robot.read_force_calibrated()["fz"] < 0


def test_zero_clears_the_reading(robot):
    robot.set_ft_goal(fz=-8.0)
    robot.set_force_zero()
    assert robot.read_force()["fz"] == 0.0


def test_unknown_command_does_not_raise(robot):
    # The console issues many setup commands; an unknown one must be inert.
    assert robot.command("SetSomethingNew,0,1,;") is not None


def test_simulated_robot_directly():
    arm = SimulatedRobot()
    assert arm.read_fsm() == FSM_STANDBY
