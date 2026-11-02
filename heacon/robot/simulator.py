"""Fake arm so the system boots, and CI can run, with no hardware attached.

It answers the same calls :class:`heacon.robot.client.RobotClient` does, with
the same shapes, so nothing downstream needs to know whether a real
controller is on the other end. Motion is integrated in software: a command
target becomes the pose over a short travel time, and force readings settle to
whatever goal was last requested.
"""

from __future__ import annotations

import math
import threading
import time

from heacon.robot.client import RobotError

#: How long the fake arm takes to reach a commanded pose at full speed.
_TRAVEL_SECONDS = 0.9
#: Standby, so the state machine sees the same FSM a real controller reports.
FSM_STANDBY = 33
FSM_MOVING = 25


def _fsm_message(fsm):
    return "机械臂正在移动" if fsm == FSM_MOVING else ""


class SimulatedRobot:
    """Minimal stand-in for the Huayan controller."""

    host = "simulated"
    port = 0

    def __init__(self, pose=None, joints=None):
        self._lock = threading.Lock()
        self._pose = dict(
            pose
            or {
                "x": -458.314,
                "y": -56.893,
                "z": 838.693,
                "rx": -179.987,
                "ry": -1.167,
                "rz": -177.441,
            }
        )
        self._joints = list(joints or [7.106, 14.041, -30.604, -0.111, -134.191, 4.470])
        self._target = dict(self._pose)
        self._moved_at = time.monotonic()
        self._fsm = FSM_STANDBY
        self._ft_state = 0
        self._goal = {"fx": 0.0, "fy": 0.0, "fz": 0.0, "mx": 0.0, "my": 0.0, "mz": 0.0}
        self._wrench = {"fx": 0.0, "fy": 0.0, "fz": 0.0, "mx": 0.0, "my": 0.0, "mz": 0.0}
        self.commands = []

    # --- the surface the console uses ------------------------------------

    def close(self):
        return None

    def command(self, message, timeout=None):
        self.commands.append(message)
        if message.startswith("ReadActPos"):
            return self._act_pos_reply()
        if message.startswith("ReadRobotModel"):
            return ",ReadRobotModel,0,heacon-sim,;"
        if message.startswith("ReadFSM"):
            return ",ReadFSM,0,%d,;" % self._fsm
        if message.startswith("GrpStop"):
            self._fsm = FSM_STANDBY
            return ",GrpStop,0,OK,;"
        if message.startswith("SetForceControlState"):
            want = 1 if message.split(",")[2].strip() == "1" else 0
            self._ft_state = 1 if want else 0
            return ",SetForceControlState,0,OK,;"
        if message.startswith("HRSetControlGoal"):
            fields = [f for f in message.split(",") if f.strip()]
            try:
                values = [float(f) for f in fields[2:8]]
            except ValueError:
                values = [0.0] * 6
            keys = ("fx", "fy", "fz", "mx", "my", "mz")
            with self._lock:
                self._goal = dict(zip(keys, values))
            return ",HRSetControlGoal,0,OK,;"
        if message.startswith("WayPoint"):
            self._apply_waypoint(message)
            return ",WayPoint,0,OK,;"
        return ",%s,0,OK,;" % message.split(",")[0].lstrip(", ")

    def state(self):
        with self._lock:
            self._settle()
            return {
                "connected": True,
                "model": "heacon-sim",
                "joints": list(self._joints),
                "pose": dict(self._pose),
                "fsm": self._fsm,
                "ready": self._fsm == FSM_STANDBY,
                "message": _fsm_message(self._fsm),
            }

    def read_pose(self):
        with self._lock:
            self._settle()
            return {"joints": list(self._joints), "pose": dict(self._pose)}

    def read_fsm(self):
        with self._lock:
            self._settle()
            return self._fsm

    def read_ft_state(self):
        return self._ft_state

    def read_force(self):
        with self._lock:
            return dict(self._wrench)

    def read_force_calibrated(self):
        with self._lock:
            return dict(self._wrench)

    def force_control_on(self):
        self._ft_state = 1
        return ",SetForceControlState,0,OK,;"

    def force_control_off(self):
        self._ft_state = 0
        return ",SetForceControlState,0,OK,;"

    def set_force_zero(self):
        with self._lock:
            self._wrench = {"fx": 0.0, "fy": 0.0, "fz": 0.0, "mx": 0.0, "my": 0.0, "mz": 0.0}
        return ",SetForceZero,0,OK,;"

    def stop(self):
        with self._lock:
            self._fsm = FSM_STANDBY
            self._target = dict(self._pose)
        return ",GrpStop,0,OK,;"

    def pause(self):
        return self.stop()

    def enable(self):
        with self._lock:
            self._fsm = FSM_STANDBY
        return ",GrpEnable,0,OK,;"

    def wake(self):
        """Same contract as the real client: (ok, message)."""
        self.enable()
        return True, "机械臂已就绪"

    def disable(self):
        with self._lock:
            self._fsm = 24
        return ",GrpDisable,0,OK,;"

    def lock(self):
        self.disable()
        return True, "机械臂已锁定"

    def jog_joint(self, axis, direction):
        with self._lock:
            if 0 <= axis < len(self._joints):
                self._joints[axis] += 2.0 * (1 if direction else -1)
        return ",MoveRelJ,0,OK,;"

    def move_horizontal(self, axis, direction, distance):
        return ",OK,;"

    # Everything the force-control wrappers call. The fake arm accepts them
    # and records that they arrived, which is what the tests assert on.
    def set_ft_free_drive(self, on):
        return ",OK,;"

    def set_ft_motion_freedom(self, *args, **kwargs):
        return ",OK,;"

    def set_ft_search_freedom(self, *args, **kwargs):
        return ",OK,;"

    def set_ft_strategy(self, mode):
        return ",OK,;"

    def set_ft_tool_frame(self, on):
        return ",OK,;"

    def set_ft_goal(self, fx=0, fy=0, fz=0, mx=0, my=0, mz=0):
        with self._lock:
            self._goal = {"fx": fx, "fy": fy, "fz": fz, "mx": mx, "my": my, "mz": mz}
            # A goal force shows up on the sensor once the arm leans into it.
            self._wrench = {
                "fx": fx * 0.9,
                "fy": fy * 0.9,
                "fz": fz * 0.9,
                "mx": mx * 0.9,
                "my": my * 0.9,
                "mz": mz * 0.9,
            }
        return ",OK,;"

    def set_ft_distance_limit(self, *args, **kwargs):
        return ",OK,;"

    def set_ft_data_limit(self, *args, **kwargs):
        return ",OK,;"

    def set_ft_free_factor(self, *args, **kwargs):
        return ",OK,;"

    def set_ft_free_speed_mode(self, mode):
        return ",OK,;"

    def set_ft_search_speed(self, *args, **kwargs):
        return ",OK,;"

    def set_ft_mass(self, *args, **kwargs):
        return ",OK,;"

    def set_ft_damp(self, *args, **kwargs):
        return ",OK,;"

    def set_ft_stiff(self, *args, **kwargs):
        return ",OK,;"

    def set_ft_pid(self, *args, **kwargs):
        return ",OK,;"

    # --- internals -------------------------------------------------------

    def _act_pos_reply(self):
        with self._lock:
            self._settle()
            values = list(self._joints) + [
                self._pose["x"],
                self._pose["y"],
                self._pose["z"],
                self._pose["rx"],
                self._pose["ry"],
                self._pose["rz"],
            ]
        return ",ReadActPos,0," + ",".join("%.3f" % v for v in values) + ",;"

    def _apply_waypoint(self, message):
        fields = [f for f in message.split(",") if f.strip()]
        try:
            values = [float(f) for f in fields[2:8]]
        except ValueError:
            return
        if len(values) < 6:
            return
        with self._lock:
            self._target = dict(zip(("x", "y", "z", "rx", "ry", "rz"), values))
            self._moved_at = time.monotonic()
            self._fsm = FSM_MOVING

    def _settle(self):
        """Walk the pose toward the last commanded target."""
        elapsed = time.monotonic() - self._moved_at
        if self._fsm != FSM_MOVING:
            return
        frac = min(1.0, elapsed / _TRAVEL_SECONDS)
        for key in ("x", "y", "z", "rx", "ry", "rz"):
            start = self._pose[key]
            end = self._target[key]
            self._pose[key] = start + (end - start) * frac
        if frac >= 1.0:
            self._pose = dict(self._target)
            self._fsm = FSM_STANDBY


def make_simulated_robot():
    return SimulatedRobot()


class SimulatedRobotError(RobotError):
    """Kept for symmetry: the fake arm never raises in normal use."""
