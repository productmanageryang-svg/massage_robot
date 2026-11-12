"""Opening the robot connection.

There is one path: TCP to the Huayan controller. No simulation, no fallback.
The field build this mirrors never had one, and an unverified extra code path
in a machine that drives a real arm is not worth the convenience.
"""

from __future__ import annotations

import heacon.config as config
from heacon.robot.client import RobotClient


def open_robot():
    """The console's single robot handle."""
    return RobotClient(host=config.ROBOT_HOST, port=config.ROBOT_PORT)
