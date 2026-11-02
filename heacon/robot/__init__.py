"""Pick a real controller or the simulator, so nothing else has to branch."""

from __future__ import annotations

import heacon.config as config
from heacon.robot.client import RobotClient


def open_robot():
    """The console's single robot handle.

    With ``HEACON_SIMULATE=1`` this is a fake arm that behaves like the real
    one. That is what lets a fresh clone boot on Ubuntu, and what CI runs
    against. Otherwise this is a TCP client to the Huayan controller.
    """
    if config.SIMULATE:
        from heacon.robot.simulator import make_simulated_robot

        return make_simulated_robot()
    return RobotClient(host=config.ROBOT_HOST, port=config.ROBOT_PORT)
