#!/usr/bin/env python3
"""One entry point for everything the field machine runs.

    python main.py console    # operator page + control API   (8765)
    python main.py tablet     # HTTPS door for the tablet     (8766)
    python main.py camera     # RealSense capture daemon
    python main.py all        # all three, console in the foreground

On the deployed machine these are separate systemd units (see deploy/systemd)
so one can restart without taking the others down. This script exists so a
developer can run the whole stack from a checkout with one command.
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import heacon.config as config


def _console():
    from heacon.http import app

    app.run()


def _tablet():
    from heacon.http import https_gateway

    https_gateway.run()


def _camera():
    """Colour + depth capture, written to the runtime dir for the console.

    This is what the NUC ran: color_live.py writes color-live.jpg and
    depth-live.png, and the console reads them. The depth image is required by
    acupoint recognition, so this path is not optional.
    """
    from heacon import camera_daemon

    camera_daemon.main()


def _all():
    # The tablet door and the camera both tolerate starting before the console
    # is listening, so they go first and the console keeps the terminal.
    config.ensure_dirs()
    threading.Thread(target=_camera, name="camera", daemon=True).start()
    time.sleep(1.0)
    threading.Thread(target=_tablet, name="tablet", daemon=True).start()
    _console()


COMMANDS = {
    "console": _console,
    "tablet": _tablet,
    "camera": _camera,
    "all": _all,
}


def main() -> int:
    parser = argparse.ArgumentParser(description="HEACON 按摩艾灸机器人控制台")
    parser.add_argument(
        "command",
        nargs="?",
        default="console",
        choices=sorted(COMMANDS),
        help="要启动的进程，默认 console",
    )
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="使用假机械臂，不需要真机也能跑完整流程",
    )
    parser.add_argument("--print-config", action="store_true", help="打印解析后的配置后退出")
    args = parser.parse_args()

    if args.print_config:
        for key, value in config.describe().items():
            print("%-16s %s" % (key, value))
        return 0

    if args.simulate:
        os.environ["HEACON_SIMULATE"] = "1"

    COMMANDS[args.command]()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
