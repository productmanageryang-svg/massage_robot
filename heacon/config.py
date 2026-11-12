"""Every path, host and port the system uses lives here.

Nothing else in the codebase may hard-code an absolute path, a robot address
or a port. Anything that differs between the Mac on site and a Ubuntu
deployment is an environment variable with a sane default.

Environment variables
---------------------
HEACON_ROBOT_HOST      robot controller IP            (default 192.168.0.10)
HEACON_ROBOT_PORT      robot command port             (default 10003)
HEACON_HTTP_HOST       console bind address           (default 0.0.0.0)
HEACON_HTTP_PORT       console port                   (default 8765)
HEACON_HTTPS_PORT      tablet HTTPS port              (default 8766)
HEACON_RUNTIME_DIR     writable scratch dir           (default <repo>/runtime)
HEACON_CONFIG_DIR      calibration and taught poses   (default <repo>/config)
HEACON_WEB_DIR         page bundle served on 8765     (default <repo>/web)
HEACON_CAMERA_BACKEND  auto | macos | linux | none    (default auto)
HEACON_LOG_DIR         where service logs go          (default <runtime>/logs)
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def _text(name: str, default: str) -> str:
    value = (os.environ.get(name) or "").strip()
    return value or default


def _number(name: str, default: int) -> int:
    raw = (os.environ.get(name) or "").strip()
    if not raw:
        return default
    try:
        return int(raw)
    except ValueError:
        return default


def _flag(name: str, default: bool = False) -> bool:
    raw = (os.environ.get(name) or "").strip().lower()
    if not raw:
        return default
    return raw not in ("0", "false", "no", "off", "")


# --- directories -----------------------------------------------------------

RUNTIME_DIR = Path(_text("HEACON_RUNTIME_DIR", str(REPO_ROOT / "runtime")))
CONFIG_DIR = Path(_text("HEACON_CONFIG_DIR", str(REPO_ROOT / "config")))
WEB_DIR = Path(_text("HEACON_WEB_DIR", str(REPO_ROOT / "web")))
LOG_DIR = Path(_text("HEACON_LOG_DIR", str(RUNTIME_DIR / "logs")))
MODEL_DIR = Path(_text("HEACON_MODEL_DIR", str(RUNTIME_DIR / "models" / "vosk-small-cn")))
#: MediaPipe pose/hand .task files used by heacon.vision.hybrid_geom.
MODEL_TASKS_DIR = Path(_text("HEACON_MODEL_TASKS_DIR", str(RUNTIME_DIR / "models")))

# --- network ---------------------------------------------------------------

ROBOT_HOST = _text("HEACON_ROBOT_HOST", "192.168.0.10")
ROBOT_PORT = _number("HEACON_ROBOT_PORT", 10003)
HTTP_HOST = _text("HEACON_HTTP_HOST", "0.0.0.0")
HTTP_PORT = _number("HEACON_HTTP_PORT", 8765)
HTTPS_HOST = _text("HEACON_HTTPS_HOST", "0.0.0.0")
HTTPS_PORT = _number("HEACON_HTTPS_PORT", 8766)

#: The colour camera runs as its own small HTTP service so the console never
#: competes with it for the USB device. On the NUC both live on one machine.
CAMERA_SERVER_HOST = _text("HEACON_CAMERA_HOST", "127.0.0.1")
CAMERA_SERVER_PORT = _number("HEACON_CAMERA_PORT", 8771)

# --- behaviour -------------------------------------------------------------

#: "macos" needs the root capture daemon; "linux" talks to the D435i directly.
CAMERA_BACKEND = _text("HEACON_CAMERA_BACKEND", "auto").lower()


def platform_tag() -> str:
    if sys.platform == "darwin":
        return "macos"
    if sys.platform.startswith("linux"):
        return "linux"
    return "other"


def camera_backend() -> str:
    """"none" when no colour capture is possible on this host."""
    if CAMERA_BACKEND in ("macos", "linux", "none"):
        return CAMERA_BACKEND
    tag = platform_tag()
    return tag if tag in ("macos", "linux") else "none"


# --- files the running system reads and writes -----------------------------

#: Written by the capture daemon, read by the console. Both processes must
#: agree, so they take the paths from here instead of naming files themselves.
COLOR_LIVE = RUNTIME_DIR / "color-live.jpg"
DEPTH_LIVE = RUNTIME_DIR / "depth-live.png"
COLOR_CALIB_LIVE = RUNTIME_DIR / "color-calib.json"
CAMERA_STATUS = RUNTIME_DIR / "camera-status.json"
CAMERA_LOG = LOG_DIR / "camera.log"

CERTS_DIR = RUNTIME_DIR / "certs"
TLS_CERT = CERTS_DIR / "heacon.crt"
TLS_KEY = CERTS_DIR / "heacon.key"


def _first_existing(*candidates: Path) -> Path:
    for path in candidates:
        if path.is_file():
            return path
    return candidates[-1]


#: Hand-eye calibration. The real file is machine specific, so the repo only
#: ships ``*.example.json``; setup copies it and the field values replace it.
HANDEYE = _first_existing(CONFIG_DIR / "handeye.json", CONFIG_DIR / "handeye.example.json")
COLOR_CALIB = _first_existing(
    CONFIG_DIR / "color_calib.json", CONFIG_DIR / "color_calib.example.json"
)
WORK_HOME = _first_existing(CONFIG_DIR / "work_home.json", CONFIG_DIR / "work_home.example.json")


def ensure_dirs() -> None:
    """Create the writable directories. Safe to call repeatedly."""
    for path in (RUNTIME_DIR, LOG_DIR, CERTS_DIR, CONFIG_DIR):
        path.mkdir(parents=True, exist_ok=True)


def describe() -> dict:
    """Printed on boot so a field engineer can see what this process resolved."""
    return {
        "robot": "%s:%d" % (ROBOT_HOST, ROBOT_PORT),
        "camera_server": "%s:%d" % (CAMERA_SERVER_HOST, CAMERA_SERVER_PORT),
        "console": "%s:%d" % (HTTP_HOST, HTTP_PORT),
        "tablet_https": "%s:%d" % (HTTPS_HOST, HTTPS_PORT),
        "runtime_dir": str(RUNTIME_DIR),
        "config_dir": str(CONFIG_DIR),
        "web_dir": str(WEB_DIR),
        "camera_backend": camera_backend(),
    }
