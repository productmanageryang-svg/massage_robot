"""The config layer is what makes the tree portable. Guard its contract."""

from __future__ import annotations

import importlib

import heacon.config as config


def test_no_source_file_hard_codes_a_user_or_tmp_path():
    # The original tree had HTML = Path("/Users/patrickyang/Desktop/...") and
    # /tmp/heacon-* frame paths sprinkled around. Anything like that breaks a
    # clone on another machine, so assert none of it came back.
    import re

    bad = re.compile(r"""["'](/Users/|/tmp/heacon|~/Desktop)""")
    offenders = []
    for path in (config.REPO_ROOT / "heacon").rglob("*.py"):
        for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if bad.search(line) and "HEACON_HID_STUB" not in line:
                offenders.append("%s:%d %s" % (path.name, number, line.strip()))
    assert not offenders, "hard-coded paths:\n" + "\n".join(offenders)


def test_env_overrides_are_honoured(monkeypatch):
    monkeypatch.setenv("HEACON_HTTP_PORT", "9911")
    monkeypatch.setenv("HEACON_ROBOT_HOST", "10.1.2.3")
    importlib.reload(config)
    try:
        assert config.HTTP_PORT == 9911
        assert config.ROBOT_HOST == "10.1.2.3"
    finally:
        monkeypatch.undo()
        importlib.reload(config)


def test_bad_number_falls_back_to_default(monkeypatch):
    monkeypatch.setenv("HEACON_HTTP_PORT", "not-a-number")
    importlib.reload(config)
    try:
        assert config.HTTP_PORT == 8765
    finally:
        monkeypatch.undo()
        importlib.reload(config)


def test_camera_backend_is_a_known_value():
    assert config.camera_backend() in ("macos", "linux", "none")


def test_describe_lists_what_an_engineer_needs():
    described = config.describe()
    for key in ("robot", "console", "tablet_https", "camera_backend"):
        assert key in described


def test_calibration_files_have_repo_defaults():
    # The repo ships examples; the resolver must fall back to them so a fresh
    # clone can boot. Real values still have to be measured on the arm.
    assert config.HANDEYE.is_file(), config.HANDEYE
    assert config.COLOR_CALIB.is_file(), config.COLOR_CALIB
