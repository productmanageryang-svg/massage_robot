"""Shared fixtures. Every test runs against the simulator, never hardware."""

from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


@pytest.fixture(autouse=True)
def _isolate(tmp_path, monkeypatch):
    """Point every writable path at a temp dir so tests never touch the check-out."""
    monkeypatch.setenv("HEACON_RUNTIME_DIR", str(tmp_path / "runtime"))
    monkeypatch.setenv("HEACON_CONFIG_DIR", str(REPO_ROOT / "config"))
    monkeypatch.setenv("HEACON_LOG_DIR", str(tmp_path / "logs"))
    monkeypatch.setenv("HEACON_SIMULATE", "1")

    # config is imported once; reload so the env vars above take effect.
    import importlib

    import heacon.config

    importlib.reload(heacon.config)
    yield
    monkeypatch.undo()
    importlib.reload(heacon.config)


@pytest.fixture
def robot():
    from heacon.robot import open_robot

    return open_robot()
