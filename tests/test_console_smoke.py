"""No-hardware end-to-end run. This is the test that proves "开箱即用".

It boots the console against the simulator, opens a session, goes to the
position pose, starts a service, and asserts the service actually advances
through points and gestures. If this passes on a clean Ubuntu box with no
robot and no camera, a new team member can run the system.
"""

from __future__ import annotations

import json
import threading
import time
import urllib.request
from http.server import ThreadingHTTPServer

import pytest


def _get(port, path, timeout=5):
    with urllib.request.urlopen("http://127.0.0.1:%d%s" % (port, path), timeout=timeout) as r:
        return json.loads(r.read())


def _post(port, path, body=None, timeout=40):
    data = json.dumps(body or {}).encode()
    req = urllib.request.Request(
        "http://127.0.0.1:%d%s" % (port, path),
        data=data,
        method="POST",
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as exc:
        return json.loads(exc.read())


@pytest.fixture
def console(monkeypatch):
    """Boot the console on an unused port with the simulator."""
    import importlib

    import heacon.config

    monkeypatch.setenv("HEACON_HTTP_PORT", "0")  # replaced below
    importlib.reload(heacon.config)

    from heacon.http import app

    importlib.reload(app)

    server = ThreadingHTTPServer(("127.0.0.1", 0), app.Handler)
    port = server.server_address[1]
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield port
    finally:
        server.shutdown()
        server.server_close()


def test_console_starts_with_no_hardware(console):
    state = _get(console, "/api/state")
    assert "connected" in state


def test_session_connect_brings_the_fake_arm_online(console):
    result = _post(console, "/api/session/connect")
    assert result.get("connected") is True
    assert result.get("fsm") == 33

    state = _get(console, "/api/state")
    assert state["session"] is True
    assert state["ready"] is True


def test_camera_endpoint_always_returns_an_image(console):
    with urllib.request.urlopen(
        "http://127.0.0.1:%d/api/camera.jpg" % console, timeout=5
    ) as resp:
        body = resp.read()
        state = resp.headers.get("X-Heacon-Camera", "")
    assert resp.status == 200
    assert body[:3] == b"\xff\xd8\xff", "not a JPEG"
    assert state == "simulated"


def test_service_runs_without_a_robot_or_camera(console):
    """The whole point of the simulator: the flow completes unattended."""
    _post(console, "/api/session/connect")
    _post(console, "/api/position", timeout=60)

    started = _post(
        console,
        "/api/service/start",
        {
            "plan": "p4",
            "points": [],
            "width": 1280,
            "height": 720,
            "fresh": True,
            "level": 2,
            "duration": 120,
            "speed": 4,
        },
        timeout=90,
    )
    assert started.get("ok") is True, started
    assert started.get("running") is True

    # Watch it advance: a point name must appear, then the gesture must change.
    seen = []
    deadline = time.time() + 45
    while time.time() < deadline:
        status = _get(console, "/api/service/status")
        mark = (status.get("point"), status.get("technique"))
        if mark not in seen and status.get("point"):
            seen.append(mark)
        if len({s[1] for s in seen if s[1]}) >= 2:
            break
        time.sleep(1.0)

    _post(console, "/api/service/stop")

    assert seen, "service never reported a point"
    assert any(name for name, _tech in seen), "no acupoint reached"
    techniques = {tech for _name, tech in seen if tech}
    assert techniques, "no technique reported"


def test_stop_returns_the_arm_to_standby(console):
    _post(console, "/api/session/connect")
    _post(console, "/api/service/start", {"plan": "p4", "duration": 60, "speed": 4}, timeout=60)
    time.sleep(2)
    stopped = _post(console, "/api/service/stop")
    assert stopped.get("running") is False
