"""Local gateway: serves the HEACON page and talks to the robot."""

import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from robot_client import RobotClient, RobotError
import demo_motion
import route_motion
import vision
import voice_tts

ROOT = Path(__file__).resolve().parents[1]
HTML = Path("/Users/patrickyang/Desktop/mmrobot-ui.html")
LINK_JS = ROOT / "public" / "heacon-link.js"
HOST = "0.0.0.0"
PORT = 8765

robot = RobotClient()
_session_on = False
_link_wanted = False
_link_phase = None
_link_lock = threading.Lock()
_link_thread = None
_fail_since = None
_lock_when_idle = True
_last_lock_ok = 0.0
_ROBOT_LIVE = {
    "/api/service/start",
    "/api/position",
    "/api/jog",
    "/api/motion/start",
    "/api/frame",
    "/api/enable",
    "/api/disable",
    "/api/wake",
    "/api/lock",
    "/api/locate",
}


def _offline_state(phase="offline", message="设备已离线"):
    return {
        "connected": False,
        "session": False,
        "live": False,
        "record": False,
        "phase": phase,
        "model": "",
        "joints": None,
        "pose": None,
        "fsm": None,
        "ready": False,
        "message": message,
    }


def _phase_message(phase):
    if phase == "unlocking":
        return "正在解锁…"
    if phase == "recovering":
        return "连接不稳，正在恢复…"
    if phase == "retrying":
        return "还没连上，正在重试…"
    if phase == "connecting":
        return "正在连接机械臂…"
    if phase == "ready":
        return "机械臂已连接"
    return "设备已离线"


def _note_fail():
    global _fail_since, _link_phase
    with _link_lock:
        if _fail_since is None:
            _fail_since = time.time()
        _link_phase = "recovering"


def _clear_fail():
    global _fail_since, _link_phase
    with _link_lock:
        _fail_since = None
        if _link_phase == "recovering":
            _link_phase = None


def _enrich(state):
    pose = state.get("pose") if isinstance(state, dict) else None
    if state.get("connected") and isinstance(pose, dict):
        try:
            state["clearance"] = round(
                vision.height_above_table((pose["x"], pose["y"], pose["z"])), 1
            )
        except (TypeError, ValueError, KeyError):
            pass
    return state


def _public_state():
    if not _session_on and not _link_wanted:
        return _offline_state()
    state = _enrich(robot.state())
    live = bool(
        _session_on
        and state.get("connected")
        and (state.get("ready") is True or state.get("fsm") in (33, 25, 27))
    )
    if _link_wanted and not _session_on:
        phase = _link_phase or "connecting"
        state.update(
            {
                "connected": False,
                "session": False,
                "live": False,
                "record": False,
                "phase": phase,
                "message": _phase_message(phase),
            }
        )
        return state
    if _session_on and not state.get("connected"):
        _note_fail()
        lost = _offline_state("recovering", _phase_message("recovering"))
        lost.update({"session": True, "record": True})
        return lost
    _clear_fail()
    state.update(
        {
            "connected": True,
            "session": True,
            "live": True,
            "record": True,
            "phase": "ready",
        }
    )
    return state


def _connect_loop():
    global _session_on, _link_phase
    while True:
        with _link_lock:
            wanted = _link_wanted
            already = _session_on
        if not wanted or already:
            with _link_lock:
                if not _link_wanted:
                    _link_phase = None
            return
        with _link_lock:
            _link_phase = "connecting"
        state = robot.state()
        if not state.get("connected"):
            with _link_lock:
                _link_phase = "retrying"
            time.sleep(1.2)
            continue
        with _link_lock:
            _link_phase = "unlocking"
        ok, _message = robot.wake()
        with _link_lock:
            if ok and _link_wanted:
                _session_on = True
                _link_phase = None
                return
            _link_phase = "retrying"
        time.sleep(1.2)


_NORMAL_COLLIDE_LEVEL = 3
_collide_level = None


def _want_collide(level):
    """Apply a collide gear once. A failed write is retried next cycle."""
    global _collide_level
    if _collide_level == level:
        return True
    try:
        robot.command(f"SetCollideLevel,0,{int(level)},;")
    except RobotError:
        return False
    _collide_level = level
    return True


def _ensure_offline_lock():
    """Tablet 设备已离线 means the arm must be locked."""
    global _last_lock_ok
    status = route_motion.status()
    if status.get("running") or status.get("positioning"):
        return False
    if _last_lock_ok and time.time() - _last_lock_ok < 6:
        return True
    try:
        ok, message = robot.lock()
    except RobotError as exc:
        print("[heacon] lock:", exc, flush=True)
        return False
    if ok:
        _last_lock_ok = time.time()
        return True
    print("[heacon] lock:", message, flush=True)
    return False


def _start_connect():
    global _link_wanted, _link_thread, _link_phase, _lock_when_idle, _last_lock_ok
    with _link_lock:
        _link_wanted = True
        _lock_when_idle = False
        _last_lock_ok = 0.0
        if _session_on:
            _link_phase = None
            return True
        _link_phase = _link_phase or "connecting"
        alive = _link_thread is not None and _link_thread.is_alive()
    if not alive:
        _link_thread = threading.Thread(target=_connect_loop, name="heacon-connect", daemon=True)
        _link_thread.start()
    return True


def _lock_then_close():
    try:
        _ensure_offline_lock()
    except Exception as exc:
        print("[heacon] lock:", exc, flush=True)
    try:
        robot.close()
    except Exception as exc:
        print("[heacon] close:", exc, flush=True)


def _stop_session(lock_arm=True):
    global _session_on, _link_wanted, _link_phase, _fail_since, _lock_when_idle
    with _link_lock:
        _link_wanted = False
        _session_on = False
        _link_phase = None
        _fail_since = None
        _lock_when_idle = True
    if lock_arm:
        threading.Thread(target=_lock_then_close, name="heacon-lock", daemon=True).start()
        return
    robot.close()


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/" or self.path.startswith("/?"):
            self._file(HTML, "text/html; charset=utf-8")
        elif path == "/heacon-link.js":
            self._file(LINK_JS, "text/javascript; charset=utf-8")
        elif path == "/api/state":
            self._json(_public_state())
        elif path == "/api/camera.jpg":
            self._camera()
        elif path == "/api/coverage":
            self._coverage()
        elif path == "/api/motion/status":
            self._json({"ok": True, **demo_motion.status()})
        elif path == "/api/service/status":
            self._json({"ok": True, **route_motion.status()})
        elif path == "/api/position":
            self._json({"ok": True, **route_motion.position_status()})
        elif path.startswith("/voice/") and path.endswith(".mp3"):
            self._file(Path(__file__).resolve().parent / "voices" / path.split("/")[-1], "audio/mpeg")
        elif path == "/api/voice":
            self._voice()
        else:
            self.send_error(404)

    def do_POST(self):
        actions = {
            "/api/stop": robot.stop,
            "/api/pause": robot.pause,
            "/api/enable": robot.enable,
            "/api/disable": robot.disable,
        }
        if self.path in ("/api/stop", "/api/pause"):
            self._stop_arm()
        elif self.path == "/api/session/connect":
            self._session_connect()
        elif self.path == "/api/session/disconnect":
            self._session_disconnect()
        elif self.path in _ROBOT_LIVE and not _session_on:
            self._json(
                {
                    "ok": False,
                    "live": False,
                    "record": False,
                    "message": "当前是离线模拟，没有写入真实记录",
                },
                409,
            )
        elif self.path == "/api/wake":
            self._wake()
        elif self.path == "/api/lock":
            self._lock()
        elif self.path in actions:
            self._run(actions[self.path])
        elif self.path == "/api/motion/start":
            self._motion("start")
        elif self.path == "/api/motion/stop":
            self._motion("stop")
        elif self.path == "/api/jog":
            self._jog()
        elif self.path == "/api/locate":
            self._locate()
        elif self.path == "/api/service/start":
            self._service_start()
        elif self.path == "/api/service/stop":
            self._service_stop()
        elif self.path == "/api/service/focus":
            self._service_focus()
        elif self.path == "/api/service/level":
            self._service_level()
        elif self.path == "/api/lead/start":
            self._lead_start()
        elif self.path == "/api/lead/stop":
            self._lead_stop()
        elif self.path == "/api/service/preview":
            self._service_preview()
        elif self.path == "/api/service/speed":
            self._service_speed()
        elif self.path == "/api/position":
            self._go_position()
        elif self.path == "/api/frame":
            self._frame()
        else:
            self.send_error(404)

    def _coverage(self):
        import body_route

        if route_motion.holding_frame(robot.state()):
            self._json({"ok": True, "complete": True})
            return
        try:
            frame = vision.camera_bgr()
        except RuntimeError as exc:
            self._json({"ok": False, "complete": False, "message": str(exc)}, 503)
            return
        view = body_route.coverage(frame)
        self._json({"ok": True, "complete": bool(view and view["complete"])})

    def _voice(self):
        from urllib.parse import parse_qs, urlparse

        text = (parse_qs(urlparse(self.path).query).get("text") or [""])[0]
        path = voice_tts.clip_path(text)
        if path is None:
            self._json({"ok": False, "message": "语音没有生成"}, 503)
            return
        self._file(path, "audio/mpeg")

    def _camera(self):
        try:
            frame, state = vision.camera_view_jpeg()
        except RuntimeError as exc:
            frame, state = b"", "missing"
            print("[heacon] camera view:", exc, flush=True)
        if not frame:
            self.send_error(503, "摄像头还没有画面")
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(frame)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Heacon-Camera", state)
        self.end_headers()
        self.wfile.write(frame)

    def _locate(self):
        try:
            raw = self.rfile.read(int(self.headers.get("Content-Length", "0")) or 0)
            body = json.loads(raw.decode("utf-8") or "{}")
            u = float(body["u"])
            v = float(body["v"])
        except (ValueError, KeyError, json.JSONDecodeError):
            self._json({"ok": False, "message": "画面坐标无效"}, 400)
            return
        state = robot.state()
        if not state.get("connected"):
            self._json({"ok": False, "message": state.get("message") or "没有连上机械臂"}, 503)
            return
        try:
            point = vision.locate(state["joints"], state["pose"], u, v)
        except (ValueError, KeyError) as exc:
            self._json({"ok": False, "message": str(exc)}, 400)
            return
        self._json({"ok": True, **point})

    def _jog(self):
        try:
            raw = self.rfile.read(int(self.headers.get("Content-Length", "0")) or 0)
            body = json.loads(raw.decode("utf-8") or "{}")
            axis = int(body["axis"])
            direction = int(body["direction"])
        except (ValueError, KeyError, json.JSONDecodeError):
            self._json({"ok": False, "message": "点动参数无效"}, 400)
            return
        self._run(lambda: robot.jog_joint(axis, direction))

    def _stop_arm(self):
        route_was = route_motion.status()["running"]
        route_ok, route_message = route_motion.stop(robot)
        demo_ok, demo_message = demo_motion.stop(robot)
        if route_was:
            self._json({"ok": route_ok, "message": route_message}, 200 if route_ok else 409)
            return
        self._json({"ok": demo_ok, "message": demo_message}, 200 if demo_ok else 409)

    def _motion(self, action):
        ok, message = demo_motion.start(robot) if action == "start" else demo_motion.stop(robot)
        self._json({"ok": ok, "message": message}, 200 if ok else 409)

    def _service_start(self):
        body = self._read_json()
        if body is None:
            self._json({"ok": False, "message": "方案参数无效"}, 400)
            return
        ok, message = route_motion.start(
            robot,
            str(body.get("plan") or ""),
            body.get("points"),
            fresh=bool(body.get("fresh", True)),
            segments=body.get("segments"),
            level=body.get("level", 1),
            duration=body.get("duration"),
            speed=body.get("speed", 1),
            width=body.get("width"),
            height=body.get("height"),
        )
        self._json({"ok": ok, "message": message, **route_motion.status()}, 200 if ok else 409)

    def _service_stop(self):
        ok, message = route_motion.stop(robot)
        self._json({"ok": ok, "message": message, **route_motion.status()}, 200 if ok else 409)

    def _lead_start(self):
        ok, message = route_motion.start_lead_preview(robot)
        self._json({**route_motion.status(), "ok": False, "message": message}, 409)

    def _lead_stop(self):
        ok, message = route_motion.stop(robot)
        self._json({"ok": ok, "message": message, **route_motion.status()}, 200 if ok else 409)

    def _frame(self):
        try:
            done, message = route_motion.frame_body(robot)
        except RobotError as exc:
            self._json({"ok": False, "done": True, "message": str(exc)}, 409)
            return
        self._json({"ok": True, "done": done, "message": message})

    def _service_preview(self):
        body = self._read_json()
        if body is None:
            self._json({"ok": False, "message": "方案参数无效"}, 400)
            return
        try:
            payload = route_motion.preview(str(body.get("plan") or ""), body.get("points"))
            route_motion.remember_scan_pose(robot.state())
        except RobotError as exc:
            self._json({"ok": False, "message": str(exc)}, 409)
            return
        self._json({"ok": True, **payload})

    def _service_focus(self):
        body = self._read_json()
        if body is None:
            self._json({"ok": False, "message": "穴位序号无效"}, 400)
            return
        try:
            index = int(body.get("index"))
        except (TypeError, ValueError):
            self._json({"ok": False, "message": "穴位序号无效"}, 400)
            return
        route_motion.focus(index)
        self._json({"ok": True, **route_motion.status()})

    def _go_position(self):
        ok, message = route_motion.go_position(robot)
        self._json(
            {"ok": ok, "message": message, **route_motion.position_status()},
            200 if ok else 409,
        )

    def _service_speed(self):
        body = self._read_json()
        if body is None:
            self._json({"ok": False, "message": "倍速无效"}, 400)
            return
        speed = route_motion.set_speed(body.get("speed", 1))
        self._json({"ok": True, "speed": speed, **route_motion.status()})

    def _service_level(self):
        body = self._read_json()
        if body is None:
            self._json({"ok": False, "message": "力度或温度档位无效"}, 400)
            return
        if body.get("delta") is not None:
            result = route_motion.nudge_level(body.get("delta"))
        else:
            result = route_motion.set_level(body.get("grade", body.get("level", 2)))
        self._json({"ok": True, **result, **route_motion.status()})

    def _read_json(self):
        try:
            length = int(self.headers.get("Content-Length", "0") or 0)
        except ValueError:
            return None
        if length < 0 or length > 20000:
            return None
        raw = self.rfile.read(length) if length else b""
        if not raw:
            return {}
        try:
            data = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError:
            return None
        return data if isinstance(data, dict) else None

    def _session_connect(self):
        _start_connect()
        deadline = time.time() + 4.0
        while time.time() < deadline:
            if _session_on:
                break
            time.sleep(0.08)
        phase = _link_phase or ("ready" if _session_on else "connecting")
        state = _public_state()
        message = state.get("message") or _phase_message(phase)
        if state.get("live"):
            message = "机械臂已连接"
        self._json(
            {
                **state,
                "ok": True,
                "message": message,
            }
        )

    def _session_disconnect(self):
        status = route_motion.status()
        if status.get("running") or status.get("positioning"):
            self._json({**_public_state(), "ok": False, "message": "服务进行中，不能断开"}, 409)
            return
        _stop_session(lock_arm=True)
        self._json({**_offline_state(), "ok": True, "message": "已断开连接"})

    def _wake(self):
        _start_connect()
        ok, message = robot.wake()
        state = _public_state()
        self._json({**state, "ok": bool(ok), "message": message}, 200 if ok else 409)

    def _lock(self):
        status = route_motion.status()
        if status.get("running") or status.get("positioning"):
            state = _public_state()
            self._json({**state, "ok": False, "message": "服务进行中，不能断开"}, 409)
            return
        _stop_session(lock_arm=True)
        self._json({**_offline_state(), "ok": True, "message": "已断开连接"})

    def _run(self, action):
        try:
            reply = action()
        except RobotError as exc:
            self._json({"ok": False, "message": str(exc), "reply": exc.reply})
            return
        self._json({"ok": True, "message": "机械臂已接受", "reply": reply})

    def _json(self, payload, status=200):
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _file(self, path, content_type):
        if not path.is_file():
            self.send_error(404)
            return
        data = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, fmt, *args):
        if args and str(args[0]).startswith("GET /api/state"):
            return
        super().log_message(fmt, *args)


def _hold_normal():
    """Standby only. Force control stays off."""
    try:
        if robot.read_ft_state():
            robot.force_control_off()
    except RobotError:
        pass
    _want_collide(_NORMAL_COLLIDE_LEVEL)
    return robot.wake()


def _keep_awake():
    last = None
    while True:
        try:
            status = route_motion.status()
            if _link_wanted and not _session_on:
                note = ("link", _phase_message(_link_phase or "connecting"))
            elif _session_on:
                if status.get("running") or status.get("positioning"):
                    note = ("busy", "服务进行中")
                else:
                    state = robot.state()
                    if state.get("connected"):
                        _clear_fail()
                        ok, message = _hold_normal()
                        note = ("ready" if ok else "wait", message)
                    else:
                        _note_fail()
                        note = ("recover", _phase_message("recovering"))
            else:
                _want_collide(_NORMAL_COLLIDE_LEVEL)
                if _lock_when_idle:
                    _ensure_offline_lock()
                robot.close()
                note = ("idle", "设备已离线")
            if note != last:
                print("[heacon] arm", note[0] + ":", note[1], flush=True)
                last = note
        except Exception as exc:
            print("[heacon] arm keep-awake:", exc, flush=True)
            last = None
        time.sleep(2)


if __name__ == "__main__":
    try:
        route_motion.hold_frame({"joints": route_motion.POSITION_JOINTS})
    except Exception:
        pass
    threading.Thread(target=_keep_awake, name="heacon-keep-awake", daemon=True).start()
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"HEACON 控制台：http://127.0.0.1:{PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n已停止")
