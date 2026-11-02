"""Prove the repo's force path issues byte-identical robot commands to 第三版.

Code that looks the same is not proof. This harness drives *both* the 第三版
module (extracted from the frozen zip) and the current package with the same
fake arm at the same pose, then compares the exact command strings each one
sends. Any drift in tuning, ordering, or arithmetic shows up as a diff.

Run: python scripts/compare_v3_force_path.py <path-to-第三版>/mmrobot-web
"""

from __future__ import annotations

import json
import subprocess
import sys
import textwrap
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]

# Runs inside a subprocess so each side gets clean module stubs.
_CHILD = r'''
import json, sys, types
from pathlib import Path

MODE = sys.argv[1]
V3_BRIDGE = sys.argv[2] if len(sys.argv) > 2 else ""

POSE = {"x": -560.256, "y": -69.074, "z": 273.544,
        "rx": -179.088, "ry": -4.403, "rz": 168.238}
JOINTS = [7.106, 14.041, -30.604, -0.111, -134.191, 4.470]

class Recorder:
    """Fake arm: records every command, answers reads deterministically."""
    host = "rec"; port = 0
    def __init__(self):
        self.log = []
        self._ft = 0
        self._fsm = 33
        self._goal = {"fx":0,"fy":0,"fz":0,"mx":0,"my":0,"mz":0}
    def command(self, message, timeout=None):
        self.log.append(message)
        return ",%s,0,OK,;" % message.split(",")[0].lstrip(", ")
    def state(self):
        return {"connected": True, "model": "rec", "joints": list(JOINTS),
                "pose": dict(POSE), "fsm": self._fsm, "ready": True, "message": ""}
    def read_pose(self):
        return {"joints": list(JOINTS), "pose": dict(POSE)}
    def read_fsm(self):
        return self._fsm
    def read_ft_state(self):
        return self._ft
    def read_force(self):
        return {"fx":0.0,"fy":0.0,"fz":0.0,"mx":0.0,"my":0.0,"mz":0.0}
    def read_force_calibrated(self):
        return self.read_force()
    def force_control_on(self):
        self._ft = 1
        return ",SetForceControlState,0,OK,;"
    def force_control_off(self):
        self._ft = 0
        return ",SetForceControlState,0,OK,;"
    def set_force_zero(self):
        return self.command("SetForceZero,0,;")
    def set_ft_goal(self, fx=0, fy=0, fz=0, mx=0, my=0, mz=0):
        self._goal = {"fx":fx,"fy":fy,"fz":fz,"mx":mx,"my":my,"mz":mz}
        return self.command(
            "HRSetControlGoal,0,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,0,0,0,0,0,0,;"
            % (fx, fy, fz, mx, my, mz))
    # remaining RobotClient surface, recorded the same way on both sides
    def __getattr__(self, name):
        if name.startswith("_"):
            raise AttributeError(name)
        def _any(*a, **k):
            return self.command("%s,0,;".strip() % name)
        return _any

def install_vision_stubs():
    """Same numbers for both sides, so only motion logic can differ."""
    import numpy as np
    frame = np.zeros((720, 1280, 3), np.uint8)
    frame[120:600, 510:770] = (168,172,178)   # torso band
    frame[200:520, 380:480] = (160,164,170)
    frame[200:520, 800:900] = (160,164,170)

    vis = types.ModuleType("vision")
    vis.height_above_table = lambda p: 40.0
    vis.up_vector = lambda: np.array([0.0, 0.0, 1.0])
    vis.camera_bgr = lambda: frame.copy()
    vis.camera_jpeg = lambda plain=True: b"\xff\xd8\xff" + b"0"*2000
    vis.note_frame_size = lambda *a, **k: (1280, 720)
    vis.ensure_camera = lambda: None
    vis.depth_mm = lambda: None
    vis.load_handeye = lambda: {
        "camera": np.eye(4),
        "matrix": np.array([[900.0,0,640.0],[0,900.0,360.0],[0,0,1.0]]),
        "dist": np.zeros(5),
        "plane_point": np.array([0.0,0.0,0.0]),
        "plane_normal": np.array([0.0,0.0,-1.0]),
        "width": 1280.0, "height": 720.0,
    }
    vis.link_pose = lambda pose, q6: np.eye(4)
    vis._rotation = lambda rx, ry, rz: np.eye(3)
    vis.camera_basis = lambda joints, pose: (
        np.zeros(3), np.array([1.0,0,0]), np.array([0,1.0,0]), np.array([0,0,-1.0]))
    vis.focal_length = lambda: 900.0
    vis.raw_pixel = lambda u, v, w=None, h=None: (float(u), float(v))
    def _project(joints, pose, u, v, lift, w=None, h=None):
        # deterministic mapping from pixel to a robot pose at the given lift
        return {"x": -560.0 + (float(u) - 640.0) * 0.5,
                "y": -69.0 + (float(v) - 360.0) * 0.5,
                "z": float(lift),
                "rx": POS["rx"], "ry": POS["ry"], "rz": POS["rz"]}
    POS = POSE
    vis.project = _project
    vis.locate = lambda joints, pose, u, v: (0.0, 0.0, 0.0)

    body = types.ModuleType("body_route")
    body.NECK_Y = 210.0; body.HIP_Y = 560.0
    body.LEFT_X = 140.0; body.RIGHT_X = 400.0
    body.find_body = lambda img: _body()
    body.median_body = lambda bodies: bodies[0] if bodies else None
    body.place_points = lambda raw, b, w, h: [(float(r[1]), float(r[2])) for r in raw]
    body.coverage = lambda img: {"complete": True, "width": 1280, "height": 720}
    def _body():
        return {"neck": (640.0, 200.0), "hip": (640.0, 560.0),
                "right": (1.0, 0.0), "width": 300.0,
                "head": (640.0, 120.0), "center": (640.0, 380.0)}
    return vis, body

vis_mod, body_mod = install_vision_stubs()

if MODE == "v3":
    sys.path.insert(0, V3_BRIDGE)
    sys.modules["vision"] = vis_mod
    sys.modules["body_route"] = body_mod
    import route_motion as R
    mod = R
else:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]) if False else "REPO_ROOT")
    from heacon.vision import geometry, body
    for name in dir(vis_mod):
        if not name.startswith("__"):
            setattr(geometry, name, getattr(vis_mod, name))
    for name in dir(body_mod):
        if not name.startswith("__"):
            setattr(body, name, getattr(body_mod, name))
    from heacon.motion import route as R
    mod = R

out = {}

# 1) Which force controller does each side pick for a moxa plan?
mod._plan_id = "p1"
rec = Recorder()

# 2) Enabling force control: exactly what happens at work height.
try:
    mod._hybrid_contact_on(rec)
except Exception as exc:
    out["contact_on_error"] = "%s: %s" % (type(exc).__name__, exc)

out["contact_on_commands"] = list(rec.log)

# 3) The restoring spring: same fixed offset must yield the same goal force.
mod._hybrid_home = dict(POSE)
mod._hybrid_xd = dict(POSE)
rec2 = Recorder()
mod._hybrid_note_force = lambda r, _orig=mod._hybrid_note_force: None
try:
    mod._hybrid_moxa_spring(rec2)
except Exception as exc:
    out["spring_error"] = "%s: %s" % (type(exc).__name__, exc)
out["spring_commands"] = list(rec2.log)

# 4) The tuning constants themselves.
out["constants"] = {
    k: getattr(mod, k) for k in dir(mod)
    if k.startswith("HYBRID_") and not callable(getattr(mod, k))
}

# 5) Gesture pose maths: same inputs, same pose out.
try:
    base = dict(POSE)
    mod._origin = dict(POSE)
    out["gesture_poses"] = [
        {k: round(float(v), 6) for k, v in mod._gesture_pose(
            base, ry=10.0, lift=40.0).items()},
        {k: round(float(v), 6) for k, v in mod._gesture_pose(
            base, rx=5.0, rz=-15.0, lift=25.0).items()},
    ]
except Exception as exc:
    out["gesture_error"] = "%s: %s" % (type(exc).__name__, exc)

print("@@JSON@@" + json.dumps(out, ensure_ascii=False, default=str))
'''


def _run(mode: str, v3_bridge: Path | None) -> dict:
    code = textwrap.dedent(_CHILD).replace("REPO_ROOT", repr(str(REPO)))
    args = [sys.executable, "-c", code, mode]
    args.append(str(v3_bridge) if v3_bridge else "")
    proc = subprocess.run(args, capture_output=True, text=True, timeout=180, cwd=str(REPO))
    for line in proc.stdout.splitlines():
        if line.startswith("@@JSON@@"):
            return json.loads(line[len("@@JSON@@"):])
    raise SystemExit(
        "side %s produced no result\n--- stdout ---\n%s\n--- stderr ---\n%s"
        % (mode, proc.stdout[-2000:], proc.stderr[-3000:])
    )


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    v3_bridge = Path(sys.argv[1]).resolve()
    if not (v3_bridge / "route_motion.py").is_file():
        raise SystemExit("找不到 %s/route_motion.py" % v3_bridge)

    print("第三版 :", v3_bridge)
    print("当前库 :", REPO / "heacon" / "motion" / "route.py")
    print("=" * 70)

    old = _run("v3", v3_bridge)
    new = _run("new", None)

    failures = 0

    def compare(label: str, a, b) -> None:
        nonlocal failures
        if a == b:
            print("[SAME] %s" % label)
            return
        failures += 1
        print("[DIFF] %s" % label)
        if isinstance(a, list) and isinstance(b, list):
            only_a = [x for x in a if x not in b]
            only_b = [x for x in b if x not in a]
            if only_a:
                print("   只在第三版:")
                for x in only_a[:10]:
                    print("     ", x)
            if only_b:
                print("   只在当前库:")
                for x in only_b[:10]:
                    print("     ", x)
            if not only_a and not only_b:
                print("   顺序不同")
        else:
            print("   第三版:", json.dumps(a, ensure_ascii=False, default=str)[:600])
            print("   当前库:", json.dumps(b, ensure_ascii=False, default=str)[:600])

    compare("力控启用的命令序列", old.get("contact_on_commands"), new.get("contact_on_commands"))
    compare("回弹刚度：同样的偏移产生同样的目标力", old.get("spring_commands"), new.get("spring_commands"))
    compare("力控调参", old.get("constants"), new.get("constants"))
    compare("手法姿态计算", old.get("gesture_poses"), new.get("gesture_poses"))

    for side, data in (("第三版", old), ("当前库", new)):
        for key in ("contact_on_error", "spring_error", "gesture_error"):
            if key in data:
                print("[ERROR] %s %s: %s" % (side, key, data[key]))
                failures += 1

    print("=" * 70)
    if failures:
        print("结果：%d 处不一致——力位混合和第三版有行为差异。" % failures)
        return 1
    print("结果：力控路径和第三版逐条命令一致。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
