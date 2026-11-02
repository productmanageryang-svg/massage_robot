"""Small client for the Huayan robot command port."""

import socket
import threading
import time


class RobotError(Exception):
    def __init__(self, message, reply=""):
        super().__init__(message)
        self.reply = reply


class RobotClient:
    def __init__(self, host="192.168.0.10", port=10003, timeout=2.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._sock = None
        self._lock = threading.Lock()
        self._model = ""
        self._last_fail_at = 0.0
        self._fail_cache = 0.8
        self._last_error = ""
        self._fsm = None

    def close(self):
        sock = self._sock
        self._sock = None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def state(self):
        now = time.time()
        if self._sock is None and self._last_fail_at and now - self._last_fail_at < self._fail_cache:
            return self._offline(self._last_error)

        try:
            if not self._model:
                model_reply = self.command("ReadRobotModel,;", timeout=1.2)
                self._model = _field(model_reply, 2)
            pose_reply = self.command("ReadActPos,0,;", timeout=1.2)
            parts = _parts(pose_reply)
            numbers = [float(item) for item in parts[2:14]]
            joints = numbers[:6]
            pose = numbers[6:12]
            try:
                self._fsm = self.read_fsm()
            except RobotError:
                self._fsm = None
            return {
                "connected": True,
                "model": self._model,
                "joints": joints,
                "pose": {
                    "x": pose[0],
                    "y": pose[1],
                    "z": pose[2],
                    "rx": pose[3],
                    "ry": pose[4],
                    "rz": pose[5],
                },
                "fsm": self._fsm,
                "ready": self._fsm == 33,
                "message": "" if self._fsm == 33 else _fsm_message(self._fsm),
            }
        except (OSError, RobotError, ValueError) as exc:
            self._last_fail_at = time.time()
            self._fail_cache = 0.8
            self._last_error = str(exc)
            self._model = ""
            self._fsm = None
            self.close()
            return self._offline(self._last_error)

    def stop(self):
        return self.command("GrpStop,0,;")

    def pause(self):
        return self.command("GrpInterrupt,0,;")

    def enable(self):
        return self.command("GrpEnable,0,;")

    def wake(self):
        """Connect and enable after power-on. Does not move the arm."""
        state = self.state()
        if not state.get("connected"):
            return False, state.get("message") or "没有连上机械臂"
        try:
            fsm = self.read_fsm()
        except RobotError as exc:
            return False, str(exc)
        if fsm in (20, 21, 22):
            try:
                self.command("GrpReset,0,;")
            except RobotError:
                pass
            time.sleep(0.3)
            try:
                fsm = self.read_fsm()
            except RobotError as exc:
                return False, str(exc)
            if fsm == 20:
                try:
                    self.command("XToStandby,0,;")
                except RobotError:
                    pass
                deadline = time.time() + 6
                while time.time() < deadline:
                    try:
                        fsm = self.read_fsm()
                        if fsm != 20:
                            break
                    except RobotError:
                        pass
                    time.sleep(0.2)
        if fsm in (7, 24):
            try:
                self.enable()
            except RobotError as exc:
                return False, str(exc)
            deadline = time.time() + 8
            while time.time() < deadline:
                try:
                    if self.read_fsm() == 33:
                        return True, "机械臂已就绪"
                except RobotError:
                    pass
                time.sleep(0.2)
            return False, "机械臂已通电，还没有进入待机"
        if fsm == 33:
            return True, "机械臂已就绪"
        if fsm in (25, 27):
            return True, "机械臂已连接"
        return False, _fsm_message(fsm)

    def disable(self):
        return self.command("GrpDisable,0,;")

    def lock(self):
        """Disable the arm after the operator confirms. Does not move it."""
        try:
            if self.read_ft_state() not in (0, None):
                self.force_control_off()
        except RobotError:
            pass
        state = self.state()
        if not state.get("connected"):
            return False, state.get("message") or "没有连上机械臂"
        try:
            fsm = self.read_fsm()
        except RobotError as exc:
            return False, str(exc)
        if fsm in (7, 24):
            return True, "机械臂已锁定"
        if fsm in (25, 27):
            return False, "机械臂正在运动，不能锁定"
        if fsm in (21, 22):
            return False, "机械臂停在故障状态，先恢复后再锁定"
        try:
            self.disable()
        except RobotError as exc:
            return False, str(exc)
        deadline = time.time() + 8
        while time.time() < deadline:
            try:
                if self.read_fsm() == 24:
                    return True, "机械臂已锁定"
            except RobotError:
                pass
            time.sleep(0.2)
        return False, "机械臂还没有进入锁定"

    def jog_joint(self, axis, direction):
        if axis not in range(6) or direction not in (0, 1):
            raise RobotError("点动参数无效")
        # ShortJogJ is rejected after the robot is enabled. A 2° relative move is allowed.
        return self.command(f"MoveRelJ,0,{axis},{direction},2,;", timeout=4)

    def read_force(self):
        """Six-axis force at the flange: Fx Fy Fz Mx My Mz."""
        return self._read_wrench("ReadForceData,0,;")

    def read_force_calibrated(self):
        """Calibrated six-axis force after the controller payload offset."""
        return self._read_wrench("ReadFTCabData,0,;")

    def read_ft_state(self):
        reply = self.command("ReadFTControlState,0,;", timeout=1.0)
        parts = _parts(reply)
        for part in parts[2:]:
            if part.lstrip("-").isdigit():
                return int(part)
        raise RobotError(f"读不到力控状态：{reply}", reply)

    def force_control_on(self):
        return self.command("SetForceControlState,0,1,;")

    def force_control_off(self):
        try:
            self.command("SetFTFreeDriveState,0,0,;")
        except RobotError:
            pass
        try:
            return self.command("SetForceControlState,0,0,;")
        except RobotError:
            return ""

    def set_ft_free_drive(self, on):
        return self.command(f"SetFTFreeDriveState,0,{1 if on else 0},;")

    def set_ft_motion_freedom(self, x, y, z, rx=0, ry=0, rz=0):
        return self.command(
            f"SetFTMotionFreedom,0,{int(bool(x))},{int(bool(y))},{int(bool(z))},{int(bool(rx))},{int(bool(ry))},{int(bool(rz))},;"
        )

    def set_ft_search_freedom(self, x, y, z, rx=0, ry=0, rz=0):
        return self.command(
            f"HRSetControlFreedom,0,{int(bool(x))},{int(bool(y))},{int(bool(z))},{int(bool(rx))},{int(bool(ry))},{int(bool(rz))},;"
        )

    def set_ft_strategy(self, mode):
        return self.command(f"HRSetForceControlStrategy,0,{int(mode)},;")

    def set_ft_tool_frame(self, on):
        return self.command(f"SetForceToolCoordinateMotion,0,{1 if on else 0},;")

    def set_force_zero(self):
        return self.command("SetForceZero,0,;")

    def set_ft_goal(self, fx=0, fy=0, fz=0, mx=0, my=0, mz=0):
        return self.command(
            f"HRSetControlGoal,0,{float(fx):.2f},{float(fy):.2f},{float(fz):.2f},{float(mx):.2f},{float(my):.2f},{float(mz):.2f},0,0,0,0,0,0,;"
        )

    def set_ft_distance_limit(self, distance_mm, strength=2):
        return self.command(
            f"HRSetForceDistanceLimit,0,{float(distance_mm):.1f},{int(strength)},;"
        )

    def set_ft_data_limit(self, force_n=500, torque_nm=50):
        """Raise the FT protection band. Over this the controller faults."""
        force = float(force_n)
        torque = float(torque_nm)
        return self.command(
            "HRSetForceDataLimit,0,"
            f"{force:.1f},{force:.1f},{force:.1f},"
            f"{torque:.1f},{torque:.1f},{torque:.1f},"
            f"{force:.1f},{force:.1f},{force:.1f},"
            f"{torque:.1f},{torque:.1f},{torque:.1f},;"
        )

    def set_ft_free_factor(self, linear, angular):
        return self.command(f"SetFTFreeFactor,0,{float(linear):.1f},{float(angular):.1f},;")

    def set_ft_free_speed_mode(self, mode):
        return self.command(f"SetFTFreeDriveSpeedMode,0,{int(mode)},;")

    def set_ft_search_speed(self, linear_mm_s, angular_deg_s):
        return self.command(
            f"HRSetMaxSearchVelocities,0,{float(linear_mm_s):.1f},{float(angular_deg_s):.1f},;"
        )

    def set_ft_mass(self, x, y, z, rx=10, ry=10, rz=10):
        return self.command(
            f"HRSetMassParams,0,{float(x):.1f},{float(y):.1f},{float(z):.1f},{float(rx):.1f},{float(ry):.1f},{float(rz):.1f},;"
        )

    def set_ft_damp(self, x, y, z, rx=40, ry=40, rz=40):
        return self.command(
            f"HRSetDampParams,0,{float(x):.1f},{float(y):.1f},{float(z):.1f},{float(rx):.1f},{float(ry):.1f},{float(rz):.1f},;"
        )

    def set_ft_stiff(self, x, y, z, rx=100, ry=100, rz=100):
        return self.command(
            f"HRSetStiffParams,0,{float(x):.1f},{float(y):.1f},{float(z):.1f},{float(rx):.1f},{float(ry):.1f},{float(rz):.1f},;"
        )

    def set_ft_pid(self, fp=1.0, fi=0.1, fd=0.0, tp=1.0, ti=0.1, td=0.0):
        return self.command(
            f"HRSetPIDControlParams,0,{float(fp):.3f},{float(fi):.3f},{float(fd):.3f},{float(tp):.3f},{float(ti):.3f},{float(td):.3f},;"
        )

    def _read_wrench(self, message):
        reply = self.command(message, timeout=1.0)
        parts = _parts(reply)
        try:
            values = [float(item) for item in parts[2:8]]
        except (TypeError, ValueError) as exc:
            raise RobotError(f"力传感器数据无法识别：{reply}", reply) from exc
        if len(values) < 6:
            raise RobotError(f"力传感器数据不完整：{reply}", reply)
        return {
            "fx": values[0],
            "fy": values[1],
            "fz": values[2],
            "mx": values[3],
            "my": values[4],
            "mz": values[5],
        }

    def read_pose(self):
        """Current joints and flange pose, without the heavier state() extras."""
        reply = self.command("ReadActPos,0,;", timeout=0.8)
        parts = _parts(reply)
        try:
            numbers = [float(item) for item in parts[2:14]]
        except (TypeError, ValueError) as exc:
            raise RobotError(f"位置数据无法识别：{reply}", reply) from exc
        if len(numbers) < 12:
            raise RobotError(f"位置数据不完整：{reply}", reply)
        joints = numbers[:6]
        pose = numbers[6:12]
        return {
            "joints": joints,
            "pose": {
                "x": pose[0],
                "y": pose[1],
                "z": pose[2],
                "rx": pose[3],
                "ry": pose[4],
                "rz": pose[5],
            },
        }

    def read_fsm(self):
        reply = self.command("ReadCurFSM,0,;")
        for part in _parts(reply):
            if part.lstrip("-").isdigit():
                return int(part)
        raise RobotError(f"读不到机械臂状态：{reply}", reply)

    def move_horizontal(self, axis, direction, distance):
        """Move in base X or Y only. Z is refused so a demo cannot descend."""
        if axis not in (0, 1) or direction not in (0, 1):
            raise RobotError("移动参数无效")
        distance = float(distance)
        if distance < 1 or distance > 60:
            raise RobotError("移动距离超出演示范围")
        millimetres = round(distance, 1)
        return self.command(
            f"MoveRelL,0,{axis},{direction},{millimetres},0,;",
            timeout=8,
        )

    def command(self, message, timeout=None):
        if not message.endswith(",;"):
            raise RobotError("命令格式无效")
        with self._lock:
            self._ensure(timeout or self.timeout)
            self._sock.settimeout(timeout or self.timeout)
            try:
                self._sock.sendall(message.encode("ascii"))
                reply = self._read_reply()
            except OSError as exc:
                self.close()
                raise RobotError(f"和机械臂通信中断：{exc}") from exc
        if ",Fail," in reply:
            raise RobotError(_fail_message(reply), reply)
        return reply

    def _ensure(self, timeout):
        if self._sock is not None:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        try:
            sock.connect((self.host, self.port))
        except OSError as exc:
            sock.close()
            raise RobotError(
                "没有连上机械臂。请确认控制器已开机，并且这台电脑连着机械臂 Wi-Fi。"
            ) from exc
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock = sock

    def _read_reply(self):
        data = b""
        while b";" not in data:
            chunk = self._sock.recv(4096)
            if not chunk:
                raise RobotError("机械臂关闭了连接")
            data += chunk
            if len(data) > 200000:
                raise RobotError("机械臂返回的内容过长")
        return data.decode("ascii", "replace")

    def _offline(self, message):
        return {
            "connected": False,
            "model": "",
            "joints": None,
            "pose": None,
            "fsm": None,
            "ready": False,
            "message": message or "正在连接机械臂",
        }


def _parts(reply):
    text = reply.strip()
    if text.endswith(";"):
        text = text[:-1]
    return text.split(",")


def _field(reply, index):
    parts = _parts(reply)
    if len(parts) <= index:
        raise RobotError(f"机械臂返回无法识别：{reply}", reply)
    return parts[index]


def _fsm_message(fsm):
    if fsm in (7, 24):
        return "机械臂已通电，正在进入待机"
    if fsm in (21, 22):
        return "机械臂停在故障状态，正在尝试恢复"
    if fsm == 6:
        return "机械臂急停还没松开"
    if fsm == 20:
        return "第3轴碰到限位，正在解锁"
    if fsm in (25, 27):
        return "机械臂已连接"
    if fsm is None:
        return "正在连接机械臂"
    return f"机械臂当前不能动作（状态 {fsm}）"


def _fail_message(reply):
    parts = _parts(reply)
    if len(parts) >= 4:
        return f"机械臂拒绝了这条命令：{parts[3]}（{parts[2]}）"
    return f"机械臂拒绝了这条命令：{reply}"
