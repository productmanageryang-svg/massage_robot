#!/usr/bin/env bash
# Install or update HEACON on Ubuntu 24.04.
#
#   sudo deploy/install-ubuntu.sh            # install / update
#   sudo deploy/install-ubuntu.sh --no-start  # stage only, do not start services
#
# Idempotent: re-running updates the code and restarts services, and leaves
# /etc/heacon (environment + calibration) alone once it exists.
set -euo pipefail

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST=/opt/heacon
CONF=/etc/heacon
STATE=/var/lib/heacon
LOGS=/var/log/heacon
USER_NAME=heacon
DO_START=1

for arg in "$@"; do
  case "$arg" in
    --no-start) DO_START=0 ;;
    -h|--help) sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "未知参数：$arg"; exit 1 ;;
  esac
done

if [[ "$(id -u)" != "0" ]]; then
  echo "请用 sudo 运行：sudo $0" >&2
  exit 1
fi

if ! grep -qi 'ubuntu' /etc/os-release 2>/dev/null; then
  echo "注意：这不是 Ubuntu，脚本按 Ubuntu 24.04 编写，继续前请确认。" >&2
fi

echo "==> 系统依赖"
apt-get update -qq
apt-get install -y -qq python3 python3-venv python3-pip git \
  libgl1 libglib2.0-0 usbutils v4l-utils openssl rsync

echo "==> 用户"
if ! id "$USER_NAME" >/dev/null 2>&1; then
  useradd --system --create-home --home-dir "$DEST" --shell /usr/sbin/nologin "$USER_NAME"
fi
usermod -aG video "$USER_NAME" 2>/dev/null || true

echo "==> 目录"
mkdir -p "$DEST" "$CONF" "$STATE" "$LOGS"
chown -R "$USER_NAME:$USER_NAME" "$DEST" "$STATE" "$LOGS"
chown root:"$USER_NAME" "$CONF"
chmod 750 "$CONF"

echo "==> 代码"
rsync -a --delete \
  --exclude .git --exclude .venv --exclude runtime --exclude __pycache__ \
  --exclude node_modules \
  "$SRC"/ "$DEST"/
chown -R "$USER_NAME:$USER_NAME" "$DEST"

echo "==> 虚拟环境"
if [[ ! -x "$DEST/.venv/bin/python" ]]; then
  sudo -u "$USER_NAME" python3 -m venv "$DEST/.venv"
fi
sudo -u "$USER_NAME" "$DEST/.venv/bin/pip" install -q -U pip
sudo -u "$USER_NAME" "$DEST/.venv/bin/pip" install -q -r "$DEST/requirements.txt"

echo "==> RealSense 依赖"
# The camera daemon is the only consumer. Without this the console still runs
# but /api/camera.jpg stays "missing" and no service can start.
if ! sudo -u "$USER_NAME" "$DEST/.venv/bin/python" -c 'import pyrealsense2' 2>/dev/null; then
  sudo -u "$USER_NAME" "$DEST/.venv/bin/pip" install -q pyrealsense2 \
    || sudo -u "$USER_NAME" "$DEST/.venv/bin/pip" install -q pyrealsense2-macosx \
    || echo "  装不上 pyrealsense2，请按 requirements-camera.txt 从源码编译。摄像头服务会起不来，其余不受影响。"
fi
sudo -u "$USER_NAME" "$DEST/.venv/bin/python" - <<'PY' || true
try:
    import pyrealsense2 as rs
    n = len(list(rs.context().query_devices()))
    print("  pyrealsense2 可用，当前看到 %d 个 RealSense 设备" % n)
    if n == 0:
        print("  !! 没看到设备：检查 USB 线、USB3 口、udev 规则是否已加载")
except Exception as exc:
    print("  pyrealsense2 检查失败：%s" % exc)
PY

echo "==> udev 规则"
install -m 644 "$DEST/deploy/udev/99-realsense-d435i.rules" /etc/udev/rules.d/
udevadm control --reload-rules || true
udevadm trigger || true

echo "==> 配置"
if [[ ! -f "$CONF/heacon.env" ]]; then
  install -m 640 -o root -g "$USER_NAME" "$DEST/.env.example" "$CONF/heacon.env"
  echo "  已写入 $CONF/heacon.env，请检查机械臂地址"
else
  echo "  保留已有 $CONF/heacon.env"
fi

for name in handeye color_calib work_home; do
  if [[ ! -f "$CONF/$name.json" && -f "$DEST/config/$name.example.json" ]]; then
    install -m 640 -o root -g "$USER_NAME" "$DEST/config/$name.example.json" "$CONF/$name.json"
    echo "  已放入示例 $name.json —— 首次使用前必须重新标定"
  fi
done

echo "==> 证书"
if [[ ! -f "$STATE/certs/heacon.key" ]]; then
  sudo -u "$USER_NAME" HEACON_RUNTIME_DIR="$STATE" "$DEST/scripts/gen-cert.sh" >/dev/null
  echo "  已生成自签证书"
else
  echo "  保留已有证书"
fi

echo "==> 识别模型（MediaPipe，必需）"
sudo -u "$USER_NAME" HEACON_RUNTIME_DIR="$STATE" "$DEST/scripts/fetch-models.sh" \
  || echo "  下载失败；没有模型无法识别穴位，服务起不来"

echo "==> 语音模型（可选）"
if [[ -d "$STATE/models/vosk-small-cn" ]]; then
  echo "  已存在"
else
  sudo -u "$USER_NAME" HEACON_RUNTIME_DIR="$STATE" "$DEST/scripts/fetch-voice-model.sh" \
    || echo "  跳过；只影响平板语音"
fi

echo "==> systemd"
install -m 644 "$DEST"/deploy/systemd/*.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable heacon-console heacon-tablet heacon-camera >/dev/null

if [[ "$DO_START" == "1" ]]; then
  systemctl restart heacon-camera heacon-console heacon-tablet
  sleep 2
  systemctl --no-pager --lines=0 status heacon-console heacon-tablet heacon-camera || true
else
  echo "  已安装，未启动（--no-start）"
fi

cat <<EOF

完成。

  查看状态   systemctl status heacon-console heacon-tablet heacon-camera
  查看日志   journalctl -u heacon-console -f
  配置       $CONF/heacon.env
  标定       $CONF/handeye.json, color_calib.json, work_home.json
  数据       $STATE

首次使用前请在真机上重新标定，再按 docs/部署.md 第 3 节逐项验证。
EOF
