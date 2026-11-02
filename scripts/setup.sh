#!/usr/bin/env bash
# One-shot setup for a fresh clone. Safe to re-run.
#
#   scripts/setup.sh              # console + tests only
#   scripts/setup.sh --camera     # also install the RealSense packages
#   scripts/setup.sh --voice      # also download the 65 MB speech model
#
# After this, `python main.py --simulate` works with no hardware at all.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

WITH_CAMERA=0
WITH_VOICE=0
for arg in "$@"; do
  case "$arg" in
    --camera) WITH_CAMERA=1 ;;
    --voice) WITH_VOICE=1 ;;
    --all) WITH_CAMERA=1; WITH_VOICE=1 ;;
    -h|--help) sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "未知参数：$arg"; exit 1 ;;
  esac
done

PY="${PYTHON:-python3}"
"$PY" - <<'PY'
import sys
if sys.version_info < (3, 9):
    raise SystemExit("需要 Python 3.9 或更高，当前是 %s" % sys.version.split()[0])
print("Python", sys.version.split()[0])
PY

echo "==> 虚拟环境"
if [[ ! -d .venv ]]; then
  "$PY" -m venv .venv
fi
# shellcheck disable=SC1091
source .venv/bin/activate

echo "==> 安装依赖"
python -m pip install --upgrade pip >/dev/null
python -m pip install -r requirements.txt

if [[ "$WITH_CAMERA" == "1" ]]; then
  echo "==> 安装 RealSense"
  python -m pip install -r requirements-camera.txt || true
  case "$(uname -s)" in
    Darwin) python -m pip install pyrealsense2-macosx || true ;;
    Linux)  python -m pip install pyrealsense2 || true ;;
  esac
fi

if [[ "$WITH_VOICE" == "1" ]]; then
  echo "==> 下载语音模型"
  scripts/fetch-voice-model.sh
fi

echo "==> 目录"
mkdir -p runtime/logs runtime/certs

echo "==> 证书"
scripts/gen-cert.sh || true

echo "==> 自检"
python -m pytest -q 2>/dev/null || echo "（未安装 pytest，跳过测试）"
python main.py --print-config

cat <<'EOF'

完成。接下来：

  无硬件试用：  source .venv/bin/activate && python main.py --simulate
  现场启动：    source .venv/bin/activate && python main.py all

现场部署用 systemd，见 deploy/systemd/README.md。
EOF
