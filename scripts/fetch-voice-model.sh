#!/usr/bin/env bash
# Download the Vosk Mandarin model used for tablet voice commands.
#
# 65 MB, so it is not committed. Without it the console still runs; only the
# microphone button on the tablet reports that speech is unavailable.
#
# Usage:  scripts/fetch-voice-model.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME="${HEACON_RUNTIME_DIR:-$ROOT/runtime}"
DEST="$RUNTIME/models/vosk-small-cn"
URL="https://alphacephei.com/vosk/models/vosk-model-small-cn-0.22.zip"

if [[ -f "$DEST/conf/model.conf" || -d "$DEST/am" ]]; then
  echo "语音模型已存在：$DEST"
  exit 0
fi

if [[ "${HEACON_SKIP_MODEL_DOWNLOAD:-0}" == "1" ]]; then
  echo "已跳过语音模型下载（HEACON_SKIP_MODEL_DOWNLOAD=1）"
  exit 0
fi

command -v curl >/dev/null || { echo "需要 curl"; exit 1; }
command -v unzip >/dev/null || { echo "需要 unzip"; exit 1; }

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "下载语音模型…"
curl -fL --progress-bar -o "$TMP/model.zip" "$URL"

echo "解压…"
unzip -q "$TMP/model.zip" -d "$TMP"

SRC="$(find "$TMP" -maxdepth 1 -type d -name 'vosk-model*' | head -1)"
[[ -n "$SRC" ]] || { echo "压缩包里没有模型目录"; exit 1; }

mkdir -p "$(dirname "$DEST")"
rm -rf "$DEST"
mv "$SRC" "$DEST"

echo "完成：$DEST"
du -sh "$DEST"
