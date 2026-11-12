#!/usr/bin/env bash
# Download the two MediaPipe task files used by acupoint recognition.
#
# 37 MB combined, so they are not in the repo. Without them
# heacon/vision/hybrid_geom.py cannot detect a body and no service can start.
#
# Usage:  scripts/fetch-models.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME="${HEACON_RUNTIME_DIR:-$ROOT/runtime}"
DEST="$RUNTIME/models"
mkdir -p "$DEST"

POSE_URL="https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/latest/pose_landmarker_full.task"
HAND_URL="https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task"

if [[ "${HEACON_SKIP_MODEL_DOWNLOAD:-0}" == "1" ]]; then
  echo "已跳过模型下载（HEACON_SKIP_MODEL_DOWNLOAD=1）"
  exit 0
fi

command -v curl >/dev/null || { echo "需要 curl"; exit 1; }

fetch() {
  local url="$1" out="$2" label="$3"
  if [[ -s "$out" ]]; then
    echo "已存在：$label ($(du -h "$out" | cut -f1))"
    return
  fi
  echo "下载 $label …"
  curl -fL --progress-bar -o "$out.part" "$url"
  mv "$out.part" "$out"
  echo "  完成 $(du -h "$out" | cut -f1)"
}

fetch "$POSE_URL" "$DEST/pose_landmarker.task" "pose_landmarker.task"
fetch "$HAND_URL" "$DEST/hand_landmarker.task" "hand_landmarker.task"

echo
echo "模型目录：$DEST"
ls -lh "$DEST"/*.task
echo
echo "确认识别能加载："
echo "  python -c \"import heacon.vision.hybrid_geom as h; print(h.POSE_MODEL, h.HAND_MODEL)\""
