#!/usr/bin/env bash
# Collect the facts needed to explain wrong acupoint placement, without hardware.
#
# Run this on the machine that shows the problem:
#
#     bash scripts/collect_debug.sh
#
# It does NOT need the camera, the arm, or the robot WiFi. It reads
# configuration, calibration and past service logs. Everything goes into
# debug-report.txt; send that one file back.
#
# What it is looking for: the capture resolution the camera actually opened at.
# The recognition code compares widths in absolute pixels, tuned for 1280x720.
# If a machine opened at 640x480 the width comparisons stop meaning anything
# and the head end can be called wrong, which turns the acupoint chart around.
#
# If the camera is connected, it also runs the full acupoint dump.
set -u

cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 1
OUT="debug-report.txt"
: > "$OUT"

say() { printf '%s\n' "$*" | tee -a "$OUT"; }
# Run a command, show it in the report, and put its output in the report.
run() {
    printf '$ %s\n' "$*" | tee -a "$OUT"
    eval "$@" 2>&1 | tee -a "$OUT"
    printf '\n' >> "$OUT"
}

say "HEACON 调试信息  $(date '+%Y-%m-%d %H:%M:%S')"
say "=============================================================="
say

say "== 1. 机器与代码版本 =="
run "uname -a"
run "hostname"
run "python3 --version"
run "git log --oneline -3"
run "git status --short"
say

say "== 2. 配置解析结果 =="
if [[ -x .venv/bin/python ]]; then
    run ".venv/bin/python main.py --print-config"
else
    say "  !! 找不到 .venv，跳过（是否在正确的目录？）"
fi
say

say "== 3. 相机实际打开的分辨率（最关键）=="
say "  从历史日志里找 color_live / camera 打印的分辨率。"
say "  期望是 1280x720；如果是 640x480 或 848x480，那就是问题根源。"
say
{
    echo "\$ journalctl -u heacon-camera --no-pager | grep -i 'opened color'"
    journalctl -u heacon-camera --no-pager 2>/dev/null | grep -i "opened color" | tail -10
    echo
    echo "\$ 日志文件里的 opened color"
    for f in /var/log/heacon/camera.log runtime/logs/camera.log /tmp/heacon-cal/color_live.log; do
        [ -f "$f" ] && grep -i "opened color" "$f" 2>/dev/null | tail -10
    done
    echo
    echo "\$ 试过但被跳过的分辨率"
    journalctl -u heacon-camera --no-pager 2>/dev/null | grep -i "skip " | tail -15
} >> "$OUT" 2>&1
tail -20 "$OUT" | sed 's/^/  /'
say

say "== 4. 相机服务近期日志 =="
run "journalctl -u heacon-camera -n 30 --no-pager"
say

say "== 5. 标定文件：本机的 vs 仓库示例 =="
say "  三个文件如果和示例完全一致，说明用的是 NUC 的标定。"
say "  如果 color_calib 被本机相机重写过，内容会不同。"
say
for name in handeye color_calib work_home; do
    live=""
    for d in config /etc/heacon ../config; do
        if [[ -f "$d/$name.json" ]]; then live="$d/$name.json"; break; fi
    done
    ex="config/$name.example.json"
    {
        echo "--- $name"
        if [[ -n "$live" ]]; then
            echo "  本机: $live  md5=$(md5sum "$live" 2>/dev/null | cut -d' ' -f1)"
        else
            echo "  本机: 未找到（会回退到示例）"
        fi
        if [[ -f "$ex" ]]; then
            echo "  示例: $ex  md5=$(md5sum "$ex" 2>/dev/null | cut -d' ' -f1)"
        fi
    } >> "$OUT" 2>&1
done
say

say "== 6. 运行时的色相标定（相机服务自动写入）=="
say "  这个文件的 width/height 决定像素坐标怎么换算。"
say
for f in runtime/color-calib.json /tmp/heacon-color-calib.json; do
    if [[ -f "$f" ]]; then
        {
            echo "--- $f"
            python3 - "$f" <<'PY' 2>/dev/null
import json, sys
d = json.load(open(sys.argv[1]))
for k in ("width", "height", "fx", "fy", "cx", "cy"):
    if k in d:
        print("  %-6s %s" % (k, d[k]))
print("  color_to_link:", "有" if d.get("color_to_link") else "无")
PY
        } >> "$OUT" 2>&1
    fi
done
say

say "== 7. 相机服务是否在跑 =="
run "systemctl is-active heacon-camera heacon-console heacon-tablet"
say

say "== 8. 如果有相机，直接跑穴位识别诊断 =="
if [[ -x .venv/bin/python ]]; then
    if .venv/bin/python scripts/diagnose_acupoints.py --out acupoint-debug >> "$OUT" 2>&1; then
        say "  已完成，见 acupoint-debug/ 目录（重点看 overlay.jpg）"
    else
        say "  没有相机或取不到画面，跳过。上面第 3 节的分辨率信息仍然有效。"
    fi
else
    say "  跳过"
fi

say
say "=============================================================="
say "请把 $OUT 发回。"
say "如果生成了 acupoint-debug/ 目录，把里面的 overlay.jpg 也一起发。"
