#!/usr/bin/env bash
# Generate the self-signed certificate the tablet needs.
#
# A browser only allows microphone access on an HTTPS origin, so the tablet
# talks to port 8766 instead of 8765. That needs a certificate. It is a local
# self-signed one: the tablet will warn once and you accept it.
#
# Usage:  scripts/gen-cert.sh [extra-host ...]
# Writes into $HEACON_RUNTIME_DIR/certs (default ./runtime/certs).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME="${HEACON_RUNTIME_DIR:-$ROOT/runtime}"
CERTS="$RUNTIME/certs"
mkdir -p "$CERTS"

CERT="$CERTS/heacon.crt"
KEY="$CERTS/heacon.key"
DAYS=3650

if [[ -f "$CERT" && -f "$KEY" && "${FORCE:-0}" != "1" ]]; then
  echo "证书已存在：$CERT"
  echo "要重新生成，先设 FORCE=1 再运行。"
  exit 0
fi

# Collect every address the tablet might use to reach this machine.
HOSTS=("localhost" "127.0.0.1" "::1")
for addr in $(hostname) $(hostname -s); do HOSTS+=("$addr"); done
if command -v ipconfig >/dev/null 2>&1; then
  # macOS
  for i in $(ipconfig getiflist 2>/dev/null || true); do
    ip="$(ipconfig getifaddr "$i" 2>/dev/null || true)"
    [[ -n "$ip" ]] && HOSTS+=("$ip")
  done
elif command -v hostname >/dev/null 2>&1; then
  # Linux
  while read -r ip; do [[ -n "$ip" ]] && HOSTS+=("$ip"); done < <(hostname -I 2>/dev/null | tr ' ' '\n')
fi
HOSTS+=("$@")

SAN=""
for h in "${HOSTS[@]}"; do
  [[ -z "$h" ]] && continue
  if [[ "$h" =~ ^[0-9.]+$ || "$h" == *:* ]]; then
    SAN="${SAN}IP:${h},"
  else
    SAN="${SAN}DNS:${h},"
  fi
done
SAN="${SAN%,}"

openssl req -x509 -newkey rsa:2048 -sha256 -days "$DAYS" -nodes \
  -keyout "$KEY" -out "$CERT" \
  -subj "/CN=heacon.local/O=HEACON" \
  -addext "subjectAltName=${SAN}" \
  -addext "basicConstraints=CA:FALSE" \
  -addext "keyUsage=digitalSignature,keyEncipherment" \
  -addext "extendedKeyUsage=serverAuth" >/dev/null 2>&1

chmod 600 "$KEY"
echo "已生成："
echo "  $CERT"
echo "  $KEY"
echo "包含地址：$SAN"
