#!/usr/bin/env bash
# Build the robot firmware and send it to a robot over Wi-Fi (no USB cable).
#
#   laptop/upload_wifi.sh 1                 # robot 1, found as robot1.local
#   laptop/upload_wifi.sh 3 192.168.1.42    # robot 3 at a known address
#
# The robot number is built into the firmware, so always give the right one.
# The robot must already be running firmware with Wi-Fi updates (load it once with
# laptop/upload_usb.sh). Its motors stop during the update; it restarts by itself.
set -euo pipefail

ID="${1:-}"
if ! [[ "$ID" =~ ^[0-9]+$ ]] || (( ID < 1 || ID > 16 )); then
  echo "usage: $0 <robot number 1-16> [ip address]" >&2
  exit 1
fi
HOST="${2:-robot$ID.local}"

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SKETCH="$ROOT/robot/robot_wifi"
BUILD="$SKETCH/build/robot$ID"
FQBN="rp2040:rp2040:rpipicow:flash=2097152_1048576"
PASS="$(sed -n 's/^#define OTA_PASSWORD "\(.*\)"/\1/p' "$SKETCH/secrets.h" 2>/dev/null)"
[[ -n "$PASS" ]] || { echo "No OTA_PASSWORD in $SKETCH/secrets.h (copy secrets.example.h to secrets.h)" >&2; exit 1; }
ESPOTA="$(ls "$HOME"/Library/Arduino15/packages/rp2040/hardware/rp2040/*/tools/espota.py 2>/dev/null | tail -1)"
[[ -n "$ESPOTA" ]] || { echo "Pico board package not installed (arduino-cli core install rp2040:rp2040)" >&2; exit 1; }

echo "== Building firmware for robot $ID"
arduino-cli compile --fqbn "$FQBN" --build-property "compiler.cpp.extra_flags=-DROBOT_ID=$ID" \
  --output-dir "$BUILD" "$SKETCH" | grep -E "Sketch uses|error" || true
[[ -f "$BUILD/robot_wifi.ino.bin" ]] || { echo "build failed" >&2; exit 1; }

IP="$(python3 -c 'import socket,sys; print(socket.gethostbyname(sys.argv[1]))' "$HOST" 2>/dev/null)" || {
  echo "Can't find $HOST. Is robot $ID on and on this network? Try: python3 laptop/robot_client.py find" >&2
  exit 1
}

echo "== Sending to robot $ID at $IP over Wi-Fi"
python3 "$ESPOTA" -i "$IP" -p 2040 -a "$PASS" -f "$BUILD/robot_wifi.ino.bin" -r
echo "== Done. Robot $ID is restarting with the new firmware (about 10 s)."
