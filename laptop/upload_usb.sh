#!/usr/bin/env bash
# Build the robot firmware and load it over USB. Needed once per new Pico W
# (or if a Wi-Fi update ever goes wrong); after that use laptop/upload_wifi.sh.
#
#   laptop/upload_usb.sh 1        # robot 1, Pico plugged in by USB
#
# A brand-new / blank Pico shows up as the RPI-RP2 drive; that works too.
set -euo pipefail

ID="${1:-}"
if ! [[ "$ID" =~ ^[0-9]+$ ]] || (( ID < 1 || ID > 16 )); then
  echo "usage: $0 <robot number 1-16>" >&2
  exit 1
fi

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SKETCH="$ROOT/robot/robot_wifi"
BUILD="$SKETCH/build/robot$ID"
FQBN="rp2040:rp2040:rpipicow:flash=2097152_1048576"

echo "== Building firmware for robot $ID"
arduino-cli compile --fqbn "$FQBN" --build-property "compiler.cpp.extra_flags=-DROBOT_ID=$ID" \
  --output-dir "$BUILD" "$SKETCH" | grep -E "Sketch uses|error" || true
[[ -f "$BUILD/robot_wifi.ino.uf2" ]] || { echo "build failed" >&2; exit 1; }

PORT="$(ls /dev/cu.usbmodem* 2>/dev/null | head -1 || true)"
if [[ -n "$PORT" ]]; then
  echo "== Uploading over $PORT"
  arduino-cli upload --fqbn "$FQBN" -p "$PORT" --input-dir "$BUILD" "$SKETCH"
elif [[ -d /Volumes/RPI-RP2 ]]; then
  echo "== Copying to the RPI-RP2 drive"
  cp "$BUILD/robot_wifi.ino.uf2" /Volumes/RPI-RP2/
else
  echo "No Pico found. Plug it in by USB (hold BOOTSEL while plugging in if it's blank)." >&2
  exit 1
fi
echo "== Done. Robot $ID is restarting."
