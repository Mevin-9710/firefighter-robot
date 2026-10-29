#!/usr/bin/env bash
# Compile + upload FireBot.  Handles the fact that this board has no
# auto-reset circuit: IO0 must be grounded and the board power-cycled
# by hand before the upload can happen.
set -euo pipefail

AC="$HOME/.local/bin/arduino-cli"
FQBN="esp32:esp32:esp32cam"
PORT="${1:-/dev/ttyUSB0}"
SKETCH="$(dirname "$0")/../firmware/firebot"

if [[ ! -e "$PORT" ]]; then
  echo "!! $PORT not found. Is the USB-TTL adapter plugged in?" >&2
  exit 1
fi

echo "==> compiling"
"$AC" compile --fqbn "$FQBN" "$SKETCH"

echo
echo "==> ready to upload"
echo "    Connect IO0 -> GND, then unplug/replug USB (or press RST)."
read -r -p "    Press Enter once the board is in bootloader... " _

echo "==> uploading to $PORT"
"$AC" upload -p "$PORT" --fqbn "$FQBN" "$SKETCH"

cat <<'EOF'

==> upload complete

    !! REMOVE THE IO0 -> GND JUMPER NOW !!
    Then unplug/replug USB so the app actually boots.

    Watch it with:
      .venv/bin/python tools/monitor.py
EOF
