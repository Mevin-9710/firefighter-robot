#!/usr/bin/env python3
"""
FireBot serial monitor.

Reads the detector's telemetry, renders it readably, and lets you push
threshold changes live.  This is the tuning loop: no reflash required.

Usage:
  monitor.py                        # watch
  monitor.py --cmd "s=95" --cmd show
  monitor.py --raw                  # dump raw lines
  monitor.py --log run1.txt
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    sys.exit("pyserial missing:  uv pip install pyserial")

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 921600

FIRE_RE = re.compile(r"FIRE n=(\d+).*d=(-?\d+)cm err=(-?\d+) st=(\w+) fps=(\d+)")
NOFIRE_RE = re.compile(r"NOFIRE blobs=(\d+) st=(\w+) fps=(\d+)")

GREEN, YELLOW, RED, DIM, RESET = "\033[32m", "\033[33m", "\033[31m", "\033[2m", "\033[0m"


def bar(err: int, width: int = 21) -> str:
    """Small visual of where the target sits relative to centre."""
    mid = width // 2
    pos = max(0, min(width - 1, mid + int(err / 160.0 * mid)))
    row = ["-"] * width
    row[mid] = "|"
    row[pos] = "#"
    return "".join(row)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--raw", action="store_true", help="print raw lines only")
    ap.add_argument("--cmd", action="append", default=[], help="command to send at start")
    ap.add_argument("--log", help="append everything to this file")
    ap.add_argument("--once", action="store_true", help="exit after first FIRE/NOFIRE")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except Exception as e:
        print(f"cannot open {args.port}: {e}", file=sys.stderr)
        return 1

    time.sleep(0.3)
    for c in args.cmd:
        ser.write((c + "\n").encode())
        print(f"{DIM}>> {c}{RESET}")

    logf = open(args.log, "a", encoding="utf-8") if args.log else None
    buf = ""
    last_state = None
    try:
        while True:
            chunk = ser.read(256)
            if not chunk:
                continue
            buf += chunk.decode("utf-8", "replace")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip("\r").strip()
                if not line:
                    continue
                if logf:
                    logf.write(line + "\n")
                    logf.flush()

                if args.raw:
                    print(line)
                else:
                    m = FIRE_RE.search(line)
                    if m:
                        n, d, err, st, fps = m.groups()
                        colour = YELLOW if int(d) <= 35 else GREEN
                        print(f"{colour}FIRE{RESET} blobs={n} dist={d:>4}cm "
                              f"err={err:>4}  [{bar(int(err))}]  {st:12} {fps}fps")
                    else:
                        m = NOFIRE_RE.search(line)
                        if m:
                            n, st, fps = m.groups()
                            print(f"{DIM}clear{RESET} blobs={n} {'':>22}  "
                                  f"{'':21}  {st:12} {fps}fps")
                        else:
                            print(f"{DIM}{line}{RESET}")
                if args.once and (FIRE_RE.search(line) or NOFIRE_RE.search(line)):
                    return 0
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        ser.close()
        if logf:
            logf.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
