#!/usr/bin/env python3
"""
Camera distance calibration.

The pinhole model is  d = focal_px * real_width_mm / bbox_width_px.
`focal_px` depends on the lens, so measure it once for your camera.

Method: put the printed flame at a known distance, read the bbox width the
firmware reports (w=...), and run this. Repeat at a couple of distances and
average -- lens distortion makes far readings less trustworthy.

  calibrate_distance.py --dist 300 --width 42 --real 100
"""

from __future__ import annotations

import argparse


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dist", type=float, required=True, help="measured distance, mm")
    ap.add_argument("--width", type=float, required=True, help="bbox width in px (w= from serial)")
    ap.add_argument("--real", type=float, default=100.0, help="real flame width, mm")
    args = ap.parse_args()

    focal = args.width * args.dist / args.real
    print(f"focal_px = {focal:.1f}")
    print(f"\nApply with:  monitor.py --cmd 'focal={focal:.0f}'")
    print(f"or hard-code it in firmware/firebot/firebot.ino (Config::focalPx)")

    print("\nSanity check at other distances:")
    for d in (150, 300, 500, 800, 1200):
        px = focal * args.real / d
        print(f"  {d:>5} mm -> {px:6.1f} px wide")


if __name__ == "__main__":
    main()
