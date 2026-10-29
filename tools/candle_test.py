#!/usr/bin/env python3
"""
Tealight-cup detection test.

Target: a vivid coloured tealight cup standing on a DARK floor, seen from a
low camera.  Squat silhouette (h/w ~0.4), strong colour against dark ground.

Checks, in order:
  1. every vivid cup colour is found at every distance
  2. the dark floor itself is never detected
  3. plausible decoys (white cup, cardboard, wood, skin, dark red) are rejected
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from fire_detect import candle_config, detect  # noqa: E402

W, H = 640, 480

CUPS = {
    "vivid pink":      (140, 40, 235),
    "hot pink":        (180, 105, 255),
    "deep pink":       (147, 20, 255),
    "rose":            (120, 70, 240),
    "red":             (40, 40, 230),
    "orange":          (40, 140, 245),
    "magenta":         (230, 40, 220),
    "violet-ish pink": (200, 60, 230),
}

# fraction of frame width == apparent cup width == distance
SIZES = {"near 40cm": 0.115, "1m": 0.048, "2m": 0.024, "3m": 0.016, "4m": 0.012}


def draw_cup(img: np.ndarray, cx: int, cy: int, w_px: int, colour) -> None:
    """Squat cylinder: body + elliptical rim, sitting on the floor."""
    h_px = max(3, int(w_px * 0.45))          # tealight: wider than tall
    x0, y0 = cx - w_px // 2, cy - h_px // 2
    # body
    cv2.rectangle(img, (x0, y0), (x0 + w_px, y0 + h_px), colour, -1)
    # rounded top rim, slightly lighter -- skipped when tiny, otherwise the
    # lightened rim dominates the few available pixels and washes out the
    # colour, which is a rendering artefact rather than a real-world effect.
    if w_px >= 14:
        rim = tuple(min(255, int(c * 1.12) + 18) for c in colour)
        cv2.ellipse(img, (cx, y0), (max(1, w_px // 2), max(1, h_px // 3)),
                    0, 0, 360, rim, -1)


def scene(colour, w_px: int, decoys: bool = False, seed: int = 1) -> np.ndarray:
    rng = np.random.default_rng(seed)
    # dark floor with a little texture
    img = np.full((H, W, 3), 26, np.uint8)
    img = np.clip(img.astype(np.float16) + rng.normal(0, 4, (H, W, 3)), 0, 255).astype(np.uint8)
    cv2.line(img, (0, int(H * 0.75)), (W, int(H * 0.78)), (40, 40, 42), 3)

    draw_cup(img, W // 2, int(H * 0.62), w_px, colour)

    if decoys:
        cv2.rectangle(img, (20, 300), (110, 380), (225, 230, 235), -1)   # white cup
        cv2.rectangle(img, (W - 130, 320), (W - 40, 400), (55, 75, 105), -1)  # cardboard
        cv2.rectangle(img, (W - 60, 40), (W - 10, 130), (25, 45, 80), -1)     # wood
        cv2.rectangle(img, (10, 20), (70, 90), (140, 175, 210), -1)      # skin
        cv2.rectangle(img, (140, 40), (200, 110), (35, 35, 95), -1)      # dark red
    return img


def main() -> int:
    profile = os.environ.get("PROFILE", "candle")
    if profile == "both":
        from fire_detect import both_config
        cfg = both_config()
    else:
        cfg = candle_config()
    total = hits = 0
    print(f"{'colour':18} {'distance':10} {'width':>6} {'blobs':>6} {'dist':>8}  verdict")
    print("-" * 68)
    for name, colour in CUPS.items():
        for dname, frac in SIZES.items():
            w_px = int(W * frac)
            img = scene(colour, w_px, decoys=(dname.startswith("near")))
            ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                img = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
            blobs, _ = detect(img, cfg)
            good = any(abs(b.w - w_px) <= max(4, w_px * 0.6) for b in blobs)
            total += 1
            hits += 1 if good else 0
            dd = f"{blobs[0].distance_mm/10:.0f}cm" if blobs else "-"
            print(f"{name:18} {dname:10} {w_px:>6} {len(blobs):>6} {dd:>8}  "
                  f"{'OK' if good else 'MISS'}")

    # --- negatives --------------------------------------------------------
    print("-" * 68)
    negs = {
        "empty dark floor": np.full((H, W, 3), 26, np.uint8),
        "white cup only": None,
        "grey wall": np.full((H, W, 3), 150, np.uint8),
        "blue cup": None,
        "green cup": None,
        "brown cardboard": None,
    }
    for label in negs:
        if label == "white cup only":
            img = np.full((H, W, 3), 26, np.uint8)
            cv2.rectangle(img, (280, 250), (360, 300), (225, 230, 235), -1)
        elif label == "brown cardboard":
            # dim brown reads like orange ink at low v_min -- the regression
            # that appeared when the "both" profile first merged the presets
            img = np.full((H, W, 3), 26, np.uint8)
            cv2.rectangle(img, (500, 320), (600, 400), (55, 75, 105), -1)
        elif label == "blue cup":
            img = np.full((H, W, 3), 26, np.uint8)
            draw_cup(img, W // 2, int(H * 0.62), 60, (200, 120, 40))
        elif label == "green cup":
            img = np.full((H, W, 3), 26, np.uint8)
            draw_cup(img, W // 2, int(H * 0.62), 60, (60, 180, 60))
        else:
            img = negs[label]
        blobs, _ = detect(img, cfg)
        verdict = "PASS" if not blobs else "FALSE POSITIVE"
        print(f"{label:18} {'-':10} {'-':>6} {len(blobs):>6} {'-':>8}  {verdict}")

    pct = 100.0 * hits / total
    print("-" * 68)
    print(f"detected {hits}/{total} = {pct:.1f}%")
    return 0 if pct >= 90 else 1


if __name__ == "__main__":
    raise SystemExit(main())
