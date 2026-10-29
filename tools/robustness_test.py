#!/usr/bin/env python3
"""
Robustness harness: does the detector survive "any hand-drawn flame"?

Generates a spread of flame drawings that a human might actually produce --
different marker colours, sizes (= distances), paper tints, JPEG noise --
and reports which ones the configured thresholds catch.
"""

from __future__ import annotations

import sys
import os
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from fire_detect import FireConfig, detect  # noqa: E402

# BGR colours roughly matching real media
INKS = {
    "crimson marker": (30, 30, 200),
    "scarlet marker": (20, 20, 235),
    "orange marker": (30, 120, 245),
    "deep red pen": (40, 40, 160),
    "dark red crayon": (35, 60, 175),
    "amber highlighter": (40, 175, 250),
    "tomato red": (60, 80, 240),
    "magenta-ish red": (70, 40, 220),
    "pink red (borderline)": (110, 80, 230),
}

# flame width as a fraction of frame width -> simulates distance
SIZES = {"near": 0.30, "mid": 0.16, "far": 0.075, "very far": 0.045}

PAPERS = {
    "white": (245, 245, 245),
    "warm white": (230, 235, 245),
    "grey-ish": (200, 200, 200),
}


def draw_flame(w: int, h: int, ink, frac: float, paper, seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    img = np.full((h, w, 3), paper, np.uint8)
    cx, cy = w // 2, int(h * 0.58)
    fw, fh = int(w * frac), int(h * frac * 2.1)

    # outer body in the ink colour, inner core lightened = hand-coloured look
    pts = np.array([
        [cx, cy - fh],
        [cx + fw // 2, cy - fh // 3],
        [cx + fw // 3, cy + fh // 2],
        [cx, cy + fh // 2],
        [cx - fw // 3, cy + fh // 2],
        [cx - fw // 2, cy - fh // 3],
    ], np.int32)
    cv2.fillPoly(img, [pts], ink)

    core = tuple(min(255, int(c * 1.25) + 45) for c in ink)
    cv2.ellipse(img, (cx, int(cy + fh * 0.12)), (max(2, fw // 4), max(2, fh // 4)),
                0, 0, 360, core, -1)

    # paper grain
    img = np.clip(img.astype(np.float16) + rng.normal(0, 5, (h, w, 3)), 0, 255).astype(np.uint8)
    return img


def add_decoys(img: np.ndarray) -> np.ndarray:
    """Things commonly in a test scene that must NOT read as fire."""
    h, w = img.shape[:2]
    cv2.rectangle(img, (5, 5), (55, 40), (200, 130, 60), -1)        # tan cardboard
    cv2.circle(img, (w - 40, 30), 18, (150, 90, 70), -1)             # skin tone
    cv2.rectangle(img, (w - 30, h - 35), (w - 5, h - 5), (90, 60, 40), -1)  # wood
    return img


def main() -> int:
    cfg = FireConfig()
    profile = os.environ.get("PROFILE", "print")
    if profile == "both":
        from fire_detect import both_config
        cfg = both_config()
    elif profile == "candle":
        from fire_detect import candle_config
        cfg = candle_config()
    W, H = 320, 240
    total = hits = 0
    print(f"{'ink':24} {'size':10} {'paper':12} {'dist':>6} {'blobs':>6}  verdict")
    print("-" * 74)

    for ink_name, ink in INKS.items():
        for size_name, frac in SIZES.items():
            for paper_name, paper in list(PAPERS.items())[:1]:
                img = draw_flame(W, H, ink, frac, paper, seed=total)
                if size_name == "near":
                    img = add_decoys(img)
                # emulate a webcam JPEG frame
                ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                img = cv2.imdecode(jpg, cv2.IMREAD_COLOR) if ok else img

                blobs, _ = detect(img, cfg)
                expected_w = int(W * frac)
                # accept if a blob exists whose width is within 2.5x of expectation
                good = any(abs(b.w - expected_w) <= max(6, expected_w * 1.5) for b in blobs)
                total += 1
                hits += 1 if good else 0
                d = f"{blobs[0].distance_mm/10:.0f}cm" if blobs else "-"
                print(f"{ink_name:24} {size_name:10} {paper_name:12} {d:>6} "
                      f"{len(blobs):>6}  {'OK' if good else 'MISS'}")
        if ink_name == "crimson marker":
            for paper_name, paper in PAPERS.items():
                img = draw_flame(W, H, ink, 0.16, paper, seed=total + 99)
                blobs, _ = detect(img, cfg)
                good = len(blobs) > 0
                total += 1
                hits += 1 if good else 0
                print(f"{ink_name:24} {'mid':10} {paper_name:12} "
                      f"{'-':>6} {len(blobs):>6}  {'OK' if good else 'MISS'}")

    pct = 100.0 * hits / total
    print("-" * 74)
    print(f"detected {hits}/{total} = {pct:.1f}%")
    return 0 if pct >= 90 else 1


if __name__ == "__main__":
    raise SystemExit(main())
