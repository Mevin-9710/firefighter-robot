#!/usr/bin/env python3
"""
Fire-blob detector -- reference implementation.

This is the ground truth for the ESP32 firmware. Tune thresholds here on real
photos, then copy the numbers from fire_config.json into the .ino.

Design goals:
  * scale invariant  -> works whether the printed flame is near or far
  * multiple blobs   -> returns every fire candidate, sorted nearest-first
  * distance estimate-> pinhole model from apparent flame width
  * cheap enough     -> the exact same maths runs on the ESP32 at 30 FPS
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, asdict, field
from pathlib import Path

import cv2
import numpy as np

# --------------------------------------------------------------------------
# Tunables.  Everything the firmware needs lives in this one object.
# --------------------------------------------------------------------------


@dataclass
class FireConfig:
    # --- HSV gate (OpenCV ranges: H 0-179, S 0-255, V 0-255) -------------
    # Fire is red/orange/yellow, plus pink-ish/magenta markers which children
    # legitimately use. Red wraps around 0, so we allow two bands.
    h_lo1: int = 0      # band 1: red -> orange -> yellow
    h_hi1: int = 38
    h_lo2: int = 145    # band 2: red wrap-around incl. pink/crimson markers
    h_hi2: int = 179
    s_min: int = 90     # flame is strongly saturated (rejects skin ~80)
    v_min: int = 110    # rejects brown cardboard, which reads like dim orange ink

    # --- RGB dominance rule (fire is red-dominant, blue-deficient) -------
    # R-B separation is the reliable discriminator; purple/blue fail it while
    # pink and magenta markers still pass.
    use_rgb_rule: bool = True
    r_min: int = 100     # absolute red floor
    r_over_b: int = 45   # R must beat B by this much
    r_over_g: int = 15   # R must at least slightly beat G

    # --- Blob filtering --------------------------------------------------
    min_area_frac: float = 0.0008   # fraction of frame area (allows far fires)
    max_area_frac: float = 0.60     # ignore "whole frame is red"
    min_fill: float = 0.28          # blob area / bbox area
    # Flames are taller than wide; this rejects cardboard boxes and wood.
    min_aspect: float = 0.80        # h/w lower bound
    max_aspect: float = 6.0         # h/w upper bound (rejects thin streaks)
    max_blobs: int = 4

    # --- Bright-glow channel ---------------------------------------------
    # A real flame's core blows out to near-white in the camera; pixels at or
    # above glow_v with saturation at or below glow_s_max count as fire with no
    # hue test.  glow_v = 255 disables the channel (paper backgrounds).
    glow_v: int = 235
    glow_s_max: int = 60

    # --- Morphology (noise cleanup) --------------------------------------
    open_ksize: int = 3

    # --- Distance model --------------------------------------------------
    # distance_mm = focal_px * target_width_mm / bbox_width_px
    # focal_px is calibrated per camera; see calibrate_distance.py
    # 250 ~= 320/2 / tan(66deg/2), typical for the AI-Thinker OV2640 lens.
    focal_px: float = 250.0
    target_width_mm: float = 100.0

    # --- Extinguish threshold -------------------------------------------
    extinguish_distance_mm: float = 350.0


# --------------------------------------------------------------------------
# Presets
# --------------------------------------------------------------------------

def flame_config() -> "FireConfig":
    """Real flame (tealight / candle).

    Real flames are much brighter and more yellow than printed ink, and their
    white-hot core is *less* saturated than the orange edge.  Decoy rejection
    leans on the temporal flicker check in flame_test.py rather than on shape.
    """
    cfg = FireConfig()
    cfg.h_hi1 = 45        # red -> yellow
    cfg.h_lo2 = 180       # band 2 disabled (real flames are never pink)
    cfg.s_min = 130
    cfg.v_min = 190       # flames are bright
    cfg.r_min = 170
    cfg.r_over_b = 70
    cfg.r_over_g = 0      # allow yellow (R == G)
    cfg.min_area_frac = 0.00025   # a tealight at 2 m is only a few pixels
    cfg.max_area_frac = 0.35
    cfg.min_fill = 0.22
    cfg.min_aspect = 0.60
    cfg.max_aspect = 6.0
    cfg.focal_px = 500.0          # calibrated for 640-wide VGA
    cfg.target_width_mm = 20.0     # tealight flame is ~1-2 cm
    cfg.extinguish_mm = 250.0
    return cfg


def candle_config() -> "FireConfig":
    """Vivid tealight cups on a dark floor.

    Tealights are squat -- 38 mm wide by 16 mm tall, so h/w is about 0.4 --
    which means the aspect filter has to be *inverted* relative to a standing
    flame.  A vivid saturated colour on a dark floor gives strong contrast, so
    colour carries most of the decision and shape only rejects obvious noise.
    """
    cfg = FireConfig()
    # warm + pink band; set hue_free for "any vivid colour"
    cfg.h_hi1 = 38
    cfg.h_lo2 = 145
    cfg.s_min = 100
    cfg.v_min = 110
    cfg.r_min = 100
    # Magenta / violet pinks have R ~= B, so the print profile's R-B >= 45
    # would reject them.  On a dark floor the saturation+brightness gate does
    # the real work, so a very loose separation is safe: white and grey have
    # R == B and S == 0, so they still fail.
    cfg.r_over_b = -15
    cfg.r_over_g = 5
    cfg.min_area_frac = 0.00006   # a 7px-wide cup at 4 m must still count
    cfg.max_area_frac = 0.40
    cfg.min_fill = 0.30
    cfg.min_aspect = 0.30      # squat: wider than tall
    cfg.max_aspect = 2.50
    cfg.focal_px = 500.0       # 640-wide VGA
    cfg.target_width_mm = 38.0 # tealight cup diameter
    cfg.extinguish_mm = 250.0
    return cfg


def both_config() -> "FireConfig":
    """Printed flame AND vivid cups in the same pass.

    The two presets disagree only on shape -- a drawn flame is tall, a tealight
    is squat -- so the aspect window is opened at both ends and the colour gates
    take the union.  Distance becomes approximate: one target width cannot
    describe both a 100 mm drawing and a 38 mm cup.
    """
    cfg = FireConfig()
    cfg.h_hi1 = 38
    cfg.h_lo2 = 145
    cfg.s_min = 100
    cfg.v_min = 110           # rejects brown cardboard, which mimics dim orange ink
    cfg.r_min = 100
    cfg.r_over_b = -15
    cfg.r_over_g = 5
    cfg.min_area_frac = 0.00006
    cfg.max_area_frac = 0.60
    cfg.min_fill = 0.28
    cfg.min_aspect = 0.30      # squat allowed
    cfg.max_aspect = 6.0       # tall allowed
    cfg.focal_px = 500.0
    cfg.target_width_mm = 60.0 # mid value, not exact for either target
    cfg.extinguish_mm = 250.0
    return cfg


# --------------------------------------------------------------------------
# Fire is not static: a real flame's pixel count oscillates.  A printed
# drawing, an orange mug or an LED do not.  This is the cheapest and most
# reliable "is this actually burning?" test.
# --------------------------------------------------------------------------


def flicker_score(counts: list[int]) -> float:
    """Normalised peak-to-peak variation of the detected fire-pixel count.

    ~0.0  static object
    >0.15 real flame
    """
    if len(counts) < 4:
        return 0.0
    lo, hi = min(counts), max(counts)
    mean = sum(counts) / len(counts)
    if mean <= 0:
        return 0.0
    return (hi - lo) / mean


@dataclass
class Blob:
    cx: int
    cy: int
    x: int
    y: int
    w: int
    h: int
    area: int
    fill: float
    aspect: float
    distance_mm: float
    score: float

    def to_dict(self) -> dict:
        return asdict(self)


# --------------------------------------------------------------------------
# Core pipeline
# --------------------------------------------------------------------------


def fire_mask(bgr: np.ndarray, cfg: FireConfig) -> np.ndarray:
    """Return a uint8 0/255 mask of fire-coloured pixels."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    band1 = cv2.inRange(hsv, (cfg.h_lo1, cfg.s_min, cfg.v_min), (cfg.h_hi1, 255, 255))
    band2 = cv2.inRange(hsv, (cfg.h_lo2, cfg.s_min, cfg.v_min), (cfg.h_hi2, 255, 255))
    mask = cv2.bitwise_or(band1, band2)

    if cfg.use_rgb_rule:
        b, g, r = cv2.split(bgr)
        ri, gi, bi = r.astype(np.int16), g.astype(np.int16), b.astype(np.int16)
        rgb = ((ri >= cfg.r_min)
               & (ri - bi >= cfg.r_over_b)
               & (ri - gi >= cfg.r_over_g))
        mask = cv2.bitwise_and(mask, rgb.astype(np.uint8) * 255)

    # bright near-white glow: blown-out flame cores
    if cfg.glow_v < 255:
        hot = (v >= cfg.glow_v) & (s <= cfg.glow_s_max)
        mask = cv2.bitwise_or(mask, hot.astype(np.uint8) * 255)

    if cfg.open_ksize > 1:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cfg.open_ksize, cfg.open_ksize))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask


def find_fire_blobs(mask: np.ndarray, cfg: FireConfig) -> list[Blob]:
    """Connected-component analysis + filtering, nearest (largest) first."""
    frame_area = mask.shape[0] * mask.shape[1]
    n, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

    blobs: list[Blob] = []
    for i in range(1, n):  # 0 is background
        x, y, w, h, area = (int(v) for v in stats[i])
        if w <= 0 or h <= 0:
            continue
        frac = area / frame_area
        if frac < cfg.min_area_frac or frac > cfg.max_area_frac:
            continue
        fill = area / float(w * h)
        if fill < cfg.min_fill:
            continue
        aspect = h / float(w)
        if aspect < cfg.min_aspect or aspect > cfg.max_aspect:
            continue
        dist = cfg.focal_px * cfg.target_width_mm / float(w)
        # prefer close + solid + compact
        score = frac * fill
        blobs.append(Blob(int(centroids[i][0]), int(centroids[i][1]),
                          x, y, w, h, area, fill, aspect, dist, score))

    blobs.sort(key=lambda b: b.area, reverse=True)
    return blobs[: cfg.max_blobs]


def detect(bgr: np.ndarray, cfg: FireConfig) -> tuple[list[Blob], np.ndarray]:
    m = fire_mask(bgr, cfg)
    return find_fire_blobs(m, cfg), m


# --------------------------------------------------------------------------
# Visualisation
# --------------------------------------------------------------------------

ACCENT = (0, 255, 0)
WARN = (0, 220, 255)


def annotate(bgr: np.ndarray, blobs: list[Blob], cfg: FireConfig) -> np.ndarray:
    out = bgr.copy()
    h, w = out.shape[:2]
    cv2.line(out, (w // 2, 0), (w // 2, h), (60, 60, 60), 1)

    for idx, b in enumerate(blobs):
        near = b.distance_mm <= cfg.extinguish_distance_mm
        col = WARN if near else ACCENT
        cv2.rectangle(out, (b.x, b.y), (b.x + b.w, b.y + b.h), col, 2)
        cv2.circle(out, (b.cx, b.cy), 3, col, -1)
        label = f"#{idx} {b.distance_mm/10:.0f}cm a={b.area}"
        cv2.putText(out, label, (b.x, max(12, b.y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)

    status = f"{len(blobs)} fire(s)" if blobs else "no fire"
    cv2.putText(out, status, (6, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                WARN if blobs else (200, 200, 200), 1, cv2.LINE_AA)
    return out


# --------------------------------------------------------------------------
# Synthetic test target (so we can validate before a photo exists)
# --------------------------------------------------------------------------


def make_synthetic_flame(w: int = 320, h: int = 240, seed: int = 7) -> np.ndarray:
    """White paper with a printed flame: red edge -> orange -> yellow core."""
    rng = np.random.default_rng(seed)
    img = np.full((h, w, 3), 245, np.uint8)  # paper

    cx, cy = w // 2, int(h * 0.58)
    fw, fh = int(w * 0.16), int(h * 0.34)
    layers = [  # (scale, BGR colour) outermost first
        (1.00, (30, 30, 210)),    # red
        (0.72, (20, 110, 245)),   # orange
        (0.42, (40, 210, 250)),   # yellow
    ]
    for scale, col in layers:
        ww, hh = int(fw * scale), int(fh * scale)
        pts = np.array([
            [cx, cy - hh],
            [cx + ww // 2, cy - hh // 3],
            [cx + ww // 3, cy + hh // 2],
            [cx, cy + hh // 2],
            [cx - ww // 3, cy + hh // 2],
            [cx - ww // 2, cy - hh // 3],
        ], np.int32)
        cv2.fillPoly(img, [pts], col)

    # a little paper grain + slight blue-ish marker noise to be realistic
    noise = rng.normal(0, 4, (h, w, 3))
    img = np.clip(img.astype(np.float16) + noise, 0, 255).astype(np.uint8)

    # a decoy: a blue-ish object that must NOT trigger
    cv2.rectangle(img, (10, 10), (60, 45), (200, 120, 40), -1)
    return img


def scale_for_camera(img: np.ndarray, width: int) -> np.ndarray:
    if width and img.shape[1] != width:
        s = width / img.shape[1]
        img = cv2.resize(img, (width, int(round(img.shape[0] * s))),
                         interpolation=cv2.INTER_AREA)
    return img


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def main() -> int:
    ap = argparse.ArgumentParser(description="Fire blob detector / threshold tuner")
    ap.add_argument("images", nargs="*", help="input image files")
    ap.add_argument("--synth", action="store_true", help="use a synthetic flame target")
    ap.add_argument("--width", type=int, default=320, help="resize width (camera sim, 0=off)")
    ap.add_argument("--out", default="out", help="output directory")
    ap.add_argument("--json", action="store_true", help="print JSON instead of a table")
    ap.add_argument("--save-config", action="store_true", help="write fire_config.json")
    ap.add_argument("--profile", default="print", choices=["print", "flame", "candle", "both"],
                    help="threshold preset: print (drawings), flame (real fire), candle (tealights), both")
    args = ap.parse_args()

    cfg = {"print": FireConfig,
           "flame": flame_config,
           "candle": candle_config,
           "both": both_config}[args.profile]()
    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)

    if args.save_config:
        p = outdir / "fire_config.json"
        p.write_text(json.dumps(asdict(cfg), indent=2))
        print(f"wrote {p}")

    jobs: list[tuple[str, np.ndarray]] = []
    if args.synth or not args.images:
        jobs.append(("synthetic", make_synthetic_flame()))
    for path in args.images:
        img = cv2.imread(path)
        if img is None:
            print(f"!! cannot read {path}", file=sys.stderr)
            continue
        jobs.append((Path(path).stem, img))

    all_stats = {}
    for name, img in jobs:
        img = scale_for_camera(img, args.width)
        blobs, mask = detect(img, cfg)
        ann = annotate(img, blobs, cfg)

        cv2.imwrite(str(outdir / f"{name}_detect.png"), ann)
        cv2.imwrite(str(outdir / f"{name}_mask.png"), mask)
        all_stats[name] = {
            "resolution": f"{img.shape[1]}x{img.shape[0]}",
            "count": len(blobs),
            "blobs": [b.to_dict() for b in blobs],
        }

        if not args.json:
            print(f"\n=== {name}  ({img.shape[1]}x{img.shape[0]}) ===")
            if not blobs:
                print("  no fire detected")
            for i, b in enumerate(blobs):
                print(f"  #{i} centre=({b.cx},{b.cy}) box={b.w}x{b.h} "
                      f"area={b.area} fill={b.fill:.2f} dist={b.distance_mm/10:.0f}cm")

    if args.json:
        print(json.dumps(all_stats, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
