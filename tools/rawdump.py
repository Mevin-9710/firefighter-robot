#!/usr/bin/env python3
"""Grab one raw framebuffer from the board and try every plausible pixel
layout, so we can tell sensor noise apart from a decoding mistake.

    python tools/rawdump.py [port]

Writes /tmp/opencode/raw_<layout>.png for each interpretation and prints a
sharpness/noise score for each one.  The layout with the highest score is the
one the camera is actually producing.
"""
from __future__ import annotations

import sys
import time

import numpy as np
import serial

DEFAULT_PORT = "/dev/ttyUSB0"
BAUD = 921600
PKT_JPEG, PKT_STATUS, PKT_RAW = 1, 2, 3
OUT_DIR = "/tmp/opencode"


def read_packet(ser: serial.Serial, deadline: float):
    """Scan for the A5 5A magic, then trust the length field."""
    while time.time() < deadline:
        if ser.read(1) != b"\xa5":
            continue
        if ser.read(1) != b"\x5a":
            continue
        head = ser.read(4)
        if len(head) != 4:
            continue
        ptype = head[0]
        length = head[1] | (head[2] << 8) | (head[3] << 16)
        if length == 0 or length > 2_000_000:
            continue
        body = ser.read(length)
        if len(body) == length:
            return ptype, body
    return None, None


def score(img: np.ndarray) -> float:
    """Lag-1 spatial correlation of luma.  Coherent images score high."""
    g = img.astype(np.float32).mean(axis=2)
    a = g[:, :-1].ravel() - g.mean()
    b = g[:, 1:].ravel() - g.mean()
    den = np.sqrt((a ** 2).sum() * (b ** 2).sum())
    return float((a * b).sum() / den) if den else 0.0


def rgb565(buf: np.ndarray, w: int, h: int, big_endian: bool) -> np.ndarray:
    px = buf[: w * h * 2].reshape(h, w, 2).astype(np.uint16)
    v = (px[:, :, 0] << 8 | px[:, :, 1]) if big_endian else (px[:, :, 1] << 8 | px[:, :, 0])
    r = ((v >> 11) & 0x1F) << 3
    g = ((v >> 5) & 0x3F) << 2
    b = (v & 0x1F) << 3
    return np.dstack([b, g, r]).astype(np.uint8)   # BGR for cv2


def yuv422(buf: np.ndarray, w: int, h: int) -> np.ndarray:
    """YUYV: Y0 U Y1 V -- two pixels share a chroma pair."""
    raw = buf[: w * h * 2].reshape(h, w // 2, 4).astype(np.float32)
    y0, u, y1, v = raw[..., 0], raw[..., 1], raw[..., 2], raw[..., 3]
    u = u - 128.0
    v = v - 128.0
    r = 1.402 * v
    g = -0.344136 * u - 0.714136 * v
    b = 1.772 * u
    out = np.empty((h, w, 3), np.float32)
    for i, y in enumerate((y0, y1)):
        out[:, i::2, 2] = y + r      # cv2 wants BGR
        out[:, i::2, 1] = y + g
        out[:, i::2, 0] = y + b
    return np.clip(out, 0, 255).astype(np.uint8)


def main() -> int:
    import cv2

    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    ser = serial.Serial(port, BAUD, timeout=0.05)
    time.sleep(0.3)
    ser.reset_input_buffer()

    ser.write(b"stream=0\n")
    ser.flush()
    time.sleep(0.5)
    ser.write(b"raw\n")
    ser.flush()

    deadline = time.time() + 25
    body = None
    while time.time() < deadline:
        ptype, payload = read_packet(ser, deadline)
        if ptype == PKT_RAW:
            body = payload
            break
    ser.close()

    if body is None:
        print("no raw packet received")
        return 1

    text = body.decode("latin-1")
    # firmware prefixes the dump with a plain-text header line
    nl = text.find("\n")
    if nl > 0 and text.startswith("RAW "):
        fields = text[:nl].split()
        w, h = (int(x) for x in fields[1].split("x"))
        buf = np.frombuffer(body[nl + 1:], np.uint8)
        print(f"header: {text[:nl]}   payload {len(buf)} bytes")
    else:
        print("warning: no RAW header, assuming QVGA RGB565")
        w, h = 320, 240
        buf = np.frombuffer(body, np.uint8)

    if len(buf) < w * h * 2:
        print(f"short payload: {len(buf)} < {w * h * 2}")
        return 1

    layouts = {
        "rgb565_le": rgb565(buf, w, h, big_endian=False),
        "rgb565_be": rgb565(buf, w, h, big_endian=True),
        "yuyv": yuv422(buf, w, h),
    }
    print(f"\n{'layout':12} {'score':>7}  bytes")
    best = None
    for name, img in layouts.items():
        path = f"{OUT_DIR}/raw_{name}.png"
        cv2.imwrite(path, img)
        sc = score(img)
        print(f"{name:12} {sc:+7.3f}  {path}")
        if best is None or sc > best[1]:
            best = (name, sc)
    print(f"\nbest: {best[0]} (score {best[1]:+.3f})")
    print("a coherent image scores ~0.95+, sensor noise ~0.1-0.5")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
