#!/usr/bin/env python3
"""
FireBot local dashboard.

The board pushes length-prefixed binary packets up the USB serial link:

    A5 5A | type | len (3 bytes LE) | payload
    type 1 = JPEG frame
    type 2 = status JSON

This script decodes them and serves the dashboard on http://127.0.0.1:8080,
so a PC with no WiFi at all can still watch the camera and tune thresholds.
Commands typed in the browser are written back down the same serial link.

    .venv/bin/python tools/dashboard.py
    .venv/bin/python tools/dashboard.py --port /dev/ttyUSB0 --baud 921600
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import serial

MAGIC = b"\xa5\x5a"
PKT_JPEG = 1
PKT_STATUS = 2
MAX_PAYLOAD = 400_000

HERE = Path(__file__).parent
PAGE = (HERE / "dashboard.html").read_bytes()


class Link:
    """Owns the serial port and the most recent decoded packet state."""

    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud
        self.ser: serial.Serial | None = None
        self.lock = threading.Lock()
        self.frame: bytes | None = None
        self.frame_id = 0
        self.status: dict = {}
        self.frame_times: deque[float] = deque(maxlen=30)
        self.error = ""
        self.running = True

    def open(self) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
            self.error = ""
            return True
        except Exception as exc:  # noqa: BLE001 - surfaced in the UI
            self.error = str(exc)
            self.ser = None
            return False

    def send(self, cmd: str) -> bool:
        if not self.ser:
            return False
        try:
            self.ser.write((cmd.strip() + "\n").encode())
            self.ser.flush()
            return True
        except Exception as exc:  # noqa: BLE001
            self.error = str(exc)
            return False

    def reader(self) -> None:
        buf = bytearray()
        started = False
        while self.running:
            if not self.ser:
                if not self.open():
                    time.sleep(1.0)
                    continue
                started = False
            if not started:
                # the port may already be open (main opens it first), so this
                # has to be driven by a flag, not by the open() transition
                self.send("stream=1")
                started = True
            try:
                chunk = self.ser.read(4096)
            except Exception as exc:  # noqa: BLE001
                self.error = str(exc)
                try:
                    self.ser.close()
                except Exception:  # noqa: BLE001
                    pass
                self.ser = None
                time.sleep(0.5)
                continue
            if not chunk:
                continue
            buf += chunk

            while True:
                i = buf.find(MAGIC)
                if i < 0:
                    # keep one byte in case the magic straddles two reads
                    if len(buf) > 1:
                        del buf[:-1]
                    break
                if i:
                    del buf[:i]
                if len(buf) < 6:
                    break
                ptype = buf[2]
                length = buf[3] | (buf[4] << 8) | (buf[5] << 16)
                if length > MAX_PAYLOAD:
                    del buf[:2]          # desync: drop magic and rescan
                    continue
                if len(buf) < 6 + length:
                    break
                payload = bytes(buf[6:6 + length])
                del buf[:6 + length]
                self._handle(ptype, payload)

    def _handle(self, ptype: int, payload: bytes) -> None:
        if ptype == PKT_JPEG:
            with self.lock:
                self.frame = payload
                self.frame_id += 1
                self.frame_times.append(time.time())
        elif ptype == PKT_STATUS:
            try:
                st = json.loads(payload)
            except json.JSONDecodeError:
                return
            with self.lock:
                self.status = st

    def snapshot(self):
        with self.lock:
            return self.frame, self.frame_id

    def status_json(self) -> bytes:
        with self.lock:
            st = dict(self.status)
            now = time.time()
            recent = [t for t in self.frame_times if now - t < 2.0]
            st["pc_fps"] = round(len(recent) / 2.0, 1)
            st["link"] = f"{self.port.split('/')[-1]} @ {self.baud}"
            if self.error:
                st["link"] = "USB error: " + self.error
        return json.dumps(st).encode()


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"
    server_version = "FireBot"

    @property
    def link(self) -> Link:
        return self.server.link  # type: ignore[attr-defined]

    def log_message(self, *_args) -> None:  # keep the console clean
        pass

    def _send(self, code: int, ctype: str, body: bytes) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        path = self.path.split("?")[0]

        if path == "/":
            self._send(200, "text/html; charset=utf-8", PAGE)
            return

        if path == "/status.json":
            self._send(200, "application/json", self.link.status_json())
            return

        if path == "/snapshot":
            frame, _ = self.link.snapshot()
            if frame is None:
                self._send(503, "text/plain", b"no frame yet")
            else:
                self._send(200, "image/jpeg", frame)
            return

        if path == "/cmd":
            query = self.path.partition("?")[2]
            cmd = ""
            for part in query.split("&"):
                if part.startswith("c="):
                    from urllib.parse import unquote_plus
                    cmd = unquote_plus(part[2:])
            ok = self.link.send(cmd) if cmd else False
            self._send(200, "text/plain", b"ok" if ok else b"link down")
            return

        if path == "/stream":
            self._stream()
            return

        self._send(404, "text/plain", b"not found")

    def _stream(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last = -1
        while True:
            frame, fid = self.link.snapshot()
            if frame is None or fid == last:
                time.sleep(0.02)
                continue
            last = fid
            try:
                self.wfile.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(frame)).encode() + b"\r\n\r\n"
                )
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError, OSError):
                return


def main() -> int:
    ap = argparse.ArgumentParser(description="FireBot local dashboard over USB serial")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--http-port", type=int, default=8080)
    args = ap.parse_args()

    link = Link(args.port, args.baud)
    if not link.open():
        print(f"[warn] cannot open {args.port}: {link.error}")
        print("       will keep retrying in the background")

    t = threading.Thread(target=link.reader, daemon=True)
    t.start()

    httpd = ThreadingHTTPServer((args.host, args.http_port), Handler)
    httpd.daemon_threads = True
    httpd.link = link  # type: ignore[attr-defined]

    print(f"FireBot dashboard -> http://{args.host}:{args.http_port}")
    print(f"serial {args.port} @ {args.baud}   (Ctrl-C to stop)")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        link.running = False
        if link.ser:
            link.send("stream=0")
            link.ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
