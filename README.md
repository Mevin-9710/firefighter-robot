# FireFighter Robot 🔥🤖

An ESP32-CAM based flame-detecting, target-extinguishing robot project. The board streams live
video over its own WiFi access point, detects fire (real flames, vivid tealight candles, or
printed/hand-drawn flame drawings) on-board, and will drive motors and a pump to seek out and
extinguish targets one at a time.

**Status:** camera + detection + WiFi dashboard working. Motors/pump inhibited by default
(`auto=0`) until a chassis + motor driver exist. Chassis work is the next milestone.

---

## Hardware

| Part | Detail |
|---|---|
| Board | AI-Thinker **ESP32-CAM**, OV2640 sensor (`pid=0x26` verified) |
| Flash LED | GPIO 4, HIGH = on (white flashlight used to see in the dark) |
| Red LED | GPIO 33, active-low (stock module) |
| Motors | To be driven via a driver (L298N / TB6612) — **never direct on GPIO** |
| Pump | GPIO 12 through a transistor |
| Power | 3.7 V Li-ion cell into the 5 V pin (≈3.1 V out of the AMS1117) works but is
  marginal; brownouts were observed under camera+WiFi peaks on a weak FTDI 5 V rail.
  A proper 5 V ≥1 A source (or a boost + 470–1000 µF bulk cap) is recommended. |

### Pin budget math (learned the hard way)

On ESP32-CAM, GPIO **16/17 are PSRAM — never use them**. GPIO **12 is an SD/strapping pin**
(board keeps the SD off the reader while driving motors: remove the SD card when flashing).
Usable free pins: `2, 4, 12, 13, 14, 15, 33`. All five of `2,12,13,14,15` are already claimed
by the motor/pump plan; `4` = flashlight, `33` = red LED, so a future buzzer shares GPIO 33.

---

## Firmware (`firmware/firebot/`)

A single sketch: `firebot.ino` builds the camera pipeline, detector and web dashboard.
`detect_core.h` is the portable C++ detector (pure, no Arduino deps) so host tools
(`tools/fire_detect.py`) mirror it byte-for-byte.

### Camera pipeline (and the bugs that shaped it)

- **JPEG capture only (`PIXFORMAT_JPEG`).** Raw RGB565 bypasses the OV2640's ISP
  (denoise, lens-shading, gamma) and produced rainbow speckle. The JPEG path gives the
  clean image; preview is the sensor's own JPEG untouched — **no re-encode, no downscale**.
- **`config.jpeg_buffer_size = 65536`.** THE big one: with
  `CONFIG_CAMERA_JPEG_MODE_FRAME_SIZE_AUTO=y`, the driver sizes each frame buffer as
  `w*h/5` — only **15,360 bytes at QVGA**, smaller than a real detailed JPEG. Every frame
  then overflows (`cam_hal: FB-OVF`), the DMA dies, and you get ~0 fps forever. With an
  explicit buffer size, capture runs solidly at >20 fps. This single line fixed what a
  whole day of power-supply theorising didn't.
- **WiFi boots strictly before the camera**, otherwise sensor frames pile up unconsumed
  during radio bring-up.
- **Brownout (BOD) loops on a FTDI/USB rail** come from the above spike loop, not a
  firmware bug — as of the fixes the sketch runs fine even on USB.
- **Detection is throttled** (`DETECT_PERIOD_MS 250`): the heavy ~210 ms decode+morphology
  run 4×/s while capture keeps flowing. The phone preview used to crawl because every
  `/snapshot` paid a full capture+detect.
- **Live MJPEG stream runs on port 81 in its own FreeRTOS task**, isolated from the
  single-threaded WebServer on port 80, so video never starves `/status.json`.

### Detection (`detect_core.h`)

Grid-space blob detection on the decoded frame:

1. **Hue channel** — red-orange-yellow band + wrap-around red band; `s`/`v` floors and an
   R-dominance rule (`r`, `rb=R−B`, `rg=R−G`) tuned to reject skin, dim inks, and pink.
   The magenta pass (used in earlier cup hunts) is now off.
2. **Glow channel** (`gv`, `gs`) — pixels ≥ `gv` brightness and ≤ `gs` saturation count as
   fire even without hue: real flame cores blow out near-white through the OV2640, so the
   white core joins its coloured fringe into a single target. Disabled in the `print`
   profile (white paper background) via `gv=255`.
3. **Flicker gate** (`flk=N`, off by default) — colour statistics cannot tell a red dress
   from a flame. When `flk` is set, a blob counts only if ≥N% of its cells moved ≥24
   brightness levels since the previous pass (a 1/4-res brightness map is kept). Enable it
   against real flames; **leave off for printed targets** (they're static).
4. **Blob filters** — min/max area fraction, fill ratio, aspect window, distance sort,
   nearest target first; "already extinguished" spots suppressed for 15 s (`suppress=`).

### Tuning presets

| profile | use case |
|---|---|
| `candle` | tealight flames — flame-colour only, cup body rejected |
| `print`  | printed/hand-drawn flames on paper |
| `both`   | union preset (boot default) |
| `flame`  | real fire anywhere in frame |

### Control keys (serial or the dashboard's command access)

`show reset pump= profile= auto= suppress= stream= led= q=4..63 swap= size=vga|qvga raw`
plus tuning keys `h1 h2lo s v rb rg gv gs flk area fill asp focal tw/fw ext gain aec autoexp bright`.

### WiFi dashboard

Boot default: the board publishes its own AP (`FireBot-CAM` / `firebot123`) and serves
`http://192.168.4.1`:
- **Live video** from port 81 (canvas overlay draws target boxes — nothing is burned in,
  so the stream stays pixel-sharp).
- **Status panel**: fps, targets, nearest distance, bearing, state, timing breakdown.
- **Buttons**: flashlight on/off, voice feedback toggle, profiles, auto mode, pump test.
- **Voice**: browser-native TTS *on the phone/PC* — the board only sends the same tiny
  status JSON as before, costing the sketch ~1 KB. Announces "Flame detected",
  "Extinguishing target", "Target eliminated". (The robot has no speaker yet; a buzzer
  share of GPIO 33 is planned.)

Dashboard daemon for PC/no-WiFi days: `tools/dashboard.py` talks serial on `/dev/ttyUSB0`
and serves the identical UI on `127.0.0.1:8080`.

---

## Tooling (`tools/`)

| file | purpose |
|---|---|
| `dashboard.html` | single-page dashboard UI, embedded into the sketch at flash time |
| `dashboard.py`   | PC serial↔browser bridge for wired testing |
| `fire_detect.py` | Python reference impl of the detector (OpenCV), parameter-identical |
| `candle_test.py` | threshold sweeps across candle/cup target images |
| `robustness_test.py` | printed-flame robustness sweeps |
| `rawdump.py`     | raw RGB565 frame dump for byte-order verification |

**Important**: `dashboard.html` is the source of truth for `PAGE_HTML` in the sketch —
edit it there, then re-embed and recompile.

---

## Build & flash

```bash
~/.local/bin/arduino-cli compile --fqbn esp32:esp32:esp32cam firmware/firebot
~/.local/bin/arduino-cli upload  -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32cam firmware/firebot
```

Flashing checklist:
- **IO0 → GND jumper at power-up** (bootloader waits indefinitely).
- **Remove the SD card** — GPIO 12 is a strapping pin used by the SD reader.
- Only one process may hold `/dev/ttyUSB0` (kill the dashboard with `fuser -k 8080/tcp`).
- After `upload --verify`, `No changed sectors found` is normal, not an error.

---

## Safety 🔥

Never run this project with real flames unattended. Keep water/sand nearby, work on a
fireproof surface, and never leave a lit candle or the robot's pump unsupervised.

## Roadmap

- [x] Clean camera frames (JPEG path, correct frame buffer size)
- [x] Flame-only detection (hue + glow + flicker gate)
- [x] SoftAP dashboard with live video + TTS
- [ ] Chassis + motor driver wiring (pins already reserved: 2/13/14/15 + PCA9685 idea)
- [ ] Pump transistor circuit
- [ ] Auto seek/align/extinguish loop field-tested
- [ ] Buzzer on GPIO 33 (shared with the red LED)
