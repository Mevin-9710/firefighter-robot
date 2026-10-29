/*
 * FireBot -- target detection + live preview
 * ----------------------------------------------------
 * Target: AI-Thinker ESP32-CAM (ESP32-D0WD-V3, 4MB flash + PSRAM)
 *
 * Detects coloured targets by HSV thresholds, finds every blob, estimates
 * distance from apparent width, and drives a hunt state machine.
 *
 * Transport: the detection frame and status are pushed over the USB serial
 * link as length-prefixed binary packets.  tools/dashboard.py runs on the PC,
 * decodes them and serves the dashboard on http://127.0.0.1:8080.  This works
 * with no WiFi anywhere in the system.
 *
 * Set ENABLE_WIFI to 1 to also serve the dashboard directly from the board's
 * own access point (needs ~670 KB more flash and a WiFi-capable client).
 *
 * All the detection maths lives in detect_core.h, which is also compiled on the
 * host by tools/host_test.cpp and checked against tools/fire_detect.py.
 *
 * Serial: SERIAL_BAUD below.  Commands are newline-terminated ASCII, the same
 * ones the HTTP /cmd endpoint accepts.
 */

#define ENABLE_WIFI 1        // 1 = board serves its own dashboard over WiFi (softAP)
#define SERIAL_BAUD 921600   // PC <-> board link; 2 Mbaud also works (see README)

#include "esp_camera.h"
#include "img_converters.h"
#include <new>
#include "esp_heap_caps.h"
#include "detect_core.h"
#if ENABLE_WIFI
#include <WiFi.h>
#include <WebServer.h>
#include "wifi_secrets.h"
#endif

// ---------------------------------------------------------------------------
// Camera pin map -- AI-Thinker ESP32-CAM
// ---------------------------------------------------------------------------
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// ---------------------------------------------------------------------------
// Actuator pins.
//
// The ESP32-CAM has almost no free GPIOs: 16/17 are PSRAM, 0/12 are strapping
// pins, and 1/3 are the serial console.  That leaves 2, 4, 13, 14, 15.
// For the real chassis use a PCA9685 (I2C PWM expander) for the motors rather
// than fighting for pins here.
// ---------------------------------------------------------------------------
#define FLASH_LED_PIN 4
#define MOTOR_IN1 13
#define MOTOR_IN2 14
#define MOTOR_IN3 15
#define MOTOR_IN4 2
// GPIO12 is a strapping pin: it must be LOW at boot or the flash is configured
// for 1.8 V and the board will not start.  Most opto-isolated relay boards pull
// their input low, so this usually works -- but if the board stops booting once
// the pump is wired, that is the cause.
#define PUMP_PIN 12

// ---------------------------------------------------------------------------
// The working grid is ALWAYS 320x240 regardless of sensor mode, so every
// buffer size is fixed at boot and the resolution can be switched at runtime:
//   VGA  640x480 with step 2   -- more sensor detail, but the OV2640 delivers
//   QVGA 320x240 with step 1      it at only ~2 fps in RGB565
// Both land on the same 320x240 cells, so this costs no memory.
#define GRID_W 320
#define GRID_H 240
#define GRID_CELLS (GRID_W * GRID_H)
static int g_step = 1;          // frame pixels per grid cell
static int g_frameW = 320;
static int g_frameH = 240;
static int g_gridW = GRID_W;
static int g_gridH = GRID_H;
static int g_framesize = FRAMESIZE_QVGA;
static bool g_swap565 = false;   // byte order of the decoded RGB565 buffer
static int g_scale = JPG_SCALE_NONE;

#define PREVIEW_W GRID_W
#define PREVIEW_H GRID_H

#define MAX_BLOBS 8
// How often the expensive detector half runs.  The capture path still runs on
// every loop iteration, so this only limits detection, not the video feed.
#define DETECT_PERIOD_MS 250

// Frame sequence: odd while processFrame is writing g_jpeg, even when stable.
// The stream task only reads on even values, so it never copies a torn frame.
volatile uint32_t g_jpegSeq = 0;

#define JPEG_CAP (128 * 1024)
#define JPEG_QUALITY 40           // boot default; runtime override is `q=<10..80>`
static int g_quality = JPEG_QUALITY;
static bool g_led = false;        // flash LED: GPIO4 HIGH = ON
static int g_sensorPid = -1;      // 0x26 = OV2640, 0x77 = OV7725, ...

static uint8_t *g_maskBuf, *g_scratchBuf;
static int32_t *g_stackBuf;
static firecore::Detector *g_det = nullptr;
static uint8_t *g_prev = nullptr;   // 320x240 RGB565: decoded frame + grid
static uint8_t *g_jpeg = nullptr;   // last JPEG frame, for the preview stream
static size_t g_jpegLen = 0;
static firecore::Blob g_blobs[MAX_BLOBS];
static int g_blobCount = 0;
static int g_targetIdx = -1;

// ---------------------------------------------------------------------------
// Runtime switches
// ---------------------------------------------------------------------------
static const char *g_profile = "both";
static bool g_auto = false;       // state machine drives motors/pump
static bool g_suppress = false;   // hide already-extinguished spots
static bool g_stream = false;     // push binary frames down the serial link
static float g_fps = 0;
static uint16_t g_tCap = 0, g_tDet = 0, g_tEnc = 0;   // ms per stage

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
enum State { ST_SEARCH, ST_TRACK, ST_ALIGN, ST_EXTINGUISH, ST_COOLDOWN };
static State g_state = ST_SEARCH;
static unsigned long g_stateSince = 0;

static const char *stateName(State s) {
  switch (s) {
    case ST_SEARCH: return "SEARCH";
    case ST_TRACK: return "TRACK";
    case ST_ALIGN: return "ALIGN";
    case ST_EXTINGUISH: return "EXTINGUISH";
    default: return "COOLDOWN";
  }
}
static void enterState(State s) { g_state = s; g_stateSince = millis(); }

// --- flicker gate ------------------------------------------------------------
// Colour alone cannot tell a flame from a red dress or a still orange object.
// What flame has that static things don't: its brightness pattern visibly
// changes between detections.  A 1/4-resolution brightness copy is kept of the
// last detection pass; a blob counts as flickering when >= g_flk percent of
// its cells changed by 24+ grey levels.  g_flk=0 disables (printed flames are
// static, so it must be optional).
#define VMAP_W (PREVIEW_W / 4)
#define VMAP_H (PREVIEW_H / 4)
static uint8_t *g_vmap;      // 1/4-res last-frame brightness
static int g_flick[MAX_BLOBS]; // percent changed, per blob, this pass
static int g_flk = 0;

static inline uint8_t cellV8(int gx, int gy) {
  const uint8_t *row = g_prev + (size_t)gy * PREVIEW_W * 2;
  uint16_t v = g_swap565
                 ? (uint16_t)(((uint16_t)row[gx * 2] << 8) | row[gx * 2 + 1])
                 : (uint16_t)((uint16_t)row[gx * 2] | ((uint16_t)row[gx * 2 + 1] << 8));
  uint8_t r = firecore::expand5((v >> 11) & 0x1F);
  uint8_t gg = firecore::expand6((v >> 5) & 0x3F);
  uint8_t b = firecore::expand5(v & 0x1F);
  uint8_t m = r; if (gg > m) m = gg; if (b > m) m = b;
  return m;
}

static void measureFlicker() {
  // 1. score each blob against the previous pass' map
  for (int b = 0; b < g_blobCount; b++) {
    firecore::Blob &bl = g_blobs[b];
    int gx0 = bl.x / 4, gy0 = bl.y / 4;
    int gx1 = (bl.x + bl.w) / 4, gy1 = (bl.y + bl.h) / 4;
    gx1 = (gx1 < VMAP_W) ? gx1 : VMAP_W - 1;
    gy1 = (gy1 < VMAP_H) ? gy1 : VMAP_H - 1;
    long changed = 0, total = 0;
    for (int gy = gy0; gy <= gy1; gy++) {
      for (int gx = gx0; gx <= gx1; gx++) {
        total++;
        if (abs((int)cellV8(gx, gy) - (int)g_vmap[(gy >> 2) * VMAP_W + (gx >> 2)]) >= 24)
          changed++;
      }
    }
    g_flick[b] = (total > 0) ? (int)((changed * 100) / total) : 100;
  }
  // 2. refresh the map for the next pass
  for (int vy = 0; vy < VMAP_H; vy++) {
    uint8_t *out = g_vmap + (size_t)vy * VMAP_W;
    for (int vx = 0; vx < VMAP_W; vx++)
      out[vx] = cellV8(vx * 4, vy * 4);
  }
}

// "already put out" memory (image-space, 15 s)
struct Done { int cx, cy; unsigned long until; };
static Done g_done[4];
static int g_doneCount = 0;
static const unsigned long DONE_HOLD_MS = 15000;

static bool isSuppressed(const firecore::Blob &b) {
  if (!g_suppress) return false;
  unsigned long now = millis();
  for (int i = 0; i < g_doneCount; i++) {
    if ((long)g_done[i].until - (long)now <= 0) continue;
    int dx = b.cx - g_done[i].cx;
    int dy = b.cy - g_done[i].cy;
    if (dx * dx + dy * dy < 70 * 70) return true;
  }
  return false;
}

static void markDone(const firecore::Blob &b) {
  if (g_doneCount >= 4) {
    memmove(g_done, g_done + 1, sizeof(Done) * 3);
    g_doneCount = 3;
  }
  g_done[g_doneCount].cx = b.cx;
  g_done[g_doneCount].cy = b.cy;
  g_done[g_doneCount].until = millis() + DONE_HOLD_MS;
  g_doneCount++;
}

// ---------------------------------------------------------------------------
// Mask build: RGB565 -> 1 bit per grid cell
// ---------------------------------------------------------------------------
// `src` is always a 320x240 RGB565 buffer (decoded from JPEG at 1/2 scale).
static bool buildMask(const uint8_t *src, int srcW) {
  const int rowBytes = srcW * 2;
  uint8_t *m = g_det->mask;
  const firecore::Config &cfg = g_det->cfg;
  int g = 0;

  for (int gy = 0; gy < g_gridH; gy++) {
    const uint8_t *row = src + (size_t)gy * rowBytes;
    for (int gx = 0; gx < g_gridW; gx++) {
      int px = gx;
      uint16_t v = g_swap565
                     ? (uint16_t)(((uint16_t)row[px * 2] << 8) | row[px * 2 + 1])
                     : (uint16_t)((uint16_t)row[px * 2] | ((uint16_t)row[px * 2 + 1] << 8));
      uint8_t r = firecore::expand5((v >> 11) & 0x1F);
      uint8_t gg = firecore::expand6((v >> 5) & 0x3F);
      uint8_t b = firecore::expand5(v & 0x1F);
      m[g++] = firecore::isFirePixel(r, gg, b, cfg) ? 1 : 0;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Pull one frame into the 320x240 detection grid.
//
// The sensor gives us JPEG; jpg2rgb565() downscales by 1/2 in the decoder, so
// a 640x480 capture lands on exactly 320x240 cells for free.
// ---------------------------------------------------------------------------
static bool decodeToGrid(const uint8_t *buf, size_t len) {
  return jpg2rgb565(buf, len, g_prev, (esp_jpeg_image_scale_t)g_scale);
}

// ---------------------------------------------------------------------------
// Sensor tuning.
// ---------------------------------------------------------------------------
static void tuneSensor() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) { Serial.println("no sensor"); return; }
  // JPEG mode gives us the sensor's own denoise and lens-shading correction,
  // so the pipeline is left auto like the stock camera example.
  s->set_gainceiling(s, GAINCEILING_4X);
  s->set_gain_ctrl(s, 1);
  s->set_exposure_ctrl(s, 1);
  s->set_ae_level(s, 0);
  s->set_brightness(s, 0);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_bpc(s, 1);                // black pixel correction
  s->set_wpc(s, 1);                // white pixel correction
  s->set_lenc(s, 1);               // lens shading correction
  g_sensorPid = s->id.PID;
  Serial.printf("sensor pid=0x%02x\n", s->id.PID);
}

// ---------------------------------------------------------------------------
// Serial transport.
//
//   A5 5A | type | len (3 bytes LE) | payload
//
//   type 1 = JPEG frame, type 2 = status JSON
//
// A 6-byte header plus a length is enough for the PC to resynchronise after a
// dropped byte: it scans for the magic, then trusts the length.  Magic is
// 0xA5 0x5A, which cannot appear inside the ASCII command echo.
// ---------------------------------------------------------------------------
#define PKT_JPEG 1
#define PKT_STATUS 2
#define PKT_RAW 3              // one-shot raw framebuffer dump
static bool g_rawOnce = false;

static void writePacket(uint8_t type, const uint8_t *data, size_t len) {
  uint8_t hdr[6] = {0xA5, 0x5A, type,
                    (uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF),
                    (uint8_t)((len >> 16) & 0xFF)};
  Serial.write(hdr, sizeof(hdr));
  Serial.write(data, len);
}

// Shared by the serial link and the optional WiFi dashboard.
static int buildStatus(char *buf, size_t cap) {
  firecore::Config &c = g_det->cfg;
  int n = snprintf(buf, cap,
                   "{\"fire\":%s,\"n\":%d,\"tgt\":%d,\"profile\":\"%s\",\"st\":\"%s\","
                   "\"res\":\"%dx%d\",\"pid\":%d,\"fps\":%.1f,\"auto\":%d,\"suppress\":%d,\"uptime\":%lu,\"free\":%u,"
                   "\"led\":%d,\"t\":{\"cap\":%u,\"det\":%u,\"enc\":%u},"
                   "\"cfg\":{\"s\":%d,\"v\":%d,\"rb\":%d,\"rg\":%d,\"area\":%.5f,"
                   "\"fill\":%.2f,\"asp\":%.2f,\"aspmax\":%.2f,\"tw\":%.0f,\"ext\":%.0f},"
                   "\"blobs\":[",
                   (g_blobCount > 0 && g_targetIdx >= 0) ? "true" : "false",
                   g_blobCount, g_targetIdx, g_profile, stateName(g_state),
                   g_frameW, g_frameH,
                   g_sensorPid,
                   g_fps, (int)g_auto, (int)g_suppress,
                   millis() / 1000, (unsigned)ESP.getFreeHeap(),
                   (int)g_led,
                   (unsigned)g_tCap, (unsigned)g_tDet, (unsigned)g_tEnc,
                   c.sMin, c.vMin, c.rOverB, c.rOverG, c.minAreaFrac,
                   c.minFill, c.minAspect, c.maxAspect, c.targetWidthMm, c.extinguishMm);
  if (n < 0 || n >= (int)cap) return 0;
  for (int i = 0; i < g_blobCount && n < (int)cap - 120; i++) {
    const firecore::Blob &b = g_blobs[i];
    n += snprintf(buf + n, cap - n,
                  "%s{\"cx\":%d,\"cy\":%d,\"w\":%d,\"h\":%d,\"d\":%d,\"err\":%d,\"sup\":%d}",
                  i ? "," : "", b.cx, b.cy, b.w, b.h,
                  (int)(b.distanceMm / 10.0f), b.cx - g_frameW / 2,
                  isSuppressed(b) ? 1 : 0);
    if (n < 0 || n >= (int)cap) return 0;
  }
  int m = snprintf(buf + n, cap - n, "]}");
  if (m < 0) return 0;
  return n + m;
}

// ---------------------------------------------------------------------------
// Actuators
// ---------------------------------------------------------------------------
static void drive(int steer, int speed) {
  if (!g_auto) return;
  // steer -100..100, speed 0..100.  Simple differential mixing.
  int l = speed + steer / 2;
  int r = speed - steer / 2;
  l = constrain(l, -100, 100);
  r = constrain(r, -100, 100);
  digitalWrite(MOTOR_IN1, l > 0);
  digitalWrite(MOTOR_IN2, l < 0);
  digitalWrite(MOTOR_IN3, r > 0);
  digitalWrite(MOTOR_IN4, r < 0);
}

static void pump(bool on) { digitalWrite(PUMP_PIN, on ? HIGH : LOW); }

// ---------------------------------------------------------------------------
// Threshold presets -- single source of truth in detect_core.h
// ---------------------------------------------------------------------------
static void printConfig();
static bool initCamera(int framesize);
static void restartCamera(int framesize);

static void applyProfile(const char *name) {
  if (!firecore::applyPreset(g_det->cfg, name)) {
    Serial.printf("unknown profile '%s'\n", name);
    return;
  }
  // Presets are written for a 640-wide frame; the pinhole model has to follow
  // whatever resolution the sensor is actually running at.
  g_det->cfg.focalPx *= (float)g_frameW / 640.0f;
  g_profile = name;
  Serial.printf("profile=%s\n", g_profile);
  printConfig();
}

static void printConfig() {
  firecore::Config &c = g_det->cfg;
  Serial.printf("cfg profile=%s h1=%d h2lo=%d s=%d v=%d rb=%d rg=%d gv=%d gs=%d\n",
                g_profile, c.hHi1, c.hLo2, c.sMin, c.vMin, c.rOverB, c.rOverG);
  Serial.printf("cfg area=%.5f fill=%.3f asp=%.2f..%.2f focal=%.0f tw=%.0f ext=%.0f\n",
                c.minAreaFrac, c.minFill, c.minAspect, c.maxAspect,
                c.focalPx, c.targetWidthMm, c.extinguishMm);
}

// ---------------------------------------------------------------------------
// One command string, shared by serial and HTTP
// ---------------------------------------------------------------------------
static void handleCommand(String s) {
  s.trim();
  if (s.length() == 0) return;
  firecore::Config &c = g_det->cfg;

  if (s == "show") {
    printConfig();
  } else if (s == "reset") {
    g_doneCount = 0;
    enterState(ST_SEARCH);
    Serial.println("reset ok");
  } else if (s.startsWith("pump=")) {
    pump(s.substring(5).toInt() != 0);
    Serial.println("pump ok");
  } else if (s.startsWith("profile=")) {
    String p = s.substring(8);
    p.trim();
    applyProfile(p.c_str());
  } else if (s.startsWith("auto=")) {
    g_auto = s.substring(5).toInt() != 0;
    if (!g_auto) { drive(0, 0); pump(false); }
    Serial.printf("auto=%d\n", (int)g_auto);
  } else if (s.startsWith("suppress=")) {
    g_suppress = s.substring(9).toInt() != 0;
    Serial.printf("suppress=%d\n", (int)g_suppress);
  } else if (s.startsWith("stream=")) {
    g_stream = s.substring(7).toInt() != 0;
    Serial.printf("stream=%d\n", (int)g_stream);
  } else if (s.startsWith("led=")) {
    g_led = s.substring(4).toInt() != 0;
    digitalWrite(FLASH_LED_PIN, g_led ? HIGH : LOW);
    Serial.printf("led=%d\n", (int)g_led);
  } else if (s == "raw") {
    g_rawOnce = true;
    Serial.println("raw dump queued");
  } else if (s.startsWith("q=")) {
    int q = s.substring(2).toInt();
    if (q < 4) q = 4;
    if (q > 63) q = 63;
    g_quality = q;
    sensor_t *sq = esp_camera_sensor_get();   // 0-63, lower = better
    if (sq) sq->set_quality(sq, q);
    Serial.printf("q=%d\n", g_quality);
  } else if (s.startsWith("size=")) {
    String m = s.substring(5);
    m.trim();
    int fs = -1;
    if (m == "qvga") { fs = FRAMESIZE_QVGA; g_step = 1; }
    else if (m == "vga") { fs = FRAMESIZE_VGA; g_step = 2; }
    else { Serial.println("size must be vga or qvga"); return; }
    restartCamera(fs);
  } else {
    int eq = s.indexOf('=');
    if (eq > 0) {
      String k = s.substring(0, eq);
      float v = s.substring(eq + 1).toFloat();
      if (k == "h1") c.hHi1 = (int)v;
      else if (k == "h2lo") c.hLo2 = (int)v;
      else if (k == "s") c.sMin = (int)v;
      else if (k == "v") c.vMin = (int)v;
      else if (k == "rb") c.rOverB = (int)v;
      else if (k == "rg") c.rOverG = (int)v;
      else if (k == "gv") c.glowV = (int)v;
      else if (k == "gs") c.glowSMax = (int)v;
      else if (k == "flk") g_flk = (int)v;
      else if (k == "area") c.minAreaFrac = v;
      else if (k == "fill") c.minFill = v;
      else if (k == "asp") c.minAspect = v;
      else if (k == "focal") c.focalPx = v;
      else if (k == "tw" || k == "fw") c.targetWidthMm = v;
      else if (k == "ext") c.extinguishMm = v;
      else if (k == "gain") {
        // 0=2x 1=4x 2=8x 3=16x 4=32x 5=64x 6=128x
        sensor_t *s = esp_camera_sensor_get();
        if (s) s->set_gainceiling(s, (gainceiling_t)(int)v);
      } else if (k == "aec") {
        sensor_t *s = esp_camera_sensor_get();
        if (s) { s->set_exposure_ctrl(s, 0); s->set_aec_value(s, (int)v); }
      } else if (k == "autoexp") {
        sensor_t *s = esp_camera_sensor_get();
        if (s) s->set_exposure_ctrl(s, (int)v);
      } else if (k == "bright") {
        sensor_t *s = esp_camera_sensor_get();
        if (s) s->set_brightness(s, (int)v);
      } else Serial.printf("unknown key '%s'\n", k.c_str());
    }
  }
}

static void pollSerial() {
  static char line[96];
  static int len = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (len) { line[len] = 0; handleCommand(String(line)); }
      len = 0;
    } else if (len < (int)sizeof(line) - 1) {
      line[len++] = ch;
    }
  }
}

// ---------------------------------------------------------------------------
// Camera
// ---------------------------------------------------------------------------
static bool initCamera(int framesize) {
  camera_config_t c;
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  // ESP-FAQ: a small resolution with a large XCLK makes the sensor's frame
  // sync fire too fast and overruns the DMA.  QVGA at 10 MHz is a matched pair.
  c.xclk_freq_hz = 20000000;
  // JPEG is the only mode in which the OV2640 runs its full image pipeline
  // (denoise, lens shading, gamma, AWB).  RGB565 output switches the sensor to
  // a reduced register set and is visibly grainy -- and the driver README says
  // not to use sizes above QVGA when not JPEG.  We decode the JPEG ourselves.
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = (framesize_t)framesize;
  c.jpeg_quality = 12;
  // THE bug we chased for hours: the driver sizes each frame buffer as
  // width*height/5 when CONFIG_CAMERA_JPEG_MODE_FRAME_SIZE_AUTO is set (it is).
  // At QVGA that is only 15,360 bytes -- smaller than a real JPEG of a detailed
  // scene, so cam_hal logs "FB-OVF", calls ll_cam_stop() and drops the DMA on
  // *every* frame.  At VGA the auto size is 61,440 bytes, which is why VGA
  // appeared to work.  Override it explicitly with jpeg_buffer_size.
  c.jpeg_buffer_size = 65536;
  // Stock security-cam config: two buffers, latest frame wins.
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("camera init failed 0x%x\n", err);
    return false;
  }
  return true;
}

// Changing resolution means a full deinit/init: the driver allocates its
// framebuffers once at init and refuses to resize them in place (it otherwise
// floods the log with "FB-SIZE: ... != ..." forever).
static void restartCamera(int framesize) {
  esp_camera_deinit();
  delay(200);
  if (!initCamera(framesize)) {
    Serial.println("camera reinit failed");
    return;
  }
  g_framesize = framesize;
  if (framesize == FRAMESIZE_VGA) {
    g_frameW = 640; g_frameH = 480; g_scale = JPG_SCALE_2X;
  } else {
    g_frameW = 320; g_frameH = 240; g_scale = JPG_SCALE_NONE;
  }
  // the decoder always lands on the same 320x240 grid
  g_gridW = GRID_W;
  g_gridH = GRID_H;
  g_step = g_frameW / GRID_W;
  g_det->gw = g_gridW;
  g_det->gh = g_gridH;
  g_det->step = g_step;
  applyProfile(g_profile);
  Serial.printf("size frame=%dx%d step=%d grid=%dx%d\n",
                g_frameW, g_frameH, g_step, g_gridW, g_gridH);
}

// ---------------------------------------------------------------------------
// Temporal persistence.  A real target is still there next frame; a speck of
// sensor noise usually is not.  This lets the area floor stay low enough to see
// a 38 mm cup at 4 m without boxing every bright pixel in a lit room.
// ---------------------------------------------------------------------------
#define PERSIST_FRAMES 2
struct Seen { int cx, cy, hits; };
static Seen g_seen[MAX_BLOBS];
static int g_seenN = 0;

static void filterPersistent() {
  Seen next[MAX_BLOBS];
  int nextN = 0;
  int kept = 0;
  for (int i = 0; i < g_blobCount; i++) {
    const firecore::Blob &b = g_blobs[i];
    int tol = b.w / 2 + 12;
    if (tol < 24) tol = 24;
    int hits = 1;
    for (int j = 0; j < g_seenN; j++) {
      if (abs(b.cx - g_seen[j].cx) <= tol && abs(b.cy - g_seen[j].cy) <= tol) {
        hits = g_seen[j].hits + 1;
        if (hits > 255) hits = 255;
        break;
      }
    }
    if (nextN < MAX_BLOBS) {
      next[nextN].cx = b.cx;
      next[nextN].cy = b.cy;
      next[nextN].hits = hits;
      nextN++;
    }
    if (hits >= (int)PERSIST_FRAMES) g_blobs[kept++] = b;
  }
  g_blobCount = kept;
  memcpy(g_seen, next, sizeof(Seen) * nextN);
  g_seenN = nextN;
}

// ---------------------------------------------------------------------------
// One detection pass.  needJpeg=false keeps the encode out of the hot path.
// ---------------------------------------------------------------------------
static unsigned long g_lastFrame = 0;
static unsigned long g_lastPrint = 0;

static void updateLed() {
  digitalWrite(FLASH_LED_PIN, g_led ? HIGH : LOW);   // GPIO4 HIGH = flash on
}

static void processFrame(bool needJpeg) {
  unsigned long t0 = millis();
  static int grabFails = 0;
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    delay(20);
    // Never leave the board stuck: if the sensor cannot deliver at this
    // resolution, drop to QVGA and keep working.
    if (++grabFails == 25 && g_frameW > 320) {
      Serial.println("no frames at this size, falling back to QVGA");
      g_step = 1;
      restartCamera(FRAMESIZE_QVGA);
      grabFails = 0;
    }
    return;
  }
  grabFails = 0;
  unsigned long t1 = millis();

  // Take a copy and hand the frame buffer straight back.  Holding it while we
  // decode and scan starves the driver: with fb_count=2 the sensor has nowhere
  // to put the next frame, which is exactly what "FB-OVF" reports.
  bool haveJpeg = (fb->format == PIXFORMAT_JPEG && fb->len <= JPEG_CAP);
  if (haveJpeg) {
    g_jpegSeq++;                       // odd: writing
    memcpy(g_jpeg, fb->buf, fb->len);
    g_jpegLen = fb->len;
    g_jpegSeq++;                       // even: stable
  }
  esp_camera_fb_return(fb);
  fb = nullptr;

  // Decode + morphology + blob scan costs ~210 ms -- far more than frame
  // delivery.  Running it on every frame is what made the preview crawl, so
  // only run it a few times a second and let the capture path spin freely in
  // between.  Detection stays far faster than anything the robot can act on.
  static unsigned long lastDetect = 0;
  bool decoded = false;
  unsigned long now = millis();
  unsigned long t2 = now;

  if (haveJpeg && (g_rawOnce || now - lastDetect >= DETECT_PERIOD_MS)) {
    lastDetect = now;
    decoded = decodeToGrid(g_jpeg, g_jpegLen);
    if (decoded && buildMask(g_prev, PREVIEW_W)) {
      g_det->morphOpenClose();
      g_blobCount = g_det->findBlobs(g_blobs, MAX_BLOBS);
      measureFlicker();
      filterPersistent();
      firecore::Detector::sortByDistance(g_blobs, g_blobCount);
    } else {
      g_blobCount = 0;
    }
    t2 = millis();

    g_targetIdx = -1;
    for (int i = 0; i < g_blobCount; i++) {
      if (isSuppressed(g_blobs[i])) continue;
      if (g_flk > 0 && g_flick[i] < g_flk) continue;  // static: not fire
      g_targetIdx = i;
      break;
    }

  firecore::Blob *target = (g_targetIdx >= 0) ? &g_blobs[g_targetIdx] : nullptr;

  if (g_auto) {
    switch (g_state) {
      case ST_SEARCH:
        if (target) enterState(ST_TRACK);
        break;
      case ST_TRACK:
        if (!target) { enterState(ST_SEARCH); break; }
        drive(target->cx - g_frameW / 2, 60);
        if (target->distanceMm <= g_det->cfg.extinguishMm) enterState(ST_ALIGN);
        break;
      case ST_ALIGN: {
        if (!target) { enterState(ST_SEARCH); break; }
        int err = target->cx - g_frameW / 2;
        drive(err, 0);
        if (abs(err) < 25) { pump(true); enterState(ST_EXTINGUISH); }
        break;
      }
      case ST_EXTINGUISH:
        drive(0, 0);
        if (now - g_stateSince > 1500) {
          pump(false);
          if (target) markDone(*target);
          enterState(ST_COOLDOWN);
        }
        break;
      case ST_COOLDOWN:
        drive(0, 0);
        if (now - g_stateSince > 600) enterState(ST_SEARCH);
        break;
    }
  }

    updateLed();
  }

  if (now > g_lastFrame) g_fps = 1000.0f / (float)(now - g_lastFrame);
  g_lastFrame = now;

  if (g_rawOnce) {
    g_rawOnce = false;
    // 5-byte binary header, then the framebuffer -- written separately so we
    // never need a second full-frame buffer in RAM
    // dump the decoded grid buffer so the PC can verify the byte order
    size_t rlen = decoded ? (size_t)PREVIEW_W * PREVIEW_H * 2 : 0;
    uint8_t meta[5] = {(uint8_t)(PREVIEW_W & 0xFF), (uint8_t)(PREVIEW_W >> 8),
                       (uint8_t)(PREVIEW_H & 0xFF), (uint8_t)(PREVIEW_H >> 8),
                       (uint8_t)PIXFORMAT_RGB565};
    size_t total = 5 + rlen;
    uint8_t hdr[6] = {0xA5, 0x5A, PKT_RAW,
                      (uint8_t)(total & 0xFF), (uint8_t)((total >> 8) & 0xFF),
                      (uint8_t)((total >> 16) & 0xFF)};
    Serial.write(hdr, sizeof(hdr));
    Serial.write(meta, sizeof(meta));
    if (rlen) Serial.write(g_prev, rlen);
    Serial.flush();
    Serial.printf("raw %dx%d %u bytes\n", PREVIEW_W, PREVIEW_H, (unsigned)rlen);
  }

  // The preview is the sensor's own JPEG, untouched: full ISP quality, no
  // re-encode, no downscale blur.  Boxes are an overlay in the browser.
  if (needJpeg && g_stream && g_jpegLen) writePacket(PKT_JPEG, g_jpeg, g_jpegLen);
  unsigned long t3 = millis();
  g_tCap = (uint16_t)(t1 - t0);
  g_tDet = (uint16_t)(t2 - t1);
  g_tEnc = (uint16_t)(t3 - t2);

  if (g_stream) {
    // Status goes out more often than video: the numbers stay responsive even
    // when the serial link is the bottleneck.
    static unsigned long lastStatus = 0;
    if (now - lastStatus >= 150) {
      lastStatus = now;
      static char sbuf[1200];
      int n = buildStatus(sbuf, sizeof(sbuf));
      if (n > 0) writePacket(PKT_STATUS, (const uint8_t *)sbuf, n);
    }
  } else if (now - g_lastPrint >= 1000) {
    g_lastPrint = now;
    if (g_targetIdx >= 0) {
      firecore::Blob &tg = g_blobs[g_targetIdx];
      Serial.printf("FIRE n=%d tgt=%d cx=%d cy=%d w=%d h=%d d=%dcm err=%d fk=%d st=%s fps=%.1f t=%u/%u/%u\n",
                    g_blobCount, g_targetIdx, tg.cx, tg.cy,
                    tg.w, tg.h, (int)(tg.distanceMm / 10.0f),
                    tg.cx - g_frameW / 2, g_flick[g_targetIdx], stateName(g_state), g_fps,
                    (unsigned)g_tCap, (unsigned)g_tDet, (unsigned)g_tEnc);
    } else {
      Serial.printf("NOFIRE blobs=%d st=%s fps=%.1f t=%u/%u/%u\n", g_blobCount,
                    stateName(g_state), g_fps,
                    (unsigned)g_tCap, (unsigned)g_tDet, (unsigned)g_tEnc);
    }
  }
}

// ---------------------------------------------------------------------------
// Optional WiFi dashboard (ENABLE_WIFI 1)
// ---------------------------------------------------------------------------
#if ENABLE_WIFI
static WebServer server(80);

// --- live MJPEG stream on port 81 -------------------------------------------
// The main WebServer is single-threaded: while it streams, it cannot answer
// /status.json.  So the video feed gets its own WiFiServer and its own task,
// on its own port.  It only ever copies from g_jpeg when g_jpegSeq is even,
// i.e. while processFrame is not mid-write.
static WiFiServer streamSrv(81);

static void streamTask(void *) {
  streamSrv.begin();
  WiFiClient c;
  for (;;) {
    WiFiClient n = streamSrv.accept();
    if (n) { c.stop(); c = n; c.setTimeout(3000); }
    if (!c || !c.connected()) { delay(20); continue; }

    c.print("HTTP/1.1 200 OK\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
            "Cache-Control: no-store\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Connection: close\r\n\r\n");

    uint32_t lastSeq = 0;
    unsigned long lastWrite = 0;
    while (c.connected()) {
      if ((g_jpegSeq & 1) || g_jpegSeq == lastSeq) {
        // waiting on the detector loop; pace the task so it never spins
        delay(5);
        continue;
      }
      if (millis() - lastWrite < 40) {   // cap the wire at ~25 fps of effort
        delay(1);
        continue;
      }
      lastSeq = g_jpegSeq;
      uint16_t len = g_jpegLen;
      if (!len) { delay(20); continue; }
      lastWrite = millis();
      if (c.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                   (unsigned)len) <= 0) break;
      if (c.write(reinterpret_cast<const uint8_t *>(g_jpeg), len) != len) break;
    }
    c.stop();
  }
}

static void sendStatus() {
  static char buf[1200];
  int n = buildStatus(buf, sizeof(buf));
  if (n <= 0) { server.send(500, "text/plain", "status overflow"); return; }
  server.send(200, "application/json", buf);
}

static void sendSnapshot() {
  // Serve the frame loop() already captured.  Calling processFrame() here would
  // charge a full capture + 210 ms detection to every image request, and the
  // request could not be answered until the loop's own pass finished -- that
  // was the stop-motion preview.
  if (!g_jpegLen) { server.send(503, "text/plain", "no frame"); return; }
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "image/jpeg", (const char *)g_jpeg, g_jpegLen);
}

static void streamHandler() {
  WiFiClient client = server.client();
  if (!client) return;
  client.print("HTTP/1.1 200 OK\r\n"
               "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
               "Cache-Control: no-store\r\n"
               "Connection: close\r\n\r\n");
  while (client.connected()) {
    pollSerial();
    processFrame(true);
    if (g_jpegLen) {
      client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                    (unsigned)g_jpegLen);
      client.write(g_jpeg, g_jpegLen);
      client.print("\r\n");
    }
  }
  client.stop();
}

static const char PAGE_HTML[] = R"HTML(<!doctype html>
<html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>FireBot</title>
<style>
*{box-sizing:border-box}
body{margin:0;background:#0b0e13;color:#dfe6f0;font:14px/1.45 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
header{display:flex;align-items:center;gap:12px;padding:12px 18px;border-bottom:1px solid #1e2634}
h1{font-size:16px;margin:0;letter-spacing:.14em;text-transform:uppercase;font-weight:600;color:#9fb0c8}
#pill{margin-left:auto;padding:6px 14px;border-radius:999px;font-weight:700;letter-spacing:.09em;
      font-size:12px;background:#24303f;color:#8fa2ba;transition:background .2s,color .2s}
#pill.fire{background:#e01b24;color:#fff;box-shadow:0 0 18px #e01b2455}
main{display:grid;grid-template-columns:minmax(0,1fr) 300px;gap:16px;padding:16px;align-items:start}
@media(max-width:900px){main{grid-template-columns:1fr}}
.view{position:relative;background:#000;border:1px solid #1e2634;border-radius:10px;overflow:hidden}
.view img{display:block;width:100%;height:auto}
#ov{position:absolute;inset:0;width:100%;height:100%;pointer-events:none}
.legend{position:absolute;left:10px;bottom:10px;display:flex;gap:14px;font-size:11px;
        background:#000a;padding:5px 10px;border-radius:6px;color:#cbd5e1}
.dot{display:inline-block;width:8px;height:8px;border-radius:2px;margin-right:5px;vertical-align:middle}
aside{display:flex;flex-direction:column;gap:10px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.card{background:#111721;border:1px solid #1e2634;border-radius:9px;padding:9px 11px}
.card .k{font-size:10px;letter-spacing:.1em;text-transform:uppercase;color:#64748b}
.card .v{font-size:19px;font-weight:650;font-variant-numeric:tabular-nums;margin-top:2px}
.card .v small{font-size:11px;font-weight:400;color:#7b8ba3}
.sec{background:#111721;border:1px solid #1e2634;border-radius:9px;padding:11px}
.sec h2{font-size:10px;letter-spacing:.12em;text-transform:uppercase;color:#64748b;margin:0 0 9px}
button{background:#1b2330;color:#c7d2e0;border:1px solid #2a3648;border-radius:7px;
       padding:7px 10px;font:inherit;font-size:12px;cursor:pointer;transition:.15s}
button:hover{background:#243040;border-color:#3a4a63}
button.on{background:#1668d6;border-color:#2f86ff;color:#fff}
.row{display:flex;gap:6px;flex-wrap:wrap}
label.sl{display:block;margin:10px 0 0}
label.sl span{display:flex;justify-content:space-between;font-size:11px;color:#8fa2ba;margin-bottom:3px}
input[type=range]{width:100%;accent-color:#2f86ff}
.note{font-size:11px;color:#5f6f86;line-height:1.5}
#link{margin-left:10px;font-size:11px;color:#5f6f86;font-variant-numeric:tabular-nums}
</style></head><body>
<header><h1>FireBot</h1><span id="link">usb</span><span id="pill">CONNECTING</span></header>
<main>
  <div class="view">
    <img id="stream" alt="camera">
    <canvas id="ov"></canvas>
    <div class="legend">
      <span><i class="dot" style="background:#22c55e"></i>target</span>
      <span><i class="dot" style="background:#fbbf24"></i>other</span>
      <span><i class="dot" style="background:#b91c1c"></i>done</span>
    </div>
  </div>
  <aside>
    <div class="grid">
      <div class="card"><div class="k">FPS</div><div class="v" id="v-fps">-</div></div>
      <div class="card"><div class="k">Targets</div><div class="v" id="v-n">-</div></div>
      <div class="card"><div class="k">Nearest</div><div class="v" id="v-d">-<small> cm</small></div></div>
      <div class="card"><div class="k">Bearing</div><div class="v" id="v-err">-</div></div>
      <div class="card"><div class="k">State</div><div class="v" id="v-st" style="font-size:15px">-</div></div>
      <div class="card"><div class="k">Profile</div><div class="v" id="v-prof" style="font-size:15px">-</div></div>
    </div>

    <div class="sec"><h2>Flashlight</h2>
      <div class="row"><button id="b-led">Flash off</button><button id="b-voice">Voice off</button></div>
      <div class="note" style="margin-top:6px">White LED on GPIO4. Steady on/off only — no RGB on this module.</div>
    </div>

    <div class="sec"><h2>Profile</h2>
      <div class="row">
        <button data-p="both">Both</button>
        <button data-p="candle">Candle</button>
        <button data-p="print">Print</button>
        <button data-p="flame">Flame</button>
      </div>
      <div class="note" style="margin-top:8px"><b>Both</b> = printed flames + vivid cups in one pass. Candle = squat cups. Print = tall drawings. Flame = bright fire, bans pink.</div>
    </div>

    <div class="sec"><h2>Thresholds</h2>
      <label class="sl"><span>Saturation <b id="l-s">-</b></span>
        <input id="s" type="range" min="0" max="255" step="1"></label>
      <label class="sl"><span>Brightness <b id="l-v">-</b></span>
        <input id="v" type="range" min="0" max="255" step="1"></label>
      <label class="sl"><span>R minus B <b id="l-rb">-</b></span>
        <input id="rb" type="range" min="-60" max="140" step="1"></label>
      <div class="note" style="margin-top:8px">Filters are strict: a box that flickers in and out means the threshold is marginal.</div>
    </div>

    <div class="sec"><h2>Actions</h2>
      <div class="row">
        <button id="b-auto">Auto off</button>
        <button id="b-sup">Suppress off</button>
        <button id="b-reset">Reset</button>
        <button id="b-pump">Pump</button>
      </div>
    </div>
    <div class="note" id="foot">-</div>
  </aside>
</main>
<script>
const $=id=>document.getElementById(id);
let cfg={},sent={};
function cmd(c){fetch('/cmd?c='+encodeURIComponent(c)).catch(()=>{});}

// The board's web server is single-threaded: an open MJPEG stream would block
// every other request, including /status.json.  So pull stills instead -- each
// fetch completes, leaving the server free to answer the status poll.
// Voice feedback. Uses the phone/PC browser's built-in speech engine
// (SpeechSynthesis): the board sends only the same tiny status JSON it always
// did, so this costs the sketch literally nothing.
let voiceOn=false, lastSt='';
function say(t){
  if(!voiceOn||!('speechSynthesis' in window)||!t) return;
  const u=new SpeechSynthesisUtterance(t);
  u.rate=1.05;
  speechSynthesis.speak(u);
}
const SAY={EXTINGUISH:'Extinguishing target',COOLDOWN:'Target eliminated'};
function sayState(st){
  if(st===lastSt) return;
  lastSt=st;
  say(SAY[st]);
}
// Flame edge: announces whether or not auto mode is running.
let prevFire=false;
function sayFire(f){
  if(f && !prevFire) say('Flame detected');
  prevFire=!!f;
}
$('b-voice').onclick=()=>{
  voiceOn=!voiceOn;
  $('b-voice').textContent='Voice '+(voiceOn?'on':'off');
  $('b-voice').classList.toggle('on',voiceOn);
  if(voiceOn){ lastSt=''; say('Voice on'); }
  else { try{speechSynthesis.cancel();}catch(e){} }
};

// The video feed runs on port 81 with its own server task on the board, so it
// never competes with /status.json.  If the stream can't start (older build,
// blocked port), fall back to polling stills from /snapshot.
let streamed=false, polling=false;
function startPolling(){
  if(polling) return;
  polling=true;
  setInterval(grabFrame,250);
}
$('stream').onload = ()=>{ streamed=true; };
$('stream').onerror = ()=>{ startPolling(); };
$('stream').src = 'http://'+location.hostname+':81/stream';
setTimeout(()=>{ if(!streamed) startPolling(); }, 3500);

let grabbing=false;
async function grabFrame(){
  if(grabbing) return;
  grabbing=true;
  try{
    const r=await fetch('/snapshot?t='+Date.now(),{cache:'no-store'});
    if(r.ok){
      const url=URL.createObjectURL(await r.blob());
      const img=$('stream'), old=img.dataset.url;
      img.onload=()=>{ if(old) URL.revokeObjectURL(old); };
      img.dataset.url=url;
      img.src=url;
    }
  }catch(e){}
  grabbing=false;
}

// Boxes are drawn here rather than burned into the frame: the preview is the
// sensor's own JPEG, so nothing is re-encoded and the image stays sharp.
function overlay(d){
  const img=$('stream'), cv=$('ov');
  const w=img.clientWidth, h=img.clientHeight;
  if(!w||!h) return;
  if(cv.width!==w||cv.height!==h){cv.width=w;cv.height=h;}
  const g=cv.getContext('2d');
  g.clearRect(0,0,w,h);
  const m=/(\d+)x(\d+)/.exec(d.res||'640x480');
  const fw=m?+m[1]:640, fh=m?+m[2]:480;
  const sx=w/fw, sy=h/fh;
  d.blobs.forEach((b,i)=>{
    const hot = i===d.tgt;
    const col = b.sup? '#b91c1c' : (hot? '#22c55e' : '#fbbf24');
    const x=(b.cx-b.w/2)*sx, y=(b.cy-b.h/2)*sy;
    g.lineWidth = hot?2.5:1.5;
    g.strokeStyle=col;
    g.strokeRect(x,y,b.w*sx,b.h*sy);
    if(hot){
      g.fillStyle=col; g.font='11px system-ui';
      g.fillText(b.d+'cm', x+2, Math.max(11,y-4));
    }
  });
}
async function tick(){
  try{
    const r=await fetch('/status.json',{cache:'no-store'});
    const d=await r.json();
    const p=$('pill');
    p.textContent=d.fire?'FIRE DETECTED':'NO TARGET';
    p.className=d.fire?'fire':'';
    $('v-fps').textContent=d.fps.toFixed(1);
    $('v-n').textContent=d.n;
    $('v-d').textContent=d.blobs.length?d.blobs[0].d+' cm':'-';
    $('v-err').textContent=d.blobs.length?(d.blobs[0].err>0?'+':'')+d.blobs[0].err:'-';
    $('v-st').textContent=d.st;
    sayState(d.st);sayFire(d.fire);
    $('v-prof').textContent=d.profile;
    document.querySelectorAll('[data-p]').forEach(b=>b.classList.toggle('on',b.dataset.p===d.profile));
    $('b-auto').textContent='Auto '+(d.auto?'on':'off');
    $('b-auto').classList.toggle('on',!!d.auto);
    $('b-led').textContent='Flash '+(d.led?'on':'off');
    $('b-led').classList.toggle('on',!!d.led);
    $('b-sup').textContent='Suppress '+(d.suppress?'on':'off');
    $('b-sup').classList.toggle('on',!!d.suppress);
    ['s','v','rb'].forEach(k=>{if(document.activeElement!==$(k))$(k).value=d.cfg[k];});
    cfg=d.cfg;
    $('l-s').textContent=d.cfg.s;$('l-v').textContent=d.cfg.v;$('l-rb').textContent=d.cfg.rb;
    $('foot').textContent='uptime '+d.uptime+'s  free heap '+(d.free/1024).toFixed(0)+' KB  '+(d.auto?'motors+pump live':'detection only, motors/pump inhibited')+(d.t?'   |   grab '+d.t.cap+'ms  detect '+d.t.det+'ms  encode '+d.t.enc+'ms':'');
    $('link').textContent = d.link ? d.link : 'usb';
    overlay(d);
  }catch(e){$('pill').textContent='OFFLINE';$('pill').className='';}
}
setInterval(tick,250);tick();
document.querySelectorAll('[data-p]').forEach(b=>b.onclick=()=>cmd('profile='+b.dataset.p));
$('b-auto').onclick=()=>cmd('auto='+($('b-auto').classList.contains('on')?0:1));
$('b-led').onclick=()=>cmd('led='+($('b-led').classList.contains('on')?0:1));
$('b-sup').onclick=()=>cmd('suppress='+($('b-sup').classList.contains('on')?0:1));
$('b-reset').onclick=()=>cmd('reset');
$('b-pump').onclick=()=>{cmd('pump=1');setTimeout(()=>cmd('pump=0'),1200);};
$('stream').onerror=()=>{setTimeout(()=>{$('stream').src='/stream?t='+Date.now();},1500);};
let deb={};
['s','v','rb'].forEach(k=>{
  $(k).oninput=()=>{
    $('l-'+k).textContent=$(k).value;
    clearTimeout(deb[k]);
    deb[k]=setTimeout(()=>cmd(k+'='+$(k).value),180);
  };
});
</script></body></html>)HTML";

static void setupWeb() {
  server.on("/", []() { server.send(200, "text/html", PAGE_HTML); });
  server.on("/status.json", sendStatus);
  server.on("/stream", streamHandler);
  server.on("/snapshot", sendSnapshot);
  server.on("/cmd", []() {
    if (server.hasArg("c")) handleCommand(server.arg("c"));
    server.send(200, "text/plain", "ok");
  });
  server.onNotFound([]() { server.send(404, "text/plain", "not found"); });
  server.begin();
  Serial.println("http server on :80");
}

static void startWifi() {
  char ip[24] = "?";
  // Live video service: its own task + port, see streamTask above.
  xTaskCreatePinnedToCore(streamTask, "stream", 4096, NULL, 1, NULL, 1);
#if WIFI_STA_ENABLED
  Serial.printf("wifi: joining \"%s\"\n", WIFI_STA_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(200);
  if (WiFi.status() == WL_CONNECTED) {
    snprintf(ip, sizeof(ip), "%s", WiFi.localIP().toString().c_str());
    Serial.printf("wifi: joined, http://%s\n", ip);
    return;
  }
  Serial.println("wifi: join failed, falling back to access point");
#endif
  WiFi.mode(WIFI_AP);
  WiFi.softAP("FireBot-CAM", "firebot123");
  snprintf(ip, sizeof(ip), "%s", WiFi.softAPIP().toString().c_str());
  Serial.printf("wifi: AP \"FireBot-CAM\" pass firebot123, http://%s\n", ip);
}
#endif  // ENABLE_WIFI

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(400);

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);    // GPIO4 HIGH = flash ON; LOW = off
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);

#if ENABLE_WIFI
  // IMPORTANT: the radio must be up BEFORE the camera.  WiFi bring-up takes a
  // couple of seconds and hogs the CPU; if the sensor is already streaming
  // nothing consumes its frames, the DMA overruns (FB-OVF), and the restart
  // spikes brown the board out on a marginal 5 V rail.
  startWifi();
  setupWeb();
#endif

  // Mask + scratch are touched several million times per frame by the
  // morphology passes, so they belong in fast internal RAM, not PSRAM.  Fall
  // back to PSRAM if internal is too fragmented.
  g_maskBuf = (uint8_t *)heap_caps_malloc(GRID_CELLS, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  g_scratchBuf = (uint8_t *)heap_caps_malloc(GRID_CELLS, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  bool maskInPsram = false;
  if (!g_maskBuf) { g_maskBuf = (uint8_t *)ps_malloc(GRID_CELLS); maskInPsram = true; }
  if (!g_scratchBuf) { g_scratchBuf = (uint8_t *)ps_malloc(GRID_CELLS); maskInPsram = true; }
  g_stackBuf = (int32_t *)ps_malloc(sizeof(int32_t) * GRID_CELLS);
  g_det = (firecore::Detector *)ps_malloc(sizeof(firecore::Detector));
  g_jpeg = (uint8_t *)ps_malloc(JPEG_CAP);
  g_prev = (uint8_t *)ps_malloc((size_t)PREVIEW_W * PREVIEW_H * 2);
  g_vmap = (uint8_t *)ps_malloc((size_t)VMAP_W * VMAP_H);
  if (!g_maskBuf || !g_scratchBuf || !g_stackBuf || !g_det || !g_jpeg || !g_prev) {
    Serial.println("PSRAM alloc failed");
    while (true) delay(1000);
  }
  new (g_det) firecore::Detector(g_gridW, g_gridH, g_step,
                                 g_maskBuf, g_scratchBuf, g_stackBuf);

  Serial.println("\nFireBot detection");
  Serial.printf("grid %dx%d (step %d), %d cells, mask in %s\n", g_gridW, g_gridH,
                g_step, GRID_CELLS, maskInPsram ? "PSRAM" : "internal RAM");

  if (!initCamera(g_framesize)) {
    Serial.println("halting");
    while (true) delay(1000);
  }
  Serial.println("camera ok");
  tuneSensor();

  applyProfile(g_profile);
  enterState(ST_SEARCH);

#if ENABLE_WIFI
  Serial.println("auto=0 (motors/pump inhibited); enable from the dashboard");
#else
  Serial.printf("serial link at %d baud -- run tools/dashboard.py on the PC\n", SERIAL_BAUD);
  Serial.println("send stream=1 to start the video feed");
#endif
}

void loop() {
  pollSerial();
  updateLed();
  processFrame(g_stream);
#if ENABLE_WIFI
  server.handleClient();
#endif
}
