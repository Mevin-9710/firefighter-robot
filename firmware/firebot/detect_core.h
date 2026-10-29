/*
 * detect_core.h -- portable fire-blob detection core.
 *
 * Pure C++ (no Arduino, no OpenCV) so the exact firmware logic can be compiled
 * and validated on the host against tools/fire_detect.py before flashing.
 *
 * Pixel rule and blob filters mirror FireConfig in tools/fire_detect.py.
 */
#pragma once

#include <stdint.h>
#include <string.h>

namespace firecore {

// ---------------------------------------------------------------------------
struct Config {
  int hHi1 = 38;      // hue band 1: 0..hHi1
  int hLo2 = 145;     // hue band 2: hLo2..179
  int sMin = 90;
  int vMin = 100;
  int rMin = 100;
  int rOverB = 45;
  int rOverG = 15;
  float minAreaFrac = 0.0008f;
  float maxAreaFrac = 0.60f;
  float minFill = 0.28f;
  float minAspect = 0.80f;
  float maxAspect = 6.0f;
  float focalPx = 250.0f;
  float targetWidthMm = 100.0f;
  float extinguishMm = 250.0f;
  int maxBlobs = 4;

  // Bright-glow channel.  A real flame through the OV2640 is not a colour
  // patch: the JPEG blows its core out to near-white.  Pixels at or above
  // glowV with saturation at or below glowSMax count as fire even without any
  // hue, so the burnt-out core joins its coloured fringe into one blob.
  // Set glowV to 255 to switch the channel off (kept strict for samples that
  // already have a white background, like a printed flame on paper).
  int glowV = 235;
  int glowSMax = 60;
};

struct Blob {
  int cx, cy;
  int x, y, w, h;
  long area;
  float fill, aspect, distanceMm;
};

// ---------------------------------------------------------------------------
// Threshold presets -- single source of truth for the firmware AND the host
// test harness.  Keep in sync with tools/fire_detect.py --profile.
// ---------------------------------------------------------------------------
static inline void presetCandle(Config &c) {
  // The tealight's FLAME, not its body: flame is hot orange/yellow with
  // R >> B and a blown-out white core.  The vivid cup, a red dress or any
  // magenta-leaning object fails the stricter R-B dominance and hue band;
  // the white core still joins via the glow channel.
  c.hHi1 = 40;   c.hLo2 = 178;    // yellow/orange + wrap-red only, no pink
  c.sMin = 85;   c.vMin = 165;    // flames out-bright fabric and dark wax
  c.rMin = 140;  c.rOverB = 25;   // true fire beats blue strongly
  c.rOverG = 8;
  c.glowV = 235; c.glowSMax = 60;
  c.minAreaFrac = 0.00006f; c.maxAreaFrac = 0.15f;
  c.minFill = 0.30f; c.minAspect = 0.35f; c.maxAspect = 4.0f;
  c.focalPx = 500.0f; c.targetWidthMm = 38.0f; c.extinguishMm = 250.0f;
}

static inline void presetPrint(Config &c) {
  // Hand-drawn / printed flame on white paper: tall silhouette.
  c.hHi1 = 38;   c.hLo2 = 145;
  c.sMin = 100;  c.vMin = 110;
  c.rMin = 100;  c.rOverB = 45;  c.rOverG = 15;
  c.glowV = 255; c.glowSMax = 0;   // off: white paper would win otherwise
  c.minAreaFrac = 0.0008f; c.maxAreaFrac = 0.60f;
  c.minFill = 0.28f; c.minAspect = 0.80f; c.maxAspect = 6.0f;
  c.focalPx = 250.0f; c.targetWidthMm = 100.0f; c.extinguishMm = 350.0f;
}

static inline void presetBoth(Config &c) {
  // Printed flame AND vivid cups in the same pass.  The two presets disagree
  // only on shape: a drawn flame is tall, a tealight is squat, so the aspect
  // window is opened at both ends and the colour gates take the union.
  // Distance is approximate here -- one target width cannot describe both a
  // 100 mm drawing and a 38 mm cup, so tw is a mid value.
  c.hHi1 = 40;   c.hLo2 = 178;    // same flame-only ruling as candle
  c.sMin = 85;   c.vMin = 165;
  c.rMin = 140;  c.rOverB = 25;   c.rOverG = 8;
  c.glowV = 235; c.glowSMax = 60;
  c.minAreaFrac = 0.00006f; c.maxAreaFrac = 0.60f;
  c.minFill = 0.28f; c.minAspect = 0.30f; c.maxAspect = 6.0f;
  c.focalPx = 500.0f; c.targetWidthMm = 60.0f; c.extinguishMm = 250.0f;
}

static inline void presetFlame(Config &c) {
  // Real flame: much brighter, bans pink, needs flicker confirmation.
  c.hHi1 = 45;   c.hLo2 = 180;
  c.sMin = 130;  c.vMin = 190;
  c.rMin = 170;  c.rOverB = 70;  c.rOverG = 0;
  c.glowV = 230; c.glowSMax = 40;
  c.minAreaFrac = 0.00025f; c.maxAreaFrac = 0.35f;
  c.minFill = 0.22f; c.minAspect = 0.60f; c.maxAspect = 6.0f;
  c.focalPx = 500.0f; c.targetWidthMm = 20.0f; c.extinguishMm = 250.0f;
}

static inline bool applyPreset(Config &c, const char *name) {
  if (!strcmp(name, "candle")) { presetCandle(c); return true; }
  if (!strcmp(name, "print")) { presetPrint(c); return true; }
  if (!strcmp(name, "both")) { presetBoth(c); return true; }
  if (!strcmp(name, "flame")) { presetFlame(c); return true; }
  return false;
}

// 5/6-bit -> 8-bit expansion without multiplies
static inline uint8_t expand5(uint8_t v) { return (uint8_t)((v << 3) | (v >> 2)); }
static inline uint8_t expand6(uint8_t v) { return (uint8_t)((v << 2) | (v >> 4)); }

// ---------------------------------------------------------------------------
// Pixel classification. Identical maths to fire_detect.fire_mask().
// ---------------------------------------------------------------------------
static inline bool isFirePixel(uint8_t r, uint8_t g, uint8_t b, const Config &c) {
  uint8_t maxc = r, minc = r;
  if (g > maxc) maxc = g;
  if (b > maxc) maxc = b;
  if (g < minc) minc = g;
  if (b < minc) minc = b;

  // Cheap common reject: not bright enough for either channel.
  if (maxc < c.vMin && maxc < c.glowV) return false;

  int delta = (int)maxc - (int)minc;
  int sat = delta ? (delta * 255) / (int)maxc : 0;

  // Bright-glow channel: a blown-out flame core is fire regardless of hue.
  if (maxc >= c.glowV && sat <= c.glowSMax) return true;

  // Hue channel below.
  if (r < c.rMin) return false;
  if ((int)r - (int)b < c.rOverB) return false;
  if ((int)r - (int)g < c.rOverG) return false;

  if (maxc < c.vMin) return false;

  if (delta == 0) return false;
  if (sat < c.sMin) return false;

  int hue;
  if (maxc == r) {
    hue = (60 * ((int)g - (int)b)) / delta;
    if (hue < 0) hue += 360;
  } else if (maxc == g) {
    hue = (60 * ((int)b - (int)r)) / delta + 120;
  } else {
    hue = (60 * ((int)r - (int)g)) / delta + 240;
  }
  hue /= 2;

  return (hue <= c.hHi1) || (hue >= c.hLo2);
}

// ---------------------------------------------------------------------------
// Detector: owns a grid-space mask, a scratch buffer and a flood-fill stack.
// All buffers are supplied by the caller so the ESP32 can place them in PSRAM.
// ---------------------------------------------------------------------------
struct Detector {
  Config cfg;
  int gw, gh, step;
  uint8_t *mask;     // gw*gh
  uint8_t *scratch;  // gw*gh
  int32_t *stack;    // gw*gh

  Detector(int gridW, int gridH, int sampleStep,
           uint8_t *maskBuf, uint8_t *scratchBuf, int32_t *stackBuf)
      : gw(gridW), gh(gridH), step(sampleStep),
        mask(maskBuf), scratch(scratchBuf), stack(stackBuf) {}

  // --- 3x3 cross morphology (matches OpenCV 3x3 ellipse) -------------------
  void erode() {
    for (int y = 1; y < gh - 1; y++) {
      for (int x = 1; x < gw - 1; x++) {
        int i = y * gw + x;
        scratch[i] = (mask[i] && mask[i - 1] && mask[i + 1] &&
                      mask[i - gw] && mask[i + gw]) ? 1 : 0;
      }
    }
    // border cells cleared
    for (int x = 0; x < gw; x++) { scratch[x] = 0; scratch[(gh - 1) * gw + x] = 0; }
    for (int y = 0; y < gh; y++) { scratch[y * gw] = 0; scratch[y * gw + gw - 1] = 0; }
    memcpy(mask, scratch, (size_t)gw * gh);
  }

  void dilate() {
    for (int y = 1; y < gh - 1; y++) {
      for (int x = 1; x < gw - 1; x++) {
        int i = y * gw + x;
        scratch[i] = (mask[i] || mask[i - 1] || mask[i + 1] ||
                      mask[i - gw] || mask[i + gw]) ? 1 : 0;
      }
    }
    for (int x = 0; x < gw; x++) { scratch[x] = mask[x]; scratch[(gh - 1) * gw + x] = mask[(gh - 1) * gw + x]; }
    for (int y = 0; y < gh; y++) { scratch[y * gw] = mask[y * gw]; scratch[y * gw + gw - 1] = mask[y * gw + gw - 1]; }
    memcpy(mask, scratch, (size_t)gw * gh);
  }

  void morphOpenClose() { erode(); dilate(); dilate(); erode(); }

  // --- connected components + filtering ------------------------------------
  // Returns blob count. out[] must hold at least maxBlobs entries.
  int findBlobs(Blob *out, int maxOut) {
    memset(scratch, 0, (size_t)gw * gh);  // reuse scratch as the "visited" map
    uint8_t *vis = scratch;

    const float frameArea = (float)((long)gw * step * gh * step);
    const float cellArea = (float)(step * step);
    int count = 0;

    for (int i = 0; i < gw * gh; i++) {
      if (!mask[i] || vis[i]) continue;

      int sp = 0;
      stack[sp++] = i;
      vis[i] = 1;
      long area = 0, sumX = 0, sumY = 0;
      int minX = gw, maxX = -1, minY = gh, maxY = -1;

      while (sp > 0) {
        int c = stack[--sp];
        int cx = c % gw, cy = c / gw;
        area++;
        sumX += cx;
        sumY += cy;
        if (cx < minX) minX = cx;
        if (cx > maxX) maxX = cx;
        if (cy < minY) minY = cy;
        if (cy > maxY) maxY = cy;

        for (int dy = -1; dy <= 1; dy++) {
          int ny = cy + dy;
          if (ny < 0 || ny >= gh) continue;
          for (int dx = -1; dx <= 1; dx++) {
            if (!dx && !dy) continue;
            int nx = cx + dx;
            if (nx < 0 || nx >= gw) continue;
            int ni = ny * gw + nx;
            if (mask[ni] && !vis[ni]) {
              vis[ni] = 1;
              stack[sp++] = ni;
            }
          }
        }
      }

      if (count >= maxOut) continue;

      int bw = maxX - minX + 1;
      int bh = maxY - minY + 1;
      float areaPx = (float)area * cellArea;
      float frac = areaPx / frameArea;
      if (frac < cfg.minAreaFrac || frac > cfg.maxAreaFrac) continue;

      float fill = (float)area / (float)(bw * bh);
      if (fill < cfg.minFill) continue;

      float aspect = (float)bh / (float)bw;
      if (aspect < cfg.minAspect || aspect > cfg.maxAspect) continue;

      Blob &b = out[count++];
      b.x = minX * step;
      b.y = minY * step;
      b.w = bw * step;
      b.h = bh * step;
      b.cx = (int)((sumX / (float)area) * step);
      b.cy = (int)((sumY / (float)area) * step);
      b.area = area;
      b.fill = fill;
      b.aspect = aspect;
      b.distanceMm = cfg.focalPx * cfg.targetWidthMm / (float)b.w;
    }
    return count;
  }

  static void sortByDistance(Blob *out, int n) {
    for (int i = 0; i < n; i++)
      for (int j = i + 1; j < n; j++)
        if (out[j].distanceMm < out[i].distanceMm) {
          Blob t = out[i];
          out[i] = out[j];
          out[j] = t;
        }
  }
};

}  // namespace firecore
