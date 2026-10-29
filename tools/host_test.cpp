/*
 * host_test.cpp -- compile the firmware's detection core on the PC and run it
 * against a PPM image, so its output can be diffed against the Python
 * reference implementation before spending a flash cycle.
 *
 *   g++ -O2 -std=c++17 -I firmware/firebot -o /tmp/host_test tools/host_test.cpp
 *   /tmp/host_test out/test.ppm
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "detect_core.h"

static bool readPPM(const char *path, int *w, int *h, std::vector<uint8_t> *rgb) {
  FILE *f = fopen(path, "rb");
  if (!f) return false;
  char magic[3] = {0};
  if (fscanf(f, "%2s", magic) != 1 || strcmp(magic, "P6") != 0) { fclose(f); return false; }
  int maxv = 0;
  if (fscanf(f, "%d %d %d", w, h, &maxv) != 3) { fclose(f); return false; }
  fgetc(f);  // single whitespace after maxval
  rgb->resize((size_t)(*w) * (*h) * 3);
  size_t got = fread(rgb->data(), 1, rgb->size(), f);
  fclose(f);
  return got == rgb->size();
}

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: %s image.ppm [step] [profile]\n", argv[0]);
    return 2;
  }
  int step = (argc > 2) ? atoi(argv[2]) : 1;
  const char *profile = (argc > 3) ? argv[3] : "candle";

  int W = 0, H = 0;
  std::vector<uint8_t> rgb;
  if (!readPPM(argv[1], &W, &H, &rgb)) {
    fprintf(stderr, "cannot read %s\n", argv[1]);
    return 1;
  }

  const int gw = W / step, gh = H / step;
  std::vector<uint8_t> mask((size_t)gw * gh), scratch((size_t)gw * gh);
  std::vector<int32_t> stack((size_t)gw * gh);

  firecore::Detector det(gw, gh, step, mask.data(), scratch.data(), stack.data());
  if (!firecore::applyPreset(det.cfg, profile)) {
    fprintf(stderr, "unknown profile '%s'\n", profile);
    return 2;
  }

  for (int y = 0; y < gh; y++)
    for (int x = 0; x < gw; x++) {
      size_t o = ((size_t)(y * step) * W + (size_t)(x * step)) * 3;
      mask[(size_t)y * gw + x] =
          firecore::isFirePixel(rgb[o], rgb[o + 1], rgb[o + 2], det.cfg) ? 1 : 0;
    }

  det.morphOpenClose();

  firecore::Blob blobs[8];
  int n = det.findBlobs(blobs, 8);
  firecore::Detector::sortByDistance(blobs, n);

  printf("resolution=%dx%d step=%d count=%d\n", W, H, step, n);
  for (int i = 0; i < n; i++)
    printf("  #%d cx=%d cy=%d box=%dx%d area=%ld fill=%.2f dist=%.0fcm\n",
           i, blobs[i].cx, blobs[i].cy, blobs[i].w, blobs[i].h,
           blobs[i].area, blobs[i].fill, blobs[i].distanceMm / 10.0f);
  return 0;
}
