/*
 * Smart Watch HD  -  KIT-GLE edition
 * ESP32-C3 + GC9A01 (240x240 round) + ICM-20948 + 1 button + buzzer
 *
 * NEW in this version
 *   - Loading screen  : "KIT-GLE" / "READY. SET. GO." / progress bar / "LOADING..."
 *   - Main screen     : seconds ring, dd/mm/yyyy, big HH:MM, seconds, STEP + BPM pills
 *   - Extra vector letters: B D E G I N O P R S T Y (plus the old K M C A L F)
 *   - Extra text sizes (SZ_T, SZ_X, SZ_N, SZ_U, SZ_B)
 *
 * Libraries (Library Manager):
 *   - GFX Library for Arduino   (moononournation)
 *   - Adafruit ICM20X  (+ BusIO, Unified Sensor)
 *   - NimBLE-Arduino (h2zero) v2.x
 *
 * Sketch folder must be named SmartWatch_KITGLE and contain fall_detect.h
 *
 * Button:
 *   short press = next screen
 *   long press  = CLOCK: main<->analog | STEPS: reset | STOPWATCH: start/stop (hold 2 s = reset)
 *   any press while the FALL alert is showing = cancel the alert
 *
 * Set time over BLE ("SmartWatch", write text to characteristic):
 *   "14:35:20"
 *   "11/19 14:35:20"            (month/day hh:mm:ss, year stays as is)
 *   "2026/11/22 14:35:20"       (yyyy/mm/dd hh:mm:ss)
 * Fall alert: BLE notify "FALL" on characteristic 6e400003-... (subscribe from the app).
 * IMU stream: binary batches on characteristic 6e400004-... for app-side fall inference.
 *
 * BPM: there is no heart-rate sensor yet. Set the global `bpm` from your sensor code later.
 */

#include <Arduino_GFX_Library.h>
#include <Adafruit_ICM20948.h>
#include <Wire.h>
#include <math.h>
#include <string>
#include <string.h>
#include <stdlib.h>
#include "fall_detect.h"

#define FALL_LOG_CSV 1   // set to 1 to stream raw IMU data over Serial (for collecting a training dataset)

#ifndef USE_BLE
#define USE_BLE 1       // BLE enabled by default
#endif
#if USE_BLE
#include <NimBLEDevice.h>       // library: NimBLE-Arduino (h2zero) v2.x
#define SVC_UUID  "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHR_UUID  "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define FALL_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define IMU_UUID  "6e400004-b5a3-f393-e0a9-e50e24dcca9e"
NimBLECharacteristic *fallChr = nullptr;
NimBLECharacteristic *imuChr = nullptr;
#endif

// ===================== PIN CONFIG (edit to match your wiring) =====================
#define PIN_SCK   4    // display SCL
#define PIN_MOSI  6    // display SDA
#define PIN_CS    7
#define PIN_DC    5
#define PIN_RST   10
#define PIN_SDA   8    // IMU SDA
#define PIN_SCL   9    // IMU SCL
#define PIN_BTN   2    // button -> GND
#define PIN_BUZ   1
// ==================================================================================

#define STEP_GOAL 10000

// ---------- types / globals (kept above all functions) ----------
enum Mode { M_CLOCK, M_STEPS, M_STOPWATCH, M_LEVEL, M_COUNT };

struct SzDef { int H; int r8; int pad; int gap; };       // text size: height px, stroke radius (1/8 px)
// SZ_T = big time, SZ_X = seconds, SZ_N = date, SZ_U = splash text, SZ_B = splash title,
// SZ_V = pill digits (thin), SZ_W = pill label (tiny)
enum { SZ_L = 0, SZ_M = 1, SZ_S = 2, SZ_T = 3, SZ_X = 4, SZ_N = 5, SZ_U = 6, SZ_B = 7, SZ_V = 8, SZ_W = 9 };
#define NSZ 10
const SzDef SZ[NSZ] = {
  {54, 24, 5, 4},   // SZ_L
  {30, 13, 3, 3},   // SZ_M
  {16,  8, 3, 2},   // SZ_S
  {34, 33, 6, 6},   // SZ_T  bold time
  {17, 18, 4, 3},   // SZ_X  bold seconds
  { 9,  7, 3, 1},   // SZ_N  date
  { 8,  4, 2, 1},   // SZ_U
  {30, 19, 4, 3},   // SZ_B
  { 8,  4, 2, 1},   // SZ_V  thin digits
  { 5,  3, 2, 1}    // SZ_W  tiny label
};

#define NGLY 34
// 0-9 digits, 10 ':', 11 '.', 12 '/', 13 K, 14 M, 15 C, 16 A, 17 L, 18 '*'(degree), 19 ' ', 20 '-', 21 F
// 22 B, 23 D, 24 E, 25 G, 26 I, 27 N, 28 O, 29 P, 30 R, 31 S, 32 T, 33 Y
const float GLY_W[NGLY] = {0.58f,0.58f,0.58f,0.58f,0.58f,0.58f,0.58f,0.58f,0.58f,0.58f,
                           0.0f,0.0f,0.40f,0.52f,0.72f,0.58f,0.62f,0.48f,0.24f,0.30f,0.40f,0.48f,
                           0.57f,0.57f,0.48f,0.60f,0.0f,0.58f,0.62f,0.52f,0.56f,0.52f,0.56f,0.58f};
struct GCache { uint8_t *p; uint8_t w; uint8_t h; bool tried; };
GCache gcache[NSZ][NGLY];
#define GLYPH_CACHE_BYTES (20 * 1024)
uint8_t glyphCachePool[GLYPH_CACHE_BYTES];
size_t glyphCacheUsed = 0;
uint8_t tmpMask[4096];

Arduino_DataBus *bus = new Arduino_ESP32SPI(PIN_DC, PIN_CS, PIN_SCK, PIN_MOSI, GFX_NOT_DEFINED);
Arduino_GFX *panel = new Arduino_GC9A01(bus, PIN_RST, 0 /*rotation*/, true /*IPS*/);
// Static framebuffer: reserve it at link time instead of allocating a large
// contiguous block from the heap during Arduino_Canvas::begin().
// RGB565: 240 * 240 * 2 = 115,200 bytes.
alignas(4) static uint16_t framebuffer[240 * 240];
uint16_t *fb = framebuffer;

Adafruit_ICM20948 icm;
bool imuOk = false;

uint16_t C_BG, C_WHITE, C_GRAY, C_DGRAY, C_CYAN, C_GREEN, C_ORANGE, C_RED, C_YELLOW;
uint16_t C_PILL, C_PINK, C_DATE, C_BRAND, C_ICON, C_TRACK;

Mode mode = M_CLOCK;
bool analogFace = false;

uint32_t steps = 0;
uint32_t lastStepMs = 0;
uint16_t bpm = 0;                        // heart rate (no sensor yet - set from your own code)
volatile float ax = 0, ay = 0, az = 1;   // written by imuTask, read by the UI

struct ImuSample {
  uint32_t timestampMs;
  float values[6];  // ax, ay, az in g; gx, gy, gz in rad/s
};

static constexpr uint8_t IMU_QUEUE_SIZE = 64;
static ImuSample imuQueue[IMU_QUEUE_SIZE];
static volatile uint8_t imuQueueHead = 0;
static volatile uint8_t imuQueueTail = 0;
static portMUX_TYPE imuQueueMux = portMUX_INITIALIZER_UNLOCKED;

uint32_t clockBaseMs = 0;
uint32_t clockBaseSec = 0;
char dateStr[12] = "";                   // "d/m" (used by the analog face)
int dateD = 1, dateM = 1, dateY = 2026;  // used by the main screen

bool swRunning = false;
uint32_t swStart = 0, swElapsed = 0;
#if USE_BLE
volatile bool bleBeepPending = false;
#endif

bool btnDown = false, longFired = false, vlongFired = false;
uint32_t btnT = 0, btnChange = 0;

// renderer state
uint8_t sqrtLut[4097];
uint16_t distTab[120 * 121 / 2];
float sinT[60], cosT[60];
uint8_t *maskBuf = nullptr;
int maskW = 0, maskH = 0;

int cX0, cY0, cX1, cY1, cUX, cUY, cLen, cR8;       // capsule state
uint16_t cCol;
float penX = 0, penY = 0;                           // glyph path state
int gX0, gY0, gH, gR;
uint16_t gCol;

#define P8(v) ((int)((v) * 8.0f + 0.5f))

static inline void flushFrame() {
  // Send the completed RGB565 frame directly to the GC9A01 panel.
  panel->draw16bitRGBBitmap(0, 0, fb, 240, 240);
}

void printRAM(const char *stage);

// ================================ pixel layer ================================
static inline uint16_t blend565(uint16_t fgc, uint16_t bgc, uint8_t a32) {
  uint32_t f = (fgc | ((uint32_t)fgc << 16)) & 0x07E0F81FUL;
  uint32_t b = (bgc | ((uint32_t)bgc << 16)) & 0x07E0F81FUL;
  uint32_t r = ((((f - b) * a32) >> 5) + b) & 0x07E0F81FUL;
  return (uint16_t)(r | (r >> 16));
}

// a = coverage 0..8
static inline void plot(int x, int y, uint16_t col, int a) {
  if (a <= 0) return;
  if (maskBuf) {
    if (x < 0 || y < 0 || x >= maskW || y >= maskH) return;
    uint8_t &m = maskBuf[y * maskW + x];
    if (a > m) m = a;
    return;
  }
  if ((unsigned)x >= 240 || (unsigned)y >= 240) return;
  uint16_t &p = fb[y * 240 + x];
  p = (a >= 8) ? col : blend565(col, p, a * 4);
}

// ================================ AA primitives ==============================
// All coordinates in 1/8 pixel units. Pixel (x,y) has its centre at (8x+4, 8y+4).

static inline void capPix(int x, int y) {
  int px = x * 8 + 4, py = y * 8 + 4;
  int vx = px - cX0, vy = py - cY0;
  int s = (vx * cUX + vy * cUY) >> 12;
  int d2;
  if (s < 0) d2 = vx * vx + vy * vy;
  else if (s > cLen) { int ex = px - cX1, ey = py - cY1; d2 = ex * ex + ey * ey; }
  else { int q = (vy * cUX - vx * cUY) >> 12; d2 = q * q; }
  if (d2 > 4096) return;
  int a = cR8 + 4 - sqrtLut[d2];
  if (a <= 0) return;
  plot(x, y, cCol, a > 8 ? 8 : a);
}

// line with round caps, half-width r8
static void capsule8(int x0, int y0, int x1, int y1, int r8, uint16_t col) {
  cX0 = x0; cY0 = y0; cX1 = x1; cY1 = y1; cR8 = r8; cCol = col;
  int dx = x1 - x0, dy = y1 - y0;
  float L = sqrtf((float)dx * dx + (float)dy * dy);
  if (L < 1.0f) { cUX = 4096; cUY = 0; cLen = 0; }
  else { cUX = (int)(dx / L * 4096.0f); cUY = (int)(dy / L * 4096.0f); cLen = (int)L; }
  int pad = r8 + 8;
  int span = (pad * 3 / 2) / 8 + 2;
  int minx = x0 < x1 ? x0 : x1, maxx = x0 < x1 ? x1 : x0;
  int miny = y0 < y1 ? y0 : y1, maxy = y0 < y1 ? y1 : y0;
  int adx = dx < 0 ? -dx : dx, ady = dy < 0 ? -dy : dy;
  if (adx >= ady) {
    for (int x = ((minx - pad) >> 3); x <= ((maxx + pad) >> 3); x++) {
      if (x < 0 || x >= 240) continue;
      int c8 = x * 8 + 4;
      int xc = c8 < minx ? minx : (c8 > maxx ? maxx : c8);
      int yc = dx ? y0 + (xc - x0) * dy / dx : y0;
      int yp = yc >> 3;
      for (int y = yp - span; y <= yp + span; y++) capPix(x, y);
    }
  } else {
    for (int y = ((miny - pad) >> 3); y <= ((maxy + pad) >> 3); y++) {
      if (y < 0 || y >= 240) continue;
      int c8 = y * 8 + 4;
      int yc = c8 < miny ? miny : (c8 > maxy ? maxy : c8);
      int xc = dy ? x0 + (yc - y0) * dx / dy : x0;
      int xp = xc >> 3;
      for (int x = xp - span; x <= xp + span; x++) capPix(x, y);
    }
  }
}

static void disc8(int cx, int cy, int r8, uint16_t col) {
  int pr = (r8 + 8) / 8 + 1;
  int lim = (r8 + 4) * (r8 + 4);
  for (int y = (cy >> 3) - pr; y <= (cy >> 3) + pr; y++) {
    for (int x = (cx >> 3) - pr; x <= (cx >> 3) + pr; x++) {
      int dx = x * 8 + 4 - cx, dy = y * 8 + 4 - cy;
      int d2 = dx * dx + dy * dy;
      if (d2 > lim) continue;
      int a = r8 + 4 - (int)(sqrtf((float)d2) + 0.5f);
      plot(x, y, col, a > 8 ? 8 : a);
    }
  }
}

// pixel-unit helpers
static void capsuleF(float x0, float y0, float x1, float y1, float rpx, uint16_t col) {
  capsule8(P8(x0), P8(y0), P8(x1), P8(y1), P8(rpx), col);
}
static void discF(float x, float y, float rpx, uint16_t col) {
  disc8(P8(x), P8(y), P8(rpx), col);
}
// arc around (cx,cy); angles in degrees, 0 = 12 o'clock, clockwise
static void strokeArc(float cx, float cy, float r, float a0, float a1, float rpx, uint16_t col) {
  int n = (int)(fabsf(a1 - a0) / 10.0f) + 2;
  float px = 0, py = 0;
  for (int k = 0; k <= n; k++) {
    float a = (a0 + (a1 - a0) * k / n) * DEG_TO_RAD;
    float x = cx + r * sinf(a), y = cy - r * cosf(a);
    if (k) capsuleF(px, py, x, y, rpx, col);
    px = x; py = y;
  }
}

// solid pixel-aligned rectangle [x0,x1) x [y0,y1)
static void fillRectPx(int x0, int y0, int x1, int y1, uint16_t col) {
  for (int y = y0; y < y1; y++)
    for (int x = x0; x < x1; x++) plot(x, y, col, 8);
}

// rounded rectangle with anti-aliased corners (integer pixel coordinates)
static void rrect(int x, int y, int w, int h, int r, uint16_t col) {
  fillRectPx(x + r, y, x + w - r, y + h, col);
  fillRectPx(x, y + r, x + r, y + h - r, col);
  fillRectPx(x + w - r, y + r, x + w, y + h - r, col);
  discF(x + r,     y + r,     r, col);
  discF(x + w - r, y + r,     r, col);
  discF(x + r,     y + h - r, r, col);
  discF(x + w - r, y + h - r, r, col);
}

// ---- table-driven AA ring centred on the screen (fast, integer only) ----
static inline int d8At(int x, int y) {
  int i = x < 120 ? 119 - x : x - 120;
  int j = y < 120 ? 119 - y : y - 120;
  if (i > j) { int t = i; i = j; j = t; }
  return distTab[j * (j + 1) / 2 + i];
}

// R1,R2 in 1/8 px. prog 0..1 sweeps clockwise from 12 o'clock.
static void ringAA(int R1, int R2, float prog, uint16_t colA, uint16_t colT) {
  bool full = prog >= 0.999f, none = prog <= 0.001f;
  int s = 0, c = 1024;
  float th = 0;
  if (!full && !none) {
    th = prog * 6.2831853f;
    s = (int)(sinf(th) * 1024.0f);
    c = (int)(cosf(th) * 1024.0f);
  }
  bool half = prog <= 0.5f;
  int ro = (R2 >> 3) + 2, ri = (R1 >> 3) - 2;
  for (int y = 120 - ro; y < 120 + ro; y++) {
    if (y < 0 || y >= 240) continue;
    float dy = y - 119.5f;
    float t = (float)ro * ro - dy * dy;
    if (t <= 0) continue;
    int ho = (int)sqrtf(t);
    int xa = 120 - ho - 1, xb = 120 + ho;
    int hi = 0;
    if (ri > 0 && fabsf(dy) < ri) hi = (int)sqrtf((float)ri * ri - dy * dy);
    int V = -(2 * y + 1 - 240);
    for (int x = xa; x <= xb; x++) {
      if (x < 0 || x >= 240) continue;
      if (hi && x >= 120 - hi && x < 120 + hi) { x = 120 + hi - 1; continue; }
      int d8 = d8At(x, y);
      int ao = R2 + 4 - d8; if (ao <= 0) continue;
      int ai = d8 - R1 + 4; if (ai <= 0) continue;
      int a = ao < ai ? ao : ai; if (a > 8) a = 8;
      bool in;
      if (full) in = true;
      else if (none) in = false;
      else {
        int U = 2 * x + 1 - 240;
        int cr = U * c - V * s;
        in = half ? (U >= 0 && cr <= 0) : (U >= 0 || cr <= 0);
      }
      plot(x, y, in ? colA : colT, a);
    }
  }
  if (!none && !full) {                      // round caps
    float rm = (R1 + R2) / 16.0f;
    int cr8 = (R2 - R1) / 2;
    disc8(960, P8(120.0f - rm), cr8, colA);
    disc8(P8(120.0f + rm * sinf(th)), P8(120.0f - rm * cosf(th)), cr8, colA);
  }
}

// ================================ vector font ================================
static void gMove(float x, float y) { penX = x; penY = y; }
static void gLine(float x, float y) {
  capsule8(gX0 + (int)(penX * gH), gY0 + (int)(penY * gH),
           gX0 + (int)(x * gH),    gY0 + (int)(y * gH), gR, gCol);
  penX = x; penY = y;
}
static void gArc(float cx, float cy, float rx, float ry, float a0, float a1, bool connect) {
  int n = (int)(fabsf(a1 - a0) / 10.0f);
  if (n < 4) n = 4;
  for (int k = 0; k <= n; k++) {
    float a = (a0 + (a1 - a0) * k / n) * DEG_TO_RAD;     // 0=right, 90=down (y down)
    float x = cx + rx * cosf(a), y = cy + ry * sinf(a);
    if (k == 0 && !connect) gMove(x, y); else gLine(x, y);
  }
}
static void gDot(float x, float y) {
  disc8(gX0 + (int)(x * gH), gY0 + (int)(y * gH), gR + gR / 6, gCol);
}

static void glyphPath(int id) {
  switch (id) {
    case 0: gArc(.29f, .5f, .29f, .5f, 0, 360, false); break;
    case 1: gMove(.04f, .24f); gLine(.30f, 0); gLine(.30f, 1); break;
    case 2: gArc(.29f, .29f, .29f, .29f, 180, 405, false); gLine(0, 1); gLine(.58f, 1); break;
    case 3: gArc(.28f, .25f, .27f, .25f, 205, 450, false);
            gArc(.28f, .75f, .30f, .25f, 270, 525, true); break;
    case 4: gMove(.44f, 1); gLine(.44f, 0); gLine(0, .68f); gLine(.58f, .68f); break;
    case 5: gMove(.54f, 0); gLine(.08f, 0); gLine(.05f, .47f);
            gArc(.28f, .73f, .30f, .27f, 250, 520, true); break;
    case 6: gArc(.55f, .69f, .55f, .64f, 268, 180, false);
            gArc(.29f, .69f, .29f, .31f, 180, 540, true); break;
    case 7: gMove(0, 0); gLine(.58f, 0); gLine(.20f, 1); break;
    case 8: gArc(.29f, .25f, .25f, .25f, 0, 360, false);
            gArc(.29f, .745f, .29f, .255f, 0, 360, false); break;
    case 9: gArc(.29f, .31f, .29f, .31f, 0, 360, false);
            gArc(.03f, .31f, .55f, .64f, 0, 90, false); break;
    case 10: gDot(0, .30f); gDot(0, .70f); break;
    case 11: gDot(0, 1.0f); break;
    case 12: gMove(.40f, 0); gLine(0, 1); break;
    case 13: gMove(0, 0); gLine(0, 1); gMove(.52f, 0); gLine(0, .58f); gLine(.52f, 1); break;
    case 14: gMove(0, 1); gLine(0, 0); gLine(.36f, .62f); gLine(.72f, 0); gLine(.72f, 1); break;
    case 15: gArc(.28f, .5f, .28f, .5f, 315, 45, false); break;
    case 16: gMove(0, 1); gLine(.31f, 0); gLine(.62f, 1); gMove(.12f, .68f); gLine(.50f, .68f); break;
    case 17: gMove(0, 0); gLine(0, 1); gLine(.48f, 1); break;
    case 18: gArc(.12f, .12f, .12f, .12f, 0, 360, false); break;
    case 20: gMove(0, .5f); gLine(.4f, .5f); break;
    case 21: gMove(.46f, 0); gLine(0, 0); gLine(0, 1); gMove(0, .48f); gLine(.38f, .48f); break;
    // ---- new letters ----
    case 22: // B
      gMove(0, 0); gLine(0, 1);
      gMove(0, 0); gLine(.30f, 0); gArc(.30f, .25f, .25f, .25f, -90, 90, true); gLine(0, .5f);
      gMove(0, .5f); gLine(.32f, .5f); gArc(.32f, .75f, .25f, .25f, -90, 90, true); gLine(0, 1);
      break;
    case 23: // D
      gMove(0, 0); gLine(.22f, 0); gArc(.22f, .5f, .34f, .5f, -90, 90, true);
      gLine(0, 1); gLine(0, 0);
      break;
    case 24: // E
      gMove(.48f, 0); gLine(0, 0); gLine(0, 1); gLine(.48f, 1);
      gMove(0, .5f); gLine(.40f, .5f);
      break;
    case 25: // G
      gArc(.30f, .5f, .30f, .5f, -40, -360, false); gLine(.32f, .5f);
      break;
    case 26: // I
      gMove(0, 0); gLine(0, 1);
      break;
    case 27: // N
      gMove(0, 1); gLine(0, 0); gLine(.58f, 1); gLine(.58f, 0);
      break;
    case 28: // O
      gArc(.31f, .5f, .31f, .5f, 0, 360, false);
      break;
    case 29: // P
      gMove(0, 1); gLine(0, 0); gLine(.26f, 0);
      gArc(.26f, .27f, .26f, .27f, -90, 90, true); gLine(0, .54f);
      break;
    case 30: // R
      gMove(0, 1); gLine(0, 0); gLine(.28f, 0);
      gArc(.28f, .26f, .26f, .26f, -90, 90, true); gLine(0, .52f);
      gMove(.26f, .52f); gLine(.56f, 1);
      break;
    case 31: // S
      gArc(.26f, .25f, .24f, .25f, -20, -270, false);
      gArc(.26f, .75f, .26f, .25f, -90, 160, true);
      break;
    case 32: // T
      gMove(0, 0); gLine(.56f, 0); gMove(.28f, 0); gLine(.28f, 1);
      break;
    case 33: // Y
      gMove(0, 0); gLine(.29f, .52f); gLine(.58f, 0);
      gMove(.29f, .52f); gLine(.29f, 1);
      break;
    default: break;
  }
}

static int glyphId(char ch) {
  if (ch >= '0' && ch <= '9') return ch - '0';
  switch (ch) {
    case ':': return 10; case '.': return 11; case '/': return 12;
    case 'K': return 13; case 'M': return 14; case 'C': return 15;
    case 'A': return 16; case 'L': return 17; case '*': return 18;
    case ' ': return 19; case '-': return 20; case 'F': return 21;
    case 'B': return 22; case 'D': return 23; case 'E': return 24;
    case 'G': return 25; case 'I': return 26; case 'N': return 27;
    case 'O': return 28; case 'P': return 29; case 'R': return 30;
    case 'S': return 31; case 'T': return 32; case 'Y': return 33;
  }
  return 19;
}

// Fixed-pool glyph cache: predictable RAM use and no heap fragmentation.
static GCache *getGlyph(int sz, int id) {
  GCache *gc = &gcache[sz][id];
  if (gc->tried) return gc;
  gc->tried = true;
  if (id == 19) return gc;

  const SzDef &d = SZ[sz];
  int w = (int)ceilf(GLY_W[id] * d.H) + 2 * d.pad + 1;
  int h = d.H + 2 * d.pad + 1;
  size_t pixels = (size_t)w * (size_t)h;
  size_t bytes = (pixels + 1) / 2;

  if (pixels > sizeof(tmpMask) ||
      bytes > GLYPH_CACHE_BYTES - glyphCacheUsed) return gc;

  memset(tmpMask, 0, pixels);
  maskBuf = tmpMask;
  maskW = w;
  maskH = h;
  gX0 = d.pad * 8;
  gY0 = d.pad * 8;
  gH = d.H * 8;
  gR = d.r8;
  gCol = 0xFFFF;
  glyphPath(id);
  maskBuf = nullptr;

  uint8_t *p = glyphCachePool + glyphCacheUsed;
  memset(p, 0, bytes);
  for (size_t i = 0; i < pixels; i++) {
    uint8_t v = tmpMask[i] & 0x0F;
    if (i & 1) p[i >> 1] |= (uint8_t)(v << 4);
    else       p[i >> 1]  = v;
  }
  glyphCacheUsed += bytes;
  gc->p = p;
  gc->w = (uint8_t)w;
  gc->h = (uint8_t)h;
  return gc;
}

static void blitMask(const uint8_t *p, int w, int h, int dx, int dy, uint16_t col) {
  for (int y = 0; y < h; y++) {
    int yy = dy + y;
    if ((unsigned)yy >= 240) continue;
    for (int x = 0; x < w; x++) {
      int i = y * w + x;
      int v = (p[i >> 1] >> ((i & 1) * 4)) & 15;
      if (v) plot(dx + x, yy, col, v);
    }
  }
}

static float glyphAdv(int sz, int id) {
  const SzDef &d = SZ[sz];
  return GLY_W[id] * d.H + d.r8 / 4.0f + d.gap;
}

// Direct fallback is used if the fixed glyph cache is full.
static void drawGlyphDirect(int sz, int id, int x, int top, uint16_t col) {
  if (id == 19) return;
  const SzDef &d = SZ[sz];
  gX0 = x * 8;
  gY0 = top * 8;
  gH = d.H * 8;
  gR = d.r8;
  gCol = col;
  glyphPath(id);
}

// width of a string in pixels; `extra` = additional letter spacing
static float textW(const char *s, int sz, float extra) {
  const SzDef &d = SZ[sz];
  int n = strlen(s);
  if (n == 0) return 0;
  float total = -d.gap;
  for (int i = 0; i < n; i++) total += glyphAdv(sz, glyphId(s[i]));
  return total + extra * (n - 1);
}

// centred text with optional letter spacing
// (digits, : . / K M C A L F B D E G I N O P R S T Y * space -)
static void drawTextSp(const char *s, int cx, int cy, int sz, uint16_t col, float extra) {
  const SzDef &d = SZ[sz];
  int n = strlen(s);
  float total = textW(s, sz, extra);
  float pen = cx - total / 2.0f + d.r8 / 8.0f;
  int top = cy - d.H / 2;

  for (int i = 0; i < n; i++) {
    int id = glyphId(s[i]);
    int x = (int)lroundf(pen) - d.pad;
    GCache *gc = getGlyph(sz, id);
    if (gc->p) blitMask(gc->p, gc->w, gc->h, x, top - d.pad, col);
    else drawGlyphDirect(sz, id, x, top - d.pad, col);
    pen += glyphAdv(sz, id) + extra;
  }
}

static void drawText(const char *s, int cx, int cy, int sz, uint16_t col) {
  drawTextSp(s, cx, cy, sz, col, 0.0f);
}

// text whose visible left edge is at `left`
static void drawTextL(const char *s, float left, int cy, int sz, uint16_t col) {
  float w = textW(s, sz, 0.0f);
  drawTextSp(s, (int)lroundf(left + w / 2.0f), cy, sz, col, 0.0f);
}

// ================================== icons ====================================
static void iconFoot(float x, float y, uint16_t col) {
  capsuleF(x, y - 5, x, y + 1, 3.3f, col);
  discF(x, y + 7, 2.5f, col);
}
static void iconFeet(float cx, float cy, uint16_t col) {
  iconFoot(cx - 6, cy + 3, col);
  iconFoot(cx + 6, cy - 3, col);
}
// STEP icon: two "head + body" figures. (ox,oy) = pill top-left
static void iconSteps(float ox, float oy, uint16_t col) {
  discF(ox + 6.65f, oy + 13.7f, 2.7f, col);                                   // left figure
  capsuleF(ox + 6.65f, oy + 18.5f, ox + 6.65f, oy + 22.35f, 2.65f, col);
  discF(ox + 13.1f, oy + 10.9f, 2.6f, col);                                   // right figure
  capsuleF(ox + 13.1f, oy + 17.45f, ox + 13.1f, oy + 18.65f, 2.65f, col);
}
// BPM icon: heart. (ox,oy) = pill top-left
static void iconHeart(float ox, float oy, uint16_t col) {
  discF(ox + 6.5f, oy + 14.6f, 4.0f, col);
  discF(ox + 13.9f, oy + 14.6f, 4.0f, col);
  for (float yy = 14.6f; yy <= 23.6f; yy += 0.5f) {
    float half = 7.7f * (23.9f - yy) / 9.3f;
    if (half < 0.4f) half = 0.4f;
    capsuleF(ox + 10.2f - half, oy + yy, ox + 10.2f + half, oy + yy, 0.45f, col);
  }
}
static void iconPause(float cx, float cy, uint16_t col) {
  capsuleF(cx - 5, cy - 7, cx - 5, cy + 7, 2.6f, col);
  capsuleF(cx + 5, cy - 7, cx + 5, cy + 7, 2.6f, col);
}
static void iconPlay(float cx, float cy, uint16_t col) {
  float ax_ = cx - 6, ay_ = cy - 9, by_ = cy + 9, px = cx + 9;
  for (int k = 0; k <= 18; k++) capsuleF(px, cy, ax_, ay_ + k, 0.8f, col);
  capsuleF(ax_, ay_, ax_, by_, 1.4f, col);
  capsuleF(ax_, ay_, px, cy, 1.4f, col);
  capsuleF(ax_, by_, px, cy, 1.4f, col);
}
static void iconStopwatch(float cx, float cy, uint16_t col) {
  strokeArc(cx, cy, 10, 0, 360, 1.2f, col);
  capsuleF(cx, cy - 14, cx, cy - 11, 1.8f, col);
  capsuleF(cx, cy, cx + 5, cy - 5, 1.0f, col);
}

static void modeDots() {
  int gap = 14;
  int x0 = 120 - (M_COUNT - 1) * gap / 2;
  for (int i = 0; i < M_COUNT; i++)
    discF(x0 + i * gap, 214, i == mode ? 3.0f : 2.0f, i == mode ? C_WHITE : C_DGRAY);
}

// ================================== helpers ==================================
void beep(int f, int ms) { tone(PIN_BUZ, f, ms); }

void nowHMS(int &h, int &m, int &s, int &cs) {
  // Unsigned subtraction safely handles millis() rollover.
  uint32_t elapsed = millis() - clockBaseMs;
  uint32_t currentSec = (clockBaseSec + elapsed / 1000UL) % 86400UL;
  h = currentSec / 3600UL;
  m = (currentSec / 60UL) % 60UL;
  s = currentSec % 60UL;
  cs = (elapsed % 1000UL) / 10UL;
}

// ================================== screens ==================================

// ---------- Screen 1: loading / splash (p = 0..1) ----------
void drawSplash(float p) {
  if (p < 0) p = 0;
  if (p > 1) p = 1;
  drawTextSp("KIT-GLE", 120, 84, SZ_B, C_WHITE, 0.0f);
  drawTextSp("READY. SET. GO.", 120, 115, SZ_U, C_ORANGE, 1.5f);

  capsuleF(64, 152, 176, 152, 1.0f, C_DGRAY);                       // track
  if (p > 0.01f) capsuleF(64, 152, 64 + 112.0f * p, 152, 1.0f, C_ORANGE);  // fill

  drawTextSp("LOADING...", 120, 171, SZ_U, C_GRAY, 3.0f);
}

void showSplash(float p) {
  memset(fb, 0, 240 * 240 * 2);
  drawSplash(p);
  if (!imuOk) {                                  // small red X = IMU not found
    capsuleF(114, 192, 126, 204, 1.6f, C_RED);
    capsuleF(126, 192, 114, 204, 1.6f, C_RED);
  }
  flushFrame();
}

void splashAnimate(float from, float to, uint32_t ms) {
  uint32_t t0 = millis();
  for (;;) {
    float k = (float)(millis() - t0) / (float)ms;
    if (k > 1.0f) k = 1.0f;
    showSplash(from + (to - from) * k);
    if (k >= 1.0f) break;
  }
}

// ---------- Screen 2: main ----------
// pill 49x35: icon | 0000 | underline + label
static void drawPill(int x, int y, bool heart, uint32_t val, const char *label) {
  rrect(x, y, 49, 35, 10, C_PILL);
  uint16_t accent = heart ? C_PINK : C_ICON;
  if (heart) iconHeart((float)x, (float)y, accent);
  else       iconSteps((float)x, (float)y, accent);

  char b[8];
  snprintf(b, sizeof(b), "%04lu", (unsigned long)val);
  drawTextL(b, x + (heart ? 20.0f : 18.0f), y + 17, SZ_V, C_WHITE);

  capsuleF(x + 14.8f, y + 25.2f, x + 30.1f, y + 25.2f, 1.0f, accent);
  drawTextL(label, x + 33.3f, y + 25, SZ_W, C_WHITE);
}

void drawMainScreen() {
  int h, m, s, cs;
  nowHMS(h, m, s, cs);
  float sec = s + cs / 100.0f;
  ringAA(P8(100), P8(120), sec / 60.0f, C_BRAND, C_TRACK);

  char buf[20];
  snprintf(buf, sizeof(buf), "%02d/%02d/%04d", dateD, dateM, dateY);
  drawText(buf, 120, 69, SZ_N, C_DATE);

  snprintf(buf, sizeof(buf), "%02d:%02d", h, m);
  drawText(buf, 120, 100, SZ_T, C_WHITE);

  snprintf(buf, sizeof(buf), "%02d", s);
  drawText(buf, 120, 139, SZ_X, C_BRAND);

  uint32_t st = steps > 9999 ? 9999 : steps;
  uint32_t hr = bpm > 9999 ? 9999 : bpm;
  drawPill(59, 155, false, st, "STP");
  drawPill(132, 155, true, hr, "BPM");
}

void drawClockAnalog() {
  int h, m, s, cs;
  nowHMS(h, m, s, cs);

  for (int i = 0; i < 60; i++) {
    bool hour = (i % 5 == 0);
    float r0 = hour ? 99.0f : 109.0f, r1 = 116.0f;
    capsuleF(120 + r0 * sinT[i], 120 - r0 * cosT[i], 120 + r1 * sinT[i], 120 - r1 * cosT[i],
             hour ? 1.6f : 0.8f, hour ? C_WHITE : C_GRAY);
  }
  drawText("12", 120, 40, SZ_S, C_WHITE);
  drawText("3", 200, 120, SZ_S, C_WHITE);
  drawText("6", 120, 196, SZ_S, C_WHITE);
  drawText("9", 40, 120, SZ_S, C_WHITE);

  drawText(dateStr, 120, 84, SZ_S, C_ORANGE);
  char buf[16];
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)steps);
  drawText(buf, 120, 160, SZ_S, C_GREEN);

  float sec = s + cs / 100.0f;
  float ha = ((h % 12) + m / 60.0f) * 30.0f * DEG_TO_RAD;
  float ma = (m + sec / 60.0f) * 6.0f * DEG_TO_RAD;
  float sa = sec * 6.0f * DEG_TO_RAD;

  capsuleF(120, 120, 120 + 52 * sinf(ha), 120 - 52 * cosf(ha), 3.6f, C_WHITE);
  capsuleF(120, 120, 120 + 84 * sinf(ma), 120 - 84 * cosf(ma), 2.5f, C_CYAN);
  capsuleF(120 - 22 * sinf(sa), 120 + 22 * cosf(sa),
           120 + 100 * sinf(sa), 120 - 100 * cosf(sa), 0.9f, C_RED);
  discF(120, 120, 6, C_RED);
  discF(120, 120, 2.5f, C_BG);
}

void drawSteps() {
  float p = (float)steps / STEP_GOAL;
  ringAA(P8(104), P8(118), p, p >= 1 ? C_YELLOW : C_GREEN, C_DGRAY);

  iconFeet(120, 46, C_GREEN);

  char buf[24];
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)steps);
  drawText(buf, 120, 96, strlen(buf) > 4 ? SZ_M : SZ_L, C_WHITE);

  drawText("/10000", 120, 138, SZ_S, C_GRAY);

  snprintf(buf, sizeof(buf), "%.2f KM", steps * 0.0007f);
  drawText(buf, 120, 168, SZ_S, C_CYAN);
  snprintf(buf, sizeof(buf), "%d KCAL", (int)(steps * 0.04f));
  drawText(buf, 120, 190, SZ_S, C_ORANGE);
}

void drawStopwatch() {
  uint32_t t = swRunning ? (swElapsed + millis() - swStart) : swElapsed;
  uint32_t totalSec = t / 1000;
  int mm = (totalSec / 60) % 100, ss = totalSec % 60, cc = (t % 1000) / 10;

  ringAA(P8(104), P8(118), (t % 60000) / 60000.0f, C_ORANGE, C_DGRAY);
  iconStopwatch(120, 52, C_ORANGE);

  char buf[16];
  snprintf(buf, sizeof(buf), "%02d:%02d", mm, ss);
  drawText(buf, 120, 92, SZ_L, C_WHITE);
  snprintf(buf, sizeof(buf), ".%02d", cc);
  drawText(buf, 120, 146, SZ_M, C_ORANGE);

  if (swRunning) iconPause(120, 184, C_GREEN);
  else iconPlay(120, 184, t ? C_YELLOW : C_GRAY);
}

void drawLevel() {
  ringAA(P8(99.2f), P8(100.8f), 1.0f, C_GRAY, C_GRAY);
  ringAA(P8(65.4f), P8(66.6f), 1.0f, C_DGRAY, C_DGRAY);
  ringAA(P8(32.4f), P8(33.6f), 1.0f, C_DGRAY, C_DGRAY);
  for (int i = 24; i < 216; i++) { plot(i, 119, C_DGRAY, 8); plot(i, 120, C_DGRAY, 8); }
  for (int i = 24; i < 216; i++) { plot(119, i, C_DGRAY, 8); plot(120, i, C_DGRAY, 8); }

  float lax = ax, lay = ay, laz = az;
  float bx = -lax * 160.0f, by = lay * 160.0f;
  float mag = sqrtf(bx * bx + by * by);
  if (mag > 78.0f) { bx *= 78.0f / mag; by *= 78.0f / mag; }
  float tilt = atan2f(sqrtf(lax * lax + lay * lay), fabsf(laz)) * RAD_TO_DEG;
  uint16_t col = tilt < 2.0f ? C_GREEN : C_ORANGE;

  strokeArc(120, 120, 15, 0, 360, 1.0f, C_WHITE);
  discF(120 + bx, 120 + by, 13, col);

  char buf[16];
  snprintf(buf, sizeof(buf), "%.1f*", tilt);
  drawText(buf, 120, 190, SZ_M, col);
}

// Fall alert overlay: countdown while the user can cancel, then a "sent" confirmation.
void drawFallAlert() {
  uint32_t now = millis();
  if (fallState == FALL_ALERT) {
    uint32_t left = fallRemainingMs(now);
    bool blink = (now / 300) & 1;
    ringAA(P8(104), P8(118), left / (float)FALL_COUNTDOWN_MS, blink ? C_RED : C_ORANGE, C_DGRAY);
    drawText("FALL", 120, 60, SZ_M, C_RED);
    char buf[8];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)((left + 999) / 1000));
    drawText(buf, 120, 120, SZ_L, C_WHITE);
  } else {
    ringAA(P8(104), P8(118), 1.0f, C_ORANGE, C_DGRAY);
    drawText("FALL", 120, 70, SZ_M, C_ORANGE);
    capsuleF(96, 130, 112, 146, 4, C_GREEN);   // check mark = alert sent
    capsuleF(112, 146, 146, 110, 4, C_GREEN);
  }
}

void renderFrame() {
  memset(fb, 0, 240 * 240 * 2);
  if (fallActive()) { drawFallAlert(); return; }
  switch (mode) {
    case M_CLOCK:     analogFace ? drawClockAnalog() : drawMainScreen(); break;
    case M_STEPS:     drawSteps(); break;
    case M_STOPWATCH: drawStopwatch(); break;
    case M_LEVEL:     drawLevel(); break;
    default: break;
  }
  modeDots();
}

// ============================== sensing & input ==============================
void updateSteps() {
  static float lp = 1, base = 1;
  static bool above = false;
  float lax = ax, lay = ay, laz = az;
  float mag = sqrtf(lax * lax + lay * lay + laz * laz);
  lp += 0.25f * (mag - lp);
  base += 0.02f * (lp - base);
  float dev = lp - base;

  uint32_t now = millis();
  if (!above && dev > 0.12f) {
    above = true;
    if (now - lastStepMs > 300) {
      steps++;
      lastStepMs = now;
      if (steps == STEP_GOAL) beep(2000, 150);
    }
  } else if (above && dev < 0.05f) {
    above = false;
  }
}

// Called from imuTask at ~100 Hz.
static void queueImuSample(uint32_t timestampMs, float axValue, float ayValue,
                           float azValue, float gxValue, float gyValue, float gzValue) {
  portENTER_CRITICAL(&imuQueueMux);
  uint8_t next = (imuQueueHead + 1) & (IMU_QUEUE_SIZE - 1);
  if (next == imuQueueTail) imuQueueTail = (imuQueueTail + 1) & (IMU_QUEUE_SIZE - 1);
  ImuSample &sample = imuQueue[imuQueueHead];
  sample.timestampMs = timestampMs;
  sample.values[0] = axValue;
  sample.values[1] = ayValue;
  sample.values[2] = azValue;
  sample.values[3] = gxValue;
  sample.values[4] = gyValue;
  sample.values[5] = gzValue;
  imuQueueHead = next;
  portEXIT_CRITICAL(&imuQueueMux);
}

void readImu() {
  if (!imuOk) return;
  sensors_event_t a, gy, t, mg;
  icm.getEvent(&a, &gy, &t, &mg);
  float x = a.acceleration.x / 9.80665f;
  float y = a.acceleration.y / 9.80665f;
  float z = a.acceleration.z / 9.80665f;
  ax = x; ay = y; az = z;

  static uint8_t n = 0;
  if ((++n & 1) == 0) updateSteps();        // step counter was tuned for 50 Hz

  uint32_t sampleTimeMs = millis();
  fallUpdate(x, y, z, sampleTimeMs);

#if FALL_LOG_CSV
  Serial.printf("%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n", (unsigned long)sampleTimeMs, x, y, z,
                gy.gyro.x, gy.gyro.y, gy.gyro.z);
#endif
  queueImuSample(sampleTimeMs, x, y, z, gy.gyro.x, gy.gyro.y, gy.gyro.z);
}

void imuTask(void *) {
  TickType_t t = xTaskGetTickCount();
  for (;;) {
    readImu();
    vTaskDelayUntil(&t, pdMS_TO_TICKS(10));   // 100 Hz
  }
}

void onShort() {
  mode = (Mode)((mode + 1) % M_COUNT);
  beep(2500, 30);
}

void onLong() {
  beep(1500, 80);
  switch (mode) {
    case M_CLOCK: analogFace = !analogFace; break;
    case M_STEPS: steps = 0; break;
    case M_STOPWATCH:
      if (swRunning) { swElapsed += millis() - swStart; swRunning = false; }
      else { swStart = millis(); swRunning = true; }
      break;
    default: break;
  }
}

void onVeryLong() {
  if (mode == M_STOPWATCH) {
    swRunning = false; swElapsed = 0;
    beep(1000, 150);
  }
}

void handleButton() {
  // While the fall alert is showing, any press cancels it and is otherwise ignored.
  if (fallActive() && !btnDown && digitalRead(PIN_BTN) == LOW) {
    fallCancel();
    beep(1500, 80);
    btnDown = true; longFired = vlongFired = true; btnChange = millis();
    return;
  }

  bool d = digitalRead(PIN_BTN) == LOW;
  uint32_t n = millis();
  if (d && !btnDown && n - btnChange > 30) {
    btnDown = true; btnT = n; btnChange = n; longFired = vlongFired = false;
  } else if (d && btnDown) {
    if (!longFired && n - btnT > 700) { longFired = true; onLong(); }
    if (!vlongFired && n - btnT > 2000) { vlongFired = true; onVeryLong(); }
  } else if (!d && btnDown && n - btnChange > 30) {
    btnDown = false; btnChange = n;
    if (!longFired) onShort();
  }
}

// ================================== time / BLE ===============================
void setClock(int h, int m, int s) {
  if (h < 0 || h > 23 || m < 0 || m > 59 || s < 0 || s > 59) return;
  clockBaseSec = (uint32_t)h * 3600UL +
                 (uint32_t)m * 60UL + (uint32_t)s;
  clockBaseMs = millis();
}

void setDate(int year, int month, int day) {
  dateY = year; dateM = month; dateD = day;
  snprintf(dateStr, sizeof(dateStr), "%d/%d", day, month);
}

void parseCompileTime() {
  int h = 0, m = 0, s = 0;
  sscanf(__TIME__, "%d:%d:%d", &h, &m, &s);
  clockBaseSec = (uint32_t)h * 3600UL +
                 (uint32_t)m * 60UL + (uint32_t)s;
  clockBaseMs = millis();

  const char *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char mon[4] = {__DATE__[0], __DATE__[1], __DATE__[2], 0};
  const char *f = strstr(months, mon);
  int mo = f ? (int)((f - months) / 3) + 1 : 1;
  setDate(atoi(__DATE__ + 7), mo, atoi(__DATE__ + 4));
}

#if USE_BLE
static bool validDate(int month, int day) {
  if (month < 1 || month > 12 || day < 1) return false;
  const uint8_t daysInMonth[12] =
      {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  return day <= daysInMonth[month - 1];
}

static bool onlyTrailingWhitespace(const char *p) {
  while (*p) {
    if (*p != ' ' && *p != 9 && *p != 13 && *p != 10) return false;
    ++p;
  }
  return true;
}

class TimeCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &connInfo) override {
    std::string value = c->getValue();
    if (value.empty() || value.size() >= 32) return;

    char input[32];
    memcpy(input, value.data(), value.size());
    input[value.size()] = 0;

    int year = 0, month = 0, day = 0, h = 0, m = 0, s = 0, used = 0;

    // format 1: "yyyy/mm/dd hh:mm:ss"
    if (sscanf(input, "%d/%d/%d %d:%d:%d%n",
               &year, &month, &day, &h, &m, &s, &used) == 6 &&
        onlyTrailingWhitespace(input + used)) {
      if (year < 2000 || year > 2099 || !validDate(month, day) ||
          h < 0 || h > 23 || m < 0 || m > 59 || s < 0 || s > 59) return;

      setClock(h, m, s);
      setDate(year, month, day);
      bleBeepPending = true;
      return;
    }

    used = 0;
    // format 2: "mm/dd hh:mm:ss"
    if (sscanf(input, "%d/%d %d:%d:%d%n",
               &month, &day, &h, &m, &s, &used) == 5 &&
        onlyTrailingWhitespace(input + used)) {
      if (!validDate(month, day) ||
          h < 0 || h > 23 || m < 0 || m > 59 || s < 0 || s > 59) return;

      setClock(h, m, s);
      setDate(dateY, month, day);
      bleBeepPending = true;
    } else {
      // format 3: "hh:mm:ss"
      used = 0;
      if (sscanf(input, "%d:%d:%d%n", &h, &m, &s, &used) != 3 ||
          !onlyTrailingWhitespace(input + used)) return;
      if (h < 0 || h > 23 || m < 0 || m > 59 || s < 0 || s > 59) return;

      setClock(h, m, s);
      bleBeepPending = true;
    }
  }
};

static void writeU16LE(uint8_t *dst, uint16_t value) {
  dst[0] = (uint8_t)(value & 0xFF);
  dst[1] = (uint8_t)(value >> 8);
}

static void writeU32LE(uint8_t *dst, uint32_t value) {
  dst[0] = (uint8_t)(value & 0xFF);
  dst[1] = (uint8_t)((value >> 8) & 0xFF);
  dst[2] = (uint8_t)((value >> 16) & 0xFF);
  dst[3] = (uint8_t)(value >> 24);
}

static int16_t scaledInt16(float value, float scale) {
  long scaled = lroundf(value * scale);
  if (scaled > 32767) scaled = 32767;
  if (scaled < -32768) scaled = -32768;
  return (int16_t)scaled;
}

static void sendImuBatch() {
  if (!imuChr) return;
  static uint32_t lastNotifyMs = 0;
  uint32_t now = millis();
  if (now - lastNotifyMs < 50) return;
  lastNotifyMs = now;

  ImuSample batch[5];
  uint8_t count = 0;
  portENTER_CRITICAL(&imuQueueMux);
  while (count < 5 && imuQueueTail != imuQueueHead) {
    batch[count++] = imuQueue[imuQueueTail];
    imuQueueTail = (imuQueueTail + 1) & (IMU_QUEUE_SIZE - 1);
  }
  portEXIT_CRITICAL(&imuQueueMux);
  if (!count) return;

  // Frame: 0xA1, count, first timestamp (u32 LE), then repeated:
  // delta from first timestamp (u16 ms), ax/ay/az (i16 at 0.001 g),
  // gx/gy/gz (i16 at 0.01 rad/s). Maximum frame size is 76 bytes.
  uint8_t packet[76];
  packet[0] = 0xA1;
  packet[1] = count;
  writeU32LE(packet + 2, batch[0].timestampMs);
  size_t offset = 6;
  for (uint8_t i = 0; i < count; i++) {
    uint32_t delta = batch[i].timestampMs - batch[0].timestampMs;
    writeU16LE(packet + offset, (uint16_t)(delta > 65535 ? 65535 : delta));
    offset += 2;
    for (uint8_t axis = 0; axis < 6; axis++) {
      float scale = axis < 3 ? 1000.0f : 100.0f;
      uint16_t encoded = (uint16_t)scaledInt16(batch[i].values[axis], scale);
      writeU16LE(packet + offset, encoded);
      offset += 2;
    }
  }
  imuChr->setValue(packet, offset);
  imuChr->notify();
}

void initBLE() {
  Serial.println("[BLE] init start");
  printRAM("before BLE init");

  NimBLEDevice::init("SmartWatch");
  Serial.println("[BLE] NimBLE initialized");
  printRAM("after NimBLE init");
  NimBLEServer *srv = NimBLEDevice::createServer();
  if (!srv) {
    Serial.println("BLE server creation failed");
    return;
  }

  NimBLEService *svc = srv->createService(SVC_UUID);
  if (!svc) {
    Serial.println("BLE service creation failed");
    return;
  }

  NimBLECharacteristic *chr = svc->createCharacteristic(
      CHR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  if (!chr) {
    Serial.println("BLE characteristic creation failed");
    return;
  }

  // Fall alert characteristic: the app subscribes (notify) and receives "FALL".
  fallChr = svc->createCharacteristic(
      FALL_UUID, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
  if (!fallChr) {
    Serial.println("BLE fall characteristic creation failed");
  }

  imuChr = svc->createCharacteristic(
      IMU_UUID, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
  if (!imuChr) {
    Serial.println("BLE IMU stream characteristic creation failed");
  }

  NimBLEDevice::setMTU(100);
  chr->setCallbacks(new TimeCb());
  svc->start();
  Serial.println("[BLE] service started");
  printRAM("after BLE service");

  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  if (adv->start()) Serial.println("[BLE] advertising started: SmartWatch");
  else Serial.println("[BLE] advertising start failed");
  printRAM("after BLE advertising");
}
#endif

// ================================== setup/loop ===============================
void initRenderer() {
  for (int i = 0; i <= 4096; i++) sqrtLut[i] = (uint8_t)(sqrtf((float)i) + 0.5f);
  for (int j = 0; j < 120; j++)
    for (int i = 0; i <= j; i++)
      distTab[j * (j + 1) / 2 + i] =
          (uint16_t)(8.0f * sqrtf((i + .5f) * (i + .5f) + (j + .5f) * (j + .5f)) + 0.5f);
  for (int i = 0; i < 60; i++) { sinT[i] = sinf(i * 6.0f * DEG_TO_RAD); cosT[i] = cosf(i * 6.0f * DEG_TO_RAD); }
}

void printRAM(const char *stage) {
  Serial.printf("[RAM] %s | free=%u | largest=%u | minimum=%u | glyph-cache=%u/%u\n",
                stage,
                (unsigned)ESP.getFreeHeap(),
                (unsigned)ESP.getMaxAllocHeap(),
                (unsigned)ESP.getMinFreeHeap(),
                (unsigned)glyphCacheUsed,
                (unsigned)GLYPH_CACHE_BYTES);
}

void setup() {
  Serial.begin(115200);
  delay(150);
  Serial.println();
  Serial.println("KIT-GLE Smart Watch boot");

  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_BUZ, OUTPUT);

  // Initialize the physical GC9A01 directly. No Arduino_Canvas allocation.
  if (!panel || !panel->begin(27000000)) {
    Serial.println("FATAL: GC9A01 panel begin failed");
    while (true) delay(1000);
  }
  memset(fb, 0, sizeof(framebuffer));
  printRAM("after panel begin (framebuffer is static)");

  C_BG = panel->color565(0, 0, 0);
  C_WHITE = panel->color565(255, 255, 255);
  C_GRAY = panel->color565(130, 130, 135);
  C_DGRAY = panel->color565(38, 40, 46);
  C_CYAN = panel->color565(0, 200, 255);
  C_GREEN = panel->color565(40, 230, 100);
  C_ORANGE = panel->color565(255, 140, 20);
  C_RED = panel->color565(255, 50, 50);
  C_YELLOW = panel->color565(255, 210, 0);
  C_PILL = panel->color565(55, 55, 55);
  C_PINK = panel->color565(200, 106, 110);
  C_DATE = panel->color565(120, 220, 150);
  C_BRAND = panel->color565(255, 122, 0);
  C_ICON = panel->color565(244, 154, 26);
  C_TRACK = panel->color565(18, 18, 18);

  initRenderer();
  showSplash(0.0f);                      // loading screen appears immediately

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  imuOk = icm.begin_I2C(0x69, &Wire) || icm.begin_I2C(0x68, &Wire);
  Serial.println(imuOk ? "IMU OK" : "IMU NOT FOUND");
  if (imuOk) {
    icm.setAccelRange(ICM20948_ACCEL_RANGE_8_G);   // wider range so impacts are not clipped
    icm.setAccelRateDivisor(10);                   // ~100 Hz output data rate
    icm.setGyroRateDivisor(10);
    // Sample the IMU in its own task (priority above loop()) so drawing never blocks it.
    xTaskCreate(imuTask, "imu", 6144, nullptr, 2, nullptr);
  }
  printRAM("after IMU");

  parseCompileTime();

  beep(2000, 80);
  splashAnimate(0.0f, 0.65f, 700);
  Serial.println("[BOOT] splash at 65%");

#if USE_BLE
  delay(250);  // Let the display finish drawing before BLE starts (as in the original sketch).
  initBLE();
  printRAM("after BLE");
#else
  Serial.println("BLE disabled (USE_BLE=0)");
#endif

  Serial.println("[BOOT] finishing splash");
  splashAnimate(0.65f, 1.0f, 500);
  delay(250);                            // hold the full bar for a moment
  Serial.println("[BOOT] splash at 100%");
  printRAM("setup complete");
}

void loop() {
  static uint32_t lastDraw = 0;
  static uint32_t lastRAM = 0;
  static uint32_t lastBuz = 0;
  uint32_t now = millis();

  handleButton();

#if USE_BLE
  if (bleBeepPending) {
    bleBeepPending = false;
    beep(2500, 60);
  }
  if (fallSendPending) {
    fallSendPending = false;
    if (fallChr) {
      fallChr->setValue((const uint8_t *)"FALL", 4);
      fallChr->notify();
      Serial.println("[FALL] BLE notify sent");
    }
  }
  sendImuBatch();
#endif

  // Alarm beeps while the countdown is running.
  if (fallState == FALL_ALERT && now - lastBuz > 500) {
    lastBuz = now;
    beep(3000, 250);
  }

  if (now - lastDraw >= 50) {
    lastDraw = now;
    renderFrame();
    flushFrame();
  }

  if (now - lastRAM >= 5000) {
    lastRAM = now;
    printRAM("running (BLE advertising)");
  }
}