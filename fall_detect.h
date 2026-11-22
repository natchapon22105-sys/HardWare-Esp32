// fall_detect.h - threshold-based fall detector for the smart watch sketch
// Put this file in the same folder as the .ino and add:  #include "fall_detect.h"
//
// Pipeline:  IDLE -> (free-fall + impact) -> SETTLE -> WATCH (stillness)
//            -> ALERT (countdown, button cancels) -> SENT (BLE notify)
//
// fallUpdate() must be called at a steady rate (~100 Hz) with accel in g.

#pragma once
#include <Arduino.h>
#include <math.h>

// ----------------------------- tunables (adjust with real data) -----------------------------
#define FALL_FREEFALL_G       0.60f   // svm below this = free-fall
#define FALL_IMPACT_G         2.80f   // svm above this = impact
#define FALL_STRONG_G         4.00f   // very strong impact counts even without free-fall
#define FALL_FF_MIN_MS        40      // free-fall must last at least this long
#define FALL_FF_TO_IMPACT_MS  1000    // impact must come within this time after free-fall
#define FALL_SETTLE_MS        1000    // ignore the bounce right after impact
#define FALL_STILL_MS         3000    // then the wrist must stay still this long
#define FALL_STILL_RANGE_G    0.35f   // svm (max-min) below this = "still"
#define FALL_COUNTDOWN_MS     15000   // time the user has to cancel
#define FALL_SENT_SHOW_MS     5000

enum FallState : uint8_t { FALL_IDLE, FALL_SETTLE, FALL_WATCH, FALL_ALERT, FALL_SENT };

volatile FallState fallState = FALL_IDLE;
volatile bool fallSendPending = false;   // set once when the countdown ends; main loop sends BLE

static uint32_t fallT = 0;               // time the current state was entered
static uint32_t ffStart = 0, lastFF = 0;
static float stillMin = 0, stillMax = 0;

static inline bool fallActive() {
  FallState s = fallState;
  return s == FALL_ALERT || s == FALL_SENT;
}

static inline uint32_t fallRemainingMs(uint32_t now) {
  uint32_t el = now - fallT;
  return el >= FALL_COUNTDOWN_MS ? 0 : (FALL_COUNTDOWN_MS - el);
}

static void fallCancel() {
  fallState = FALL_IDLE;
  ffStart = 0;
  lastFF = 0;
}

static void fallUpdate(float ax, float ay, float az, uint32_t now) {
  float svm = sqrtf(ax * ax + ay * ay + az * az);
  FallState st = fallState;

  switch (st) {
    case FALL_IDLE:
      // 1) free-fall
      if (svm < FALL_FREEFALL_G) {
        if (!ffStart) ffStart = now;
        else if (now - ffStart >= FALL_FF_MIN_MS) lastFF = now;
      } else {
        ffStart = 0;
      }
      // 2) impact
      if (svm > FALL_IMPACT_G) {
        bool hadFF = lastFF && (now - lastFF < FALL_FF_TO_IMPACT_MS);
        if (hadFF || svm > FALL_STRONG_G) {
          Serial.printf("[FALL] impact %.2fg (free-fall=%d)\n", svm, hadFF ? 1 : 0);
          fallState = FALL_SETTLE;
          fallT = now;
          ffStart = 0;
          lastFF = 0;
        }
      }
      break;

    case FALL_SETTLE:
      if (now - fallT >= FALL_SETTLE_MS) {
        fallState = FALL_WATCH;
        fallT = now;
        stillMin = stillMax = svm;
      }
      break;

    case FALL_WATCH:
      if (svm < stillMin) stillMin = svm;
      if (svm > stillMax) stillMax = svm;
      if (stillMax - stillMin > FALL_STILL_RANGE_G) {
        Serial.println("[FALL] movement after impact -> cancelled");
        fallCancel();
      } else if (now - fallT >= FALL_STILL_MS) {
        Serial.println("[FALL] still after impact -> ALERT");
        fallState = FALL_ALERT;
        fallT = now;
      }
      break;

    case FALL_ALERT:
      if (now - fallT >= FALL_COUNTDOWN_MS) {
        Serial.println("[FALL] countdown ended -> send");
        fallSendPending = true;
        fallState = FALL_SENT;
        fallT = now;
      }
      break;

    case FALL_SENT:
      if (now - fallT >= FALL_SENT_SHOW_MS) fallCancel();
      break;
  }
}
