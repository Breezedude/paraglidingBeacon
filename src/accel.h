#pragma once
// ===========================================================================
//  QMA6100P accelerometer, used for the motion timeout
// ---------------------------------------------------------------------------
//  Deliberately minimal: it reads the raw acceleration vector and compares it
//  with the previous one. No interrupts, no filter configuration, so it rests
//  on as few register assumptions as possible.
//
//  If the sensor does not answer, begin() returns false and motion detection
//  stays disabled. A silent sensor must never shut the device down by itself.
// ===========================================================================

#include <Arduino.h>

class Accel {
public:
  bool begin();                       // false: not found, timeout disabled
  bool ok() const { return _ok; }

  // Reads the sensor and reports movement since the previous call.
  bool motionSince();

  float lastDelta() const { return _lastDelta; }

private:
  bool readRaw(int16_t &x, int16_t &y, int16_t &z);

  bool    _ok = false;
  int16_t _px = 0, _py = 0, _pz = 0;
  bool    _havePrev = false;
  float   _lastDelta = 0;
};

extern Accel accel;
