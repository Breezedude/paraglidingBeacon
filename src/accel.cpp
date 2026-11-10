#include "accel.h"
#include "pins_t1000e.h"
#include <Wire.h>

Accel accel;

// --- Registers --------------------------------------------------------------
#define QMA_REG_CHIPID   0x00     // expects 0x90
#define QMA_REG_XOUT_L   0x01     // six bytes, X/Y/Z, 14 bit left aligned
#define QMA_REG_RANGE    0x0F
#define QMA_REG_BW       0x10
#define QMA_REG_PM       0x11     // bit 7 selects active mode
#define QMA_REG_RESET    0x36

// Threshold for what counts as movement. Deliberately coarse: it separates
// lying still from being carried, it does not recognise gestures.
// Raw values are 14 bit at +/-2 g, so roughly 4096 LSB per g.
#define MOTION_THRESHOLD  350.0f

// ---------------------------------------------------------------------------
static bool wr(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(QMA6100P_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool rd(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(QMA6100P_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)QMA6100P_ADDR, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// ---------------------------------------------------------------------------
bool Accel::begin() {
  _ok = false;

  uint8_t id = 0;
  if (!rd(QMA_REG_CHIPID, &id, 1)) return false;
  if (id != QMA6100P_CHIPID) return false;

  wr(QMA_REG_RESET, 0xB6);   delay(5);            // soft reset
  wr(QMA_REG_RESET, 0x00);   delay(10);

  wr(QMA_REG_RANGE, 0x01);                        // +/- 2 g
  wr(QMA_REG_BW, 0x05);                           // moderate output rate
  if (!wr(QMA_REG_PM, 0x80)) return false;        // leave standby
  delay(20);

  int16_t x, y, z;
  if (!readRaw(x, y, z)) return false;            // does it return data at all?

  _ok = true;
  return true;
}

// ---------------------------------------------------------------------------
bool Accel::readRaw(int16_t &x, int16_t &y, int16_t &z) {
  uint8_t b[6];
  if (!rd(QMA_REG_XOUT_L, b, 6)) return false;
  // 14 bit, left aligned in a 16 bit word
  x = (int16_t)((b[1] << 8) | b[0]) >> 2;
  y = (int16_t)((b[3] << 8) | b[2]) >> 2;
  z = (int16_t)((b[5] << 8) | b[4]) >> 2;
  return true;
}

bool Accel::motionSince() {
  if (!_ok) return false;

  int16_t x, y, z;
  if (!readRaw(x, y, z)) return false;

  if (!_havePrev) {
    _px = x; _py = y; _pz = z;
    _havePrev = true;
    return false;
  }

  float dx = (float)(x - _px), dy = (float)(y - _py), dz = (float)(z - _pz);
  _lastDelta = sqrtf(dx * dx + dy * dy + dz * dz);

  _px = x; _py = y; _pz = z;
  return _lastDelta > MOTION_THRESHOLD;
}
