#include "proto/ads_l.h"

#include "proto/bits.h"
#include "proto/xxtea.h"

namespace proto::ads_l {

namespace {

// ---------------------------------------------------------------------------
//  Variable-range encoding (spec G.1.6). The thresholds are 1, 3, 7, 15 times
//  the base scale; every step halves the resolution and covers the previous.
//  Verified against the spec examples G.1.7..G.1.9 in the host tests.
// ---------------------------------------------------------------------------
template <typename T, unsigned Bits>
constexpr T encodeUnsVR(T value) noexcept {
  if (value < (T(1) << Bits)) {
  } else if (value < (T(3) << Bits)) {
    value = (T(1) << Bits) | ((value - (T(1) << Bits)) >> 1);
  } else if (value < (T(7) << Bits)) {
    value = (T(2) << Bits) | ((value - (T(3) << Bits)) >> 2);
  } else if (value < (T(15) << Bits)) {
    value = (T(3) << Bits) | ((value - (T(7) << Bits)) >> 3);
  } else {
    value = (T(4) << Bits) - 1;
  }
  return value;
}

template <typename T, unsigned Bits>
constexpr T decodeUnsVR(T value) noexcept {
  const T range = T(value >> Bits);
  value &= (T(1) << Bits) - 1;
  if (range == 0) return value;
  if (range == 1) return (T(1) << Bits) + 1 + (value << 1);
  if (range == 2) return (T(3) << Bits) + 2 + (value << 2);
  return (T(7) << Bits) + 4 + (value << 3);
}

// Signed view of the same scheme: magnitude in the VR field, sign bit above.
template <typename T, unsigned Bits>
constexpr T encodeSignVR(T value) noexcept {
  T sign = 0;
  if (value < 0) { value = T(-value); sign = T(1) << (Bits + 2); }
  return T(encodeUnsVR<T, Bits>(value)) | sign;
}

template <typename T, unsigned Bits>
constexpr T decodeSignVR(T value) noexcept {
  const T sign = value & (T(1) << (Bits + 2));
  value = decodeUnsVR<T, Bits>(value & ((T(1) << (Bits + 2)) - 1));
  return sign ? T(-value) : value;
}

// ---------------------------------------------------------------------------
//  Coordinates: the air form is the FANET-cordic scale, 2^30 over the range
//  of +-90 degrees. Converting doubles through the ublox 1e-7 scale keeps the
//  rounding behaviour identical to the reference implementation, which the
//  golden-frame test locks in.
// ---------------------------------------------------------------------------
constexpr double kFntDeg = 90.0007295677 / double(uint64_t(1) << 30);

int32_t degToFnt(double deg) noexcept {
  const int64_t ubx = int64_t(deg * 1e7);
  return int32_t((ubx * 5003959 + (int64_t(1) << 21)) >> 22);
}

double fntToDeg(int32_t fnt) noexcept { return double(fnt) * kFntDeg; }

// ---------------------------------------------------------------------------
//  CRC-24, the Mode-S polynomial in MSB-first form: the byte is OR-ed into the
//  register, then the polynomial is XOR-ed before the shift on every bit, as in
//  the worked example of the EASA ADS-L 4 SRD 860 specification, section E.3.1.
//  The order matters: XOR after the shift produces a different checksum that
//  receiving stations reject as a mismatch.
// ---------------------------------------------------------------------------
constexpr uint32_t kCrcPoly = 0xFFFA0480;

uint32_t polyPass(uint32_t crc, uint8_t byte) noexcept {
  crc |= byte;
  for (int bit = 0; bit < 8; ++bit) {
    if (crc & 0x80000000u) crc ^= kCrcPoly;
    crc <<= 1;
  }
  return crc;
}

uint32_t polyCheckPI(const uint8_t* data, std::size_t bytes) noexcept {
  uint32_t crc = 0;
  for (std::size_t i = 0; i < bytes; ++i) crc = polyPass(crc, data[i]);
  return crc >> 8;
}

// The CRC residual produced by a single flipped bit at the given position,
// counted MSB-first through the frame. Positions 0..191.
uint32_t crcSyndrome(unsigned bit) noexcept {
  uint8_t packet[kFrameBytes] = {};
  packet[bit >> 3] = uint8_t(1u << (7u - (bit & 7u)));
  return polyCheckPI(packet, kFrameBytes);
}

// ---------------------------------------------------------------------------
//  Scrambling: XXTEA with the fixed key variant, 6 rounds. The reference
//  applies it to the five 32-bit words of the body.
// ---------------------------------------------------------------------------
void scramble(uint8_t body[kPayloadBytes]) noexcept {
  uint32_t w[kPayloadBytes / 4];
  for (int i = 0; i < 5; ++i) w[i] = loadLE32(body + 4 * i);
  xxteaEncryptKey0(w, 5, 6);
  for (int i = 0; i < 5; ++i) storeLE32(body + 4 * i, w[i]);
}

void descramble(uint8_t body[kPayloadBytes]) noexcept {
  uint32_t w[kPayloadBytes / 4];
  for (int i = 0; i < 5; ++i) w[i] = loadLE32(body + 4 * i);
  xxteaDecryptKey0(w, 5, 6);
  for (int i = 0; i < 5; ++i) storeLE32(body + 4 * i, w[i]);
}

}  // namespace

// ---------------------------------------------------------------------------
void buildBody(const State& s, uint8_t out[kPayloadBytes]) {
  uint8_t b[kPayloadBytes] = {};
  b[0] = 0x02;                              // iConspicuity position

  const uint32_t addr32 = (s.addressTable & 0x3F) | ((s.address & 0xFFFFFF) << 6);
  b[1] = uint8_t(addr32);                   // bits 30/31: reserved, relay=0
  b[2] = uint8_t(addr32 >> 8);
  b[3] = uint8_t(addr32 >> 16);
  b[4] = uint8_t(addr32 >> 24);

  b[5] = (s.timestamp & 0x3F) | ((s.flightState & 0x03) << 6);
  b[6] = (s.aircraft & 0x1F) | ((s.emergency & 0x07) << 5);

  const int32_t lat = degToFnt(s.latitude);
  const int32_t lon = degToFnt(s.longitude);
  const int32_t lat24 = (lat + 0x40) >> 7;
  const int32_t lon24 = (lon + 0x80) >> 8;
  b[7]  = uint8_t(lat24);
  b[8]  = uint8_t(lat24 >> 8);
  b[9]  = uint8_t(lat24 >> 16);
  b[10] = uint8_t(lon24);
  b[11] = uint8_t(lon24 >> 8);
  b[12] = uint8_t(lon24 >> 16);

  b[13] = uint8_t(encodeUnsVR<uint16_t, 6>(
      uint16_t(s.groundSpeed * 4.0f + 0.5f)));

  int64_t alt = int64_t(s.altitude + 320.0f);
  if (alt < 0) alt = 0;
  if (alt > 61432) alt = 61432;
  const uint16_t altWord = encodeUnsVR<uint32_t, 12>(uint32_t(alt));
  b[14] = uint8_t(altWord);
  b[15] = uint8_t((altWord >> 8) & 0x3F);

  const int16_t climbWord = encodeSignVR<int16_t, 6>(
      int16_t(s.climbRate * 8.0f + (s.climbRate >= 0.0f ? 0.5f : -0.5f)));
  b[15] = uint8_t(b[15] | ((climbWord & 0x03) << 6));
  b[16] = uint8_t((climbWord >> 2) & 0x7F);

  uint16_t track = uint16_t(s.track * (64.0f / 45.0f) + 0.5f);
  if (track >= 512) track = 0;              // 360 degrees wraps to north
  b[16] = uint8_t(b[16] | ((track & 0x01) << 7));
  b[17] = uint8_t(track >> 1);

  // Integrity word: all zeros, matching the reference transmitter.
  b[18] = 0;
  b[19] = 0;

  for (std::size_t i = 0; i < kPayloadBytes; ++i) out[i] = b[i];
}

// ---------------------------------------------------------------------------
void buildFrame(const State& s, uint8_t out[kFrameBytes]) {
  out[0] = 0x00;                            // key bits 0: fixed-key scrambling
  buildBody(s, out + 1);
  scramble(out + 1);
  const uint32_t crc = crcAppend(out, 1 + kPayloadBytes);
  out[kFrameBytes - 3] = uint8_t(crc >> 16);
  out[kFrameBytes - 2] = uint8_t(crc >> 8);
  out[kFrameBytes - 1] = uint8_t(crc);
}

// ---------------------------------------------------------------------------
uint32_t crcAppend(const uint8_t* data, std::size_t bytes) {
  uint32_t crc = 0;
  for (std::size_t i = 0; i < bytes; ++i) crc = polyPass(crc, data[i]);
  crc = polyPass(crc, 0);                   // pad to 24 bits
  crc = polyPass(crc, 0);
  crc = polyPass(crc, 0);
  return crc >> 8;
}

uint32_t checkPI(const uint8_t* frame, std::size_t bytes) {
  return polyCheckPI(frame, bytes);
}

// ---------------------------------------------------------------------------
int correctFrame(uint8_t* frame, std::size_t bytes) {
  if (polyCheckPI(frame, bytes) == 0) return 0;
  for (unsigned bit = 0; bit < bytes * 8; ++bit) {
    if (polyCheckPI(frame, bytes) == crcSyndrome(bit)) {
      frame[bit >> 3] ^= uint8_t(1u << (7u - (bit & 7u)));
      return 1;
    }
  }
  return -1;
}

// ---------------------------------------------------------------------------
bool decodeFrame(const uint8_t in[kFrameBytes], Decoded& out) {
  uint8_t pkt[kFrameBytes];
  for (std::size_t i = 0; i < kFrameBytes; ++i) pkt[i] = in[i];

  const int corrected = correctFrame(pkt, kFrameBytes);
  if (corrected < 0) return false;

  const uint8_t version = pkt[0];
  const uint8_t key     = (version >> 5) & 0x03;
  if (key != 0 && key != 3) return false;   // derived-key frames unsupported

  uint8_t* body = pkt + 1;
  if (key == 0) descramble(body);
  if (body[0] != 0x02) return false;        // position type only

  out.version      = version;
  const uint32_t addr32 = loadLE32(body + 1);
  out.addressTable = addr32 & 0x3F;
  out.address      = (addr32 >> 6) & 0xFFFFFF;
  out.relay        = (addr32 >> 31) & 1;
  out.timestamp    = body[5] & 0x3F;
  out.flightState  = (body[5] >> 6) & 0x03;
  out.aircraft     = body[6] & 0x1F;
  out.emergency    = (body[6] >> 5) & 0x07;

  int32_t lat = int32_t(body[7]) | (int32_t(body[8]) << 8) |
                (int32_t(body[9]) << 16);
  lat <<= 8; lat >>= 1;                     // sign-extend the 24 bit field
  int32_t lon = int32_t(body[10]) | (int32_t(body[11]) << 8) |
                (int32_t(body[12]) << 16);
  lon <<= 8;
  out.latitude  = fntToDeg(lat);
  out.longitude = fntToDeg(lon);

  out.groundSpeed = 0.25f * float(decodeUnsVR<uint16_t, 6>(body[13]));

  const uint16_t altWord = uint16_t(body[14] | ((body[15] & 0x3F) << 8));
  out.altitude = float(int32_t(decodeUnsVR<uint32_t, 12>(altWord)) - 320);

  const uint16_t climbWord = uint16_t((body[16] << 2) | (body[15] >> 6));
  out.climbRate = 0.125f * float(decodeSignVR<int16_t, 6>(climbWord));

  const uint16_t trackWord = uint16_t((body[17] << 1) | (body[16] >> 7));
  out.track = (45.0f / 64.0f) * float(trackWord);
  return true;
}

}  // namespace proto::ads_l