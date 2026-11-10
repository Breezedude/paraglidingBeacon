#include "flarm.h"

#include <cmath>
#include <cstring>

#include "bits.h"
#include "xxtea.h"

namespace proto::flarm {
namespace {

// ---------------------------------------------------------------------------
//  Protocol constants
// ---------------------------------------------------------------------------
// The checksum seed is 0x051E, not the 0xFFFF that the usual CCITT naming
// implies. It was recovered by searching all 65536 possible seeds against three
// independently recorded frames that a receiving ground station had accepted as
// valid; exactly one seed satisfies all three. Those frames are kept as host
// tests so the value cannot quietly regress.
constexpr uint16_t kCrcSeed = 0x051E;

constexpr uint32_t kV6Table[8] = {
    0xE43276DFu, 0xDCA83759u, 0x9802B8ACu, 0x4675A56Bu,
    0xFC78EA65u, 0x804B90EAu, 0xB76542CDu, 0x329DFA32u};
constexpr uint32_t kV6MixA = 0x045D9F3Bu;
constexpr uint32_t kV6MixB = 0x87B562F4u;

constexpr uint32_t kV7Salt      = 0x956F6C77u;
constexpr uint32_t kV7Key[4]    = {0xA5F9B21Cu, 0xAB3F9D12u,
                                   0xC6F34E34u, 0xD72FA378u};
constexpr unsigned kCipherRounds = 6;

// The address tag states where the identifier comes from: 0 random, 1 ICAO,
// 2 assigned by FLARM, 3 self assigned (OGN). Ours is chosen by the operator
// and registered with OGN, so claiming a FLARM assignment would be wrong.
constexpr uint8_t kAddressTypeOgn = 3;

// Ground speed above which an aircraft counts as airborne, per type [m/s].
constexpr uint8_t kAirborneThreshold[16] = {
    0, 2, 10, 2, 5, 10, 2, 2, 10, 10, 0, 0, 0, 2, 0, 0};

// Longitude divisor by latitude for V7, indexed by |latitude| - 14.
constexpr uint16_t kLonDivisor[] = {
     53,  53,  54,  54,  55,  55,  56,  56,  57,  57,  58,  58,  59,  59,
     60,  60,  61,  61,  62,  62,  63,  63,  64,  64,  65,  65,  67,  68,
     70,  71,  73,  74,  76,  77,  79,  80,  82,  83,  85,  86,  88,  89,
     91,  94,  98, 101, 105, 108, 112, 115, 119, 122, 126, 129, 137, 144,
    152, 159, 167, 174, 190, 205, 221, 236, 252, 267, 299, 330, 362, 425,
    489, 552, 616, 679, 743, 806, 806};

// ---------------------------------------------------------------------------
//  Helpers
// ---------------------------------------------------------------------------
constexpr int32_t toMicroDegrees(double deg) {
  return int32_t(deg * 1e7);
}

// Rounds a scaled coordinate towards the nearer grid point. Bit 6 of the
// micro-degree value decides, which is the halfway mark of the 7 bit shift.
constexpr int32_t roundCoarse(int32_t micro, int32_t divisor) {
  const int32_t base = (divisor == 128) ? (micro >> 7) : (micro / divisor);
  const int32_t bump = (micro & 0x40) ? (micro < 0 ? -1 : 1) : 0;
  return base + bump;
}

// Pseudo floating point used by V7: `mantissa` bits of mantissa followed by
// `exponent` bits of binary exponent.
constexpr uint32_t compressUnsigned(uint32_t value, unsigned mantissa,
                                    unsigned exponent) {
  const uint32_t offset = uint32_t(1) << mantissa;
  const uint32_t limit  = (offset << exponent) - 1u;
  if (value < offset) return value;

  uint32_t e = 0;
  uint32_t m = offset + value;
  while (m > (offset + offset - 1u)) {
    m >>= 1;
    e += offset;
    if (e > limit) return limit;
  }
  return e | (m - offset);
}

constexpr uint32_t compressSigned(int32_t value, unsigned mantissa,
                                  unsigned exponent) {
  const uint32_t signBit = uint32_t(1) << (mantissa + exponent);
  uint32_t sign = 0;
  if (value < 0) { value = -value; sign = signBit; }
  const uint32_t magnitude = compressUnsigned(uint32_t(value), mantissa, exponent);
  return sign | magnitude;
}

uint8_t airborneThreshold(Aircraft a) {
  return kAirborneThreshold[uint8_t(a) & 0x0F];
}

// ---------------------------------------------------------------------------
//  Key derivation
// ---------------------------------------------------------------------------
constexpr uint32_t scramble(uint32_t value, uint32_t seed) {
  const uint32_t a = seed * (value ^ (value >> 16));
  const uint32_t b = seed * (a ^ (a >> 16));
  return b ^ (b >> 16);
}

void deriveV6Key(uint32_t key[4], uint32_t utc, uint32_t address) {
  const unsigned bank = ((utc >> 23) & 1u) ? 4u : 0u;
  const uint32_t salt = (utc >> 6) ^ ((address << 8) & 0xFFFFFFu);
  for (unsigned i = 0; i < 4; ++i) {
    key[i] = scramble(kV6Table[bank + i] ^ salt, kV6MixA) ^ kV6MixB;
  }
}

// Two rounds of a byte wise Feistel style mixing over the 16 key bytes.
void deriveV7Key(uint32_t key[4]) {
  auto* bytes = reinterpret_cast<uint8_t*>(key);
  uint32_t sum  = 0;
  uint32_t prev = bytes[15];

  for (int round = 0; round < 2; ++round) {
    sum += kXxteaDelta;
    for (unsigned i = 0; i < 16; ++i) {
      const uint32_t z = prev & 0xFFu;
      const uint32_t y = bytes[(i + 1) % 16];
      uint32_t x = bytes[i];
      x += (((z >> 5) ^ (y << 2)) + ((y >> 3) ^ (z << 4))) ^ (sum ^ y);
      bytes[i] = uint8_t(x);
      prev = x;
    }
  }
}

// ---------------------------------------------------------------------------
//  V6 payload
// ---------------------------------------------------------------------------
void buildV6(const State& s, uint8_t out[kPayloadBytes]) {
  const int32_t lat = toMicroDegrees(s.latitude);
  const int32_t lon = toMicroDegrees(s.longitude);

  // Speed shares a two bit shift with the vertical speed and the vector
  // components, so that fast aircraft still fit into the narrow fields.
  uint32_t quarters = uint32_t(std::lround(s.groundSpeed * 4.0f));
  if (quarters > 0x3FFu) quarters = 0x3FFu;

  unsigned shift = 0;
  if      (quarters & 0x200u) shift = 3;
  else if (quarters & 0x100u) shift = 2;
  else if (quarters & 0x080u) shift = 1;

  const uint32_t speed = quarters >> shift;
  const double   rad   = double(s.track) * 3.14159265358979323846 / 180.0;
  const int32_t  north = int32_t(int8_t(speed * std::cos(rad)));
  const int32_t  east  = int32_t(int8_t(speed * std::sin(rad)));
  const int32_t  climb = int32_t(std::lround(s.climbRate * 10.0f)) >> shift;

  int32_t altitude = int32_t(std::lround(s.altitude));
  if (altitude < 0) altitude = 0;

  std::memset(out, 0, kPayloadBytes);
  BitWriter w(out, kPayloadBytes);

  w.put(s.address & 0xFFFFFFu, 24);
  w.put(0, 4);                                  // format marker
  w.put(kAddressTypeOgn, 3);
  w.skip(1);

  w.putSigned(climb, 10);
  w.skip(2);
  w.put(uint32_t(s.groundSpeed) >= airborneThreshold(s.aircraft) ? 1u : 0u, 1);
  w.skip(1);                                    // stealth
  w.skip(1);                                    // no-track
  w.skip(1);                                    // parity, filled in below
  w.put(323, 12);                               // fix quality indicator
  w.put(uint8_t(s.aircraft), 4);

  w.put(uint32_t(roundCoarse(lat, 128)) & 0x7FFFFu, 19);
  w.put(uint32_t(altitude), 13);

  w.put(uint32_t(roundCoarse(lon, 128)) & 0xFFFFFu, 20);
  w.skip(10);
  w.put(shift, 2);

  for (int i = 0; i < 4; ++i) w.putSigned(north, 8);
  for (int i = 0; i < 4; ++i) w.putSigned(east, 8);

  // The parity bit covers the whole plaintext payload and therefore has to be
  // written last. It sits in the most significant bit of byte 5.
  if (parityOf(out, kPayloadBytes)) out[5] |= 0x80u;

  // Encrypt everything but the first word, which carries the address.
  uint32_t key[4];
  deriveV6Key(key, s.utcSeconds, s.address);

  uint32_t words[5];
  for (int i = 0; i < 5; ++i) words[i] = loadLE32(out + 4 + 4 * i);
  xxteaEncrypt(words, 5, key, kCipherRounds);
  for (int i = 0; i < 5; ++i) storeLE32(out + 4 + 4 * i, words[i]);
}

// ---------------------------------------------------------------------------
//  V7 payload
// ---------------------------------------------------------------------------
void buildV7(const State& s, uint8_t out[kPayloadBytes]) {
  const int32_t lat = toMicroDegrees(s.latitude);
  const int32_t lon = toMicroDegrees(s.longitude);

  int32_t altitude = int32_t(std::lround(s.altitude)) + 1000;
  if (altitude < 0) altitude = 0;

  unsigned degrees = unsigned(std::fabs(s.latitude));
  if (degrees > 89) degrees = 89;
  const int32_t lonDivisor = (degrees < 14) ? 52 : kLonDivisor[degrees - 14];

  std::memset(out, 0, kPayloadBytes);
  BitWriter w(out, kPayloadBytes);

  w.put(s.address & 0xFFFFFFu, 24);
  w.put(2, 4);                                  // format marker
  w.put(kAddressTypeOgn, 3);
  w.skip(1);

  w.skip(22);
  w.skip(1);                                    // stealth
  w.skip(1);                                    // no-track
  w.put(3, 2);                                  // fixed
  w.skip(2);
  w.put(3, 2);                                  // fixed
  w.skip(2);

  w.skip(2);
  w.put(s.utcSeconds & 0x0Fu, 4);               // low bits of the timestamp
  w.put(uint8_t(s.aircraft), 4);
  w.skip(1);
  w.put(compressUnsigned(uint32_t(altitude), 12, 1), 13);

  w.put(uint32_t(roundCoarse(lat, 52)) & 0xFFFFFu, 20);
  w.put(uint32_t(lon / lonDivisor) & 0xFFFFFu, 20);
  w.skip(9);                                    // turn rate, not derived
  w.put(compressUnsigned(uint32_t(std::lround(s.groundSpeed * 10.0f)), 8, 2), 10);
  w.put(compressSigned(int32_t(std::lround(s.climbRate * 10.0f)), 6, 2), 9);
  w.put(uint32_t(s.track * 2.0f) & 0x3FFu, 10);
  w.put(uint32_t(s.groundSpeed) >= airborneThreshold(s.aircraft) ? 2u : 1u, 2);

  w.put(0x1D, 6);                               // horizontal accuracy, ~10 m
  w.put(0x11, 5);                               // vertical accuracy, ~20 m
  w.skip(5);
  w.skip(8);

  // The key depends on the plaintext header, so it has to be taken before the
  // payload is encrypted.
  uint32_t key[4] = {loadLE32(out), loadLE32(out + 4), s.utcSeconds >> 4,
                     kV7Salt};
  deriveV7Key(key);

  uint32_t words[4];
  for (int i = 0; i < 4; ++i) words[i] = loadLE32(out + 8 + 4 * i);
  for (int i = 0; i < 4; ++i) words[i] ^= key[i];
  xxteaEncrypt(words, 4, kV7Key, kCipherRounds);
  for (int i = 0; i < 4; ++i) storeLE32(out + 8 + 4 * i, words[i]);
}

}  // namespace

// ---------------------------------------------------------------------------
void buildPayload(const State& s, Format fmt, uint8_t out[kPayloadBytes]) {
  if (fmt == Format::V7) buildV7(s, out);
  else                   buildV6(s, out);
}

void buildFrame(const State& s, Format fmt, uint8_t out[kFrameBytes]) {
  // The payload is not bit-inverted, contrary to descriptions of this protocol
  // that call for an inverted payload. Inverting it here and using the
  // Manchester polarity above cancel out, which is why both mistakes together
  // can look like a working implementation while neither alone does.
  uint8_t payload[kPayloadBytes + 2];
  buildPayload(s, fmt, payload);

  const uint16_t crc = crc16Ccitt(payload, kPayloadBytes, kCrcSeed);
  payload[kPayloadBytes]     = uint8_t(crc >> 8);
  payload[kPayloadBytes + 1] = uint8_t(crc);

  manchesterEncode(payload, kPayloadBytes + 2, out);
}


// ===========================================================================
//  Receive side
// ===========================================================================
namespace {

constexpr double kPi = 3.14159265358979323846;

// Rebuilds an absolute coordinate from the transmitted low bits.
// The sender drops the most significant bits, so the value is only meaningful
// against a nearby reference; the modulus folds it back around that reference.
int32_t restoreCoordinate(uint32_t raw, int32_t reference, unsigned bits,
                          int32_t divisor) {
  const uint32_t span = uint32_t(1) << bits;
  int32_t offset = int32_t((raw - uint32_t(reference)) & (span - 1u));
  if (offset >= int32_t(span >> 1)) offset -= int32_t(span);
  return (offset + reference) * divisor;
}

void decodeV6(const uint8_t payload[kPayloadBytes], const Reference& ref,
              Traffic& out) {
  BitReader r(payload, kPayloadBytes);
  r.skip(32);                                   // header, already parsed

  const int32_t  climbRaw = r.getSigned(10);
  r.skip(2);
  const bool     airborne = r.get(1) != 0;
  out.stealth  = r.get(1) != 0;
  out.noTrack  = r.get(1) != 0;
  r.skip(1);                                    // parity
  r.skip(12);                                   // fix quality indicator
  out.aircraft = AircraftType(r.get(4));

  const uint32_t latRaw = r.get(19);
  const int32_t  alt    = int32_t(r.get(13));
  const uint32_t lonRaw = r.get(20);
  r.skip(10);
  const unsigned shift  = r.get(2);

  int32_t north = 0, east = 0;
  for (int i = 0; i < 4; ++i) north += r.getSigned(8);
  for (int i = 0; i < 4; ++i) east  += r.getSigned(8);
  north /= 4;
  east  /= 4;

  const int32_t lat = restoreCoordinate(latRaw, int32_t(ref.latitude * 1e7) >> 7,
                                        19, 128);
  const int32_t lon = restoreCoordinate(lonRaw, int32_t(ref.longitude * 1e7) >> 7,
                                        20, 128);
  out.latitude    = double(lat) / 1e7;
  out.longitude   = double(lon) / 1e7;
  out.hasPosition = true;

  // Speed and the vector components share the same two bit shift.
  const double quarters =
      std::sqrt(double(east) * east + double(north) * north) * (1 << shift);
  out.groundSpeed = float(quarters / 4.0);
  out.climbRate   = float(double(climbRaw << shift) / 10.0);
  out.altitude    = float(alt);
  out.hasVelocity = true;

  if (quarters > 0.0) {
    double bearing = std::atan2(double(north), double(east)) * 180.0 / kPi;
    bearing = (bearing <= 90.0) ? (90.0 - bearing) : (450.0 - bearing);
    out.track = float(bearing);
  }
  (void)airborne;
}

void decodeV7(const uint8_t payload[kPayloadBytes], const Reference& ref,
              Traffic& out) {
  BitReader r(payload, kPayloadBytes);
  r.skip(32 + 22);                              // header and constant filler
  out.stealth = r.get(1) != 0;
  out.noTrack = r.get(1) != 0;

  BitReader body(payload, kPayloadBytes);
  body.skip(64 + 2);                            // start of the encrypted block
  body.skip(4);                                 // timestamp low bits
  out.aircraft = AircraftType(body.get(4));
  body.skip(1);
  const uint32_t altRaw   = body.get(13);
  const uint32_t latRaw   = body.get(20);
  const uint32_t lonRaw   = body.get(20);
  body.skip(9);                                 // turn rate
  const uint32_t speedRaw = body.get(10);
  const uint32_t climbRaw = body.get(9);
  const uint32_t course   = body.get(10);

  out.altitude = float(expandUnsigned(altRaw, 12, 1) - 1000);

  const int32_t lat = restoreCoordinate(latRaw, int32_t(ref.latitude * 1e7) / 52,
                                        20, 52);
  out.latitude = double(lat) / 1e7;

  // The longitude divisor depends on the latitude that was just recovered.
  unsigned degrees = unsigned(std::fabs(out.latitude));
  if (degrees > 89) degrees = 89;
  const int32_t divisor = (degrees < 14) ? 52 : kLonDivisor[degrees - 14];

  const int32_t lon = restoreCoordinate(
      lonRaw, int32_t(ref.longitude * 1e7) / divisor, 20, divisor);
  out.longitude   = double(lon) / 1e7;
  out.hasPosition = true;

  out.groundSpeed = float(expandUnsigned(speedRaw, 8, 2)) / 10.0f;
  out.climbRate   = float(expandSigned(climbRaw, 6, 2)) / 10.0f;
  out.track       = float(course) / 2.0f;
  out.hasVelocity = true;
}

}  // namespace

bool decodeFrame(const uint8_t in[kFrameBytes], const Reference& ref,
                 Traffic& out) {
  uint8_t payload[kPayloadBytes + 2];
  if (!manchesterDecode(in, kPayloadBytes + 2, payload)) return false;

  const uint16_t expected =
      uint16_t(uint16_t(payload[kPayloadBytes] << 8) | payload[kPayloadBytes + 1]);
  if (crc16Ccitt(payload, kPayloadBytes, kCrcSeed) != expected) return false;

  BitReader head(payload, kPayloadBytes);
  const uint32_t address = head.get(24);
  const uint32_t format  = head.get(4);
  const uint32_t addrTag = head.get(3);

  out = Traffic{};
  out.source      = Source::Flarm;
  out.address     = address;
  // The on-air tag distinguishes random, ICAO and FLARM assigned addresses;
  // the NMEA field uses the same meaning for its first three values.
  out.addressType = uint8_t(addrTag <= 2 ? addrTag : 0);

  uint32_t words[5];
  switch (format) {
    case 0: {
      uint32_t key[4];
      deriveV6Key(key, ref.utcSeconds, address);
      for (int i = 0; i < 5; ++i) words[i] = loadLE32(payload + 4 + 4 * i);
      xxteaDecrypt(words, 5, key, kCipherRounds);
      for (int i = 0; i < 5; ++i) storeLE32(payload + 4 + 4 * i, words[i]);
      decodeV6(payload, ref, out);
      return true;
    }
    case 2: {
      // Encoding applied the key first and the cipher second, so undo them in
      // the opposite order.
      uint32_t key[4] = {loadLE32(payload), loadLE32(payload + 4),
                         ref.utcSeconds >> 4, kV7Salt};
      deriveV7Key(key);

      uint32_t body[4];
      for (int i = 0; i < 4; ++i) body[i] = loadLE32(payload + 8 + 4 * i);
      xxteaDecrypt(body, 4, kV7Key, kCipherRounds);
      for (int i = 0; i < 4; ++i) body[i] ^= key[i];
      for (int i = 0; i < 4; ++i) storeLE32(payload + 8 + 4 * i, body[i]);
      decodeV7(payload, ref, out);
      return true;
    }
    default:
      return false;
  }
}

}  // namespace proto::flarm
