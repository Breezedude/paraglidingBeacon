#pragma once
// ===========================================================================
//  FLARM legacy air protocol, transmit side
// ---------------------------------------------------------------------------
//  Two frame formats are in use. The older one (V6) is still required by
//  receivers that predate the 2024 update; the newer one (V7) carries the same
//  information with finer quantisation.
//
//  This layer is pure: it turns a position into the bytes that go on the air
//  and knows nothing about any radio. That keeps it testable on a host.
//
//  Frame construction:
//    24 byte payload  ->  CRC-16 appended  ->  Manchester encoded  ->  52 bytes
// ===========================================================================

#include <cstddef>
#include <cstdint>

#include "report.h"

namespace proto::flarm {

enum class Format : uint8_t { V6, V7 };

enum class Aircraft : uint8_t {
  Unknown = 0, Glider, TowPlane, Helicopter, Parachute, DropPlane,
  HangGlider, Paraglider, Powered, Jet, Ufo, Balloon, Zeppelin, Uav,
  Reserved, Static
};

inline constexpr std::size_t kPayloadBytes = 24;
inline constexpr std::size_t kFrameBytes   = 2 * (kPayloadBytes + 2);

// Everything the encoder needs. Units are SI unless stated otherwise.
struct State {
  double   latitude    = 0.0;   // degrees
  double   longitude   = 0.0;   // degrees
  float    altitude    = 0.0f;  // metres above the WGS84 ellipsoid
  float    groundSpeed = 0.0f;  // m/s
  float    climbRate   = 0.0f;  // m/s, positive up
  float    track       = 0.0f;  // degrees true
  uint32_t utcSeconds  = 0;     // unix time, must be exact: it keys the cipher
  uint32_t address     = 0;     // 24 bit device address
  Aircraft aircraft    = Aircraft::Paraglider;
};

// Builds the plaintext 24 byte payload. Exposed for testing.
void buildPayload(const State& s, Format fmt, uint8_t out[kPayloadBytes]);

// Builds the complete on-air frame: payload, CRC, Manchester encoding.
void buildFrame(const State& s, Format fmt, uint8_t out[kFrameBytes]);


// ---------------------------------------------------------------------------
//  Receive side
// ---------------------------------------------------------------------------
//  Decoding needs the receiver's own position and clock. Positions are sent
//  with their most significant bits removed, so they are only meaningful
//  relative to a nearby reference, and the timestamp keys the cipher. A clock
//  that is off by more than a second will simply fail to decrypt.
struct Reference {
  double   latitude   = 0.0;
  double   longitude  = 0.0;
  uint32_t utcSeconds = 0;
};

// Validates and decodes a received frame. Returns false for a corrupted
// symbol, a bad checksum or an unknown frame format.
bool decodeFrame(const uint8_t in[kFrameBytes], const Reference& ref,
                 Traffic& out);

}  // namespace proto::flarm
