#pragma once
// ===========================================================================
//  ADS-L (SRD-860) air protocol, per EASA ED Decision 2022/024/R
// ---------------------------------------------------------------------------
//  Transmit side and receive side. Frame construction mirrors the reference
//  implementation (ADS-L Tx/Rx demo, pjalocha):
//    Version 1 byte, 20 byte body, 24 bit Mode-S CRC  -> 24 bytes.
//  The body is XXTEA scrambled with the fixed "Key0" variant (6 rounds);
//  deciphering the air protocol expects the receiver to own (or derive) keys,
//  but Key0 is the unscrambled baseline that all receivers must handle, and
//  key bits "3" carry the body in plaintext.
//
//  This layer is pure: it turns a position into the bytes that go on the air
//  and knows nothing about any radio. That keeps it testable on a host.
// ===========================================================================

#include <cstddef>
#include <cstdint>

namespace proto::ads_l {

inline constexpr std::size_t kPayloadBytes = 20;
inline constexpr std::size_t kCrcBytes     = 3;
inline constexpr std::size_t kFrameBytes   = 1 + kPayloadBytes + kCrcBytes;

// Aircraft categories, spec G.1.3.
enum class Aircraft : uint8_t {
  None = 0, LightFixedWing, SmallToHeavyFixedWing, Rotorcraft, Glider,
  LighterThanAir, Ultralight, HangGliderParaglider, Parachutist, Evtol,
  Gyrocopter, UasOpen, UasSpecific, UasCertified
};

// Everything the encoder needs. Units are SI unless stated otherwise.
struct State {
  double   latitude    = 0.0;   // degrees
  double   longitude   = 0.0;   // degrees
  float    altitude    = 0.0f;  // metres above the WGS84 ellipsoid
  float    groundSpeed = 0.0f;  // m/s
  float    climbRate   = 0.0f;  // m/s, positive up
  float    track       = 0.0f;  // degrees true
  uint8_t  timestamp   = 0;     // quarter seconds into the current hour, 0..59
  uint8_t  flightState = 1;     // 1 = on ground, 2 = airborne (spec G.1.2)
  uint8_t  aircraft    = uint8_t(Aircraft::None);  // spec G.1.3
  uint8_t  emergency   = 1;     // 1 = no emergency (spec G.1.4)
  uint8_t  addressTable = 7;    // OGN table
  uint32_t address      = 0;    // 24 bit device address within the table
};

// What decodeFrame recovers from a position packet.
struct Decoded {
  uint8_t  version      = 0;
  uint8_t  addressTable = 0;
  uint32_t address      = 0;
  bool     relay        = false;
  uint8_t  timestamp    = 0;    // quarter seconds, 0..59
  uint8_t  flightState  = 0;
  uint8_t  aircraft     = 0;
  uint8_t  emergency    = 0;
  double   latitude     = 0.0;  // degrees
  double   longitude    = 0.0;  // degrees
  float    altitude     = 0.0f; // metres HAE
  float    groundSpeed  = 0.0f; // m/s
  float    climbRate    = 0.0f; // m/s
  float    track        = 0.0f; // degrees true
};

// Builds the complete 24 byte frame: Version, scrambled body, CRC.
void buildFrame(const State& s, uint8_t out[kFrameBytes]);

// Builds the plaintext 20 byte body before scrambling. Exposed for testing.
void buildBody(const State& s, uint8_t out[kPayloadBytes]);

// The three CRC bytes to append to `bytes` payload bytes (MSB first).
uint32_t crcAppend(const uint8_t* data, std::size_t bytes);

// CRC run over the whole frame including the CRC bytes; 0 marks a valid frame.
uint32_t checkPI(const uint8_t* frame, std::size_t bytes);

// Repairs a single corrupted bit using the CRC residual syndrome.
// Returns 0 (already correct), 1 (one bit repaired), -1 (not repairable).
int correctFrame(uint8_t* frame, std::size_t bytes);

// Validates, descrambles and parses a 24 byte frame. Handles the Key0 and the
// plaintext variant; frames with derived-key scrambling are rejected.
bool decodeFrame(const uint8_t in[kFrameBytes], Decoded& out);

}  // namespace proto::ads_l