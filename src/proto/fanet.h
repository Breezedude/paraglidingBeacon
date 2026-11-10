#pragma once
// ===========================================================================
//  FANET air protocol, transmit side
// ---------------------------------------------------------------------------
//  Pure encoder: turns a position or a name into the payload bytes. Radio
//  handling lives elsewhere, so this can be unit tested on a host.
// ===========================================================================

#include <cstddef>
#include <cstdint>

#include "report.h"

namespace proto::fanet {

enum class Aircraft : uint8_t {
  Other = 0, Paraglider, HangGlider, Balloon, Glider, Powered, Helicopter, Uav
};

enum class Type : uint8_t { Tracking = 1, Name = 2, GroundTracking = 7 };

// What a device on the ground reports about itself. The values above Boot are
// rescue states and are not sent automatically by this firmware.
enum class Ground : uint8_t {
  Other = 0, Walking = 1, Vehicle = 2, Bike = 3, Boot = 4,
  NeedRide = 8, LandedWell = 9,
  NeedTechnical = 12, NeedMedical = 13, Distress = 14, DistressAuto = 15
};

inline constexpr std::size_t kHeaderBytes   = 4;
inline constexpr std::size_t kTrackingBytes = kHeaderBytes + 11;
inline constexpr std::size_t kGroundBytes   = kHeaderBytes + 7;
inline constexpr std::size_t kNameMaxBytes  = kHeaderBytes + 24;

struct Identity {
  uint8_t  vendor  = 0;
  uint16_t device  = 0;
};

struct State {
  double   latitude    = 0.0;   // degrees
  double   longitude    = 0.0;  // degrees
  float    altitude    = 0.0f;  // metres
  float    groundSpeed = 0.0f;  // km/h
  float    climbRate   = 0.0f;  // m/s, positive up
  float    track       = 0.0f;  // degrees true
  Aircraft aircraft    = Aircraft::Paraglider;
  bool     onlineTracking = true;
};

// Writes a tracking payload. Returns the number of bytes used.
std::size_t buildTracking(const Identity& id, const State& s,
                          uint8_t out[kTrackingBytes]);

// Writes a ground tracking payload: position and what the device is doing,
// with no aircraft type, altitude or velocity, because on the ground there is
// nothing to report about any of them. Returns the number of bytes used.
std::size_t buildGround(const Identity& id, double latitude, double longitude,
                        Ground status, bool onlineTracking,
                        uint8_t out[kGroundBytes]);

// Writes a name payload, truncated to fit. Returns the number of bytes used.
std::size_t buildName(const Identity& id, const char* name,
                      uint8_t out[kNameMaxBytes]);


// ---------------------------------------------------------------------------
//  Receive side
// ---------------------------------------------------------------------------
//  FANET sends absolute coordinates and no encryption, so a frame decodes on
//  its own without a reference position or a synchronised clock.
//
//  Recognised frame types: tracking (1), name (2), service (4, used by weather
//  stations) and ground tracking (7). Anything else is reported as unknown.
bool decodeFrame(const uint8_t* data, std::size_t len, Report& out);

}  // namespace proto::fanet
