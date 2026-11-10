#pragma once
// ===========================================================================
//  Decoded reports from received frames
// ---------------------------------------------------------------------------
//  All three air protocols carry roughly the same information about an
//  aircraft, so they decode into one common shape. Everything downstream, the
//  NMEA output and the status stream, works on these types alone and never has
//  to branch per protocol.
//
//  Fields that a protocol does not carry are left at their default and marked
//  through the `has*` flags, so a consumer can tell "zero" from "unknown".
// ===========================================================================

#include <cstdint>

namespace proto {

enum class Source : uint8_t { Flarm, Fanet, Adsl };

// Aircraft categories follow the FLARM data port enumeration, because that is
// what the NMEA output expects. The other protocols are mapped onto it.
enum class AircraftType : uint8_t {
  Reserved = 0, Glider, TowPlane, Helicopter, Parachute, DropPlane,
  HangGlider, Paraglider, Powered, Jet, Unknown, Balloon, Airship, Uav,
  Reserved2, StaticObstacle
};

struct Traffic {
  Source       source      = Source::Flarm;
  uint32_t     address     = 0;
  uint8_t      addressType = 0;      // NMEA <IDType>: 0 random, 1 ICAO, 2 FLARM
  double       latitude    = 0.0;
  double       longitude   = 0.0;
  float        altitude    = 0.0f;   // metres
  float        groundSpeed = 0.0f;   // m/s
  float        climbRate   = 0.0f;   // m/s, positive up
  float        track       = 0.0f;   // degrees true
  AircraftType aircraft    = AircraftType::Unknown;
  bool         hasPosition = false;
  bool         hasVelocity = false;
  bool         stealth     = false;
  bool         noTrack     = false;
  int16_t      rssi        = 0;      // dBm, 0 when unknown
};

// FANET service frames, which weather stations use.
struct Weather {
  uint32_t address     = 0;
  double   latitude    = 0.0;
  double   longitude   = 0.0;
  bool     hasPosition = false;
  bool     hasTemperature = false;
  bool     hasWind        = false;
  bool     hasHumidity    = false;
  bool     hasPressure    = false;
  bool     hasBattery     = false;
  bool     internetGateway = false;
  float    temperature = 0.0f;   // degrees Celsius
  float    windDir     = 0.0f;   // degrees the wind comes from
  float    windSpeed   = 0.0f;   // km/h
  float    windGust    = 0.0f;   // km/h
  float    humidity    = 0.0f;   // percent
  float    pressure    = 0.0f;   // hPa
  uint8_t  battery     = 0;      // percent
  int16_t  rssi        = 0;      // dBm
};

// FANET name frames.
struct Name {
  uint32_t address = 0;
  char     text[24] = {};
};

enum class ReportKind : uint8_t { None, Traffic, Weather, Name };

struct Report {
  ReportKind kind = ReportKind::None;
  Traffic    traffic;
  Weather    weather;
  Name       name;
};

}  // namespace proto
