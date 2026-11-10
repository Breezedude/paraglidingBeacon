#pragma once
// ===========================================================================
//  Plausibility check and smoothing for the receiver's own position.
//
//  A bare NMEA stream is not good enough to transmit unchanged. Fixes jump
//  when satellites come and go, altitude is the noisiest quantity a GNSS
//  produces, and heading is meaningless while standing still. Everything here
//  works on plain numbers so it can be tested without hardware.
// ===========================================================================

#include <cstdint>

namespace gnss {

// One reading as it arrives from the receiver.
struct Fix {
  double   latitude    = 0.0;
  double   longitude   = 0.0;
  float    altitude    = 0.0f;   // m above mean sea level
  float    groundSpeed = 0.0f;   // m/s
  float    track       = 0.0f;   // degrees, 0..360
  uint8_t  satellites  = 0;
  uint32_t timeMs      = 0;      // monotonic milliseconds
};

// What the rest of the firmware transmits.
struct Position {
  double latitude    = 0.0;
  double longitude   = 0.0;
  float  altitude    = 0.0f;     // smoothed
  float  groundSpeed = 0.0f;     // smoothed
  float  track       = 0.0f;     // smoothed, held while stationary
  float  climbRate   = 0.0f;     // m/s, from the smoothed altitude
  bool   valid       = false;
};

// A fix has to be at least this good before it is used at all. Four satellites
// are the minimum for a three dimensional solution.
constexpr uint8_t kMinSatellites = 4;

// Limits for a single step between two fixes. Both are far above anything a
// free flight aircraft reaches and are meant to catch receiver artefacts, not
// to judge the flight.
constexpr float kMaxGroundSpeed = 100.0f;   // m/s, about 360 km/h
constexpr float kMaxClimbRate   = 30.0f;    // m/s

// A step that fails the check is dropped, but not forever: after this many
// consecutive rejections the filter accepts the new position and starts over.
// Without it a genuine jump, such as a fix regained far from where the device
// last saw one, would lock the filter out permanently.
constexpr uint8_t kMaxRejections = 4;

// Below this speed the reported heading is noise, so the last usable one is
// held instead of following it.
constexpr float kHeadingHoldSpeed = 1.0f;   // m/s

class Filter {
 public:
  // Feeds one reading. Returns true when it was accepted; a rejected reading
  // leaves the previous position in place.
  bool update(const Fix& fix);

  // Drops the history, for instance after the fix was lost.
  void reset();

  const Position& position() const { return pos_; }
  bool     valid()      const { return pos_.valid; }
  uint32_t rejected()   const { return rejected_; }

 private:
  Position pos_;
  double   lastLat_   = 0.0;
  double   lastLon_   = 0.0;
  float    lastAlt_   = 0.0f;
  uint32_t lastMs_    = 0;
  bool     seeded_    = false;
  uint8_t  rejects_   = 0;
  uint32_t rejected_  = 0;

  void seed(const Fix& fix);
};

// Great circle distance in metres, accurate enough over the short steps
// between two fixes.
double distanceMetres(double lat1, double lon1, double lat2, double lon2);

// Shortest signed difference between two headings, in degrees, -180..180.
float headingDelta(float from, float to);

}  // namespace gnss
