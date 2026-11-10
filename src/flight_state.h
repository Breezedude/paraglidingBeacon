#pragma once
// ===========================================================================
//  Telling walking from flying.
//
//  Ground speed alone does not do it. A paraglider into wind sits still over
//  the ground for minutes at a time, and hiking to a launch in the mountains
//  gains altitude steadily, so neither speed nor a changing altitude decides
//  anything on its own. What separates the two is that flying keeps producing
//  vertical movement of a magnitude walking never reaches, and that the state
//  should be slow to be given up once it is established.
//
//  Plain arithmetic, no hardware, so it can be tested on a host.
// ===========================================================================

#include <cstdint>

namespace flight {

enum class State : uint8_t { Ground, Airborne };

// Above this ground speed the device is moving faster than a person walks.
constexpr float kAirborneSpeed = 4.2f;    // m/s, about 15 km/h

// Below this it is walking pace or slower.
constexpr float kGroundSpeed = 3.3f;      // m/s, 12 km/h

// Vertical speed that walking does not reach. A brisk ascent on foot is around
// 0.3 m/s; thermalling and gliding are several times that.
constexpr float kAirborneClimb = 1.0f;    // m/s

// Vertical speed low enough to be consistent with standing or walking.
constexpr float kGroundClimb = 0.5f;      // m/s

// How much the altitude may wander over a period that otherwise looks like
// standing still. This is what keeps a slope landing from reading as flight
// and, more importantly, a hovering glider from reading as a walk.
constexpr float kGroundAltitudeRange = 15.0f;   // m

// Evidence has to persist. Taking off is recognised quickly because being
// shown as walking while airborne is the worse error; landing is recognised
// slowly, because a moment of hanging still in the air is not a landing.
constexpr uint32_t kTakeoffMs = 5000;
constexpr uint32_t kLandingMs = 45000;

class Detector {
 public:
  // One step per position report. Speed and climb are in m/s, altitude in
  // metres, and nowMs is monotonic. Returns the state after this reading.
  State update(uint32_t nowMs, float groundSpeed, float climbRate, float altitude);

  State state() const { return state_; }
  bool  airborne() const { return state_ == State::Airborne; }

  // Forgets the evidence collected so far but keeps the state; used when the
  // fix was lost and the readings in between mean nothing.
  void reset();

 private:
  State    state_       = State::Ground;
  uint32_t sinceMs_     = 0;      // when the current evidence streak began
  bool     streakAir_   = false;  // what the streak is evidence for
  bool     haveStreak_  = false;
  float    altMin_      = 0.0f;
  float    altMax_      = 0.0f;

  void startStreak(uint32_t nowMs, bool air, float altitude);
};

}  // namespace flight
