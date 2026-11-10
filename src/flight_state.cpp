#include "flight_state.h"

#include <cmath>

namespace flight {

void Detector::reset() {
  haveStreak_ = false;
  sinceMs_    = 0;
}

void Detector::startStreak(uint32_t nowMs, bool air, float altitude) {
  haveStreak_ = true;
  streakAir_  = air;
  sinceMs_    = nowMs;
  altMin_     = altitude;
  altMax_     = altitude;
}

State Detector::update(uint32_t nowMs, float groundSpeed, float climbRate, float altitude) {
  const float climb = std::fabs(climbRate);

  // Either being fast or moving vertically counts as flying. They are taken
  // separately on purpose: into wind the speed disappears, and in still air
  // above flat ground the climb does.
  const bool airEvidence = groundSpeed >= kAirborneSpeed || climb >= kAirborneClimb;

  // Being on the ground is the stronger claim and needs everything at once.
  const bool groundEvidence = groundSpeed <= kGroundSpeed && climb <= kGroundClimb;

  // Only the evidence against the current state is worth accumulating; the
  // state already holds the other case.
  const bool wanted = (state_ == State::Ground);   // looking for the opposite
  const bool evidence = wanted ? airEvidence : groundEvidence;

  if (!evidence) {
    haveStreak_ = false;
    return state_;
  }

  if (!haveStreak_ || streakAir_ != wanted) {
    startStreak(nowMs, wanted, altitude);
    return state_;
  }

  if (altitude < altMin_) altMin_ = altitude;
  if (altitude > altMax_) altMax_ = altitude;

  // Standing still for a while but drifting up or down all the time is not
  // standing still. This is what a glider parked in ridge lift looks like, and
  // it is also why walking uphill has to stay well inside the range.
  if (!wanted && (altMax_ - altMin_) > kGroundAltitudeRange) {
    haveStreak_ = false;
    return state_;
  }

  const uint32_t needed = wanted ? kTakeoffMs : kLandingMs;
  if (nowMs - sinceMs_ >= needed) {
    state_      = wanted ? State::Airborne : State::Ground;
    haveStreak_ = false;
  }
  return state_;
}

}  // namespace flight
