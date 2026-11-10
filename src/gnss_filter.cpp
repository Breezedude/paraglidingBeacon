#include "gnss_filter.h"

#include <cmath>

namespace gnss {
namespace {

constexpr double kEarthRadius = 6371000.0;
constexpr double kDegToRad    = 3.14159265358979323846 / 180.0;

// Time constants for the exponential smoothing, in seconds. Altitude is the
// noisiest quantity and gets the most; speed and heading only need enough to
// take the jitter off.
constexpr float kTauAltitude = 1.5f;
constexpr float kTauClimb    = 3.0f;
constexpr float kTauSpeed    = 1.5f;
constexpr float kTauTrack    = 2.0f;

// Smoothing factor for a step of dt seconds, so the result does not depend on
// how often the receiver happens to report.
float alphaFor(float dt, float tau) {
  if (dt <= 0.0f) return 0.0f;
  if (tau <= 0.0f) return 1.0f;
  return 1.0f - std::exp(-dt / tau);
}

float blend(float previous, float current, float alpha) {
  return previous + (current - previous) * alpha;
}

}  // namespace

// ---------------------------------------------------------------------------
double distanceMetres(double lat1, double lon1, double lat2, double lon2) {
  // Equirectangular approximation: the error is negligible over the distance
  // covered between two fixes and it costs one trigonometric call.
  const double meanLat = (lat1 + lat2) * 0.5 * kDegToRad;
  const double dLat    = (lat2 - lat1) * kDegToRad;
  const double dLon    = (lon2 - lon1) * kDegToRad * std::cos(meanLat);
  return std::sqrt(dLat * dLat + dLon * dLon) * kEarthRadius;
}

float headingDelta(float from, float to) {
  float d = std::fmod(to - from + 540.0f, 360.0f) - 180.0f;
  return d;
}

// ---------------------------------------------------------------------------
void Filter::reset() {
  pos_    = Position{};
  seeded_ = false;
  rejects_ = 0;
}

void Filter::seed(const Fix& fix) {
  pos_.latitude    = fix.latitude;
  pos_.longitude   = fix.longitude;
  pos_.altitude    = fix.altitude;
  pos_.groundSpeed = fix.groundSpeed;
  pos_.track       = fix.track;
  pos_.climbRate   = 0.0f;
  pos_.valid       = true;

  lastLat_ = fix.latitude;
  lastLon_ = fix.longitude;
  lastAlt_ = fix.altitude;
  lastMs_  = fix.timeMs;
  seeded_  = true;
  rejects_ = 0;
}

bool Filter::update(const Fix& fix) {
  // A solution built on too few satellites is not worth filtering; it is not
  // wrong so much as unknown, and transmitting it would mislead.
  if (fix.satellites < kMinSatellites) {
    ++rejected_;
    return false;
  }

  if (!seeded_) {
    seed(fix);
    return true;
  }

  // Time only ever moves forward here. A step of zero carries no new
  // information and a step backwards means the clock wrapped, which is a
  // reason to start over rather than to divide by it.
  if (fix.timeMs <= lastMs_) {
    ++rejected_;
    return false;
  }

  const float dt = float(fix.timeMs - lastMs_) / 1000.0f;

  const double moved  = distanceMetres(lastLat_, lastLon_, fix.latitude, fix.longitude);
  const float  speed  = float(moved) / dt;
  const float  climb  = (fix.altitude - lastAlt_) / dt;

  if (speed > kMaxGroundSpeed || std::fabs(climb) > kMaxClimbRate) {
    ++rejected_;
    // Repeated rejections mean the old reference is the wrong one, not the new
    // reading, so the filter re-seeds instead of rejecting forever.
    if (++rejects_ >= kMaxRejections) {
      seed(fix);
      return true;
    }
    return false;
  }
  rejects_ = 0;

  pos_.latitude  = fix.latitude;
  pos_.longitude = fix.longitude;

  pos_.altitude    = blend(pos_.altitude, fix.altitude, alphaFor(dt, kTauAltitude));
  pos_.climbRate   = blend(pos_.climbRate, climb, alphaFor(dt, kTauClimb));
  pos_.groundSpeed = blend(pos_.groundSpeed, fix.groundSpeed, alphaFor(dt, kTauSpeed));

  // Heading is only meaningful once the device is actually going somewhere,
  // and it wraps, so it is smoothed along the shorter way round.
  if (fix.groundSpeed >= kHeadingHoldSpeed) {
    const float delta = headingDelta(pos_.track, fix.track);
    pos_.track = std::fmod(pos_.track + delta * alphaFor(dt, kTauTrack) + 360.0f, 360.0f);
  }

  lastLat_ = fix.latitude;
  lastLon_ = fix.longitude;
  lastAlt_ = fix.altitude;
  lastMs_  = fix.timeMs;
  pos_.valid = true;
  return true;
}

}  // namespace gnss
