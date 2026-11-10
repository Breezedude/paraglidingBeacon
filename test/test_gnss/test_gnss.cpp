// Host tests for the position filter. No hardware, no Arduino.

#include <unity.h>

#include <cmath>

#include "gnss_filter.h"

using gnss::Filter;
using gnss::Fix;

namespace {

Fix at(double lat, double lon, float alt, uint32_t ms,
       float speed = 0.0f, float track = 0.0f, uint8_t sats = 8) {
  Fix f;
  f.latitude    = lat;
  f.longitude   = lon;
  f.altitude    = alt;
  f.groundSpeed = speed;
  f.track       = track;
  f.satellites  = sats;
  f.timeMs      = ms;
  return f;
}

}  // namespace

// ---------------------------------------------------------------------------
void test_distance_matches_known_separation() {
  // One degree of latitude is close to 111.2 km anywhere on the globe.
  const double d = gnss::distanceMetres(47.0, 12.0, 48.0, 12.0);
  TEST_ASSERT_FLOAT_WITHIN(500.0, 111200.0, float(d));

  // A degree of longitude shrinks with the cosine of the latitude.
  const double e = gnss::distanceMetres(47.0, 12.0, 47.0, 13.0);
  TEST_ASSERT_FLOAT_WITHIN(500.0, 111200.0 * std::cos(47.0 * 3.14159265 / 180.0), float(e));
}

void test_heading_delta_takes_the_short_way() {
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, gnss::headingDelta(10.0f, 30.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -20.0f, gnss::headingDelta(30.0f, 10.0f));
  // Across north the short way is 20 degrees, not 340.
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, gnss::headingDelta(350.0f, 10.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -20.0f, gnss::headingDelta(10.0f, 350.0f));
}

void test_too_few_satellites_is_not_a_fix() {
  Filter f;
  TEST_ASSERT_FALSE(f.update(at(47.0, 12.0, 1000.0f, 1000, 0.0f, 0.0f, 3)));
  TEST_ASSERT_FALSE(f.valid());
  TEST_ASSERT_TRUE(f.update(at(47.0, 12.0, 1000.0f, 2000, 0.0f, 0.0f, 4)));
  TEST_ASSERT_TRUE(f.valid());
}

void test_first_fix_is_taken_unchanged() {
  Filter f;
  TEST_ASSERT_TRUE(f.update(at(47.5, 12.5, 1200.0f, 1000, 10.0f, 90.0f)));
  TEST_ASSERT_EQUAL_DOUBLE(47.5, f.position().latitude);
  TEST_ASSERT_EQUAL_DOUBLE(12.5, f.position().longitude);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1200.0f, f.position().altitude);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, f.position().track);
  // Nothing to compare against yet, so no climb is claimed.
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, f.position().climbRate);
}

void test_position_jump_is_rejected() {
  Filter f;
  f.update(at(47.0, 12.0, 1000.0f, 1000));

  // Half a degree in one second is about 55 km/s, which no aircraft does.
  TEST_ASSERT_FALSE(f.update(at(47.5, 12.0, 1000.0f, 2000)));
  TEST_ASSERT_EQUAL_DOUBLE(47.0, f.position().latitude);
  TEST_ASSERT_EQUAL_UINT32(1, f.rejected());
}

void test_altitude_spike_is_rejected() {
  Filter f;
  f.update(at(47.0, 12.0, 1000.0f, 1000));
  TEST_ASSERT_FALSE(f.update(at(47.0, 12.0, 1500.0f, 2000)));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1000.0f, f.position().altitude);
}

void test_filter_reseeds_after_repeated_rejections() {
  Filter f;
  f.update(at(47.0, 12.0, 1000.0f, 1000));

  // A fix regained far away looks like a jump, but insisting on the stale
  // reference would leave the device transmitting the wrong position forever.
  uint32_t t = 2000;
  bool accepted = false;
  for (int i = 0; i < gnss::kMaxRejections; ++i, t += 1000) {
    accepted = f.update(at(48.0, 13.0, 1000.0f, t));
  }
  TEST_ASSERT_TRUE(accepted);
  TEST_ASSERT_EQUAL_DOUBLE(48.0, f.position().latitude);
}

void test_altitude_is_smoothed_not_followed() {
  Filter f;
  f.update(at(47.0, 12.0, 1000.0f, 1000));

  // A ten metre step within the rate limit is accepted, but only part of it
  // shows immediately.
  TEST_ASSERT_TRUE(f.update(at(47.0, 12.0, 1010.0f, 2000)));
  const float a = f.position().altitude;
  TEST_ASSERT_TRUE(a > 1000.0f);
  TEST_ASSERT_TRUE(a < 1010.0f);

  // Held there, it converges.
  for (uint32_t t = 3000; t <= 15000; t += 1000) f.update(at(47.0, 12.0, 1010.0f, t));
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 1010.0f, f.position().altitude);
}

void test_steady_climb_is_reported() {
  Filter f;
  float alt = 1000.0f;
  f.update(at(47.0, 12.0, alt, 1000));
  for (uint32_t t = 2000; t <= 40000; t += 1000) {
    alt += 2.0f;                                   // two metres per second
    f.update(at(47.0, 12.0, alt, t));
  }
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 2.0f, f.position().climbRate);
}

void test_heading_is_held_while_stationary() {
  Filter f;
  f.update(at(47.0, 12.0, 1000.0f, 1000, 10.0f, 90.0f));

  // Standing still the receiver reports nonsense headings; the last usable one
  // stays put.
  for (uint32_t t = 2000; t <= 10000; t += 1000) {
    f.update(at(47.0, 12.0, 1000.0f, t, 0.2f, float((t / 1000) * 37 % 360)));
  }
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, f.position().track);
}

void test_heading_follows_once_moving() {
  Filter f;
  f.update(at(47.0, 12.0, 1000.0f, 1000, 10.0f, 350.0f));
  // Turning to 10 degrees crosses north: the result must not swing the long
  // way round through 180.
  for (uint32_t t = 2000; t <= 20000; t += 1000) {
    f.update(at(47.0, 12.0, 1000.0f, t, 10.0f, 10.0f));
  }
  const float track = f.position().track;
  const bool nearTen = track < 12.0f || track > 358.0f;
  TEST_ASSERT_TRUE(nearTen);
}

void test_reset_forgets_everything() {
  Filter f;
  f.update(at(47.0, 12.0, 1000.0f, 1000));
  f.reset();
  TEST_ASSERT_FALSE(f.valid());
  // After a reset the next fix is taken as it comes, however far away.
  TEST_ASSERT_TRUE(f.update(at(48.0, 13.0, 2000.0f, 2000)));
}

void test_repeated_timestamp_carries_nothing() {
  Filter f;
  f.update(at(47.0, 12.0, 1000.0f, 1000));
  TEST_ASSERT_FALSE(f.update(at(47.0, 12.0, 1000.0f, 1000)));
}

// ---------------------------------------------------------------------------
extern "C" void setUp() {}
extern "C" void tearDown() {}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_distance_matches_known_separation);
  RUN_TEST(test_heading_delta_takes_the_short_way);
  RUN_TEST(test_too_few_satellites_is_not_a_fix);
  RUN_TEST(test_first_fix_is_taken_unchanged);
  RUN_TEST(test_position_jump_is_rejected);
  RUN_TEST(test_altitude_spike_is_rejected);
  RUN_TEST(test_filter_reseeds_after_repeated_rejections);
  RUN_TEST(test_altitude_is_smoothed_not_followed);
  RUN_TEST(test_steady_climb_is_reported);
  RUN_TEST(test_heading_is_held_while_stationary);
  RUN_TEST(test_heading_follows_once_moving);
  RUN_TEST(test_reset_forgets_everything);
  RUN_TEST(test_repeated_timestamp_carries_nothing);
  return UNITY_END();
}
