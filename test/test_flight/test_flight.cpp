// Host tests for the walking / flying detector, and for the FANET ground
// tracking frame it selects.

#include <unity.h>

#include <cstring>

#include "flight_state.h"
#include "proto/fanet.h"

using flight::Detector;
using flight::State;

namespace {

// Feeds a steady situation for a number of seconds at 1 Hz, which is what the
// receiver reports at.
State feed(Detector& d, uint32_t& t, unsigned seconds,
           float speed, float climb, float altitude) {
  State s = d.state();
  for (unsigned i = 0; i < seconds; ++i) {
    t += 1000;
    s = d.update(t, speed, climb, altitude);
    altitude += climb;
  }
  return s;
}

}  // namespace

// ---------------------------------------------------------------------------
void test_starts_on_the_ground() {
  Detector d;
  TEST_ASSERT_FALSE(d.airborne());
}

void test_standing_still_stays_on_the_ground() {
  Detector d;
  uint32_t t = 0;
  TEST_ASSERT_TRUE(feed(d, t, 120, 0.0f, 0.0f, 700.0f) == State::Ground);
}

void test_walking_uphill_in_the_mountains_is_not_flying() {
  Detector d;
  uint32_t t = 0;
  // Four km/h on foot, climbing 1000 m in an hour: steep for a walker and
  // still nowhere near flight.
  TEST_ASSERT_TRUE(feed(d, t, 900, 1.1f, 0.28f, 900.0f) == State::Ground);
}

void test_running_downhill_is_not_flying() {
  Detector d;
  uint32_t t = 0;
  TEST_ASSERT_TRUE(feed(d, t, 300, 2.5f, -0.45f, 1600.0f) == State::Ground);
}

void test_flying_is_recognised_by_speed() {
  Detector d;
  uint32_t t = 0;
  TEST_ASSERT_TRUE(feed(d, t, 4, 10.0f, 0.0f, 1000.0f) == State::Ground);
  TEST_ASSERT_TRUE(feed(d, t, 3, 10.0f, 0.0f, 1000.0f) == State::Airborne);
}

void test_flying_is_recognised_by_climb_alone() {
  Detector d;
  uint32_t t = 0;
  // Thermalling with no ground speed at all, which a speed rule would miss.
  TEST_ASSERT_TRUE(feed(d, t, 10, 0.5f, 2.0f, 1000.0f) == State::Airborne);
}

void test_headwind_does_not_land_the_aircraft() {
  Detector d;
  uint32_t t = 0;
  feed(d, t, 10, 12.0f, 1.5f, 1200.0f);
  TEST_ASSERT_TRUE(d.airborne());

  // Soaring into wind: no progress over the ground for two minutes, but the
  // glider keeps moving up and down.
  for (int i = 0; i < 60; ++i) {
    feed(d, t, 1, 0.3f, (i % 2) ? 1.4f : -1.2f, 1200.0f);
    feed(d, t, 1, 0.0f, (i % 2) ? -1.0f : 1.6f, 1200.0f);
  }
  TEST_ASSERT_TRUE(d.airborne());
}

void test_hanging_still_in_lift_does_not_land_the_aircraft() {
  Detector d;
  uint32_t t = 0;
  feed(d, t, 10, 12.0f, 1.5f, 1200.0f);

  // Almost no speed and only gentle vertical movement, but drifting upwards
  // the whole time: the altitude gives it away.
  TEST_ASSERT_TRUE(feed(d, t, 120, 0.5f, 0.4f, 1200.0f) == State::Airborne);
}

void test_landing_is_recognised_once_it_lasts() {
  Detector d;
  uint32_t t = 0;
  feed(d, t, 10, 12.0f, 1.5f, 1500.0f);
  TEST_ASSERT_TRUE(d.airborne());

  // Standing in the landing field. Nothing happens for the first half minute.
  TEST_ASSERT_TRUE(feed(d, t, 30, 0.2f, 0.0f, 800.0f) == State::Airborne);
  TEST_ASSERT_TRUE(feed(d, t, 20, 0.2f, 0.0f, 800.0f) == State::Ground);
}

void test_a_moment_of_slow_flight_does_not_land_the_aircraft() {
  Detector d;
  uint32_t t = 0;
  feed(d, t, 10, 12.0f, 1.5f, 1500.0f);

  // Twenty seconds of hanging about, then flying on: well short of the
  // landing threshold, so nothing changes.
  feed(d, t, 20, 0.4f, 0.1f, 1500.0f);
  TEST_ASSERT_TRUE(d.airborne());
  TEST_ASSERT_TRUE(feed(d, t, 30, 11.0f, -1.0f, 1500.0f) == State::Airborne);
}

void test_takeoff_after_a_walk_to_launch() {
  Detector d;
  uint32_t t = 0;
  feed(d, t, 600, 1.2f, 0.3f, 1000.0f);        // the hike up
  TEST_ASSERT_FALSE(d.airborne());
  feed(d, t, 20, 0.0f, 0.0f, 1200.0f);         // laying out the wing
  TEST_ASSERT_FALSE(d.airborne());
  TEST_ASSERT_TRUE(feed(d, t, 8, 9.0f, 1.0f, 1200.0f) == State::Airborne);
}

void test_reset_keeps_the_state_but_drops_the_evidence() {
  Detector d;
  uint32_t t = 0;
  feed(d, t, 4, 10.0f, 0.0f, 1000.0f);         // nearly airborne
  d.reset();
  TEST_ASSERT_FALSE(d.airborne());
  // The evidence has to be collected again from scratch.
  TEST_ASSERT_TRUE(feed(d, t, 3, 10.0f, 0.0f, 1000.0f) == State::Ground);
  TEST_ASSERT_TRUE(feed(d, t, 3, 10.0f, 0.0f, 1000.0f) == State::Airborne);
}

// ---------------------------------------------------------------------------
void test_ground_frame_layout() {
  const proto::fanet::Identity id{0xBD, 0x4A17};
  uint8_t frame[proto::fanet::kGroundBytes];

  const std::size_t n = proto::fanet::buildGround(
      id, 47.891290, 12.194160, proto::fanet::Ground::Walking, true, frame);
  TEST_ASSERT_EQUAL_UINT32(proto::fanet::kGroundBytes, n);

  // Header: type 7, no forwarding, no extended header.
  TEST_ASSERT_EQUAL_HEX8(0x07, frame[0]);
  TEST_ASSERT_EQUAL_HEX8(0xBD, frame[1]);
  TEST_ASSERT_EQUAL_HEX8(0x17, frame[2]);
  TEST_ASSERT_EQUAL_HEX8(0x4A, frame[3]);

  // Status in the upper nibble, online tracking in the lowest bit.
  TEST_ASSERT_EQUAL_HEX8(0x11, frame[10]);

  proto::Report out;
  TEST_ASSERT_TRUE(proto::fanet::decodeFrame(frame, n, out));
  TEST_ASSERT_TRUE(out.kind == proto::ReportKind::Traffic);
  TEST_ASSERT_TRUE(out.traffic.hasPosition);
  TEST_ASSERT_FALSE(out.traffic.hasVelocity);
  TEST_ASSERT_DOUBLE_WITHIN(0.0001, 47.891290, out.traffic.latitude);
  TEST_ASSERT_DOUBLE_WITHIN(0.0001, 12.194160, out.traffic.longitude);
  TEST_ASSERT_EQUAL_UINT32(0x00BD4A17u, out.traffic.address);
}

void test_ground_frame_without_online_tracking() {
  const proto::fanet::Identity id{0xBD, 0x0001};
  uint8_t frame[proto::fanet::kGroundBytes];
  proto::fanet::buildGround(id, 47.0, 12.0, proto::fanet::Ground::Other, false, frame);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[10]);
}

// ---------------------------------------------------------------------------
extern "C" void setUp() {}
extern "C" void tearDown() {}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_starts_on_the_ground);
  RUN_TEST(test_standing_still_stays_on_the_ground);
  RUN_TEST(test_walking_uphill_in_the_mountains_is_not_flying);
  RUN_TEST(test_running_downhill_is_not_flying);
  RUN_TEST(test_flying_is_recognised_by_speed);
  RUN_TEST(test_flying_is_recognised_by_climb_alone);
  RUN_TEST(test_headwind_does_not_land_the_aircraft);
  RUN_TEST(test_hanging_still_in_lift_does_not_land_the_aircraft);
  RUN_TEST(test_landing_is_recognised_once_it_lasts);
  RUN_TEST(test_a_moment_of_slow_flight_does_not_land_the_aircraft);
  RUN_TEST(test_takeoff_after_a_walk_to_launch);
  RUN_TEST(test_reset_keeps_the_state_but_drops_the_evidence);
  RUN_TEST(test_ground_frame_layout);
  RUN_TEST(test_ground_frame_without_online_tracking);
  return UNITY_END();
}
