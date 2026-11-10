// Host tests for the state of charge curve.

#include <unity.h>

#include "battery.h"

using battery::percentFromMillivolts;

void test_full_and_empty_are_clamped() {
  TEST_ASSERT_EQUAL_UINT8(100, percentFromMillivolts(4300));
  TEST_ASSERT_EQUAL_UINT8(100, percentFromMillivolts(4190));
  TEST_ASSERT_EQUAL_UINT8(0,   percentFromMillivolts(3100));
  TEST_ASSERT_EQUAL_UINT8(0,   percentFromMillivolts(2500));
}

void test_points_on_the_curve() {
  TEST_ASSERT_EQUAL_UINT8(90, percentFromMillivolts(4042));
  TEST_ASSERT_EQUAL_UINT8(50, percentFromMillivolts(3776));
  TEST_ASSERT_EQUAL_UINT8(10, percentFromMillivolts(3644));
}

void test_between_points_is_interpolated() {
  // Half way between the 50 and 60 percent points.
  TEST_ASSERT_EQUAL_UINT8(55, percentFromMillivolts(3798));
}

void test_never_decreases_with_rising_voltage() {
  uint8_t previous = 0;
  for (uint16_t mv = 3000; mv <= 4300; ++mv) {
    const uint8_t now = percentFromMillivolts(mv);
    TEST_ASSERT_TRUE(now >= previous);
    previous = now;
  }
}

void test_the_middle_of_the_curve_is_not_linear() {
  // 3.75 V is 40 percent of the cell, where a straight line from 3.5 to 4.2 V
  // would claim 35 and one from 3.1 V would claim 59.
  TEST_ASSERT_UINT8_WITHIN(2, 41, percentFromMillivolts(3750));
}

extern "C" void setUp() {}
extern "C" void tearDown() {}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_full_and_empty_are_clamped);
  RUN_TEST(test_points_on_the_curve);
  RUN_TEST(test_between_points_is_interpolated);
  RUN_TEST(test_never_decreases_with_rising_voltage);
  RUN_TEST(test_the_middle_of_the_curve_is_not_linear);
  return UNITY_END();
}
