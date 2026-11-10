// Host tests for the airtime account. The point of the module is a limit that
// holds whatever the pattern of transmissions, so most of these try to break
// it: a sender that takes every chance, over hours, and the worst moment to
// look at the result.

#include <unity.h>

#include <cstdint>
#include <deque>

#include "airtime.h"

using namespace airtime;

namespace {

// The most time on air any one-hour window holds, for transmissions given as
// (start, duration) in milliseconds.
uint64_t worstHourUs(const std::deque<std::pair<uint64_t, uint32_t>>& sent) {
  uint64_t worst = 0, inWindow = 0;
  size_t   first = 0;
  for (size_t i = 0; i < sent.size(); ++i) {
    inWindow += sent[i].second;
    while (sent[i].first - sent[first].first >= kHourMs) {
      inWindow -= sent[first].second;
      ++first;
    }
    if (inWindow > worst) worst = inWindow;
  }
  return worst;
}

// Sends a frame of `costUs` whenever the budget allows, checking every
// `stepMs`, for `hours`. Returns the worst hour.
uint64_t greedy(const Rule& rule, uint32_t costUs, uint32_t stepMs, unsigned hours,
                uint32_t startMs = 0) {
  Budget budget;
  budget.setRule(rule, startMs);
  std::deque<std::pair<uint64_t, uint32_t>> sent;
  for (uint64_t t = 0; t < uint64_t(hours) * kHourMs; t += stepMs) {
    const uint32_t now = uint32_t(startMs + t);    // wraps like millis()
    if (budget.affordable(now, costUs)) {
      budget.charge(now, costUs);
      sent.emplace_back(t, costUs);
    }
  }
  return worstHourUs(sent);
}

}  // namespace

// ---- time on air -------------------------------------------------------------

void test_lora_time_on_air() {
  // SF7, 250 kHz, 4/5, preamble 12, 15 bytes: 16.25 preamble symbols and 33
  // payload symbols of 0.512 ms each.
  TEST_ASSERT_UINT32_WITHIN(2, 25216, loraUs(15, 7, 250000, 5, 12));
  // Longer payloads cost whole blocks of five symbols.
  TEST_ASSERT_TRUE(loraUs(16, 7, 250000, 5, 12) >= loraUs(15, 7, 250000, 5, 12));
  TEST_ASSERT_UINT32_WITHIN(2, 25216 + 5 * 512, loraUs(18, 7, 250000, 5, 12));
}

void test_fsk_time_on_air() {
  // 8 preamble bits, 8 sync bytes and 52 bytes at 100 kbit/s: 488 bits.
  TEST_ASSERT_EQUAL_UINT32(4880, fskUs(52, 8, 8, 100000));
  TEST_ASSERT_EQUAL_UINT32(4560, fskUs(48, 8, 8, 100000));
}

// ---- the account ---------------------------------------------------------------

void test_starts_full_and_empties() {
  Budget budget;
  budget.setRule(kDutyCycle, 1000);
  TEST_ASSERT_EQUAL_INT64(kDutyCycle.capacityUs, budget.levelUs(1000));
  TEST_ASSERT_TRUE(budget.affordable(1000, kDutyCycle.capacityUs));
  budget.charge(1000, kDutyCycle.capacityUs);
  TEST_ASSERT_FALSE(budget.affordable(1000, 1));
}

void test_refills_at_the_rate() {
  Budget budget;
  budget.setRule(kDutyCycle, 0);
  budget.charge(0, kDutyCycle.capacityUs);
  // One second refills 9 ms.
  TEST_ASSERT_EQUAL_INT64(9000, budget.levelUs(1000));
  TEST_ASSERT_TRUE(budget.affordable(1000, 9000));
  TEST_ASSERT_FALSE(budget.affordable(1000, 9001));
}

void test_never_exceeds_capacity() {
  Budget budget;
  budget.setRule(kPolite, 0);
  TEST_ASSERT_EQUAL_INT64(kPolite.capacityUs, budget.levelUs(10 * kHourMs));
}

void test_reserve_holds_back() {
  Budget budget;
  budget.setRule(kDutyCycle, 0);
  budget.charge(0, kDutyCycle.capacityUs - 30000);       // 30 ms left
  TEST_ASSERT_TRUE(budget.affordable(0, 25000));
  TEST_ASSERT_FALSE(budget.affordable(0, 25000, 10000));
  TEST_ASSERT_TRUE(budget.affordable(0, 20000, 10000));
}

void test_overrun_goes_negative_and_is_paid_back() {
  Budget budget;
  budget.setRule(kDutyCycle, 0);
  budget.charge(0, kDutyCycle.capacityUs + 18000);       // 18 ms over
  TEST_ASSERT_EQUAL_INT64(-18000, budget.levelUs(0));
  TEST_ASSERT_FALSE(budget.affordable(1000, 1));
  TEST_ASSERT_TRUE(budget.affordable(3000, 9000));
}

void test_duty_cycle_holds_for_any_hour() {
  // A FANET frame, tried every 10 ms for three hours.
  TEST_ASSERT_TRUE(greedy(kDutyCycle, 25216, 10, 3) <= 36000000ULL);
  // Short FSK frames, tried every millisecond.
  TEST_ASSERT_TRUE(greedy(kDutyCycle, 4880, 1, 3) <= 36000000ULL);
}

void test_polite_limit_holds_for_any_hour() {
  TEST_ASSERT_TRUE(greedy(kPolite, 25216, 10, 3) <= 100000000ULL);
  TEST_ASSERT_TRUE(greedy(kPolite, 4880, 1, 3) <= 100000000ULL);
}

void test_holds_across_the_millis_wrap() {
  const uint32_t nearWrap = 0xFFFFFFFFu - kHourMs / 2;
  TEST_ASSERT_TRUE(greedy(kDutyCycle, 25216, 10, 2, nearWrap) <= 36000000ULL);
}

void test_the_budget_is_used_not_just_respected() {
  // A limiter that never sends would pass the tests above. Over three hours a
  // greedy sender should get close to the refill rate.
  const uint64_t worst = greedy(kDutyCycle, 25216, 10, 3);
  TEST_ASSERT_TRUE(worst >= 32000000ULL);
}

void test_switching_to_the_stricter_rule_clips() {
  Budget budget;
  budget.setRule(kPolite, 0);
  budget.setRule(kDutyCycle, 0);
  TEST_ASSERT_EQUAL_INT64(kDutyCycle.capacityUs, budget.levelUs(0));
}

// ---- the pause per frequency ---------------------------------------------------

void test_off_time() {
  OffTime off;
  TEST_ASSERT_EQUAL_UINT32(0, off.waitMs(500, 0));
  off.transmitted(1000, 0);
  TEST_ASSERT_EQUAL_UINT32(100, off.waitMs(1000, 0));
  TEST_ASSERT_EQUAL_UINT32(40, off.waitMs(1060, 0));
  TEST_ASSERT_EQUAL_UINT32(0, off.waitMs(1100, 0));
  // The other channel is not affected.
  TEST_ASSERT_EQUAL_UINT32(0, off.waitMs(1000, 1));
  // Out of range is never held back.
  TEST_ASSERT_EQUAL_UINT32(0, off.waitMs(1000, 7));
}

void test_off_time_across_the_wrap() {
  OffTime off;
  off.transmitted(0xFFFFFFF0u, 1);
  TEST_ASSERT_EQUAL_UINT32(100 - 0x20, off.waitMs(0x10, 1));
}

void setUp() {}
void tearDown() {}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_lora_time_on_air);
  RUN_TEST(test_fsk_time_on_air);
  RUN_TEST(test_starts_full_and_empties);
  RUN_TEST(test_refills_at_the_rate);
  RUN_TEST(test_never_exceeds_capacity);
  RUN_TEST(test_reserve_holds_back);
  RUN_TEST(test_overrun_goes_negative_and_is_paid_back);
  RUN_TEST(test_duty_cycle_holds_for_any_hour);
  RUN_TEST(test_polite_limit_holds_for_any_hour);
  RUN_TEST(test_holds_across_the_millis_wrap);
  RUN_TEST(test_the_budget_is_used_not_just_respected);
  RUN_TEST(test_switching_to_the_stricter_rule_clips);
  RUN_TEST(test_off_time);
  RUN_TEST(test_off_time_across_the_wrap);
  return UNITY_END();
}
