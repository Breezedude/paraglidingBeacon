// Host tests for the UTC second estimate and the transmit slot timing.

#include <unity.h>

#include "slot_clock.h"

using timing::SecondClock;
using timing::SlotTimer;
using timing::Window;

namespace {

// Local time at which UTC seconds start in these tests, and the moment the
// report of second n is seen when nothing delays it.
constexpr uint32_t kOrigin = 100000;

uint32_t reportAt(uint32_t second, uint32_t delay = 0) {
  return kOrigin + second * 1000 + timing::kSentenceLatencyMs + delay;
}

}  // namespace

// ---------------------------------------------------------------------------
void test_unsynced_without_a_report() {
  SecondClock c;
  TEST_ASSERT_FALSE(c.synced(5000));
}

void test_phase_follows_the_report() {
  SecondClock c;
  c.onReport(reportAt(0));
  TEST_ASSERT_TRUE(c.synced(reportAt(0)));
  TEST_ASSERT_EQUAL_UINT32(0,   c.msIntoSecond(kOrigin));
  TEST_ASSERT_EQUAL_UINT32(500, c.msIntoSecond(kOrigin + 500));
  TEST_ASSERT_EQUAL_UINT32(250, c.msIntoSecond(kOrigin + 3250));
  TEST_ASSERT_EQUAL_UINT32(kOrigin + 3000, c.secondStart(kOrigin + 3250));
}

void test_a_late_report_does_not_move_the_phase() {
  SecondClock c;
  c.onReport(reportAt(0));
  c.onReport(reportAt(1));
  // The loop was busy: this one is seen 300 ms after it arrived.
  c.onReport(reportAt(2, 300));
  TEST_ASSERT_EQUAL_UINT32(400, c.msIntoSecond(kOrigin + 2400));
}

void test_an_earlier_report_is_taken_at_once() {
  SecondClock c;
  // The first reports were all seen late, so the estimate starts out wrong.
  c.onReport(reportAt(0, 200));
  c.onReport(reportAt(1, 200));
  TEST_ASSERT_EQUAL_UINT32(200, c.msIntoSecond(kOrigin + 1400));
  // One undelayed report is enough to correct it.
  c.onReport(reportAt(2));
  TEST_ASSERT_EQUAL_UINT32(400, c.msIntoSecond(kOrigin + 2400));
}

void test_a_lasting_shift_later_is_followed() {
  SecondClock c;
  for (uint32_t s = 0; s < 5; ++s) c.onReport(reportAt(s));
  // The receiver moves its output by 150 ms and keeps it there, as happens
  // when it aligns to UTC after the first fix.
  for (uint32_t s = 5; s < 5 + timing::kReportHistory; ++s) c.onReport(reportAt(s, 150));
  const uint32_t now = kOrigin + (5 + timing::kReportHistory) * 1000 + 500;
  TEST_ASSERT_EQUAL_UINT32(350, c.msIntoSecond(now));
}

void test_sync_expires() {
  SecondClock c;
  c.onReport(reportAt(0));
  TEST_ASSERT_TRUE(c.synced(reportAt(0) + timing::kSyncTimeoutMs - 1));
  TEST_ASSERT_FALSE(c.synced(reportAt(0) + timing::kSyncTimeoutMs));
}

void test_history_is_dropped_after_a_gap() {
  SecondClock c;
  c.onReport(reportAt(0));
  // Ten seconds of silence, then reports at a completely different phase.
  c.onReport(reportAt(10, 400));
  TEST_ASSERT_EQUAL_UINT32(100, c.msIntoSecond(kOrigin + 10500));
}

void test_survives_the_millisecond_counter_wrapping() {
  SecondClock c;
  const uint32_t nearEnd = 0xFFFFFFFFu - 1500;
  c.onReport(nearEnd + timing::kSentenceLatencyMs);
  c.onReport(nearEnd + 1000 + timing::kSentenceLatencyMs);
  c.onReport(nearEnd + 2000 + timing::kSentenceLatencyMs);     // wrapped
  TEST_ASSERT_EQUAL_UINT32(300, c.msIntoSecond(nearEnd + 2300));
}

// ---------------------------------------------------------------------------
void test_slot_fires_once_per_second_inside_the_window() {
  SecondClock c;
  SlotTimer t;
  const Window w{470, 540, 600};

  for (uint32_t s = 0; s < 20; ++s) {
    c.onReport(reportAt(s));
    unsigned fired = 0;
    uint32_t at = 0;
    for (uint32_t ms = 100; ms < 1000; ++ms) {
      if (t.due(c, w, kOrigin + s * 1000 + ms, s * 7919u)) { ++fired; at = ms; }
    }
    TEST_ASSERT_EQUAL_UINT32(1, fired);
    TEST_ASSERT_TRUE(at >= w.begin && at < w.end);
  }
}

void test_slot_moment_varies_between_seconds() {
  SecondClock c;
  SlotTimer t;
  const Window w{470, 540, 600};
  uint32_t first = 0;
  bool differs = false;

  for (uint32_t s = 0; s < 10; ++s) {
    c.onReport(reportAt(s));
    for (uint32_t ms = 100; ms < 1000; ++ms) {
      if (t.due(c, w, kOrigin + s * 1000 + ms, s * 7919u + 13)) {
        if (s == 0) first = ms; else if (ms != first) differs = true;
      }
    }
  }
  TEST_ASSERT_TRUE(differs);
}

void test_slot_is_skipped_when_the_radio_was_busy_too_long() {
  SecondClock c;
  SlotTimer t;
  const Window w{470, 540, 600};
  c.onReport(reportAt(0));

  // First asked after the deadline: nothing goes out in this second at all.
  bool fired = false;
  for (uint32_t ms = 650; ms < 1000; ++ms) fired |= t.due(c, w, kOrigin + ms, 5);
  TEST_ASSERT_FALSE(fired);

  // The next second is unaffected.
  c.onReport(reportAt(1));
  fired = false;
  for (uint32_t ms = 100; ms < 1000; ++ms) fired |= t.due(c, w, kOrigin + 1000 + ms, 5);
  TEST_ASSERT_TRUE(fired);
}

void test_slot_goes_out_late_but_before_the_deadline() {
  SecondClock c;
  SlotTimer t;
  const Window w{470, 540, 600};
  c.onReport(reportAt(0));
  // Busy until 580 ms: still inside the slot, so it is sent rather than lost.
  TEST_ASSERT_TRUE(t.due(c, w, kOrigin + 580, 0));
  TEST_ASSERT_FALSE(t.due(c, w, kOrigin + 590, 0));
}

void test_a_deferred_slot_fires_again_later() {
  SecondClock c;
  SlotTimer t;
  const Window w{470, 540, 600};
  c.onReport(reportAt(0));
  TEST_ASSERT_TRUE(t.due(c, w, kOrigin + 470, 0));
  // The channel was busy: try again 20 ms later, and only then.
  TEST_ASSERT_TRUE(t.defer(c, w, kOrigin + 470, 20));
  TEST_ASSERT_FALSE(t.due(c, w, kOrigin + 480, 0));
  TEST_ASSERT_TRUE(t.due(c, w, kOrigin + 490, 0));
  TEST_ASSERT_FALSE(t.due(c, w, kOrigin + 495, 0));
}

void test_a_deferral_past_the_deadline_skips_the_second() {
  SecondClock c;
  SlotTimer t;
  const Window w{470, 540, 600};
  c.onReport(reportAt(0));
  TEST_ASSERT_TRUE(t.due(c, w, kOrigin + 590, 0));
  TEST_ASSERT_FALSE(t.defer(c, w, kOrigin + 590, 20));
  bool fired = false;
  for (uint32_t ms = 590; ms < 1000; ++ms) fired |= t.due(c, w, kOrigin + ms, 0);
  TEST_ASSERT_FALSE(fired);
  // The next second is unaffected.
  c.onReport(reportAt(1));
  fired = false;
  for (uint32_t ms = 100; ms < 1000; ++ms) fired |= t.due(c, w, kOrigin + 1000 + ms, 5);
  TEST_ASSERT_TRUE(fired);
}

void test_slot_never_fires_without_sync() {
  SecondClock c;
  SlotTimer t;
  const Window w{470, 540, 600};
  bool fired = false;
  for (uint32_t ms = 0; ms < 3000; ++ms) fired |= t.due(c, w, ms, ms);
  TEST_ASSERT_FALSE(fired);
}

void test_slot_does_not_fire_twice_when_the_phase_is_corrected() {
  SecondClock c;
  SlotTimer t;
  const Window w{470, 540, 600};

  c.onReport(reportAt(0, 200));              // estimate 200 ms late
  unsigned fired = 0;
  // By the late estimate this is the window.
  for (uint32_t ms = 600; ms < 800; ++ms) fired += t.due(c, w, kOrigin + ms, 0);
  TEST_ASSERT_EQUAL_UINT32(1, fired);

  // Now the estimate snaps 200 ms earlier. The rest of this second must not
  // produce a second frame.
  c.onReport(kOrigin + 800 + 0);             // seen as early as it can be
  for (uint32_t ms = 800; ms < 1000; ++ms) fired += t.due(c, w, kOrigin + ms, 0);
  TEST_ASSERT_EQUAL_UINT32(1, fired);
}

void test_the_windows_stay_inside_their_slots() {
  // With the receiver's latency off by -60..+120 ms, every emission still has
  // to fall inside its slot, including the few milliseconds it is on the air.
  constexpr int early = -60, late = 120, air = 8;
  using namespace timing;

  TEST_ASSERT_TRUE(kFlarmWindow.begin + early >= 400);
  TEST_ASSERT_TRUE(kFlarmWindow.deadline + late + air <= 800);

  TEST_ASSERT_TRUE(kAdslUpperWindow.begin + early >= 400);
  TEST_ASSERT_TRUE(kAdslUpperWindow.deadline + late + air <= 800);

  TEST_ASSERT_TRUE(kAdslLowerWindow.begin + early >= 800);
  TEST_ASSERT_TRUE(kAdslLowerWindow.deadline + late + air <= 1200);

  // The two frames that share slot 0 must not be due at the same moment.
  TEST_ASSERT_TRUE(kFlarmWindow.end <= kAdslUpperWindow.begin);
  // And LoRa stays clear of all of them.
  TEST_ASSERT_TRUE(kFanetWindow.deadline + 60 <= kFlarmWindow.begin);
}

// ---------------------------------------------------------------------------
extern "C" void setUp() {}
extern "C" void tearDown() {}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_unsynced_without_a_report);
  RUN_TEST(test_phase_follows_the_report);
  RUN_TEST(test_a_late_report_does_not_move_the_phase);
  RUN_TEST(test_an_earlier_report_is_taken_at_once);
  RUN_TEST(test_a_lasting_shift_later_is_followed);
  RUN_TEST(test_sync_expires);
  RUN_TEST(test_history_is_dropped_after_a_gap);
  RUN_TEST(test_survives_the_millisecond_counter_wrapping);
  RUN_TEST(test_slot_fires_once_per_second_inside_the_window);
  RUN_TEST(test_slot_moment_varies_between_seconds);
  RUN_TEST(test_slot_is_skipped_when_the_radio_was_busy_too_long);
  RUN_TEST(test_slot_goes_out_late_but_before_the_deadline);
  RUN_TEST(test_a_deferred_slot_fires_again_later);
  RUN_TEST(test_a_deferral_past_the_deadline_skips_the_second);
  RUN_TEST(test_slot_never_fires_without_sync);
  RUN_TEST(test_slot_does_not_fire_twice_when_the_phase_is_corrected);
  RUN_TEST(test_the_windows_stay_inside_their_slots);
  return UNITY_END();
}
