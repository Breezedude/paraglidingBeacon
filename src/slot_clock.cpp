#include "slot_clock.h"

namespace timing {

// ---------------------------------------------------------------------------
void SecondClock::onReport(uint32_t nowMs) {
  const uint32_t estimate = nowMs - kSentenceLatencyMs;

  // After a gap the old reports say nothing about the present.
  if (count_ && nowMs - lastMs_ > kSyncTimeoutMs) {
    count_ = 0;
    next_  = 0;
  }

  history_[next_] = estimate;
  next_ = (next_ + 1) % kReportHistory;
  if (count_ < kReportHistory) ++count_;
  lastMs_ = nowMs;

  // Each stored report implies a second boundary. Measured against the newest
  // one, pick the earliest: -500..499 ms, wrapping within the second. The
  // stored values are older than the estimate, so the subtraction is done that
  // way round to stay clear of unsigned wrap-around.
  int32_t earliest = 0;
  for (unsigned i = 0; i < count_; ++i) {
    int32_t offset = -int32_t((estimate - history_[i]) % 1000U);
    if (offset < -500) offset += 1000;
    if (offset < earliest) earliest = offset;
  }
  refMs_ = estimate + uint32_t(earliest);
}

bool SecondClock::synced(uint32_t nowMs) const {
  return count_ > 0 && (nowMs - lastMs_) < kSyncTimeoutMs;
}

uint32_t SecondClock::msIntoSecond(uint32_t nowMs) const {
  return (nowMs - refMs_) % 1000U;
}

// ---------------------------------------------------------------------------
bool SlotTimer::due(const SecondClock& clock, const Window& window,
                    uint32_t nowMs, uint32_t entropy) {
  if (!clock.synced(nowMs)) {
    armed_ = false;
    seen_  = false;
    return false;
  }

  const uint32_t start = clock.secondStart(nowMs);

  // The reference moves by a few milliseconds from report to report, so two
  // readings belong to the same second when they are close, not when they are
  // equal. The subtraction wraps in either direction.
  const uint32_t apart = start - second_;
  const bool sameSecond = seen_ && (apart < 500U || apart > 0xFFFFFFFFU - 500U);

  if (!sameSecond) {
    const uint32_t span = uint32_t(window.end) - uint32_t(window.begin);
    second_ = start;
    seen_   = true;
    armed_  = true;
    offset_ = uint16_t(window.begin + (span ? entropy % span : 0U));
  }

  if (!armed_) return false;

  const uint32_t ms = clock.msIntoSecond(nowMs);
  if (ms < offset_) return false;

  // One chance per second. Past the deadline the frame would leave in the
  // wrong slot, where it helps nobody and may collide with someone else.
  armed_ = false;
  return ms < window.deadline;
}

bool SlotTimer::defer(const SecondClock& clock, const Window& window,
                      uint32_t nowMs, uint32_t delayMs) {
  if (!clock.synced(nowMs) || !seen_) return false;
  const uint32_t target = clock.msIntoSecond(nowMs) + delayMs;
  if (target >= window.deadline) return false;
  offset_ = uint16_t(target);
  armed_  = true;
  return true;
}

}  // namespace timing
