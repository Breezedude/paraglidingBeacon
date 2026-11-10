#include "airtime.h"

namespace airtime {

uint32_t loraUs(size_t payloadBytes, uint8_t spreadingFactor, uint32_t bandwidthHz,
                uint8_t codingRateDenominator, uint16_t preambleSymbols) {
  // Everything in quarter symbols, so the 4.25 symbols the chip adds to the
  // preamble stay an integer.
  const int32_t sf = spreadingFactor;
  const int32_t cr = int32_t(codingRateDenominator) - 4;            // 4/5 -> 1
  const int32_t numerator = 8 * int32_t(payloadBytes) - 4 * sf + 28 + 16;
  const int32_t blocks    = numerator > 0 ? (numerator + 4 * sf - 1) / (4 * sf) : 0;
  const int64_t quarterSymbols =
      4 * (int64_t(preambleSymbols) + 8) + 17 + 4 * int64_t(blocks) * (cr + 4);

  const uint64_t symbolNs = (uint64_t(1) << sf) * 1000000000ULL / bandwidthHz;
  return uint32_t((uint64_t(quarterSymbols) * symbolNs / 4 + 999) / 1000);
}

uint32_t fskUs(size_t airBytes, uint16_t preambleBits, uint8_t syncBytes,
               uint32_t bitsPerSecond) {
  const uint64_t bits = uint64_t(preambleBits) + 8ULL * syncBytes + 8ULL * airBytes;
  return uint32_t((bits * 1000000ULL + bitsPerSecond - 1) / bitsPerSecond);
}

// ---------------------------------------------------------------------------
void Budget::refill(uint32_t nowMs) {
  if (!started_) {
    started_ = true;
    lastMs_  = nowMs;
    levelUs_ = rule_.capacityUs;
    return;
  }
  const uint32_t elapsed = nowMs - lastMs_;
  lastMs_ = nowMs;

  // Rounded down, which only ever errs on the safe side.
  levelUs_ += int64_t(uint64_t(elapsed) * rule_.refillUsPerSecond / 1000U);
  if (levelUs_ > int64_t(rule_.capacityUs)) levelUs_ = rule_.capacityUs;
}

void Budget::setRule(const Rule& rule, uint32_t nowMs) {
  refill(nowMs);
  rule_ = rule;
  if (levelUs_ > int64_t(rule_.capacityUs)) levelUs_ = rule_.capacityUs;
}

bool Budget::affordable(uint32_t nowMs, uint32_t costUs, uint32_t reserveUs) {
  refill(nowMs);
  return levelUs_ >= int64_t(costUs) + int64_t(reserveUs);
}

void Budget::charge(uint32_t nowMs, uint32_t costUs) {
  refill(nowMs);
  levelUs_ -= costUs;
}

int64_t Budget::levelUs(uint32_t nowMs) {
  refill(nowMs);
  return levelUs_;
}

// ---------------------------------------------------------------------------
uint32_t OffTime::waitMs(uint32_t nowMs, unsigned channel) const {
  if (channel >= kChannels || !used_[channel]) return 0;
  const uint32_t since = nowMs - lastEndMs_[channel];
  return since >= kMinOffMs ? 0 : kMinOffMs - since;
}

void OffTime::transmitted(uint32_t endMs, unsigned channel) {
  if (channel >= kChannels) return;
  lastEndMs_[channel] = endMs;
  used_[channel]      = true;
}

}  // namespace airtime
