#pragma once
// ===========================================================================
//  Sharing the band
// ---------------------------------------------------------------------------
//  All three protocols transmit between 868.0 and 868.6 MHz, band M of
//  ETSI EN 300 220-2 (V3.3.1, Table 4): 25 mW e.r.p., and either
//
//    at most 1 % duty cycle, measured over an hour across the whole band
//    (clause 4.4.3), or
//
//    polite spectrum access (clause 4.6): listen before every transmission
//    and hold back while the channel is in use. Then the limits of Table 18
//    apply instead: at most 100 s on air per hour for each 200 kHz of
//    spectrum, and at least 100 ms between two transmissions on the same
//    frequency.
//
//  This module keeps the account for either rule. Listening itself needs the
//  radio and lives in radio.cpp. Plain arithmetic, no hardware, so it can be
//  tested on a host.
// ===========================================================================

#include <cstddef>
#include <cstdint>

namespace airtime {

// ---- time on air -----------------------------------------------------------
// LoRa with an explicit header and a payload CRC, low data rate optimisation
// off. The formula from the transceiver's data sheet.
uint32_t loraUs(size_t payloadBytes, uint8_t spreadingFactor, uint32_t bandwidthHz,
                uint8_t codingRateDenominator, uint16_t preambleSymbols);

// FSK: preamble, sync word and the bytes as they go on air.
uint32_t fskUs(size_t airBytes, uint16_t preambleBits, uint8_t syncBytes,
               uint32_t bitsPerSecond);

// ---- the account -----------------------------------------------------------
// A bucket of microseconds that fills at a steady rate and is emptied by every
// transmission. Whatever the pattern of transmissions, no hour can hold more
// than the capacity plus an hour's worth of refill, so both are chosen to stay
// below the limit with a little to spare.
struct Rule {
  uint32_t refillUsPerSecond;
  uint32_t capacityUs;
};

// 1 % of an hour is 36 s. 0.9 % refill and 3 s capacity: at most 35.4 s.
constexpr Rule kDutyCycle{9000, 3000000};

// 100 s per hour. Counted once for both channels rather than per 200 kHz,
// which is stricter than required. 2.5 % refill and 9 s capacity: at most 99 s.
constexpr Rule kPolite{25000, 9000000};

// With polite spectrum access, the shortest pause on one frequency.
constexpr uint32_t kMinOffMs = 100;

constexpr uint32_t kHourMs = 3600000;

class Budget {
 public:
  // A change of rule applies from now on. The level is clipped to the new
  // capacity; the hour before the change is not reconciled.
  void setRule(const Rule& rule, uint32_t nowMs);
  const Rule& rule() const { return rule_; }

  // Whether a transmission of `costUs` fits now, leaving at least `reserveUs`
  // for others. The reserve is how one protocol gives way to another.
  bool affordable(uint32_t nowMs, uint32_t costUs, uint32_t reserveUs = 0);

  // Books a transmission. The level may go below zero when the actual time on
  // air was longer than the estimate it was admitted with; it is then simply
  // longer before the next one fits.
  void charge(uint32_t nowMs, uint32_t costUs);

  int64_t levelUs(uint32_t nowMs);

 private:
  void refill(uint32_t nowMs);

  Rule     rule_    = kDutyCycle;
  int64_t  levelUs_ = 0;
  uint32_t lastMs_  = 0;
  bool     started_ = false;   // the bucket starts full at the first use
};

// The pause per frequency. Two channels: 868.2 and 868.4 MHz.
class OffTime {
 public:
  static constexpr unsigned kChannels = 2;

  // Milliseconds to wait before `channel` may be used again, 0 if it may now.
  uint32_t waitMs(uint32_t nowMs, unsigned channel) const;
  void transmitted(uint32_t endMs, unsigned channel);

 private:
  uint32_t lastEndMs_[kChannels] = {};
  bool     used_[kChannels]      = {};
};

}  // namespace airtime
