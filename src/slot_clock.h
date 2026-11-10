#pragma once
// ===========================================================================
//  Where in the UTC second are we?
//
//  FLARM and ADS-L do not transmit whenever they like. Each second is divided
//  into two slots, 400 to 800 ms and 800 to 1200 ms after the second starts,
//  and in Europe each slot belongs to one of two frequencies:
//
//                 slot 0 (400..800)   slot 1 (800..1200)
//      FLARM         868.2 MHz           868.4 MHz
//      ADS-L, OGN    868.4 MHz           868.2 MHz
//
//  Receivers hop between the two in step with this, so a frame sent at the
//  right frequency but in the wrong slot is simply not heard. A ground station
//  listening on one frequency all the time does not show the problem, which is
//  how it goes unnoticed.
//
//  This board has no pulse-per-second line from the GNSS receiver, so the
//  start of the second is estimated from when its report arrives. Plain
//  arithmetic, no hardware, so it can be tested on a host.
// ===========================================================================

#include <cstdint>

namespace timing {

// How long after the start of a second the position sentence of the receiver's
// report has arrived in full. Not a measured figure for this receiver: it is
// the value other firmware for the same chip assumes, and the transmit windows
// below are placed so that being wrong by a good hundred milliseconds either
// way still lands inside the slot.
constexpr uint32_t kSentenceLatencyMs = 90;

// A report can only ever be seen late, never early: the loop may have been
// busy when it came in. The earliest of the last few reports is therefore the
// best estimate. Keeping only a few of them means a genuine shift, such as the
// receiver aligning its output to UTC once it has a fix, is followed within
// seconds rather than held against forever.
constexpr unsigned kReportHistory = 5;

// Without a report for this long the estimate is no longer trusted.
constexpr uint32_t kSyncTimeoutMs = 3000;

class SecondClock {
 public:
  // Call when the position sentence of a report has been received in full.
  void onReport(uint32_t nowMs);

  bool synced(uint32_t nowMs) const;

  // Milliseconds since the start of the current UTC second, 0..999.
  uint32_t msIntoSecond(uint32_t nowMs) const;

  // Local time at which the current UTC second started.
  uint32_t secondStart(uint32_t nowMs) const { return nowMs - msIntoSecond(nowMs); }

  void reset() { count_ = 0; }

 private:
  uint32_t history_[kReportHistory] = {};   // estimated second starts
  unsigned count_  = 0;                     // how many of them are filled
  unsigned next_   = 0;
  uint32_t refMs_  = 0;                     // local time of a second boundary
  uint32_t lastMs_ = 0;                     // when the last report came in
};

// A stretch of the second in which one transmission is allowed to start.
// A moment between `begin` and `end` is picked at random for each second, so
// that devices whose clocks agree do not transmit on top of each other. If the
// radio was busy at that moment the frame may still go out up to `deadline`;
// after that the second is skipped rather than sent in the wrong slot.
struct Window {
  uint16_t begin;
  uint16_t end;
  uint16_t deadline;
};

// Nominal times. With the receiver's latency unknown to within roughly
// -60..+120 ms, the actual emissions fall between 410 and 790 ms for the two
// windows in slot 0 and between 810 and 1115 ms for the one in slot 1.
constexpr Window kFlarmWindow     {470, 540, 600};   // slot 0, 868.2 MHz
constexpr Window kAdslUpperWindow {600, 660, 670};   // slot 0, 868.4 MHz
constexpr Window kAdslLowerWindow {870, 980, 995};   // slot 1, 868.2 MHz

// LoRa frames are not slotted, but they occupy the radio for some tens of
// milliseconds, so they are kept clear of the windows above.
constexpr Window kFanetWindow     { 20, 300, 380};

// Decides, once per UTC second, when a transmission is due.
class SlotTimer {
 public:
  // True exactly once per second, at the moment picked for it. `entropy` is
  // any random number; it is only consumed when a new second begins.
  bool due(const SecondClock& clock, const Window& window, uint32_t nowMs,
           uint32_t entropy);

  // For a frame that was due but could not go out, because the channel was in
  // use or the frequency still had to rest: picks a new moment `delayMs` from
  // now, still in this second. Returns false, and leaves the second skipped,
  // when that moment would be past the deadline.
  bool defer(const SecondClock& clock, const Window& window, uint32_t nowMs,
             uint32_t delayMs);

 private:
  uint32_t second_ = 0;       // start of the second the state belongs to
  uint16_t offset_ = 0;       // chosen moment within it
  bool     armed_  = false;   // a moment was chosen and not yet used
  bool     seen_   = false;
};

}  // namespace timing
