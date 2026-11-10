#pragma once
// ===========================================================================
//  What has been heard lately.
//
//  Decoded reports are otherwise handed straight to whoever is connected and
//  forgotten, so a client that connects a minute later sees an empty list
//  until the next frame happens to arrive. This keeps the last few minutes so
//  a session can start with what is actually in the air.
//
//  One entry per address, because a station sends its position, its name and,
//  for a weather station, its measurements in separate frames that belong
//  together. Plain data, no hardware, so it can be tested on a host.
// ===========================================================================

#include <cstdint>

#include "proto/report.h"

namespace rx {

// Enough for a busy site. Beyond this the oldest entry is dropped, which is
// the one least likely to still be in the air.
inline constexpr unsigned kMaxEntries = 24;

// How long a station stays in the cache without being heard from again.
inline constexpr uint32_t kEntryTimeoutMs = 300000;   // five minutes

struct Entry {
  uint32_t     address  = 0;
  uint32_t     lastSeen = 0;
  bool         used     = false;

  bool           hasTraffic = false;
  proto::Traffic traffic;

  bool           hasWeather = false;
  proto::Weather weather;

  bool           hasName = false;
  proto::Name    name;
};

class Cache {
 public:
  // Each of these merges into the entry for that address, leaving the other
  // kinds of information about it untouched.
  void note(uint32_t nowMs, const proto::Traffic& t);
  void note(uint32_t nowMs, const proto::Weather& w);
  void note(uint32_t nowMs, const proto::Name& n);

  // Drops what has gone quiet. Called before handing the cache out.
  void expire(uint32_t nowMs);

  void clear();

  unsigned      size() const;
  const Entry&  at(unsigned index) const { return entries_[index]; }
  unsigned      capacity() const { return kMaxEntries; }

 private:
  Entry entries_[kMaxEntries];

  // Returns the entry for an address, creating or recycling one if needed.
  Entry& obtain(uint32_t nowMs, uint32_t address);
};

}  // namespace rx
