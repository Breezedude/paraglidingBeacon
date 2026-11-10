#include "rx_cache.h"

namespace rx {

Entry& Cache::obtain(uint32_t nowMs, uint32_t address) {
  Entry* free = nullptr;
  Entry* oldest = nullptr;

  for (auto& e : entries_) {
    if (e.used && e.address == address) {
      e.lastSeen = nowMs;
      return e;
    }
    if (!e.used) {
      if (!free) free = &e;
      continue;
    }
    if (!oldest || int32_t(e.lastSeen - oldest->lastSeen) < 0) oldest = &e;
  }

  // A fresh entry starts empty rather than inheriting whatever the recycled
  // slot happened to hold.
  Entry& e = free ? *free : *oldest;
  e = Entry{};
  e.address  = address;
  e.lastSeen = nowMs;
  e.used     = true;
  return e;
}

void Cache::note(uint32_t nowMs, const proto::Traffic& t) {
  Entry& e = obtain(nowMs, t.address);
  e.traffic    = t;
  e.hasTraffic = true;
}

void Cache::note(uint32_t nowMs, const proto::Weather& w) {
  Entry& e = obtain(nowMs, w.address);
  e.weather    = w;
  e.hasWeather = true;
}

void Cache::note(uint32_t nowMs, const proto::Name& n) {
  Entry& e = obtain(nowMs, n.address);
  e.name    = n;
  e.hasName = true;
}

void Cache::expire(uint32_t nowMs) {
  for (auto& e : entries_)
    if (e.used && nowMs - e.lastSeen > kEntryTimeoutMs) e = Entry{};
}

void Cache::clear() {
  for (auto& e : entries_) e = Entry{};
}

unsigned Cache::size() const {
  unsigned n = 0;
  for (const auto& e : entries_) if (e.used) ++n;
  return n;
}

}  // namespace rx
