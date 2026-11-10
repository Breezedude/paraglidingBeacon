#pragma once
// ===========================================================================
//  NMEA output for received traffic
// ---------------------------------------------------------------------------
//  Navigation applications understand the FLARM data port sentences, so every
//  protocol is reported through those regardless of how it arrived:
//
//    $PFLAU  heartbeat and status, once per second
//    $PFLAA  one sentence per aircraft currently known
//
//  Field layout follows the FLARM Data Port Interface Control Document
//  (FTD-012). Positions are relative to our own, so a fix is required before
//  any traffic can be reported.
//
//  FANET weather stations have no equivalent standard sentence, so they use a
//  proprietary one documented in DEVELOPMENT.md.
// ===========================================================================

#include <cstddef>
#include <cstdint>

#include "report.h"

namespace proto::nmea {

// Our own position, needed to express traffic relative to it.
struct Own {
  double latitude  = 0.0;
  double longitude = 0.0;
  float  altitude  = 0.0f;   // metres
  bool   valid     = false;
  bool   airborne  = false;
};

// All builders append a checksum and CRLF and return the number of characters
// written, or 0 if the sentence could not be produced.

// <RX> is the number of aircraft currently tracked, <TX> whether we transmit.
std::size_t buildStatus(char* out, std::size_t cap, unsigned tracked,
                        bool transmitting, const Own& own);

// One aircraft. Returns 0 when the target or our own position is unknown,
// because the sentence is defined in relative terms.
std::size_t buildTraffic(char* out, std::size_t cap, const Traffic& t,
                         const Own& own);

// FANET weather station, proprietary sentence.
std::size_t buildWeather(char* out, std::size_t cap, const Weather& w);

// Appends "*hh\r\n" to a sentence that already starts with '$'.
std::size_t finish(char* out, std::size_t cap, std::size_t len);

}  // namespace proto::nmea
