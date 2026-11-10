#include "nmea.h"

#include <cmath>
#include <cstdio>

namespace proto::nmea {
namespace {

constexpr double kPi          = 3.14159265358979323846;
constexpr double kMetresPerDeg = 111320.0;   // one degree of latitude

// Aircraft category as a single hexadecimal digit, per the data port document.
char aircraftDigit(AircraftType type) {
  const uint8_t v = uint8_t(type) & 0x0Fu;
  return char(v < 10 ? ('0' + v) : ('A' + (v - 10)));
}

}  // namespace

std::size_t finish(char* out, std::size_t cap, std::size_t len) {
  if (len == 0 || len + 5 >= cap) return 0;

  uint8_t sum = 0;
  for (std::size_t i = 1; i < len; ++i) sum = uint8_t(sum ^ uint8_t(out[i]));

  const int extra = std::snprintf(out + len, cap - len, "*%02X\r\n", sum);
  return (extra > 0) ? len + std::size_t(extra) : 0;
}

// ---------------------------------------------------------------------------
std::size_t buildStatus(char* out, std::size_t cap, unsigned tracked,
                        bool transmitting, const Own& own) {
  // <GPS>: 0 no reception, 1 fix on the ground, 2 fix airborne.
  const int gps = !own.valid ? 0 : (own.airborne ? 2 : 1);
  if (tracked > 99) tracked = 99;

  // No collision computation happens here, so the alarm fields stay empty and
  // the alarm level is zero. Consumers treat that as traffic information only.
  const int len = std::snprintf(out, cap, "$PFLAU,%u,%d,%d,1,0,,0,,",
                                tracked, transmitting ? 1 : 0, gps);
  return (len > 0) ? finish(out, cap, std::size_t(len)) : 0;
}

// ---------------------------------------------------------------------------
std::size_t buildTraffic(char* out, std::size_t cap, const Traffic& t,
                         const Own& own) {
  if (!own.valid || !t.hasPosition) return 0;

  // A local flat earth approximation is accurate well beyond the range of
  // these radios, and it is what the relative fields are defined against.
  const double north = (t.latitude - own.latitude) * kMetresPerDeg;
  const double east  = (t.longitude - own.longitude) * kMetresPerDeg *
                       std::cos(own.latitude * kPi / 180.0);
  const double up    = double(t.altitude) - double(own.altitude);

  char track[8]  = "";
  char speed[8]  = "";
  char climb[10] = "";
  if (t.hasVelocity && !t.stealth) {
    int heading = int(std::lround(t.track)) % 360;
    if (heading < 0) heading += 360;
    std::snprintf(track, sizeof(track), "%d", heading);
    std::snprintf(speed, sizeof(speed), "%d", int(std::lround(t.groundSpeed)));
    std::snprintf(climb, sizeof(climb), "%.1f", double(t.climbRate));
  }

  // <TurnRate> is specified as always empty.
  const int len = std::snprintf(
      out, cap, "$PFLAA,0,%d,%d,%d,%u,%06X,%s,,%s,%s,%c",
      int(std::lround(north)), int(std::lround(east)), int(std::lround(up)),
      unsigned(t.addressType), unsigned(t.address & 0xFFFFFFu),
      track, speed, climb, aircraftDigit(t.aircraft));
  return (len > 0) ? finish(out, cap, std::size_t(len)) : 0;
}

// ---------------------------------------------------------------------------
//  No standard sentence exists for weather stations, so this one is our own.
//  Empty fields mean the station did not report that value.
//    $PBDWX,<id>,<lat>,<lon>,<temp C>,<dir deg>,<wind km/h>,<gust km/h>,
//           <humidity %>,<pressure hPa>,<battery %>
std::size_t buildWeather(char* out, std::size_t cap, const Weather& w) {
  char lat[16] = "", lon[16] = "", temp[10] = "", dir[8] = "";
  char wind[10] = "", gust[10] = "", hum[8] = "", press[10] = "", batt[8] = "";

  if (w.hasPosition) {
    std::snprintf(lat, sizeof(lat), "%.6f", w.latitude);
    std::snprintf(lon, sizeof(lon), "%.6f", w.longitude);
  }
  if (w.hasTemperature) std::snprintf(temp, sizeof(temp), "%.1f", double(w.temperature));
  if (w.hasWind) {
    std::snprintf(dir, sizeof(dir), "%d", int(std::lround(w.windDir)));
    std::snprintf(wind, sizeof(wind), "%.1f", double(w.windSpeed));
    std::snprintf(gust, sizeof(gust), "%.1f", double(w.windGust));
  }
  if (w.hasHumidity) std::snprintf(hum, sizeof(hum), "%.0f", double(w.humidity));
  if (w.hasPressure) std::snprintf(press, sizeof(press), "%.1f", double(w.pressure));
  if (w.hasBattery)  std::snprintf(batt, sizeof(batt), "%u", unsigned(w.battery));

  const int len = std::snprintf(out, cap, "$PBDWX,%06X,%s,%s,%s,%s,%s,%s,%s,%s,%s",
                                unsigned(w.address & 0xFFFFFFu), lat, lon, temp,
                                dir, wind, gust, hum, press, batt);
  return (len > 0) ? finish(out, cap, std::size_t(len)) : 0;
}

}  // namespace proto::nmea
