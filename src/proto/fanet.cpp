#include "fanet.h"

#include <cmath>
#include <cstring>

namespace proto::fanet {
namespace {

// Coordinates are transmitted as scaled 24 bit integers.
constexpr double kLatitudeScale  = 93206.0;
constexpr double kLongitudeScale = 46603.0;

// Several fields share one encoding: seven value bits plus a scale bit that
// multiplies by five when the value would not otherwise fit.
uint8_t compress(int32_t value, bool allowNegative) {
  const int32_t limit = allowNegative ? 63 : 127;
  const int32_t floor = allowNegative ? -63 : 0;

  bool scaled = false;
  if (value > limit || value < floor) { value /= 5; scaled = true; }
  if (value > limit) value = limit;
  if (value < floor) value = floor;

  const uint8_t bits = uint8_t(uint32_t(value) & 0x7Fu);
  return scaled ? uint8_t(bits | 0x80u) : bits;
}

std::size_t writeHeader(const Identity& id, Type type, uint8_t* out) {
  out[0] = uint8_t(uint8_t(type) & 0x3Fu);   // no forwarding, no extended header
  out[1] = id.vendor;
  out[2] = uint8_t(id.device);
  out[3] = uint8_t(id.device >> 8);
  return kHeaderBytes;
}

}  // namespace

// ---------------------------------------------------------------------------
std::size_t buildTracking(const Identity& id, const State& s,
                          uint8_t out[kTrackingBytes]) {
  std::memset(out, 0, kTrackingBytes);
  std::size_t n = writeHeader(id, Type::Tracking, out);

  const int32_t lat = int32_t(std::lround(s.latitude * kLatitudeScale));
  const int32_t lon = int32_t(std::lround(s.longitude * kLongitudeScale));

  out[n++] = uint8_t(lat);
  out[n++] = uint8_t(lat >> 8);
  out[n++] = uint8_t(lat >> 16);
  out[n++] = uint8_t(lon);
  out[n++] = uint8_t(lon >> 8);
  out[n++] = uint8_t(lon >> 16);

  // Altitude is metres up to 2047, in four metre steps above that.
  int32_t metres = int32_t(std::lround(s.altitude));
  if (metres < 0) metres = 0;

  uint16_t altitude;
  if (metres > 2047) {
    int32_t coarse = metres / 4;
    if (coarse > 2047) coarse = 2047;
    altitude = uint16_t(uint16_t(coarse) | (1u << 11));
  } else {
    altitude = uint16_t(metres);
  }
  altitude = uint16_t(altitude | uint16_t((uint8_t(s.aircraft) & 0x07u) << 12));
  if (s.onlineTracking) altitude = uint16_t(altitude | (1u << 15));

  out[n++] = uint8_t(altitude);
  out[n++] = uint8_t(altitude >> 8);

  out[n++] = compress(int32_t(std::lround(s.groundSpeed * 2.0f)), false);
  out[n++] = compress(int32_t(std::lround(s.climbRate * 10.0f)), true);

  float heading = std::fmod(s.track, 360.0f);
  if (heading < 0.0f) heading += 360.0f;
  out[n++] = uint8_t(std::lround(heading * 256.0f / 360.0f)) ;

  return n;
}

// ---------------------------------------------------------------------------
std::size_t buildGround(const Identity& id, double latitude, double longitude,
                        Ground status, bool onlineTracking,
                        uint8_t out[kGroundBytes]) {
  std::memset(out, 0, kGroundBytes);
  std::size_t n = writeHeader(id, Type::GroundTracking, out);

  const int32_t lat = int32_t(std::lround(latitude * kLatitudeScale));
  const int32_t lon = int32_t(std::lround(longitude * kLongitudeScale));

  out[n++] = uint8_t(lat);
  out[n++] = uint8_t(lat >> 8);
  out[n++] = uint8_t(lat >> 16);
  out[n++] = uint8_t(lon);
  out[n++] = uint8_t(lon >> 8);
  out[n++] = uint8_t(lon >> 16);

  // Status in the upper nibble, online tracking in the lowest bit; the three
  // bits between them are reserved.
  out[n++] = uint8_t(uint8_t(uint8_t(status) & 0x0Fu) << 4) |
             uint8_t(onlineTracking ? 0x01u : 0x00u);
  return n;
}

// ---------------------------------------------------------------------------
std::size_t buildName(const Identity& id, const char* name,
                      uint8_t out[kNameMaxBytes]) {
  std::size_t n = writeHeader(id, Type::Name, out);
  if (!name) return n;

  const std::size_t room = kNameMaxBytes - kHeaderBytes;
  std::size_t len = 0;
  while (len < room && name[len] != '\0') ++len;

  std::memcpy(out + n, name, len);
  return n + len;
}


// ===========================================================================
//  Receive side
// ===========================================================================
namespace {

constexpr double kLatDecode = 1.0 / 93206.0;
constexpr double kLonDecode = 1.0 / 46603.0;

// Sign extends a 24 bit little endian coordinate.
int32_t readCoordinate(const uint8_t* p) {
  int32_t v = int32_t(uint32_t(p[0]) | (uint32_t(p[1]) << 8) |
                      (uint32_t(p[2]) << 16));
  if (v & 0x800000) v -= 0x1000000;
  return v;
}

// The seven bit value plus scale bit shared by the speed style fields.
float expandScaled(uint8_t raw, float unit) {
  const float value = float(raw & 0x7Fu) * unit;
  return (raw & 0x80u) ? value * 5.0f : value;
}

// FANET categories differ from the ones the NMEA output expects.
AircraftType toNmeaAircraft(uint8_t fanetType) {
  switch (fanetType & 0x07u) {
    case 1:  return AircraftType::Paraglider;
    case 2:  return AircraftType::HangGlider;
    case 3:  return AircraftType::Balloon;
    case 4:  return AircraftType::Glider;
    case 5:  return AircraftType::Powered;
    case 6:  return AircraftType::Helicopter;
    case 7:  return AircraftType::Uav;
    default: return AircraftType::Unknown;
  }
}

void fillIdentity(Traffic& t, uint8_t vendor, uint16_t device) {
  t.source      = Source::Fanet;
  t.address     = (uint32_t(vendor) << 16) | device;
  t.addressType = 0;                 // not an ICAO or FLARM assigned address
}

}  // namespace

bool decodeFrame(const uint8_t* data, std::size_t len, Report& out) {
  out = Report{};
  if (len < kHeaderBytes) return false;

  const uint8_t type     = uint8_t(data[0] & 0x3Fu);
  const bool    extended = (data[0] & 0x80u) != 0;
  const uint8_t vendor   = data[1];
  const uint16_t device  = uint16_t(uint16_t(data[2]) | (uint16_t(data[3]) << 8));

  std::size_t offset = kHeaderBytes;
  if (extended) ++offset;            // optional extended header byte
  if (offset > len) return false;

  const uint8_t*    body      = data + offset;
  const std::size_t bodyBytes = len - offset;

  switch (type) {
    case 1: {                                   // tracking
      if (bodyBytes < 11) return false;
      Traffic& t = out.traffic;
      fillIdentity(t, vendor, device);

      t.latitude    = double(readCoordinate(body)) * kLatDecode;
      t.longitude   = double(readCoordinate(body + 3)) * kLonDecode;
      t.hasPosition = true;

      const uint16_t altField = uint16_t(uint16_t(body[6]) | (uint16_t(body[7]) << 8));
      const uint16_t metres   = uint16_t(altField & 0x7FFu);
      t.altitude = float((altField & (1u << 11)) ? metres * 4u : metres);
      t.aircraft = toNmeaAircraft(uint8_t((altField >> 12) & 0x07u));

      // FANET reports km/h, the report structure and NMEA use m/s.
      t.groundSpeed = expandScaled(body[8], 0.5f) / 3.6f;

      // Climb is a seven bit two's complement value in 0.1 m/s, with the top
      // bit selecting a five times scale. Shifting left and back sign extends
      // the seven bit field without hand written branches.
      const int8_t climbRaw = int8_t(int8_t(body[9] << 1) / 2);
      t.climbRate = float(climbRaw) * 0.1f;
      if (body[9] & 0x80u) t.climbRate *= 5.0f;

      t.track       = float(body[10]) * 360.0f / 256.0f;
      t.hasVelocity = true;

      out.kind = ReportKind::Traffic;
      return true;
    }

    case 2: {                                   // name
      Name& n = out.name;
      n.address = (uint32_t(vendor) << 16) | device;
      std::size_t count = bodyBytes;
      if (count > sizeof(n.text) - 1) count = sizeof(n.text) - 1;
      std::memcpy(n.text, body, count);
      n.text[count] = '\0';
      out.kind = ReportKind::Name;
      return true;
    }

    case 4: {                                   // service, used by weather stations
      if (bodyBytes < 1) return false;
      const uint8_t flags = body[0];

      std::size_t at = 1;
      if (flags & 0x01u) ++at;                  // service extended header

      Weather& w = out.weather;
      w.address         = (uint32_t(vendor) << 16) | device;
      w.internetGateway = (flags & 0x80u) != 0;

      if (bodyBytes >= at + 6) {
        w.latitude    = double(readCoordinate(body + at)) * kLatDecode;
        w.longitude   = double(readCoordinate(body + at + 3)) * kLonDecode;
        w.hasPosition = true;
        at += 6;
      }

      // Optional values follow in descending flag order.
      if ((flags & 0x40u) && bodyBytes >= at + 1) {
        w.temperature    = float(int8_t(body[at++])) / 2.0f;
        w.hasTemperature = true;
      }
      if ((flags & 0x20u) && bodyBytes >= at + 3) {
        w.windDir   = float(body[at]) * 360.0f / 256.0f;
        w.windSpeed = expandScaled(body[at + 1], 0.2f);
        w.windGust  = expandScaled(body[at + 2], 0.2f);
        w.hasWind   = true;
        at += 3;
      }
      if ((flags & 0x10u) && bodyBytes >= at + 1) {
        w.humidity    = float(body[at++]) * 0.4f;
        w.hasHumidity = true;
      }
      if ((flags & 0x08u) && bodyBytes >= at + 2) {
        const uint16_t raw = uint16_t(uint16_t(body[at]) | (uint16_t(body[at + 1]) << 8));
        w.pressure    = 430.0f + float(raw) / 10.0f;
        w.hasPressure = true;
        at += 2;
      }
      if ((flags & 0x02u) && bodyBytes >= at + 1) {
        w.battery    = uint8_t((body[at++] & 0x0Fu) * 100u / 15u);
        w.hasBattery = true;
      }

      out.kind = ReportKind::Weather;
      return true;
    }

    case 7: {                                   // ground tracking
      if (bodyBytes < 7) return false;
      Traffic& t = out.traffic;
      fillIdentity(t, vendor, device);
      t.latitude    = double(readCoordinate(body)) * kLatDecode;
      t.longitude   = double(readCoordinate(body + 3)) * kLonDecode;
      t.hasPosition = true;
      // Ground tracking carries walkers, vehicles and rescue states rather
      // than an aircraft category, so nothing sensible maps onto the NMEA
      // enumeration.
      t.aircraft    = AircraftType::Unknown;
      t.hasVelocity = false;
      out.kind      = ReportKind::Traffic;
      return true;
    }

    default:
      return false;
  }
}

}  // namespace proto::fanet
