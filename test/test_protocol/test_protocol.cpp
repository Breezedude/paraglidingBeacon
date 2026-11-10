// ===========================================================================
//  Host tests for the protocol layer
// ---------------------------------------------------------------------------
//  These check the properties that are easy to get wrong and impossible to see
//  from the transmitting side: bit ordering, Manchester polarity, the CRC seed
//  and the encoding scales. All expectations come from decoded recordings of
//  real traffic.
//
//  Run with:  pio test -e native
// ===========================================================================

#include <unity.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "proto/bits.h"
#include "proto/fanet.h"
#include "proto/flarm.h"
#include "proto/nmea.h"
#include "proto/report.h"
#include "proto/ads_l.h"

using namespace proto;

// Required by the test framework; nothing to set up or tear down here.
extern "C" void setUp() {}
extern "C" void tearDown() {}

// ---------------------------------------------------------------------------
//  Bit packing
// ---------------------------------------------------------------------------
void test_bitwriter_is_little_endian_lsb_first() {
  uint8_t buf[4] = {};
  BitWriter w(buf, sizeof(buf));
  w.put(0x123456u, 24);          // a 24 bit value fills the first three bytes
  w.put(0x0, 4);                 // format marker
  w.put(0x2, 3);                 // address type

  TEST_ASSERT_EQUAL_HEX8(0x56, buf[0]);
  TEST_ASSERT_EQUAL_HEX8(0x34, buf[1]);
  TEST_ASSERT_EQUAL_HEX8(0x12, buf[2]);
  TEST_ASSERT_EQUAL_HEX8(0x20, buf[3]);   // type in the low nibble, then type 2
}

void test_bitwriter_signed_is_twos_complement() {
  uint8_t buf[2] = {};
  BitWriter w(buf, sizeof(buf));
  w.putSigned(-1, 10);
  TEST_ASSERT_EQUAL_HEX8(0xFF, buf[0]);
  TEST_ASSERT_EQUAL_HEX8(0x03, buf[1]);
}

void test_word_access_is_endianness_independent() {
  const uint8_t bytes[4] = {0x78, 0x56, 0x34, 0x12};
  TEST_ASSERT_EQUAL_HEX32(0x12345678u, loadLE32(bytes));

  uint8_t out[4] = {};
  storeLE32(out, 0x12345678u);
  TEST_ASSERT_EQUAL_MEMORY(bytes, out, 4);
}

// ---------------------------------------------------------------------------
//  Manchester
// ---------------------------------------------------------------------------
//  The convention is 1 -> 01 and 0 -> 10. With the opposite mapping a receiver
//  decodes the bitwise complement, which is exactly how this was found.
void test_manchester_polarity() {
  TEST_ASSERT_EQUAL_HEX8(0xAA, manchesterNibble(0x0));
  TEST_ASSERT_EQUAL_HEX8(0x55, manchesterNibble(0xF));

  const uint8_t in[4] = {0x33, 0x05, 0x53, 0x20};
  uint8_t out[8] = {};
  manchesterEncode(in, sizeof(in), out);

  // Byte pairs taken from a decoded recording of a real frame.
  const uint8_t expect[8] = {0xA5, 0xA5, 0xAA, 0x99, 0x99, 0xA5, 0xA6, 0xAA};
  TEST_ASSERT_EQUAL_MEMORY(expect, out, sizeof(expect));
}

void test_manchester_doubles_length() {
  uint8_t out[2 * (flarm::kPayloadBytes + 2)] = {};
  TEST_ASSERT_EQUAL_UINT32(sizeof(out), flarm::kFrameBytes);
}

// ---------------------------------------------------------------------------
//  CRC
// ---------------------------------------------------------------------------
//  Three payloads recovered from frames a receiver had accepted as valid,
//  together with the checksum it expected. Only one seed satisfies all three.
void test_crc_seed_against_recorded_frames() {
  struct Sample { uint8_t payload[24]; uint16_t crc; };
  const Sample samples[3] = {
    {{0x33,0x05,0x53,0x20,0x5C,0x8F,0x3D,0xC1,0xB6,0x1B,0xFC,0x08,
      0x8E,0xDC,0x78,0xB0,0x50,0xCC,0x04,0xFA,0x2E,0x2B,0x10,0x29}, 0x1A39},
    {{0x33,0x05,0x53,0x22,0x00,0x00,0x00,0x33,0xD4,0x41,0x91,0xFA,
      0x15,0x47,0xCA,0xBA,0x57,0x4B,0x09,0x51,0xC3,0x2D,0xE6,0x9D}, 0x09AB},
    {{0x33,0x05,0x53,0x20,0x95,0x7B,0xF2,0xE7,0xE5,0x57,0x9B,0x4A,
      0x47,0xAB,0x4B,0x42,0x76,0xB7,0xFC,0x66,0x19,0x46,0xB5,0xCC}, 0x5760},
  };

  for (const auto& s : samples) {
    TEST_ASSERT_EQUAL_HEX16(s.crc, crc16Ccitt(s.payload, 24, 0x051E));
  }
}

void test_crc_rejects_the_conventional_seed() {
  const uint8_t payload[24] = {0x33,0x05,0x53,0x20,0x5C,0x8F,0x3D,0xC1,
                               0xB6,0x1B,0xFC,0x08,0x8E,0xDC,0x78,0xB0,
                               0x50,0xCC,0x04,0xFA,0x2E,0x2B,0x10,0x29};
  TEST_ASSERT_NOT_EQUAL(0x1A39, crc16Ccitt(payload, 24, 0xFFFF));
}

// ---------------------------------------------------------------------------
//  FLARM frames
// ---------------------------------------------------------------------------
flarm::State sampleState() {
  flarm::State s;
  s.latitude    = 47.89129;
  s.longitude   = 12.19416;
  s.altitude    = 564.0f;
  s.groundSpeed = 0.0f;
  s.climbRate   = 0.0f;
  s.track       = 0.0f;
  s.utcSeconds  = 1700000000u;
  s.address     = 0xBD080Bu;
  s.aircraft    = flarm::Aircraft::Paraglider;
  return s;
}

void test_v6_header_is_plaintext() {
  uint8_t payload[flarm::kPayloadBytes] = {};
  flarm::buildPayload(sampleState(), flarm::Format::V6, payload);

  // The first word carries the address and is not encrypted.
  TEST_ASSERT_EQUAL_HEX8(0x0B, payload[0]);
  TEST_ASSERT_EQUAL_HEX8(0x08, payload[1]);
  TEST_ASSERT_EQUAL_HEX8(0xBD, payload[2]);
  TEST_ASSERT_EQUAL_HEX8(0x30, payload[3]);   // format 0, address type 3 (OGN)
}

void test_v7_header_and_fixed_fields() {
  uint8_t payload[flarm::kPayloadBytes] = {};
  flarm::buildPayload(sampleState(), flarm::Format::V7, payload);

  TEST_ASSERT_EQUAL_HEX8(0x0B, payload[0]);
  TEST_ASSERT_EQUAL_HEX8(0x08, payload[1]);
  TEST_ASSERT_EQUAL_HEX8(0xBD, payload[2]);
  TEST_ASSERT_EQUAL_HEX8(0x32, payload[3]);   // format 2, address type 3 (OGN)

  // The second word holds only constants, so it has a fixed value. Recordings
  // of real traffic show it as bytes 00 00 00 33.
  TEST_ASSERT_EQUAL_HEX32(0x33000000u, loadLE32(payload + 4));
}

void test_encryption_leaves_the_header_alone_and_changes_the_body() {
  auto a = sampleState();
  auto b = sampleState();
  b.utcSeconds += 64;                 // the timestamp keys the cipher

  uint8_t pa[flarm::kPayloadBytes] = {};
  uint8_t pb[flarm::kPayloadBytes] = {};
  flarm::buildPayload(a, flarm::Format::V6, pa);
  flarm::buildPayload(b, flarm::Format::V6, pb);

  TEST_ASSERT_EQUAL_MEMORY(pa, pb, 4);                    // header unchanged
  TEST_ASSERT_NOT_EQUAL(0, std::memcmp(pa + 4, pb + 4, flarm::kPayloadBytes - 4));
}

void test_frame_carries_the_checksum_of_its_payload() {
  const auto s = sampleState();

  uint8_t payload[flarm::kPayloadBytes] = {};
  flarm::buildPayload(s, flarm::Format::V7, payload);
  const uint16_t crc = crc16Ccitt(payload, flarm::kPayloadBytes, 0x051E);

  uint8_t frame[flarm::kFrameBytes] = {};
  flarm::buildFrame(s, flarm::Format::V7, frame);

  // Undo the Manchester encoding of the last two payload bytes.
  uint8_t expected[4] = {};
  const uint8_t tail[2] = {uint8_t(crc >> 8), uint8_t(crc)};
  manchesterEncode(tail, 2, expected);

  TEST_ASSERT_EQUAL_MEMORY(expected, frame + flarm::kFrameBytes - 4, 4);
}

void test_both_formats_produce_distinct_frames() {
  const auto s = sampleState();
  uint8_t v6[flarm::kFrameBytes] = {};
  uint8_t v7[flarm::kFrameBytes] = {};
  flarm::buildFrame(s, flarm::Format::V6, v6);
  flarm::buildFrame(s, flarm::Format::V7, v7);
  TEST_ASSERT_NOT_EQUAL(0, std::memcmp(v6, v7, flarm::kFrameBytes));
}

// ---------------------------------------------------------------------------
//  FANET frames
// ---------------------------------------------------------------------------
void test_fanet_header_and_coordinates() {
  const fanet::Identity id{0xBD, 0x080B};
  fanet::State s;
  s.latitude  = 47.89129;
  s.longitude = 12.19416;
  s.altitude  = 564.0f;

  uint8_t out[fanet::kTrackingBytes] = {};
  const size_t len = fanet::buildTracking(id, s, out);
  TEST_ASSERT_EQUAL_UINT32(fanet::kTrackingBytes, len);

  TEST_ASSERT_EQUAL_HEX8(0x01, out[0]);      // tracking, no forwarding
  TEST_ASSERT_EQUAL_HEX8(0xBD, out[1]);
  TEST_ASSERT_EQUAL_HEX8(0x0B, out[2]);
  TEST_ASSERT_EQUAL_HEX8(0x08, out[3]);

  const int32_t lat = int32_t(out[4] | (out[5] << 8) | (out[6] << 16));
  TEST_ASSERT_INT32_WITHIN(2, int32_t(47.89129 * 93206.0), lat);
}

void test_fanet_altitude_switches_to_coarse_steps() {
  const fanet::Identity id{1, 1};
  fanet::State s;
  s.onlineTracking = false;
  s.aircraft = fanet::Aircraft::Other;

  uint8_t low[fanet::kTrackingBytes] = {};
  s.altitude = 1000.0f;
  fanet::buildTracking(id, s, low);
  const uint16_t fieldLow = uint16_t(low[10] | (low[11] << 8));
  TEST_ASSERT_EQUAL_UINT16(1000, fieldLow & 0x7FF);
  TEST_ASSERT_EQUAL_UINT16(0, fieldLow & (1 << 11));      // no scaling yet

  uint8_t high[fanet::kTrackingBytes] = {};
  s.altitude = 4000.0f;
  fanet::buildTracking(id, s, high);
  const uint16_t fieldHigh = uint16_t(high[10] | (high[11] << 8));
  TEST_ASSERT_NOT_EQUAL(0, fieldHigh & (1 << 11));        // four metre steps
  TEST_ASSERT_EQUAL_UINT16(1000, fieldHigh & 0x7FF);
}

void test_fanet_name_is_truncated_to_fit() {
  const fanet::Identity id{1, 1};
  uint8_t out[fanet::kNameMaxBytes] = {};
  const char* longName = "0123456789012345678901234567890123456789";
  const size_t len = fanet::buildName(id, longName, out);
  TEST_ASSERT_EQUAL_UINT32(fanet::kNameMaxBytes, len);
  TEST_ASSERT_EQUAL_HEX8(0x02, out[0]);                  // name frame
}

// ---------------------------------------------------------------------------
//  ADS-L frames
// ---------------------------------------------------------------------------
ads_l::State sampleAdslState() {
  ads_l::State s;
  s.latitude     = 52.0;
  s.longitude    = -1.0;
  s.altitude     = 148.0f;
  s.groundSpeed  = 0.0f;
  s.climbRate    = 0.0f;
  s.track        = 0.0f;
  s.timestamp    = 37;
  s.flightState  = 1;
  s.aircraft     = uint8_t(ads_l::Aircraft::HangGliderParaglider);
  s.emergency    = 1;
  s.addressTable = 7;
  s.address      = 0x123456;
  return s;
}

// Reference frame generated and cross-checked with the reference
// implementation (ADS-L Tx-demo: Init, Type=0x02, Address/Table 7, Meta,
// Lat/Lon/Alt, FlightState 1), then independently verified with a Python
// model of the frame layout, the XXTEA-Key0 scramble and the CRC-24
// polynomial.
//
// The reference frame carries the 0x18 Data-Link length byte in front
// (golden[0]); the protocol layer emits only the 24 byte network packet, the
// radio sends the length byte. The Tx-demo "Init" state used aircraft 1
// (our sample uses 7) and a non-zero integrity word, so those two fields are
// not part of the comparison.
//
// The two checks pin the properties the encoder is expected to reproduce:
// the reference CRC must satisfy the same residue check our encoder
// produces (this locks in the polynomial step order of spec E.3.1), and
// decoding the reference frame must recover the sample state (this locks in
// the XXTEA-Key0 scramble and the field layout).
void test_ads_l_golden_frame() {
  const uint8_t golden[ads_l::kFrameBytes + 1] = {
      0x18, 0x00, 0xBF, 0x08, 0x36, 0x8F, 0x50, 0x92,
      0x4C, 0xFC, 0x4E, 0xFE, 0xDD, 0xCA, 0x7A, 0xE2,
      0xE9, 0x63, 0x1D, 0x5C, 0x26, 0xD8, 0x70, 0xC5,
      0xDD};
  TEST_ASSERT_EQUAL_UINT32(0, ads_l::checkPI(golden + 1, ads_l::kFrameBytes));

  ads_l::Decoded d;
  TEST_ASSERT_TRUE(ads_l::decodeFrame(golden + 1, d));
  TEST_ASSERT_EQUAL_HEX8(0x00, d.version);
  TEST_ASSERT_EQUAL_HEX8(7, d.addressTable);
  TEST_ASSERT_EQUAL_HEX32(0x123456, d.address);
  TEST_ASSERT_FALSE(d.relay);
  TEST_ASSERT_EQUAL_HEX8(37, d.timestamp);
  TEST_ASSERT_EQUAL_HEX8(1, d.flightState);
  TEST_ASSERT_EQUAL_HEX8(1, d.emergency);
  TEST_ASSERT_FLOAT_WITHIN(1.1e-5, 52.0, d.latitude);
  TEST_ASSERT_FLOAT_WITHIN(2.2e-5, -1.0, d.longitude);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 148.0f, d.altitude);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, d.groundSpeed);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, d.climbRate);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, d.track);
}

void test_ads_l_check_pi_is_zero() {
  uint8_t frame[ads_l::kFrameBytes] = {};
  ads_l::buildFrame(sampleAdslState(), frame);
  TEST_ASSERT_EQUAL_UINT32(0, ads_l::checkPI(frame, ads_l::kFrameBytes));
}

void test_ads_l_decode_roundtrip() {
  uint8_t frame[ads_l::kFrameBytes] = {};
  ads_l::buildFrame(sampleAdslState(), frame);

  ads_l::Decoded d;
  TEST_ASSERT_TRUE(ads_l::decodeFrame(frame, d));

  TEST_ASSERT_EQUAL_HEX8(0x00, d.version);
  TEST_ASSERT_EQUAL_HEX8(7, d.addressTable);
  TEST_ASSERT_EQUAL_HEX32(0x123456, d.address);
  TEST_ASSERT_FALSE(d.relay);
  TEST_ASSERT_EQUAL_HEX8(37, d.timestamp);
  TEST_ASSERT_EQUAL_HEX8(1, d.flightState);
  TEST_ASSERT_EQUAL_HEX8(uint8_t(ads_l::Aircraft::HangGliderParaglider),
                         d.aircraft);
  TEST_ASSERT_EQUAL_HEX8(1, d.emergency);
  TEST_ASSERT_FLOAT_WITHIN(1.1e-5, 52.0, d.latitude);
  TEST_ASSERT_FLOAT_WITHIN(2.2e-5, -1.0, d.longitude);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 148.0f, d.altitude);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, d.groundSpeed);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, d.climbRate);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, d.track);
}

void test_ads_l_single_bit_error_is_repaired() {
  uint8_t frame[ads_l::kFrameBytes] = {};
  ads_l::buildFrame(sampleAdslState(), frame);

  frame[5] ^= 0x04;                             // one flake in the body
  TEST_ASSERT_NOT_EQUAL(0, ads_l::checkPI(frame, ads_l::kFrameBytes));
  TEST_ASSERT_EQUAL_INT(1, ads_l::correctFrame(frame, ads_l::kFrameBytes));
  TEST_ASSERT_EQUAL_UINT32(0, ads_l::checkPI(frame, ads_l::kFrameBytes));

  ads_l::Decoded d;
  TEST_ASSERT_TRUE(ads_l::decodeFrame(frame, d));
  TEST_ASSERT_FLOAT_WITHIN(2.2e-5, -1.0, d.longitude);
}

void test_ads_l_multiple_bit_errors_are_detected() {
  uint8_t frame[ads_l::kFrameBytes] = {};
  ads_l::buildFrame(sampleAdslState(), frame);

  frame[5] ^= 0x04;
  frame[6] ^= 0x02;
  TEST_ASSERT_NOT_EQUAL(0, ads_l::checkPI(frame, ads_l::kFrameBytes));
  TEST_ASSERT_EQUAL_INT(-1, ads_l::correctFrame(frame, ads_l::kFrameBytes));
}

// The spec examples G.1.7..G.1.9 exercised through the same code paths the
// encoder uses: altitude (VR-12) and speed/climb (VR-6).
void test_ads_l_exp_encoding_examples() {
  ads_l::State s = sampleAdslState();
  s.altitude    = 1000.0f;
  s.groundSpeed = 120.0f;
  s.climbRate   = 10.0f;

  uint8_t body[ads_l::kPayloadBytes] = {};
  ads_l::buildBody(s, body);

  // G.1.7: altitude VR-12, 1320 (1000 m + 320 m offset) encodes as 0x0528.
  TEST_ASSERT_EQUAL_HEX8(0x28, body[14]);
  TEST_ASSERT_EQUAL_HEX8(0x05, body[15] & 0x3F);
  // G.1.8: speed VR-6, 480 quarter-m/s (120 m/s) encodes as 0xC4.
  TEST_ASSERT_EQUAL_HEX8(0xC4, body[13]);
  // G.1.9: climb VR-6 signed, 80 eighth-m/s (10 m/s) encodes as 0x48.
  const uint8_t climbWord = uint8_t((body[16] << 2) | (body[15] >> 6));
  TEST_ASSERT_EQUAL_HEX8(0x48, climbWord);
}


// ---------------------------------------------------------------------------
//  Receive side
// ---------------------------------------------------------------------------
//  A round trip is the strongest check available without a second radio: if a
//  frame survives encoding and decoding with its values intact, the bit
//  layout, the cipher and the scaling all agree with themselves.
void roundTrip(flarm::Format fmt) {
  auto s = sampleState();
  s.groundSpeed = 12.0f;
  s.climbRate   = 1.5f;
  s.track       = 250.0f;

  uint8_t frame[flarm::kFrameBytes] = {};
  flarm::buildFrame(s, fmt, frame);

  flarm::Reference ref;
  ref.latitude   = s.latitude + 0.01;      // a nearby receiver
  ref.longitude  = s.longitude - 0.01;
  ref.utcSeconds = s.utcSeconds;

  Traffic t;
  TEST_ASSERT_TRUE(flarm::decodeFrame(frame, ref, t));
  TEST_ASSERT_EQUAL_HEX32(s.address, t.address);
  TEST_ASSERT_TRUE(t.hasPosition);
  TEST_ASSERT_DOUBLE_WITHIN(0.001, s.latitude, t.latitude);
  TEST_ASSERT_DOUBLE_WITHIN(0.001, s.longitude, t.longitude);
  TEST_ASSERT_FLOAT_WITHIN(30.0f, s.altitude, t.altitude);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(s.aircraft), uint8_t(t.aircraft));
  TEST_ASSERT_FLOAT_WITHIN(3.0f, s.groundSpeed, t.groundSpeed);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, s.climbRate, t.climbRate);
  TEST_ASSERT_FLOAT_WITHIN(15.0f, s.track, t.track);
}

void test_flarm_v6_round_trip() { roundTrip(flarm::Format::V6); }
void test_flarm_v7_round_trip() { roundTrip(flarm::Format::V7); }

void test_flarm_decode_rejects_a_corrupted_frame() {
  const auto s = sampleState();
  uint8_t frame[flarm::kFrameBytes] = {};
  flarm::buildFrame(s, flarm::Format::V7, frame);
  frame[10] = uint8_t(frame[10] ^ 0xFFu);      // damage one byte

  flarm::Reference ref;
  ref.latitude   = s.latitude;
  ref.longitude  = s.longitude;
  ref.utcSeconds = s.utcSeconds;

  Traffic t;
  TEST_ASSERT_FALSE(flarm::decodeFrame(frame, ref, t));
}

void test_flarm_decode_needs_the_right_time() {
  const auto s = sampleState();
  uint8_t frame[flarm::kFrameBytes] = {};
  flarm::buildFrame(s, flarm::Format::V6, frame);

  flarm::Reference ref;
  ref.latitude   = s.latitude;
  ref.longitude  = s.longitude;
  ref.utcSeconds = s.utcSeconds + 4096;        // clock far out of step

  // The header is plaintext so it still parses, but the body will not come
  // back the way it went out.
  Traffic t;
  if (flarm::decodeFrame(frame, ref, t)) {
    const bool same = std::fabs(t.latitude - s.latitude) < 0.001 &&
                      std::fabs(t.longitude - s.longitude) < 0.001;
    TEST_ASSERT_FALSE(same);
  }
}

void test_manchester_decode_is_the_inverse_of_encode() {
  const uint8_t in[6] = {0x00, 0xFF, 0x5A, 0xA5, 0x12, 0xBD};
  uint8_t air[12] = {};
  uint8_t back[6] = {};
  manchesterEncode(in, sizeof(in), air);
  TEST_ASSERT_TRUE(manchesterDecode(air, sizeof(in), back));
  TEST_ASSERT_EQUAL_MEMORY(in, back, sizeof(in));

  air[3] = 0x00;                                // an illegal symbol pair
  TEST_ASSERT_FALSE(manchesterDecode(air, sizeof(in), back));
}

void test_fanet_tracking_round_trip() {
  const fanet::Identity id{0xBD, 0x080B};
  fanet::State s;
  s.latitude    = 47.89129;
  s.longitude   = 12.19416;
  s.altitude    = 1200.0f;
  s.groundSpeed = 36.0f;                        // km/h
  s.climbRate   = -2.5f;
  s.track       = 90.0f;
  s.aircraft    = fanet::Aircraft::Paraglider;

  uint8_t frame[fanet::kTrackingBytes] = {};
  const size_t len = fanet::buildTracking(id, s, frame);

  Report r;
  TEST_ASSERT_TRUE(fanet::decodeFrame(frame, len, r));
  TEST_ASSERT_EQUAL(int(ReportKind::Traffic), int(r.kind));

  const Traffic& t = r.traffic;
  TEST_ASSERT_EQUAL_HEX32(0xBD080Bu, t.address);
  TEST_ASSERT_DOUBLE_WITHIN(0.0001, s.latitude, t.latitude);
  TEST_ASSERT_DOUBLE_WITHIN(0.0001, s.longitude, t.longitude);
  TEST_ASSERT_FLOAT_WITHIN(5.0f, s.altitude, t.altitude);
  // The report structure uses m/s while FANET transmits km/h.
  TEST_ASSERT_FLOAT_WITHIN(0.5f, s.groundSpeed / 3.6f, t.groundSpeed);
  TEST_ASSERT_FLOAT_WITHIN(0.3f, s.climbRate, t.climbRate);
  TEST_ASSERT_FLOAT_WITHIN(2.0f, s.track, t.track);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(AircraftType::Paraglider), uint8_t(t.aircraft));
}

void test_fanet_weather_decode() {
  // Service frame carrying temperature, wind, humidity, pressure and charge.
  const uint8_t frame[] = {
      0x04, 0xBD, 0x0B, 0x08,                   // header: service, address
      0x7A,                                     // flags: temp, wind, hum, baro, soc
      0x2B, 0x2C, 0x43,                         // latitude
      0x9A, 0x1D, 0x0B,                         // longitude
      0x2A,                                     // temperature 21.0 C
      0x40, 0x32, 0x50,                         // wind 90 deg, 10.0, 16.0 km/h
      0x7D,                                     // humidity 50 %
      0x86, 0x1C,                               // pressure
      0x0F};                                    // battery full

  Report r;
  TEST_ASSERT_TRUE(fanet::decodeFrame(frame, sizeof(frame), r));
  TEST_ASSERT_EQUAL(int(ReportKind::Weather), int(r.kind));

  const Weather& w = r.weather;
  TEST_ASSERT_EQUAL_HEX32(0xBD080Bu, w.address);
  TEST_ASSERT_TRUE(w.hasPosition);
  TEST_ASSERT_TRUE(w.hasTemperature);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 21.0f, w.temperature);
  TEST_ASSERT_TRUE(w.hasWind);
  TEST_ASSERT_FLOAT_WITHIN(2.0f, 90.0f, w.windDir);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 10.0f, w.windSpeed);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 16.0f, w.windGust);
  TEST_ASSERT_TRUE(w.hasHumidity);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 50.0f, w.humidity);
  TEST_ASSERT_TRUE(w.hasPressure);
  TEST_ASSERT_TRUE(w.hasBattery);
  TEST_ASSERT_EQUAL_UINT8(100, w.battery);
}

void test_fanet_name_decode() {
  const fanet::Identity id{0x11, 0x2233};
  uint8_t frame[fanet::kNameMaxBytes] = {};
  const size_t len = fanet::buildName(id, "Test Pilot", frame);

  Report r;
  TEST_ASSERT_TRUE(fanet::decodeFrame(frame, len, r));
  TEST_ASSERT_EQUAL(int(ReportKind::Name), int(r.kind));
  TEST_ASSERT_EQUAL_STRING("Test Pilot", r.name.text);
}

// ---------------------------------------------------------------------------
//  NMEA output
// ---------------------------------------------------------------------------
//  A sentence is only useful if the checksum is right, so it is recomputed
//  here the way a consumer would.
bool checksumValid(const char* sentence, std::size_t len) {
  if (len < 6 || sentence[0] != '$') return false;
  std::size_t star = 0;
  for (std::size_t i = 0; i < len; ++i)
    if (sentence[i] == '*') star = i;
  if (star == 0) return false;

  uint8_t sum = 0;
  for (std::size_t i = 1; i < star; ++i)
    sum = uint8_t(sum ^ uint8_t(sentence[i]));

  char expect[4];
  std::snprintf(expect, sizeof(expect), "%02X", sum);
  return sentence[star + 1] == expect[0] && sentence[star + 2] == expect[1];
}

void test_nmea_status_sentence() {
  nmea::Own own;
  own.valid    = true;
  own.airborne = true;

  char line[96];
  const std::size_t len = nmea::buildStatus(line, sizeof(line), 3, true, own);
  TEST_ASSERT_TRUE(len > 0);
  TEST_ASSERT_TRUE(checksumValid(line, len));
  TEST_ASSERT_EQUAL_STRING_LEN("$PFLAU,3,1,2,1,0,,0,,", line, 21);
  TEST_ASSERT_EQUAL('\r', line[len - 2]);
  TEST_ASSERT_EQUAL('\n', line[len - 1]);
}

void test_nmea_status_reports_no_fix() {
  nmea::Own own;                                 // valid stays false
  char line[96];
  const std::size_t len = nmea::buildStatus(line, sizeof(line), 0, false, own);
  TEST_ASSERT_TRUE(len > 0);
  TEST_ASSERT_EQUAL_STRING_LEN("$PFLAU,0,0,0,", line, 13);
}

void test_nmea_traffic_is_relative_to_own_position() {
  nmea::Own own;
  own.latitude  = 47.0;
  own.longitude = 12.0;
  own.altitude  = 1000.0f;
  own.valid     = true;

  Traffic t;
  t.address     = 0xDD8F12u;
  t.addressType = 2;
  t.latitude    = 47.001;                        // roughly 111 m north
  t.longitude   = 12.0;
  t.altitude    = 1220.0f;                       // 220 m above
  t.track       = 180.0f;
  t.groundSpeed = 30.0f;
  t.climbRate   = -1.4f;
  t.aircraft    = AircraftType::Glider;
  t.hasPosition = true;
  t.hasVelocity = true;

  char line[128];
  const std::size_t len = nmea::buildTraffic(line, sizeof(line), t, own);
  TEST_ASSERT_TRUE(len > 0);
  TEST_ASSERT_TRUE(checksumValid(line, len));
  TEST_ASSERT_EQUAL_STRING_LEN("$PFLAA,0,111,0,220,2,DD8F12,180,,30,-1.4,1",
                               line, 41);
}

void test_nmea_traffic_needs_both_positions() {
  nmea::Own own;                                 // no fix
  Traffic t;
  t.hasPosition = true;
  char line[128];
  TEST_ASSERT_EQUAL_UINT32(0, nmea::buildTraffic(line, sizeof(line), t, own));

  own.valid     = true;
  t.hasPosition = false;                         // target without a position
  TEST_ASSERT_EQUAL_UINT32(0, nmea::buildTraffic(line, sizeof(line), t, own));
}

void test_nmea_weather_leaves_unknown_fields_empty() {
  Weather w;
  w.address        = 0xBD080Bu;
  w.hasTemperature = true;
  w.temperature    = -3.5f;

  char line[160];
  const std::size_t len = nmea::buildWeather(line, sizeof(line), w);
  TEST_ASSERT_TRUE(len > 0);
  TEST_ASSERT_TRUE(checksumValid(line, len));
  // Position, wind, humidity, pressure and battery are all absent.
  TEST_ASSERT_EQUAL_STRING_LEN("$PBDWX,BD080B,,,-3.5,,,,,,", line, 26);
}

// ---------------------------------------------------------------------------
int main(int, char**) {
  UNITY_BEGIN();

  RUN_TEST(test_bitwriter_is_little_endian_lsb_first);
  RUN_TEST(test_bitwriter_signed_is_twos_complement);
  RUN_TEST(test_word_access_is_endianness_independent);

  RUN_TEST(test_manchester_polarity);
  RUN_TEST(test_manchester_doubles_length);

  RUN_TEST(test_crc_seed_against_recorded_frames);
  RUN_TEST(test_crc_rejects_the_conventional_seed);

  RUN_TEST(test_v6_header_is_plaintext);
  RUN_TEST(test_v7_header_and_fixed_fields);
  RUN_TEST(test_encryption_leaves_the_header_alone_and_changes_the_body);
  RUN_TEST(test_frame_carries_the_checksum_of_its_payload);
  RUN_TEST(test_both_formats_produce_distinct_frames);

  RUN_TEST(test_fanet_header_and_coordinates);
  RUN_TEST(test_fanet_altitude_switches_to_coarse_steps);
  RUN_TEST(test_fanet_name_is_truncated_to_fit);

  RUN_TEST(test_ads_l_golden_frame);
  RUN_TEST(test_ads_l_check_pi_is_zero);
  RUN_TEST(test_ads_l_decode_roundtrip);
  RUN_TEST(test_ads_l_single_bit_error_is_repaired);
  RUN_TEST(test_ads_l_multiple_bit_errors_are_detected);
  RUN_TEST(test_ads_l_exp_encoding_examples);

  RUN_TEST(test_manchester_decode_is_the_inverse_of_encode);
  RUN_TEST(test_flarm_v6_round_trip);
  RUN_TEST(test_flarm_v7_round_trip);
  RUN_TEST(test_flarm_decode_rejects_a_corrupted_frame);
  RUN_TEST(test_flarm_decode_needs_the_right_time);
  RUN_TEST(test_fanet_tracking_round_trip);
  RUN_TEST(test_fanet_weather_decode);
  RUN_TEST(test_fanet_name_decode);
  RUN_TEST(test_nmea_status_sentence);
  RUN_TEST(test_nmea_status_reports_no_fix);
  RUN_TEST(test_nmea_traffic_is_relative_to_own_position);
  RUN_TEST(test_nmea_traffic_needs_both_positions);
  RUN_TEST(test_nmea_weather_leaves_unknown_fields_empty);

  return UNITY_END();
}
