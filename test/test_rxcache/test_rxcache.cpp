// Host tests for the received station cache.

#include <unity.h>

#include <cstring>

#include "rx_cache.h"

using proto::Name;
using proto::Traffic;
using proto::Weather;

namespace {

Traffic traffic(uint32_t address, double lat = 47.0) {
  Traffic t;
  t.address     = address;
  t.latitude    = lat;
  t.longitude   = 12.0;
  t.hasPosition = true;
  return t;
}

Weather weather(uint32_t address, float temp = 21.0f) {
  Weather w;
  w.address        = address;
  w.temperature    = temp;
  w.hasTemperature = true;
  return w;
}

Name named(uint32_t address, const char* text) {
  Name n;
  n.address = address;
  std::strncpy(n.text, text, sizeof(n.text) - 1);
  return n;
}

const rx::Entry* find(const rx::Cache& c, uint32_t address) {
  for (unsigned i = 0; i < c.capacity(); ++i)
    if (c.at(i).used && c.at(i).address == address) return &c.at(i);
  return nullptr;
}

}  // namespace

// ---------------------------------------------------------------------------
void test_starts_empty() {
  rx::Cache c;
  TEST_ASSERT_EQUAL_UINT32(0, c.size());
}

void test_one_entry_per_address() {
  rx::Cache c;
  c.note(1000, traffic(0xBD1111));
  c.note(2000, traffic(0xBD1111, 48.0));
  TEST_ASSERT_EQUAL_UINT32(1, c.size());
  TEST_ASSERT_EQUAL_DOUBLE(48.0, find(c, 0xBD1111)->traffic.latitude);
}

void test_weather_and_name_merge_into_one_station() {
  rx::Cache c;
  c.note(1000, weather(0xBD9E2E, 27.5f));
  c.note(2000, named(0xBD9E2E, "Teststation"));
  TEST_ASSERT_EQUAL_UINT32(1, c.size());

  const rx::Entry* e = find(c, 0xBD9E2E);
  TEST_ASSERT_TRUE(e->hasWeather);
  TEST_ASSERT_TRUE(e->hasName);
  TEST_ASSERT_FALSE(e->hasTraffic);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 27.5f, e->weather.temperature);
  TEST_ASSERT_EQUAL_STRING("Teststation", e->name.text);
}

void test_a_name_does_not_erase_the_position() {
  rx::Cache c;
  c.note(1000, traffic(0xBD5555, 47.5));
  c.note(2000, named(0xBD5555, "Ein Pilot"));

  const rx::Entry* e = find(c, 0xBD5555);
  TEST_ASSERT_TRUE(e->hasTraffic);
  TEST_ASSERT_EQUAL_DOUBLE(47.5, e->traffic.latitude);
  TEST_ASSERT_EQUAL_STRING("Ein Pilot", e->name.text);
}

void test_stations_expire_after_the_timeout() {
  rx::Cache c;
  c.note(1000, traffic(0xBD1111));
  c.note(200000, traffic(0xBD2222));

  c.expire(1000 + rx::kEntryTimeoutMs);          // exactly on the limit
  TEST_ASSERT_EQUAL_UINT32(2, c.size());

  c.expire(1001 + rx::kEntryTimeoutMs);
  TEST_ASSERT_EQUAL_UINT32(1, c.size());
  TEST_ASSERT_NULL(find(c, 0xBD1111));
  TEST_ASSERT_NOT_NULL(find(c, 0xBD2222));
}

void test_hearing_a_station_again_keeps_it_alive() {
  rx::Cache c;
  c.note(1000, traffic(0xBD1111));
  c.note(200000, named(0xBD1111, "noch da"));    // any frame counts
  c.expire(1001 + rx::kEntryTimeoutMs);
  TEST_ASSERT_EQUAL_UINT32(1, c.size());
}

void test_a_full_cache_drops_the_oldest() {
  rx::Cache c;
  for (unsigned i = 0; i < rx::kMaxEntries; ++i)
    c.note(1000 + i * 10, traffic(0xBD0000 + i));
  TEST_ASSERT_EQUAL_UINT32(rx::kMaxEntries, c.size());

  c.note(9000, traffic(0xBDFFFF));
  TEST_ASSERT_EQUAL_UINT32(rx::kMaxEntries, c.size());
  TEST_ASSERT_NULL(find(c, 0xBD0000));           // the first one heard
  TEST_ASSERT_NOT_NULL(find(c, 0xBDFFFF));
  TEST_ASSERT_NOT_NULL(find(c, 0xBD0001));
}

void test_a_recycled_slot_does_not_inherit_the_previous_station() {
  rx::Cache c;
  for (unsigned i = 0; i < rx::kMaxEntries; ++i)
    c.note(1000 + i * 10, weather(0xBD0000 + i, 5.0f));

  c.note(9000, traffic(0xBDFFFF));
  const rx::Entry* e = find(c, 0xBDFFFF);
  TEST_ASSERT_TRUE(e->hasTraffic);
  TEST_ASSERT_FALSE(e->hasWeather);
  TEST_ASSERT_FALSE(e->hasName);
}

void test_clear_forgets_everything() {
  rx::Cache c;
  c.note(1000, traffic(0xBD1111));
  c.note(1000, weather(0xBD2222));
  c.clear();
  TEST_ASSERT_EQUAL_UINT32(0, c.size());
}

// ---------------------------------------------------------------------------
extern "C" void setUp() {}
extern "C" void tearDown() {}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_starts_empty);
  RUN_TEST(test_one_entry_per_address);
  RUN_TEST(test_weather_and_name_merge_into_one_station);
  RUN_TEST(test_a_name_does_not_erase_the_position);
  RUN_TEST(test_stations_expire_after_the_timeout);
  RUN_TEST(test_hearing_a_station_again_keeps_it_alive);
  RUN_TEST(test_a_full_cache_drops_the_oldest);
  RUN_TEST(test_a_recycled_slot_does_not_inherit_the_previous_station);
  RUN_TEST(test_clear_forgets_everything);
  return UNITY_END();
}
