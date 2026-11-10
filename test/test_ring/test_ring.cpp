// Host tests for the byte queue behind the Bluetooth link.

#include <unity.h>

#include <cstring>

#include "byte_ring.h"

namespace {

const uint8_t* bytes(const char* text) { return reinterpret_cast<const uint8_t*>(text); }

}  // namespace

void test_bytes_come_out_in_order() {
  ByteRing<16> ring;
  TEST_ASSERT_TRUE(ring.empty());
  TEST_ASSERT_TRUE(ring.push(bytes("abc"), 3));
  TEST_ASSERT_EQUAL_UINT(3, ring.size());
  TEST_ASSERT_EQUAL_INT('a', ring.pop());
  TEST_ASSERT_EQUAL_INT('b', ring.pop());
  TEST_ASSERT_EQUAL_INT('c', ring.pop());
  TEST_ASSERT_EQUAL_INT(-1, ring.pop());
  TEST_ASSERT_TRUE(ring.empty());
}

void test_a_message_that_does_not_fit_is_refused_whole() {
  ByteRing<8> ring;
  TEST_ASSERT_TRUE(ring.push(bytes("12345"), 5));
  TEST_ASSERT_FALSE(ring.push(bytes("6789"), 4));
  // Nothing of the refused message may have gone in.
  TEST_ASSERT_EQUAL_UINT(5, ring.size());
  TEST_ASSERT_EQUAL_UINT(3, ring.room());
  TEST_ASSERT_TRUE(ring.push(bytes("678"), 3));
  TEST_ASSERT_EQUAL_UINT(0, ring.room());
}

void test_peek_leaves_the_data_in_place() {
  ByteRing<16> ring;
  ring.push(bytes("hello"), 5);

  uint8_t out[8] = {};
  TEST_ASSERT_EQUAL_UINT(3, ring.peek(out, 3));
  TEST_ASSERT_EQUAL_MEMORY("hel", out, 3);
  TEST_ASSERT_EQUAL_UINT(5, ring.size());

  ring.drop(3);
  TEST_ASSERT_EQUAL_UINT(2, ring.peek(out, sizeof(out)));
  TEST_ASSERT_EQUAL_MEMORY("lo", out, 2);
}

void test_data_survives_wrapping_around_the_end() {
  ByteRing<8> ring;
  uint8_t out[8];

  // Walk the indices round the buffer several times with an odd step, so the
  // data straddles the end at every possible offset.
  for (int round = 0; round < 40; ++round) {
    TEST_ASSERT_TRUE(ring.push(bytes("ABCDE"), 5));
    TEST_ASSERT_EQUAL_UINT(5, ring.peek(out, sizeof(out)));
    TEST_ASSERT_EQUAL_MEMORY("ABCDE", out, 5);
    ring.drop(5);
    TEST_ASSERT_TRUE(ring.empty());
  }
}

void test_dropping_more_than_is_held_empties_it() {
  ByteRing<8> ring;
  ring.push(bytes("ab"), 2);
  ring.drop(100);
  TEST_ASSERT_TRUE(ring.empty());
  TEST_ASSERT_EQUAL_UINT(8, ring.room());
}

void test_clear_discards_what_is_waiting() {
  ByteRing<8> ring;
  ring.push(bytes("abcdef"), 6);
  ring.clear();
  TEST_ASSERT_TRUE(ring.empty());
  TEST_ASSERT_TRUE(ring.push(bytes("12345678"), 8));
  TEST_ASSERT_EQUAL_INT('1', ring.pop());
}

void setUp() {}
void tearDown() {}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_bytes_come_out_in_order);
  RUN_TEST(test_a_message_that_does_not_fit_is_refused_whole);
  RUN_TEST(test_peek_leaves_the_data_in_place);
  RUN_TEST(test_data_survives_wrapping_around_the_end);
  RUN_TEST(test_dropping_more_than_is_held_empties_it);
  RUN_TEST(test_clear_discards_what_is_waiting);
  return UNITY_END();
}
