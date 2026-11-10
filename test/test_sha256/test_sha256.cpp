// Host tests for the hash behind the client key check, against the published
// vectors: FIPS 180-4 examples for SHA-256 and RFC 4231 for the HMAC.

#include <unity.h>

#include <cstdio>
#include <cstring>

#include "sha256.h"

namespace {

void expectHex(const char* expected, const uint8_t* actual) {
  char text[2 * crypto::kSha256Length + 1];
  for (size_t i = 0; i < crypto::kSha256Length; ++i)
    std::snprintf(text + 2 * i, 3, "%02x", actual[i]);
  TEST_ASSERT_EQUAL_STRING(expected, text);
}

const uint8_t* bytes(const char* text) { return reinterpret_cast<const uint8_t*>(text); }

}  // namespace

void test_hash_of_nothing() {
  uint8_t digest[crypto::kSha256Length];
  crypto::sha256(nullptr, 0, digest);
  expectHex("e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855", digest);
}

void test_hash_of_abc() {
  uint8_t digest[crypto::kSha256Length];
  crypto::sha256(bytes("abc"), 3, digest);
  expectHex("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad", digest);
}

void test_hash_spanning_two_blocks() {
  const char* message = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
  uint8_t digest[crypto::kSha256Length];
  crypto::sha256(bytes(message), std::strlen(message), digest);
  expectHex("248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1", digest);
}

void test_hash_fed_in_pieces_matches_one_call() {
  // Lengths around the block boundary are where the padding goes wrong.
  uint8_t data[200];
  for (size_t i = 0; i < sizeof(data); ++i) data[i] = uint8_t(i * 7 + 3);

  const size_t lengths[] = {55, 56, 57, 63, 64, 65, 119, 120, 128, 200};
  for (size_t length : lengths) {
    uint8_t whole[crypto::kSha256Length];
    crypto::sha256(data, length, whole);

    crypto::Sha256 hash;
    for (size_t i = 0; i < length; i += 13) hash.update(data + i, (length - i) < 13 ? (length - i) : 13);
    uint8_t pieces[crypto::kSha256Length];
    hash.finish(pieces);

    TEST_ASSERT_EQUAL_MEMORY(whole, pieces, sizeof(whole));
  }
}

void test_hmac_rfc4231_case_1() {
  uint8_t key[20];
  std::memset(key, 0x0b, sizeof(key));
  uint8_t mac[crypto::kSha256Length];
  crypto::hmacSha256(key, sizeof(key), bytes("Hi There"), 8, mac);
  expectHex("b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7", mac);
}

void test_hmac_rfc4231_case_2() {
  const char* message = "what do ya want for nothing?";
  uint8_t mac[crypto::kSha256Length];
  crypto::hmacSha256(bytes("Jefe"), 4, bytes(message), std::strlen(message), mac);
  expectHex("5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843", mac);
}

void test_hmac_with_a_key_longer_than_a_block() {
  uint8_t key[131];
  std::memset(key, 0xaa, sizeof(key));
  const char* message = "Test Using Larger Than Block-Size Key - Hash Key First";
  uint8_t mac[crypto::kSha256Length];
  crypto::hmacSha256(key, sizeof(key), bytes(message), std::strlen(message), mac);
  expectHex("60e431591ee0b67f0d8a26aacbf5b77f8e0bc6213728c5140546040f0ee37f54", mac);
}

void test_comparison() {
  const uint8_t a[4] = {1, 2, 3, 4};
  const uint8_t b[4] = {1, 2, 3, 4};
  const uint8_t c[4] = {1, 2, 3, 5};
  const uint8_t d[4] = {9, 2, 3, 4};
  TEST_ASSERT_TRUE(crypto::equalConstantTime(a, b, 4));
  TEST_ASSERT_FALSE(crypto::equalConstantTime(a, c, 4));
  TEST_ASSERT_FALSE(crypto::equalConstantTime(a, d, 4));
}

void setUp() {}
void tearDown() {}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_hash_of_nothing);
  RUN_TEST(test_hash_of_abc);
  RUN_TEST(test_hash_spanning_two_blocks);
  RUN_TEST(test_hash_fed_in_pieces_matches_one_call);
  RUN_TEST(test_hmac_rfc4231_case_1);
  RUN_TEST(test_hmac_rfc4231_case_2);
  RUN_TEST(test_hmac_with_a_key_longer_than_a_block);
  RUN_TEST(test_comparison);
  return UNITY_END();
}
