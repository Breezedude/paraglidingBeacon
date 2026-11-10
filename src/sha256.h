#pragma once
// ===========================================================================
//  SHA-256 and HMAC-SHA-256
// ---------------------------------------------------------------------------
//  Used to let a configuration client prove that it holds the key it was given
//  at the button, without the key going over the air again. Plain C++ with no
//  hardware behind it, so that it can be checked against the published test
//  vectors on the host: a hash that is almost right is of no use to anyone.
// ===========================================================================

#include <cstddef>
#include <cstdint>

namespace crypto {

inline constexpr size_t kSha256Length = 32;

class Sha256 {
 public:
  Sha256();
  void update(const uint8_t* data, size_t length);
  void finish(uint8_t digest[kSha256Length]);

 private:
  void compress(const uint8_t block[64]);

  uint32_t state_[8];
  uint8_t  block_[64];
  size_t   filled_ = 0;
  uint64_t total_  = 0;
};

void sha256(const uint8_t* data, size_t length, uint8_t digest[kSha256Length]);

void hmacSha256(const uint8_t* key, size_t keyLength,
                const uint8_t* message, size_t messageLength,
                uint8_t mac[kSha256Length]);

// Compares without stopping at the first difference, so the time taken does
// not tell a guesser how many leading bytes were right.
bool equalConstantTime(const uint8_t* a, const uint8_t* b, size_t length);

}  // namespace crypto
