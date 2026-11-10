#pragma once
// ===========================================================================
//  XXTEA (Corrected Block TEA), Needham and Wheeler
// ---------------------------------------------------------------------------
//  The reference algorithm is in the public domain. The number of rounds is a
//  parameter here because the air protocol uses a fixed count rather than the
//  6 + 52/n of the original note.
// ===========================================================================

#include <cstdint>

namespace proto {

inline constexpr uint32_t kXxteaDelta = 0x9E3779B9u;

// Encrypts `n` words in place. `n` must be at least two.
inline void xxteaEncrypt(uint32_t* v, int n, const uint32_t key[4],
                         unsigned rounds) noexcept {
  if (n < 2) return;

  const auto mix = [&](uint32_t y, uint32_t z, uint32_t sum,
                       unsigned p, uint32_t e) -> uint32_t {
    const uint32_t a = ((z >> 5) ^ (y << 2)) + ((y >> 3) ^ (z << 4));
    const uint32_t b = (sum ^ y) + (key[(p & 3u) ^ e] ^ z);
    return a ^ b;
  };

  uint32_t sum = 0;
  uint32_t z   = v[n - 1];
  while (rounds--) {
    sum += kXxteaDelta;
    const uint32_t e = (sum >> 2) & 3u;
    uint32_t y;
    for (int p = 0; p < n - 1; ++p) {
      y = v[p + 1];
      z = v[p] += mix(y, z, sum, unsigned(p), e);
    }
    y = v[0];
    z = v[n - 1] += mix(y, z, sum, unsigned(n - 1), e);
  }
}

// ---------------------------------------------------------------------------
//  The "Key0" variant the ADS-L air protocol uses for scrambling: no key is
//  fed in, so the round counter step and the key selection term vanish.
//  Five words, six rounds, exactly as the reference implementation.
// ---------------------------------------------------------------------------
inline void xxteaEncryptKey0(uint32_t* v, int n, unsigned rounds) noexcept {
  if (n < 2) return;

  const auto mix = [](uint32_t y, uint32_t z, uint32_t sum) -> uint32_t {
    const uint32_t a = ((z >> 5) ^ (y << 2)) + ((y >> 3) ^ (z << 4));
    const uint32_t b = (sum ^ y) + z;
    return a ^ b;
  };

  uint32_t sum = 0;
  uint32_t z   = v[n - 1];
  while (rounds--) {
    sum += kXxteaDelta;
    uint32_t y;
    for (int p = 0; p < n - 1; ++p) {
      y = v[p + 1];
      z = v[p] += mix(y, z, sum);
    }
    y = v[0];
    z = v[n - 1] += mix(y, z, sum);
  }
}

inline void xxteaDecryptKey0(uint32_t* v, int n, unsigned rounds) noexcept {
  if (n < 2) return;

  const auto mix = [](uint32_t y, uint32_t z, uint32_t sum) -> uint32_t {
    const uint32_t a = ((z >> 5) ^ (y << 2)) + ((y >> 3) ^ (z << 4));
    const uint32_t b = (sum ^ y) + z;
    return a ^ b;
  };

  uint32_t sum = rounds * kXxteaDelta;
  uint32_t y   = v[0];
  while (rounds--) {
    uint32_t z;
    for (int p = n - 1; p > 0; --p) {
      z = v[p - 1];
      y = v[p] -= mix(y, z, sum);
    }
    z = v[n - 1];
    y = v[0] -= mix(y, z, sum);
    sum -= kXxteaDelta;
  }
}


// Decrypts `n` words in place, the exact inverse of xxteaEncrypt.
inline void xxteaDecrypt(uint32_t* v, int n, const uint32_t key[4],
                         unsigned rounds) noexcept {
  if (n < 2) return;

  const auto mix = [&](uint32_t y, uint32_t z, uint32_t sum,
                       unsigned p, uint32_t e) -> uint32_t {
    const uint32_t a = ((z >> 5) ^ (y << 2)) + ((y >> 3) ^ (z << 4));
    const uint32_t b = (sum ^ y) + (key[(p & 3u) ^ e] ^ z);
    return a ^ b;
  };

  uint32_t sum = uint32_t(rounds) * kXxteaDelta;
  uint32_t y   = v[0];
  while (rounds--) {
    const uint32_t e = (sum >> 2) & 3u;
    uint32_t z;
    for (int p = n - 1; p > 0; --p) {
      z = v[p - 1];
      y = v[p] -= mix(y, z, sum, unsigned(p), e);
    }
    z = v[n - 1];
    y = v[0] -= mix(y, z, sum, 0u, e);
    sum -= kXxteaDelta;
  }
}

}  // namespace proto
