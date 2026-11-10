#pragma once
// ===========================================================================
//  Bit and byte helpers for wire formats
// ---------------------------------------------------------------------------
//  Freestanding: depends only on the standard library, so the protocol layer
//  can be unit tested on a host without any hardware or framework headers.
// ===========================================================================

#include <cstddef>
#include <cstdint>

namespace proto {

// ---------------------------------------------------------------------------
//  Little-endian 32 bit access
// ---------------------------------------------------------------------------
//  Explicit byte assembly rather than casting the buffer to uint32_t*. That
//  would depend on host endianness and violate alignment and aliasing rules.
inline uint32_t loadLE32(const uint8_t* p) noexcept {
  return uint32_t(p[0]) | (uint32_t(p[1]) << 8) |
         (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}

inline void storeLE32(uint8_t* p, uint32_t v) noexcept {
  p[0] = uint8_t(v);
  p[1] = uint8_t(v >> 8);
  p[2] = uint8_t(v >> 16);
  p[3] = uint8_t(v >> 24);
}

// ---------------------------------------------------------------------------
//  BitWriter
// ---------------------------------------------------------------------------
//  Appends fields least significant bit first, starting at bit 0 of byte 0 and
//  spilling into the following bytes. This is the layout the wire formats use.
//
//  The destination must be zeroed beforehand; put() only sets bits.
class BitWriter {
 public:
  constexpr BitWriter(uint8_t* data, std::size_t size) noexcept
      : data_(data), size_(size) {}

  // Append the low `width` bits of `value`.
  void put(uint32_t value, unsigned width) noexcept {
    for (unsigned i = 0; i < width; ++i, ++pos_) {
      if ((pos_ >> 3) >= size_) return;          // never write past the buffer
      if (value & (uint32_t(1) << i)) {
        data_[pos_ >> 3] |= uint8_t(1u << (pos_ & 7u));
      }
    }
  }

  // Append a signed value in two's complement, truncated to `width` bits.
  void putSigned(int32_t value, unsigned width) noexcept {
    const uint32_t mask = (width >= 32) ? 0xFFFFFFFFu
                                       : ((uint32_t(1) << width) - 1u);
    put(uint32_t(value) & mask, width);
  }

  // Leave `width` bits at zero.
  void skip(unsigned width) noexcept { pos_ += width; }

  std::size_t bitsWritten() const noexcept { return pos_; }

 private:
  uint8_t*    data_;
  std::size_t size_;
  std::size_t pos_ = 0;
};

// ---------------------------------------------------------------------------
//  CRC-16/CCITT, polynomial 0x1021, most significant bit first
// ---------------------------------------------------------------------------
constexpr uint16_t crc16Ccitt(const uint8_t* data, std::size_t len,
                              uint16_t seed) noexcept {
  uint16_t crc = seed;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= uint16_t(uint16_t(data[i]) << 8);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000u) ? uint16_t(uint16_t(crc << 1) ^ 0x1021u)
                            : uint16_t(crc << 1);
    }
  }
  return crc;
}

// Number of set bits, modulo two.
constexpr unsigned parityOf(const uint8_t* data, std::size_t len) noexcept {
  unsigned ones = 0;
  for (std::size_t i = 0; i < len; ++i) {
    uint8_t b = data[i];
    while (b) { ones += b & 1u; b = uint8_t(b >> 1); }
  }
  return ones & 1u;
}

// ---------------------------------------------------------------------------
//  Manchester encoding
// ---------------------------------------------------------------------------
//  Convention: 1 -> 01, 0 -> 10 (G.E. Thomas), the inverse of IEEE 802.3.
//
//  Getting this backwards is silent and confusing: the receiver synchronises,
//  reports a plausible frame and decodes the exact bitwise complement of every
//  byte. An address of 0xBD080B arrives as 0x42F7F4 and the checksum simply
//  disagrees. The polarity here was settled by comparing the encoder output
//  against recordings of frames a receiver had accepted, and is covered by a
//  host test.
//
//  The lookup table is generated at compile time from the rule above, so the
//  rule itself stays readable instead of hiding in magic constants.
constexpr uint8_t manchesterNibble(uint8_t nibble) noexcept {
  uint8_t out = 0;
  for (int bit = 3; bit >= 0; --bit) {
    out = uint8_t(out << 2);
    out = uint8_t(out | (((nibble >> bit) & 1u) ? 0b01u : 0b10u));
  }
  return out;
}

// Each input byte becomes two output bytes; `out` needs 2 * len bytes.
inline void manchesterEncode(const uint8_t* in, std::size_t len,
                             uint8_t* out) noexcept {
  for (std::size_t i = 0; i < len; ++i) {
    out[2 * i]     = manchesterNibble(uint8_t(in[i] >> 4));
    out[2 * i + 1] = manchesterNibble(uint8_t(in[i] & 0x0F));
  }
}


// ---------------------------------------------------------------------------
//  BitReader
// ---------------------------------------------------------------------------
//  The exact mirror of BitWriter: fields are read least significant bit first,
//  starting at bit 0 of byte 0.
class BitReader {
 public:
  constexpr BitReader(const uint8_t* data, std::size_t size) noexcept
      : data_(data), size_(size) {}

  uint32_t get(unsigned width) noexcept {
    uint32_t value = 0;
    for (unsigned i = 0; i < width; ++i, ++pos_) {
      if ((pos_ >> 3) >= size_) break;
      if (data_[pos_ >> 3] & uint8_t(1u << (pos_ & 7u))) {
        value |= uint32_t(1) << i;
      }
    }
    return value;
  }

  // Reads `width` bits and sign extends them.
  int32_t getSigned(unsigned width) noexcept {
    const uint32_t raw = get(width);
    if (width == 0 || width >= 32) return int32_t(raw);
    const uint32_t sign = uint32_t(1) << (width - 1);
    return (raw & sign) ? int32_t(raw | ~((uint32_t(1) << width) - 1))
                        : int32_t(raw);
  }

  void skip(unsigned width) noexcept { pos_ += width; }
  std::size_t bitsRead() const noexcept { return pos_; }

 private:
  const uint8_t* data_;
  std::size_t    size_;
  std::size_t    pos_ = 0;
};

// ---------------------------------------------------------------------------
//  Manchester decoding
// ---------------------------------------------------------------------------
//  Inverse of manchesterEncode. Every pair of transmitted bits must be either
//  01 or 10; anything else means the symbol was corrupted, which is reported so
//  the caller can drop the frame instead of acting on noise.
//  `len` counts output bytes and requires 2 * len input bytes.
inline bool manchesterDecode(const uint8_t* in, std::size_t len,
                             uint8_t* out) noexcept {
  bool ok = true;
  for (std::size_t i = 0; i < len; ++i) {
    uint8_t value = 0;
    for (int half = 0; half < 2; ++half) {
      const uint8_t encoded = in[2 * i + half];
      for (int pair = 3; pair >= 0; --pair) {
        const uint8_t symbol = uint8_t((encoded >> (pair * 2)) & 0x03u);
        if (symbol == 0b01)      value = uint8_t((value << 1) | 1u);
        else if (symbol == 0b10) value = uint8_t(value << 1);
        else { value = uint8_t(value << 1); ok = false; }
      }
    }
    out[i] = value;
  }
  return ok;
}

// Inverse of the pseudo floating point compression used by some formats.
constexpr int32_t expandUnsigned(uint32_t value, unsigned mantissa,
                                 unsigned exponent) noexcept {
  const uint32_t offset = uint32_t(1) << mantissa;
  (void)exponent;
  if (value < offset) return int32_t(value);
  const uint32_t e = value >> mantissa;
  uint32_t m = (value & (offset - 1u)) + offset;
  m <<= e;
  return int32_t(m - offset);
}

constexpr int32_t expandSigned(uint32_t value, unsigned mantissa,
                               unsigned exponent) noexcept {
  const uint32_t signBit  = uint32_t(1) << (mantissa + exponent);
  const bool     negative = (value & signBit) != 0;
  const int32_t  magnitude =
      expandUnsigned(value & (signBit - 1u), mantissa, exponent);
  return negative ? -magnitude : magnitude;
}

}  // namespace proto
