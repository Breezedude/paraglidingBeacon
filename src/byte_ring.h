#pragma once
// ===========================================================================
//  Fixed size byte queue
// ---------------------------------------------------------------------------
//  Holds what is waiting to be sent or parsed, so that neither side has to
//  wait for the other. A message goes in whole or not at all: a queue that
//  accepted half a line would hand the reader two lines run together.
//
//  One writer and one reader may be in different tasks. The writer only moves
//  the head and the reader only moves the tail, and each index is a single
//  word, so no lock is needed. clear() belongs to the reader.
// ===========================================================================

#include <cstddef>
#include <cstdint>

template <size_t N>
class ByteRing {
  // The indices run freely and wrap with the word size, which only lines up
  // with the buffer when its length is a power of two.
  static_assert(N != 0 && (N & (N - 1)) == 0, "length must be a power of two");

 public:
  size_t size()  const { return size_t(uint32_t(head_ - tail_)); }
  size_t room()  const { return N - size(); }
  bool   empty() const { return head_ == tail_; }

  // Appends all of the data, or nothing when it does not fit.
  bool push(const uint8_t* data, size_t length) {
    if (length > room()) return false;
    const uint32_t head = head_;
    for (size_t i = 0; i < length; ++i) buffer_[(head + i) & (N - 1)] = data[i];
    head_ = head + uint32_t(length);
    return true;
  }

  // Copies up to `limit` bytes from the front without removing them.
  size_t peek(uint8_t* out, size_t limit) const {
    const uint32_t tail  = tail_;
    const size_t   count = size() < limit ? size() : limit;
    for (size_t i = 0; i < count; ++i) out[i] = buffer_[(tail + i) & (N - 1)];
    return count;
  }

  // Removes up to `count` bytes from the front.
  void drop(size_t count) {
    const size_t held = size();
    tail_ = tail_ + uint32_t(count < held ? count : held);
  }

  // Removes and returns the front byte, or -1 when there is none.
  int pop() {
    if (empty()) return -1;
    const uint32_t tail = tail_;
    const uint8_t  byte = buffer_[tail & (N - 1)];
    tail_ = tail + 1;
    return byte;
  }

  void clear() { tail_ = head_; }

 private:
  uint8_t           buffer_[N];
  volatile uint32_t head_ = 0;   // moved by the writer only
  volatile uint32_t tail_ = 0;   // moved by the reader only
};
