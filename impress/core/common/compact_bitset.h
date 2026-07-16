// Copyright 2026 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_COMPACT_BITSET_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_COMPACT_BITSET_H_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "absl/numeric/bits.h"

namespace imp {

// A memory-efficient bitset that uses inline storage. Once a bit is set that is
// outside the bounds of the inline storage, the bitset will automatically
// switch to using heap storage.
//
// The key benefit of this class is that it has **zero** memory overhead beyond
// the inline storage while also supporting arbitrary bit indices allowing the
// bitset to grow as needed.
//
// The number of bits that can be stored is `InlineWords * 64 - 1`. For example,
// `CompactBitset<2>` can store up to 127 bits while `CompactBitset<4>` can
// store up to 255 bits. The reason for the `-1` is that the last bit of the
// last inline word is used as a flag to indicate whether the bitset is on the
// heap or not.
//
// Template parameter `InlineWords` specifies the number of 64-bit words to use
// as inline storage. It must be at least 2 to support zero-overhead heap
// spilling.
template <size_t InlineWords = 4>
class CompactBitset {
  static_assert(InlineWords >= 2, "InlineWords must be at least 2");

 public:
  CompactBitset();
  ~CompactBitset();

  CompactBitset(const CompactBitset& other);
  CompactBitset& operator=(const CompactBitset& other);

  CompactBitset(CompactBitset&& other) noexcept;
  CompactBitset& operator=(CompactBitset&& other) noexcept;

  // Sets the bit at index `bit_idx`.
  void SetBit(size_t bit_idx);

  // Clears the bit at index `bit_idx` if it is set, otherwise does nothing.
  void ClearBit(size_t bit_idx);

  // Returns true if the bit at index `bit_idx` is set.
  bool GetBit(size_t bit_idx) const noexcept;

  // Calls the callback function for each set bit.
  //
  // **Important:** It is not safe to call 'SetBit' or 'ClearBit' during
  // iteration.
  template <typename Fn>
  void ForEachSetBit(Fn&& callback) const;

 private:
  // The last bit of the last inline word indicates whether the bitset is on the
  // heap or using the inline storage.
  static constexpr uint64_t kHeapFlag = 1ULL << 63;

  bool IsOnHeap() const noexcept;
  uint64_t* GetHeapPtr() const noexcept;
  uint32_t GetHeapCapacity() const noexcept;

  void SetHeapState(uint64_t* ptr, uint32_t capacity) noexcept;
  void Cleanup() noexcept;

  void HandleHeapSet(size_t bit_idx);
  void Inflate(size_t bit_idx);
  void EnsureCapacity(size_t bit_idx);

  alignas(uint64_t) std::array<uint64_t, InlineWords> storage_ = {};
};

template <size_t InlineWords>
CompactBitset<InlineWords>::CompactBitset() = default;

template <size_t InlineWords>
CompactBitset<InlineWords>::~CompactBitset() {
  Cleanup();
}

template <size_t InlineWords>
CompactBitset<InlineWords>::CompactBitset(const CompactBitset& other) {
  if (other.IsOnHeap()) {
    uint32_t cap = other.GetHeapCapacity();
    uint64_t* p = new uint64_t[cap]();
    std::copy(other.GetHeapPtr(), other.GetHeapPtr() + cap, p);
    SetHeapState(p, cap);
  } else {
    storage_ = other.storage_;
  }
}

template <size_t InlineWords>
CompactBitset<InlineWords>& CompactBitset<InlineWords>::operator=(
    const CompactBitset& other) {
  if (this != &other) {
    CompactBitset temp(other);
    std::swap(storage_, temp.storage_);
  }
  return *this;
}

template <size_t InlineWords>
CompactBitset<InlineWords>::CompactBitset(CompactBitset&& other) noexcept {
  storage_ = other.storage_;
  other.storage_.fill(0);
}

template <size_t InlineWords>
CompactBitset<InlineWords>& CompactBitset<InlineWords>::operator=(
    CompactBitset&& other) noexcept {
  if (this != &other) {
    Cleanup();
    storage_ = other.storage_;
    other.storage_.fill(0);
  }
  return *this;
}

template <size_t InlineWords>
void CompactBitset<InlineWords>::SetBit(size_t bit_idx) {
  constexpr size_t kInlineHeapFlagBit = InlineWords * 64 - 1;
  if (IsOnHeap()) {
    HandleHeapSet(bit_idx);
  } else if (bit_idx >= kInlineHeapFlagBit) {
    Inflate(bit_idx);
    HandleHeapSet(bit_idx);
  } else {
    storage_[bit_idx / 64] |= (1ULL << (bit_idx % 64));
  }
}

template <size_t InlineWords>
void CompactBitset<InlineWords>::ClearBit(size_t bit_idx) {
  if (IsOnHeap()) {
    size_t word_idx = bit_idx / 64;
    if (word_idx < GetHeapCapacity()) {
      GetHeapPtr()[word_idx] &= ~(1ULL << (bit_idx % 64));
    }
  } else {
    constexpr size_t kInlineHeapFlagBit = InlineWords * 64 - 1;
    if (bit_idx < kInlineHeapFlagBit) {
      storage_[bit_idx / 64] &= ~(1ULL << (bit_idx % 64));
    }
  }
}

template <size_t InlineWords>
bool CompactBitset<InlineWords>::GetBit(size_t bit_idx) const noexcept {
  if (IsOnHeap()) {
    size_t word_idx = bit_idx / 64;
    if (word_idx >= GetHeapCapacity()) {
      return false;
    }
    return (GetHeapPtr()[word_idx] & (1ULL << (bit_idx % 64))) != 0;
  } else {
    constexpr size_t kInlineHeapFlagBit = InlineWords * 64 - 1;
    if (bit_idx >= kInlineHeapFlagBit) {
      return false;
    }
    return (storage_[bit_idx / 64] & (1ULL << (bit_idx % 64))) != 0;
  }
}

template <size_t InlineWords>
template <typename Fn>
void CompactBitset<InlineWords>::ForEachSetBit(Fn&& callback) const {
  if (IsOnHeap()) {
    const uint64_t* ptr = GetHeapPtr();
    const size_t cap = GetHeapCapacity();
    for (size_t i = 0; i < cap; ++i) {
      uint64_t word = ptr[i];
      while (word != 0) {
        int bit = absl::countr_zero(word);
        callback(i * 64 + bit);
        word &= (word - 1);
      }
    }
  } else {
    // This logic is intentionally duplicated from the heap case instead of
    // sharing the cap and ptr variables dynamically. Since InlineWords is known
    // at compile time, it allows the compiler to perform optimizations that
    // would be inhibited otherwise.
    for (size_t i = 0; i < InlineWords; ++i) {
      uint64_t word = storage_[i];
      while (word != 0) {
        int bit = absl::countr_zero(word);
        callback(i * 64 + bit);
        word &= (word - 1);
      }
    }
  }
}

template <size_t InlineWords>
bool CompactBitset<InlineWords>::IsOnHeap() const noexcept {
  return (storage_[InlineWords - 1] & kHeapFlag) != 0;
}

template <size_t InlineWords>
uint64_t* CompactBitset<InlineWords>::GetHeapPtr() const noexcept {
  return reinterpret_cast<uint64_t*>(storage_[0]);
}

template <size_t InlineWords>
uint32_t CompactBitset<InlineWords>::GetHeapCapacity() const noexcept {
  // The heap capacity is stored in the second word of the storage. The word is
  // uint64_t and capacity is limited to uint32_t. Casting the storage to
  // uint32_t intentionally truncates the value, this is because when
  // InlineWords is 2, the heap flag is stored in the same word as the capacity,
  // truncating the value ensures the capacity isn't impacted by the heap flag.
  return static_cast<uint32_t>(storage_[1]);
}

template <size_t InlineWords>
void CompactBitset<InlineWords>::SetHeapState(uint64_t* ptr,
                                              uint32_t capacity) noexcept {
  storage_[0] = reinterpret_cast<uint64_t>(ptr);
  storage_[1] = capacity;
  storage_[InlineWords - 1] |= kHeapFlag;
}

template <size_t InlineWords>
void CompactBitset<InlineWords>::Cleanup() noexcept {
  if (IsOnHeap()) {
    delete[] GetHeapPtr();
  }
}

template <size_t InlineWords>
void CompactBitset<InlineWords>::HandleHeapSet(size_t bit_idx) {
  EnsureCapacity(bit_idx);
  GetHeapPtr()[bit_idx / 64] |= (1ULL << (bit_idx % 64));
}

template <size_t InlineWords>
void CompactBitset<InlineWords>::Inflate(size_t bit_idx) {
  uint32_t needed_words = static_cast<uint32_t>((bit_idx / 64) + 1);
  // Double capacity (or take required size if larger) to amortize reallocation
  // cost.
  uint32_t cap = std::max(static_cast<uint32_t>(InlineWords * 2), needed_words);
  uint64_t* p = new uint64_t[cap]();

  for (size_t i = 0; i < InlineWords; ++i) {
    p[i] = storage_[i];
  }

  SetHeapState(p, cap);
}

template <size_t InlineWords>
void CompactBitset<InlineWords>::EnsureCapacity(size_t bit_idx) {
  uint32_t needed_idx = static_cast<uint32_t>(bit_idx / 64);
  uint32_t current_cap = GetHeapCapacity();
  if (needed_idx >= current_cap) {
    // Double capacity (or take required size if larger) to achieve O(1)
    // amortized growth.
    uint32_t new_cap = std::max(current_cap * 2, needed_idx + 1);
    uint64_t* p = new uint64_t[new_cap]();
    if (GetHeapPtr()) {
      std::copy(GetHeapPtr(), GetHeapPtr() + current_cap, p);
      delete[] GetHeapPtr();
    }
    SetHeapState(p, new_cap);
  }
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_COMPACT_BITSET_H_
