/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_BIT_VECTOR_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_BIT_VECTOR_H_

#include <algorithm>
#include <cassert>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <vector>

#include "absl/types/span.h"
#include "filament/libs/utils/include/utils/algorithm.h"
#include "filament/libs/utils/include/utils/compiler.h"

namespace imp {
namespace details {

constexpr size_t FloorLog2(size_t x) {
  return x == 1 ? 0 : 1 + FloorLog2(x >> 1);
}

constexpr size_t CeilLog2(size_t x) {
  return x == 1 ? 0 : FloorLog2(x - 1) + 1;
}

}  // namespace details

class BitVector {
  using Word = uint32_t;
  static constexpr size_t kWordBits = sizeof(Word) * CHAR_BIT;
  static constexpr size_t kWordShift = details::CeilLog2(kWordBits);
  static constexpr size_t kWordMask = (1 << kWordShift) - 1;

 public:
  using value_type = bool;

  BitVector() : words_(), count_(0) {}

  explicit BitVector(std::vector<uint32_t> words, size_t count)
      : words_(std::move(words)), count_(count) {}

  absl::Span<const uint32_t> Words() const { return absl::MakeSpan(words_); }

  void Resize(int size) {
    int word_count = size ? (((size - 1) >> kWordShift) + 1) : 0;
    words_.resize(word_count, Word{0});
    count_ = size;
  }

  size_t FindFirstWithValue(bool value) const {
    size_t partial_count = count_ & kWordMask;
    size_t filled = partial_count ? words_.size() - 1 : words_.size();
    size_t i;
    auto filled_mask = ~Word{0};
    for (i = 0; i < filled; ++i) {
      Word test = value ? words_[i] : ~words_[i];
      if (test && test != filled_mask) {
        size_t lowest_set_bit = ::utils::ctz(test);
        return lowest_set_bit + (i << kWordShift);
      }
    }

    if (partial_count) {
      auto partial_mask = static_cast<Word>((1 << partial_count) - 1);
      Word test = (value ? words_[i] : ~words_[i]) | ~partial_mask;
      size_t lowest_set_bit = ::utils::ctz(test);
      return std::min(lowest_set_bit, partial_count) + (i << kWordShift);
    } else {
      return i << kWordShift;
    }
  }

  bool Get(size_t index) const {
    const size_t word_index = index >> kWordShift;
    const size_t bit_index = index & kWordMask;
    const auto bit_mask = static_cast<Word>(1 << bit_index);
    return !!(words_.at(word_index) & bit_mask);
  }

  void Set(size_t index, bool value = true) {
    const size_t word_index = index >> kWordShift;
    const size_t bit_index = index & kWordMask;
    const uint32_t bit_mask = 1 << bit_index;
    if (value) {
      words_.at(word_index) |= bit_mask;
    } else {
      words_.at(word_index) &= ~bit_mask;
    }
  }

  void SetAll(bool value = true) {
    size_t partial_count = count_ & kWordMask;
    size_t filled = partial_count ? words_.size() - 1 : words_.size();
    size_t i;
    auto filled_mask = value ? ~Word{0} : Word{0};
    for (i = 0; i < filled; ++i) {
      words_[i] = filled_mask;
    }
    if (partial_count) {
      auto partial_mask =
          value ? static_cast<Word>((1 << partial_count) - 1) : Word{0};
      words_[i] = partial_mask;
    }
  }

  // Enable std::back_inserter() and friends.
  void push_back(bool value) {
    if (!(count_ & kWordMask)) {
      // incrementing our count will require a new word.
      words_.push_back(Word{0});
    }
    Set(count_++, value);
  }
  size_t size() const { return count_; }
  bool empty() const { return count_ == 0; }

  bool Any(bool set = true) const {
    size_t partial_count = count_ & kWordMask;
    size_t filled = partial_count ? words_.size() - 1 : words_.size();
    size_t i;
    for (i = 0; i < filled; ++i) {
      Word test = set ? words_[i] : ~words_[i];
      if (test) {
        return true;
      }
    }
    if (partial_count) {
      auto partial_mask = static_cast<Word>((1 << partial_count) - 1);
      Word word = (set ? words_[i] : ~words_[i]) & partial_mask;
      if (word) {
        return true;
      }
    }
    return false;
  }

  template <typename F>
  void ForEachBit(F f, bool set_bits = true) const noexcept {
    size_t partial_count = count_ & kWordMask;
    size_t filled = partial_count ? words_.size() - 1 : words_.size();
    size_t i;
    for (i = 0; i < filled; ++i) {
      Word word = set_bits ? words_[i] : ~words_[i];
      while (word) {
        size_t lowest_set_bit = ::utils::ctz(word);
        word &= ~(Word(1) << lowest_set_bit);
        f(size_t((i << kWordShift) + lowest_set_bit));
      }
    }
    if (partial_count) {
      auto partial_mask = static_cast<Word>((1 << partial_count) - 1);
      Word word = (set_bits ? words_[i] : ~words_[i]) & partial_mask;
      while (word) {
        size_t lowest_set_bit = ::utils::ctz(word);
        word &= ~(Word(1) << lowest_set_bit);
        f(size_t((i << kWordShift) + lowest_set_bit));
      }
    }
  }

  class Iterator {
   public:
    typedef bool value_type;
    typedef Iterator reference;
    typedef void pointer;
    typedef std::output_iterator_tag iterator_category;
    typedef size_t difference_type;

    explicit Iterator(BitVector &container, size_t index = 0)
        : container_(container), index_(index) {}
    bool operator*() const { return GetValue(); }
    explicit operator bool() const { return GetValue(); }
    Iterator &operator=(bool value) {
      container_.Set(index_, value);
      return *this;
    }
    Iterator &operator++() {
      ++index_;
      return *this;
    }
    Iterator operator++(int) {
      Iterator ret = *this;
      ++index_;
      return ret;
    }
    bool operator==(const Iterator &rhs) {
      return &container_ == &rhs.container_ && index_ == rhs.index_;
    }
    bool operator!=(const Iterator &rhs) { return !(*this == rhs); }

   private:
    bool GetValue() const {
      assert(index_ < container_.size());
      return container_.Get(index_);
    }
    BitVector &container_;
    size_t index_;
  };

  Iterator operator[](size_t index) { return Iterator(*this, index); }

  Iterator begin() { return Iterator(*this, 0); }
  Iterator end() { return Iterator(*this, size()); }

 protected:
  std::vector<Word> words_;
  size_t count_;
};

template <typename IdReferredType>
class PairedBitVector : public BitVector {
 public:
  PairedBitVector() : BitVector() {}
  explicit PairedBitVector(std::vector<uint32_t> words, size_t count)
      : BitVector(std::move(words), count) {}

  using Iterator = BitVector::Iterator;
  template <typename C>
  void Pair(const C &container, bool default_value = false) {
    static_assert(
        std::is_same<std::remove_const_t<
                         std::remove_reference_t<decltype(container.front())>>,
                     std::remove_const_t<IdReferredType>>::value,
        "container has wrong type");
    Resize(container.size(), default_value);
  }

  template <typename I>
  Iterator operator[](I id) {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Incompatible Id types");
    return Iterator(*this, static_cast<typename I::ValueType>(id));
  }
  Iterator operator[](size_t index) = delete;
  bool Get(size_t index) const = delete;
  void Set(size_t index, bool value) = delete;
  template <typename I>
  bool Get(I id) const {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Incompatible Id types");
    return BitVector::Get(static_cast<typename I::ValueType>(id));
  }
  template <typename I>
  void Set(I id, bool value = true) {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Incompatible Id types");
    BitVector::Set(static_cast<typename I::ValueType>(id), value);
  }
  template <typename I>
  I FindFirstWithValue(bool value) const {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Incompatible Id types");
    return I{static_cast<typename I::ValueType>(
        BitVector::FindFirstWithValue(value))};
  }
  template <typename I, typename F>
  void ForEachBit(F &&f, bool set_bits = true) const noexcept {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Incompatible Id types");
    BitVector::ForEachBit(
        [f = std::forward<F>(f)](size_t bit_index) {
          f(I{static_cast<typename I::ValueType>(bit_index)});
        },
        set_bits);
  }
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_BIT_VECTOR_H_
