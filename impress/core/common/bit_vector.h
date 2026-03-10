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
#include <limits>
#include <type_traits>
#include <vector>

#include "absl/numeric/bits.h"
#include "absl/types/span.h"

namespace imp {
namespace details {

constexpr size_t FloorLog2(size_t x) {
  return x == 1 ? 0 : 1 + FloorLog2(x >> 1);
}

constexpr size_t CeilLog2(size_t x) {
  return x == 1 ? 0 : FloorLog2(x - 1) + 1;
}

}  // namespace details

template <typename WordType>
class BasicBitVector {
  using Word = WordType;
  static constexpr size_t kWordBits = sizeof(Word) * CHAR_BIT;
  static constexpr size_t kWordShift = details::CeilLog2(kWordBits);
  static constexpr size_t kWordMask = (1 << kWordShift) - 1;

 public:
  using value_type = bool;

  BasicBitVector() : words_(), count_(0) {}

  explicit BasicBitVector(std::vector<Word> words, size_t count)
      : words_(std::move(words)), count_(count) {}

  absl::Span<const Word> Words() const { return absl::MakeSpan(words_); }

  void Resize(int size) {
    int word_count = size ? (((size - 1) >> kWordShift) + 1) : 0;
    words_.resize(word_count, Word{0});
    count_ = size;
  }

  // Finds the index of the first bit with the specified `value` in the range
  // [start_index, end_index).
  //
  // Returns:
  // - The index of the first matching bit if found within [start_index, limit),
  //   where limit = min(end_index, count_).
  // - `limit` if no such bit is found.
  //
  // Notes:
  // - The range is half-open: `start_index` is inclusive, `end_index` is
  //   exclusive.
  // - `end_index` is clamped to `count_` (the size of the bit vector).
  size_t FindFirstWithValue(
      bool value, size_t start_index = 0,
      size_t end_index = std::numeric_limits<size_t>::max()) const {
    const size_t limit = std::min(end_index, count_);
    if (start_index >= limit) {
      return limit;
    }

    size_t word_i = start_index >> kWordShift;
    const size_t bit_i = start_index & kWordMask;
    const size_t end_word_i = (limit == 0) ? 0 : ((limit - 1) >> kWordShift);

    // 1. Test bits in the first partial word: words_[word_i].
    // We are interested in bits from start_index onwards.
    Word bits_to_consider = value ? words_[word_i] : ~words_[word_i];
    // Mask to only consider bits >= bit_i.
    // Example: start_index=5, bit_i=5. Mask = 111...11100000
    bits_to_consider &= (~Word{0} << bit_i);

    // If word_i is also the *last* word we are interested in, we must
    // additionally mask off any bits that are >= limit, because they are
    // outside the requested range [start_index, limit).
    if (word_i == end_word_i) {
      const size_t bits_in_last_word = limit & kWordMask;
      // If bits_in_last_word is 0, it means count_ is a multiple of kWordBits,
      // so all bits 0..31 in this word are valid and no masking is needed.
      // Otherwise, we create a mask for only the valid bits, e.g., if
      // bits_in_last_word = 5, we want bits 0,1,2,3,4, so mask = 0b11111.
      if (bits_in_last_word) {
        bits_to_consider &= (Word{1} << bits_in_last_word) - 1;
      }
    }

    // If any bits were found in the first word checked, ctz finds the
    // lowest bit. Return its global index.
    if (bits_to_consider) {
      return (word_i << kWordShift) + absl::countr_zero(bits_to_consider);
    }

    // If we only needed to check one word and we didn't find it, we're done.
    if (word_i == end_word_i) {
      return limit;
    }

    // 2. Test bits in subsequent full words: words_[word_i + 1, ...,
    // end_word_i].
    for (++word_i; word_i <= end_word_i; ++word_i) {
      bits_to_consider = value ? words_[word_i] : ~words_[word_i];
      // If this is the last word we are checking, mask off bits >= limit as
      // above.
      if (word_i == end_word_i) {
        const size_t bits_in_last_word = limit & kWordMask;
        if (bits_in_last_word) {
          bits_to_consider &= (Word{1} << bits_in_last_word) - 1;
        }
      }
      if (bits_to_consider) {
        return (word_i << kWordShift) + absl::countr_zero(bits_to_consider);
      }
    }

    // No bits found in any words from start_index to limit.
    return limit;
  }

  bool Get(size_t index) const {
    const size_t word_index = index >> kWordShift;
    const size_t bit_index = index & kWordMask;
    const auto bit_mask = static_cast<Word>(Word{1} << bit_index);
    return !!(words_.at(word_index) & bit_mask);
  }

  void Set(size_t index, bool value = true) {
    const size_t word_index = index >> kWordShift;
    const size_t bit_index = index & kWordMask;
    const Word bit_mask = Word{1} << bit_index;
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
          value ? static_cast<Word>((Word{1} << partial_count) - 1) : Word{0};
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
      auto partial_mask = static_cast<Word>((Word{1} << partial_count) - 1);
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
        size_t lowest_set_bit = absl::countr_zero(word);
        word &= ~(Word{1} << lowest_set_bit);
        f(size_t((i << kWordShift) + lowest_set_bit));
      }
    }
    if (partial_count) {
      auto partial_mask = static_cast<Word>((Word{1} << partial_count) - 1);
      Word word = (set_bits ? words_[i] : ~words_[i]) & partial_mask;
      while (word) {
        size_t lowest_set_bit = absl::countr_zero(word);
        word &= ~(Word{1} << lowest_set_bit);
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

    explicit Iterator(BasicBitVector& container, size_t index = 0)
        : container_(container), index_(index) {}
    bool operator*() const { return GetValue(); }
    explicit operator bool() const { return GetValue(); }
    Iterator& operator=(bool value) {
      container_.Set(index_, value);
      return *this;
    }
    Iterator& operator++() {
      ++index_;
      return *this;
    }
    Iterator operator++(int) {
      Iterator ret = *this;
      ++index_;
      return ret;
    }
    bool operator==(const Iterator& rhs) {
      return &container_ == &rhs.container_ && index_ == rhs.index_;
    }
    bool operator!=(const Iterator& rhs) { return !(*this == rhs); }

   private:
    bool GetValue() const {
      assert(index_ < container_.size());
      return container_.Get(index_);
    }
    BasicBitVector& container_;
    size_t index_;
  };

  Iterator operator[](size_t index) { return Iterator(*this, index); }

  Iterator begin() { return Iterator(*this, 0); }
  Iterator end() { return Iterator(*this, size()); }

 protected:
  std::vector<Word> words_;
  size_t count_;
};

using BitVector = BasicBitVector<uint32_t>;
using BitVector64 = BasicBitVector<size_t>;

template <typename WordType, typename IdReferredType>
class BasicPairedBitVector : public BasicBitVector<WordType> {
 public:
  using Base = BasicBitVector<WordType>;
  BasicPairedBitVector() : Base() {}
  explicit BasicPairedBitVector(std::vector<WordType> words, size_t count)
      : Base(std::move(words), count) {}

  using Iterator = typename Base::Iterator;
  template <typename C>
  void Pair(const C& container, bool default_value = false) {
    static_assert(
        std::is_same<std::remove_const_t<
                         std::remove_reference_t<decltype(container.front())>>,
                     std::remove_const_t<IdReferredType>>::value,
        "container has wrong type");
    this->Resize(container.size());
    this->SetAll(default_value);
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
    return Base::Get(static_cast<typename I::ValueType>(id));
  }
  template <typename I>
  void Set(I id, bool value = true) {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Incompatible Id types");
    Base::Set(static_cast<typename I::ValueType>(id), value);
  }
  template <typename I>
  I FindFirstWithValue(bool value) const {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Incompatible Id types");
    return I{
        static_cast<typename I::ValueType>(Base::FindFirstWithValue(value))};
  }
  template <typename I, typename F>
  void ForEachBit(F&& f, bool set_bits = true) const noexcept {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Incompatible Id types");
    Base::ForEachBit(
        [f = std::forward<F>(f)](size_t bit_index) {
          f(I{static_cast<typename I::ValueType>(bit_index)});
        },
        set_bits);
  }
};

template <typename IdReferredType>
using PairedBitVector = BasicPairedBitVector<uint32_t, IdReferredType>;

template <typename IdReferredType>
using PairedBitVector64 = BasicPairedBitVector<size_t, IdReferredType>;

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_BIT_VECTOR_H_
