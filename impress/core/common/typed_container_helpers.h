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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_TYPED_CONTAINER_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_TYPED_CONTAINER_HELPERS_H_

#include <cassert>
#include <cstddef>
#include <type_traits>

#include "absl/algorithm/container.h"
#include "absl/meta/type_traits.h"
#include "absl/types/span.h"
#include "core/common/data_helpers.h"
#include "core/common/typed_id.h"

namespace imp {

// Helper templates designed for CRTP usage
// (see: (broken link)).

// Make a derived class behave like a standard c++ container (e.g. std::vector)
// - Ranged-for compatibility (e.g. begin(), end())
// - Convenience/expectation (e.g. empty(), resize(), reserve())
// - <algorithm> and absl/algorithm/container compatibility (e.g. push_back())
// - Does _not_ define index operators; those are provided elsewhere.
// For use with a derived class with a private field container_.
template <class Derived, typename Container>
class TGenericContainerMethods {
 public:
  using iterator = typename Container::iterator;
  using const_iterator = typename Container::const_iterator;
  using value_type = typename Container::value_type;

  const_iterator begin() const {
    const auto& derived = *static_cast<const Derived*>(this);
    return derived.container_.begin();
  }
  iterator begin() {
    auto& derived = *static_cast<Derived*>(this);
    return derived.container_.begin();
  }
  const_iterator end() const {
    const auto& derived = *static_cast<const Derived*>(this);
    return derived.container_.end();
  }
  iterator end() {
    auto& derived = *static_cast<Derived*>(this);
    return derived.container_.end();
  }
  value_type* data() {
    Derived& derived = *static_cast<Derived*>(this);
    return derived.container_.data();
  }
  const value_type* data() const {
    const Derived& derived = *static_cast<const Derived*>(this);
    return derived.container_.data();
  }
  size_t size() const {
    const Derived& derived = *static_cast<const Derived*>(this);
    return derived.container_.size();
  }
  void resize(size_t new_size, value_type default_value = {}) {
    Derived& derived = *static_cast<Derived*>(this);
    return derived.container_.resize(new_size, default_value);
  }
  bool empty() const { return !size(); }
  const value_type& front() const {
    auto& derived = *static_cast<const Derived*>(this);
    return derived.container_.front();
  }
  value_type& front() {
    auto& derived = *static_cast<Derived*>(this);
    return derived.container_.front();
  }
  const value_type& back() const {
    auto& derived = *static_cast<const Derived*>(this);
    return derived.container_.back();
  }
  value_type& back() {
    Derived& derived = *static_cast<Derived*>(this);
    return derived.container_.back();
  }
  void push_back(value_type&& value) {
    Derived& derived = *static_cast<Derived*>(this);
    return derived.container_.push_back(std::move(value));
  }
  void push_back(const value_type& value) {
    Derived& derived = *static_cast<Derived*>(this);
    return derived.container_.push_back(value);
  }
  void pop_back() {
    Derived& derived = *static_cast<Derived*>(this);
    return derived.container_.pop_back();
  }
  void clear() {
    Derived& derived = *static_cast<Derived*>(this);
    derived.container_.clear();
  }
  void reserve(size_t new_reserve) {
    Derived& derived = *static_cast<Derived*>(this);
    derived.container_.reserve(new_reserve);
  }
  template <class... Args>
  void emplace_back(Args&&... args) {
    Derived& derived = *static_cast<Derived*>(this);
    derived.container_.emplace_back(std::forward<Args>(args)...);
  }
};

// Typed Containers (currently TypedVector and TypedView) provide type safety
// in a sea of indices.  Index-based Id types (TypedId) are declared for a given
// type T, and TTypedContainerMethods allow for containers of type T which are
// only indexable with a compatible TypedId.
template <template <typename T> class D, typename T>
class TTypedContainerMethods {
  using Derived = D<T>;
  static constexpr bool kIsCopyable = absl::is_copy_assignable<T>::value;

 public:
  // Makes Derived indexable with TypedId's of our container's value type T.
  template <typename V>
  const T& operator[](TypedId<T, V> id) const {
    const auto& derived = *static_cast<const Derived*>(this);
    return derived.container_[static_cast<V>(id)];
  }

  // Non-const version of above.
  template <typename V>
  T& operator[](TypedId<T, V> id) {
    auto& derived = *static_cast<Derived*>(this);
    return derived.container_[static_cast<V>(id)];
  }

  T& operator[](size_t id) = delete;
  const T& operator[](size_t id) const = delete;
  
  // Given an element reference, conjure a valid TypedId to that element.
  template <typename I = TypedId<T, size_t>>
  I IdOf(const T& elem) const {
    static_assert(std::is_same_v<typename I::ReferredType, T>, "Wrong Id type");
    const auto& derived = *static_cast<const Derived*>(this);
    size_t elem_index = &elem - &derived.container_.front();
    assert(elem_index < derived.container_.size());
    return TypedId<T, typename I::ValueType>(
        static_cast<typename I::ValueType>(elem_index));
  }

  // Sanity check the provided TypedId `id` for use with this container.
  template <typename I>
  bool IsValid(I id) const {
    static_assert(std::is_same_v<typename I::ReferredType, T>, "Wrong Id type");
    const auto& derived = *static_cast<const Derived*>(this);
    return static_cast<typename I::ValueType>(id) < derived.size() &&
           ((!kIsSigned<typename I::ValueType>) ||
            static_cast<typename I::ValueType>(id) >= 0);
  }

  // Append() pushes and returns the id of the created item.
  // Disambiguate our two ways of Appending based on whether the element type
  // is copy-assignable.  If it is, push by const value; otherwise, by rvalue.
  template <typename I = TypedId<T, size_t>>
  typename std::enable_if<kIsCopyable, I>::type Append(
      const T default_value = T()) {
    static_assert(std::is_same_v<typename I::ReferredType, T>, "Wrong Id type");
    auto& derived = *static_cast<Derived*>(this);
    derived.container_.push_back(default_value);
    return I(this->template IdOf<I>(derived.container_.back()));
  }
  template <typename I = TypedId<T, size_t>>
  typename std::enable_if<!kIsCopyable, I>::type Append(T&& value) {
    static_assert(std::is_same_v<typename I::ReferredType, T>, "Wrong Id type");
    auto& derived = *static_cast<Derived*>(this);
    derived.container_.push_back(std::move(value));
    return I(this->template IdOf<I>(derived.container_.back()));
  }
};

template <typename IdReferredType,  //
          template <typename... T> class D, typename... T>
class TTypedIdProviderMethods {
  using Derived = D<T...>;

 public:
  // Declare a private dummy type designed for use in ranged-for expressions
  template <typename I>
  struct IdProvider {
    const Derived& derived;
  };

  // Return a dummy type which can be called by begin() and end()
  template <typename I = TypedId<IdReferredType, size_t>>
  IdProvider<I> Ids() const {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Wrong Id type");
    const auto& derived = *static_cast<const Derived*>(this);
    return IdProvider<I>{derived};
  }

  // Custom type used as the iterand in a ranged-for Ids() call.
  template <typename I, bool IsReversed = false>
  class IdIterator {
   public:
    // Construct with an ID
    explicit IdIterator(I value)
        : value_(static_cast<typename I::ValueType>(value)) {
      static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                    "Wrong Id type");
    }
    // De-reference returns the value as an ID
    I operator*() const {
      if constexpr (IsReversed) {
        return I(static_cast<typename I::ValueType>(value_ - 1));
      } else {
        return I(value_);
      }
    }

    bool operator==(const IdIterator<I, IsReversed>& other) const {
      return value_ == other.value_;
    }
    bool operator!=(const IdIterator<I, IsReversed>& other) const {
      return !(*this == other);
    }
    bool operator==(const I& other) const {
      return value_ == static_cast<typename I::ValueType>(other);
    }
    bool operator!=(const I& other) const { return !(*this == other); }
    IdIterator<I, IsReversed> operator++(int) {
      IdIterator<I, IsReversed> ret(
          I::At(static_cast<typename I::ValueType>(value_)));
      if constexpr (IsReversed) {
        --value_;
      } else {
        ++value_;
      }
      return ret;
    }
    IdIterator<I, IsReversed>& operator++() {
      if constexpr (IsReversed) {
        --value_;
      } else {
        ++value_;
      }
      return *this;
    }

   private:
    typename I::ValueType value_;
  };

  // Return a TypedId corresponding to the first indexable value (#0)
  template <typename I, bool IsReversed = false>
  inline IdIterator<I, IsReversed> BeginId() const {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Wrong Id type");
    return IdIterator<I, IsReversed>(I(static_cast<typename I::ValueType>(0)));
  }
  // Return a TypedId corresponding to the first non-indexable value (#size())
  template <typename I, bool IsReversed = false>
  inline IdIterator<I, IsReversed> EndId() const {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Wrong Id type");
    const auto& derived = *static_cast<const Derived*>(this);
    return IdIterator<I, IsReversed>(
        I(static_cast<typename I::ValueType>(derived.size())));
  }

  // Return an IdIterator at the first valid index.
  template <typename I>
  friend inline constexpr IdIterator<I> begin(const IdProvider<I>& provider) {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Wrong Id type");
    return provider.derived.template BeginId<I>();
  }
  // Return an IdIterator at the first invalid index.
  template <typename I>
  friend inline constexpr IdIterator<I> end(const IdProvider<I>& provider) {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Wrong Id type");
    return provider.derived.template EndId<I>();
  }

  // Return a reverse IdIterator at the first invalid index.
  template <typename I>
  friend inline constexpr IdIterator<I, true> rbegin(
      const IdProvider<I>& provider) {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Wrong Id type");
    return provider.derived.template EndId<I, true>();
  }
  // Return a reverse IdIterator at the first valid index.
  template <typename I>
  friend inline constexpr IdIterator<I, true> rend(
      const IdProvider<I>& provider) {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Wrong Id type");
    return provider.derived.template BeginId<I, true>();
  }
};

// Paired Containers (currently PairedVector and PairedSpan) are strongly
// coupled (paired) with a specific instance of a TypedId, and are only
// indexable by that index type.
template <template <typename T, typename IdReferredType> class D, typename T,
          typename IdReferredType>
class TPairedContainerMethods {
  using Derived = D<T, IdReferredType>;
  static constexpr bool kIsCopyable = absl::is_copy_assignable<T>::value;

 public:
  template <typename I>
  T& operator[](const I& index) {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Incompatible Id types");
    Derived& derived = *static_cast<Derived*>(this);
    return derived.container_[static_cast<typename I::ValueType>(index)];
  }
  template <typename I>
  const T& operator[](const I& index) const {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Incompatible Id types");
    const Derived& derived = *static_cast<const Derived*>(this);
    return derived.container_[static_cast<typename I::ValueType>(index)];
  }

  template <typename I>
  I IdOf(const T& elem) const {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Incompatible Id types");
    const Derived& derived = *static_cast<const Derived*>(this);

    size_t elem_index = &elem - &derived.container_.front();
    assert(elem_index < derived.container_.size());
    return I(static_cast<typename I::ValueType>(elem_index));
  }

  // Sanity check the provided TypedId `id` for use with this container.
  template <typename I>
  bool IsValid(const I& id) const {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Incompatible Id types");
    const auto& derived = *static_cast<const Derived*>(this);
    return static_cast<typename I::ValueType>(id) < derived.container_.size() &&
           ((!kIsSigned<typename I::ValueType>) ||
            static_cast<typename I::ValueType>(id) >= 0);
  }
  // Append() pushes and returns the id of the created item.
  // Disambiguate our two ways of Appending based on whether the element type
  // is copy-assignable.  If it is, push by const value; otherwise, by rvalue.
  template <typename I = TypedId<T, size_t>>
  typename std::enable_if<kIsCopyable, I>::type Append(
      const T default_value = T()) {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Incompatible Id types");
    auto& derived = *static_cast<Derived*>(this);
    derived.container_.push_back(default_value);
    return I(this->template IdOf<I>(derived.container_.back()));
  }
  template <typename I = TypedId<T, size_t>>
  typename std::enable_if<!kIsCopyable, I>::type Append(T&& value) {
    static_assert(std::is_same_v<typename I::ReferredType, IdReferredType>,
                  "Incompatible Id types");
    auto& derived = *static_cast<Derived*>(this);
    derived.container_.push_back(std::move(value));
    return I(this->template IdOf<I>(derived.container_.back()));
  }
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_TYPED_CONTAINER_HELPERS_H_
