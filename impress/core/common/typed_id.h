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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_TYPED_ID_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_TYPED_ID_H_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <type_traits>
#include <typeinfo>

#include "absl/hash/hash.h"
#include "core/common/log.h"
#include "absl/strings/str_format.h"
#include "core/common/data_helpers.h"
#include "core/common/platform_helpers.h"
#include "core/common/type_traits.h"

namespace imp {

// TypedId, and the container types that use it (TypedVector, PairedSpan, &c)
// are a means to have strongly typed indices.  We refer to them as Id's partly
// for brevity, and partly to telegraph that the boolean operator is a validity
// check (i.e. it can return 'true' for the value 0, and 'false' for the value
// -1).

// A TypedId<T,V> is an index-style reference (represented as a V) to a T.
// Default construction is only available for signed value types, and defaults
// to a sentinel of -1.  When aliased via a using statement, it establishes that
// e.g. a `ThingId` is a holdable, possibly nullable reference to a `Thing`.
template <class T, class V = int32_t>
class TypedId {
  // Defined for all types because c++14 doesn't have constexpr-if
  static constexpr V kSentinelValue = static_cast<V>(-1);

 public:
  using ReferredType = T;
  using ValueType = V;
  template <bool B = kIsSigned<V>, typename std::enable_if<B, V>::type = 0>
  constexpr TypedId() noexcept : val_(Sentinel()) {}
  // We can construct with our storage type.  Use enable_if to force the
  // provided value to have _literally_ our value type.  This prevents e.g. a
  // literal that exceeds the storage size from being silently truncated.
  template <typename VOther,
            std::enable_if_t<std::is_same<V, VOther>{}, int> = 0>
  explicit constexpr TypedId(VOther val) noexcept : val_(val) {
    if constexpr (kIsSigned<V>) {
      assert(val_ >= 0 || val_ == kSentinelValue);
    }
  }
  // We can construct with a TypedId with matching T but different V
  template <typename VOther>
  constexpr TypedId(
      TypedId<T, VOther> val) noexcept  // NOLINT(runtime/explicit)
      : val_(static_cast<V>(static_cast<VOther>(val))) {
    assert(static_cast<VOther>(val) <= kMaxValue<V>);
    if constexpr (kIsSigned<V>) {
      assert(val_ >= 0 || val_ == kSentinelValue);
    }
  }
  // We can construct with a size_t via the At() method (to disambiguate from
  // the constructor that takes the explicit value type).
  static TypedId At(size_t val) {
    // Ensure we were constructed with a legitimate value.
    assert(val <= kMaxValue<V> ||
           (kIsSigned<V> && val == static_cast<size_t>(-1)));
    return (kIsSigned<V> && val == static_cast<size_t>(-1))
               ? TypedId{kSentinelValue}
               : TypedId{static_cast<V>(val)};
  }
  explicit constexpr operator V() const noexcept { return val_; }
  friend constexpr bool operator==(TypedId a, TypedId b) {
    return a.val_ == b.val_;
  }
  friend constexpr bool operator!=(TypedId a, TypedId b) {
    return a.val_ != b.val_;
  }
  template <typename VOther>
  friend constexpr bool operator==(TypedId a, TypedId<T, VOther> b) {
    if constexpr (kMaxValue < V >> kMaxValue<VOther>)
      return a.val_ == static_cast<V>(static_cast<VOther>(b));
    else
      return static_cast<VOther>(a.val_) == static_cast<VOther>(b);
  }
  template <typename VOther>
  friend constexpr bool operator!=(TypedId a, TypedId b) {
    if constexpr (kMaxValue < V >> kMaxValue<VOther>)
      return a.val_ != static_cast<V>(static_cast<VOther>(b));
    else
      return static_cast<VOther>(a.val_) != static_cast<VOther>(b);
  }

  constexpr TypedId& operator+=(size_t offset) noexcept {
    val_ += offset;
    return *this;
  }
  constexpr TypedId& operator-=(size_t offset) noexcept {
    val_ -= offset;
    return *this;
  }
  friend inline constexpr TypedId operator+(TypedId lv,
                                            size_t offset) noexcept {
    // We pass by value since we're returning a copy anyway
    return lv += offset;
  }
  friend inline constexpr TypedId operator-(TypedId lv,
                                            size_t offset) noexcept {
    // We pass by value since we're returning a copy anyway
    return lv -= offset;
  }
  friend inline constexpr TypedId operator++(TypedId& lv) noexcept {
    return lv += 1;
  }
  // Only TypedIndices with signed storage can store an invalid value.
  explicit operator bool() const noexcept {
    return kIsSigned<V> ? val_ != kSentinelValue : true;
  }

  // Pending the need for more draconian measures, support casting to different
  // Id types.  This should obviously be used with care, and is intended for
  // transferring id's between different systems (e.g. each EntityInfo will
  // match 1-1 with EntityData, so allow EntityInfoId to cast into EntityId).
  template <typename I>
  constexpr I CastTo() const {
    using VOther = typename I::ValueType;
    if (val_ >= 0 && val_ > kMaxValue<VOther>) {
      IMP_LOG(imp::FATAL) << "ID value " << val_ << " cannot fit in " << sizeof(I)
                 << " bytes";
    }
    if (!kIsSigned<typename I::ValueType>) {
      assert(val_ >= 0);
    }
    return I(static_cast<typename I::ValueType>(val_));
  }

  template <typename H>
  friend H AbslHashValue(H h, const TypedId<T, V>& id) {
    return H::combine(std::move(h), id.val_);
  }

 protected:
  friend struct std::hash<TypedId<T, V>>;
  V val_;
  // Sentinel function is only defined for signed id types.
  template <bool B = kIsSigned<V>, typename std::enable_if<B, V>::type = 0>
  static constexpr V Sentinel() {
    return kSentinelValue;
  }
};

// A specialization of TypedId that uses a specific unsigned sentinel value
// instead of -1.  It is primarily used for adjacency information, e.g. since
// children must appear after parents, index 0 cannot be a valid child index,
// and is used as a sentinel for children indices (similarly, kMaxValue<V>
// cannot be a valid parent index, and is used as a sentinel for parent
// indices).
template <class T, class V,
          uint64_t kSentinel = static_cast<size_t>(kMaxValue<V>)>
class TypedIdWithSentinel : public TypedId<T, V> {
  // Otherwise, we would have two sentinel values.
  static_assert(!kIsSigned<V>, "TypedIdWithSentinel is for unsigned integers");
  static constexpr V kSentinelValue = static_cast<V>(kSentinel);

 public:
  TypedIdWithSentinel() : TypedId<T, V>(kSentinelValue) {}
  template <typename VOther,
            std::enable_if_t<std::is_same<V, VOther>{}, int> = 0>
  explicit constexpr TypedIdWithSentinel(VOther val) : TypedId<T, V>(val) {}
  explicit constexpr TypedIdWithSentinel(TypedId<T, V> rhs)
      : TypedId<T, V>(static_cast<V>(rhs)) {
    assert(static_cast<V>(rhs) != kSentinelValue);
  }
  // We can construct with a TypedId with matching T but different V
  template <typename VOther>
  constexpr TypedIdWithSentinel(
      TypedId<T, VOther> val)  // NOLINT(runtime/explicit)
      : TypedId<T, V>(static_cast<V>(static_cast<VOther>(val))) {
    assert(static_cast<VOther>(val) <= kMaxValue<V>);
  }

  // We can construct with a size_t via the At() method (to disambiguate from
  // the constructor that takes the explicit value type).
  static TypedIdWithSentinel At(size_t val) {
    assert(static_cast<V>(val) != kSentinelValue);
    return TypedIdWithSentinel<T, V, kSentinelValue>(static_cast<V>(val));
  }

  TypedIdWithSentinel& operator=(TypedId<T, V> rhs) {
    assert(static_cast<V>(rhs) != kSentinelValue);
    this->val_ = static_cast<V>(rhs);
    return *this;
  }
  explicit operator V() const { return this->val_; }
  explicit operator bool() const noexcept {
    return this->val_ != kSentinelValue;
  }
};

template <class T>
std::string ToString(TypedId<T, uint8_t> id) {
  constexpr auto type_name = type_traits::GetTypeName<T>();
  return absl::StrFormat("%sId@%hx", type_name, static_cast<uint8_t>(id));
}
template <class T>
std::string ToString(TypedId<T, uint16_t> id) {
  constexpr auto type_name = type_traits::GetTypeName<T>();
  return absl::StrFormat("%sId@%hx", type_name, static_cast<uint16_t>(id));
}
template <class T>
std::string ToString(TypedId<T, uint32_t> id) {
  constexpr auto type_name = type_traits::GetTypeName<T>();
  return absl::StrFormat("%sId@%x", type_name, static_cast<uint32_t>(id));
}
// Guarded because otherwise size_t and uint32_t are the same type.
#ifdef _LP64
template <class T>
std::string ToString(TypedId<T, size_t> id) {
  constexpr auto type_name = type_traits::GetTypeName<T>();
  return absl::StrFormat("%sId@%lx", type_name, static_cast<size_t>(id));
}
#endif  // _LP64
template <class T>
std::string ToString(TypedId<T, int16_t> id) {
  constexpr auto type_name = type_traits::GetTypeName<T>();
  return absl::StrFormat("%sId@%hd", type_name, static_cast<int16_t>(id));
}
template <class T>
std::string ToString(TypedId<T, int32_t> id) {
  constexpr auto type_name = type_traits::GetTypeName<T>();
  return absl::StrFormat("%sId@%d", type_name, static_cast<int32_t>(id));
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_TYPED_ID_H_
