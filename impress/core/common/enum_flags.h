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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_ENUM_FLAGS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_ENUM_FLAGS_H_

#include <initializer_list>
#include <tuple>
#include <type_traits>

namespace imp {

// Defines a type that implements C-style bit flag semantics using scoped enums.
// It also works with unscoped enums, but they provide less type safety.
template <typename E>
class Flags {
  using StorageType = typename std::underlying_type<E>::type;

 public:
  constexpr Flags() : storage_(0) {}
  constexpr explicit Flags(E value) : storage_(AsStorage(value)) {}
  bool Test(E value) const {
    return (storage_ & AsStorage(value)) == AsStorage(value);
  }
  bool AnyOf(E value) const { return (storage_ & AsStorage(value)) > 0; }
  void Set(E value, bool set = true) {
    storage_ = set ? storage_ | AsStorage(value) : storage_ & ~AsStorage(value);
  }
  explicit operator bool() const { return storage_ != 0; }
  E Value() const { return static_cast<E>(storage_); }
  StorageType RawValue() const { return storage_; }

  friend Flags operator|(Flags lhs, E rhs) {
    return Flags(lhs.storage_ | AsStorage(rhs));
  }
  friend Flags operator|(Flags lhs, Flags rhs) {
    return Flags(lhs.storage_ | rhs.storage_);
  }
  friend Flags& operator|=(Flags& lhs, Flags rhs) {
    lhs.storage_ |= rhs.storage_;
    return lhs;
  }
  friend Flags& operator|=(Flags& lhs, E rhs) {
    lhs.storage_ |= AsStorage(rhs);
    return lhs;
  }
  friend Flags operator&(Flags lhs, E rhs) {
    return Flags(lhs.storage_ & AsStorage(rhs));
  }
  friend Flags operator&(Flags lhs, Flags rhs) {
    return Flags(lhs.storage_ & rhs.storage_);
  }
  friend Flags& operator&=(Flags& lhs, Flags rhs) {
    lhs.storage_ &= rhs.storage_;
    return lhs;
  }
  friend Flags& operator&=(Flags& lhs, E rhs) {
    lhs.storage_ &= AsStorage(rhs);
    return lhs;
  }
  friend bool operator==(const Flags& lhs, const Flags& rhs) {
    return lhs.storage_ == rhs.storage_;
  }
  friend bool operator!=(const Flags& lhs, const Flags& rhs) {
    return lhs.storage_ != rhs.storage_;
  }
  // We cannot define unary negation for enums; Instead an enum can be wrapped
  // via ToFlags, and the resultant Flags<> object supports unary negation.
  friend Flags operator~(const Flags& in) { return Flags(~in.storage_); }

 private:
  static StorageType AsStorage(E in) { return static_cast<StorageType>(in); }
  explicit Flags(StorageType storage) : storage_(storage) {}

  StorageType storage_;
};

namespace internal {
template <typename... E>
using FirstType = std::tuple_element_t<0, std::tuple<E...>>;
template <class E, class... Es>
constexpr void OrFlags(Flags<E>& accum, E next, Es... rest) {
  accum |= Flags<E>(next);
  if constexpr (sizeof...(rest) > 0) {
    OrFlags(accum, rest...);
  }
}
}  // namespace internal

template <class... Es>
constexpr Flags<internal::FirstType<Es...>> ToFlags(Es... in) {
  using EnumType = std::tuple_element_t<0, std::tuple<Es...>>;
  Flags<EnumType> result;
  internal::OrFlags<EnumType>(result, in...);
  return result;
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_ENUM_FLAGS_H_
