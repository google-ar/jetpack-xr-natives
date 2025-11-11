// Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_OPTIONAL_WITH_DEFAULT_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_OPTIONAL_WITH_DEFAULT_H_

#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

namespace imp {

namespace imp_internal {
template <typename T>
struct OptionalWithDefaultAccessTypes {
  using RefT = const T&;
  using PtrT = const T*;
};

template <>
struct OptionalWithDefaultAccessTypes<std::string> {
  using RefT = std::string_view;
  using PtrT = const std::string_view*;
};

template <typename T>
struct AdditionalFields {};

template <>
struct AdditionalFields<std::string> {
  // Used so that operator-> can return a non-temporary string_view.
  mutable std::string_view cached_view;
};
}  // namespace imp_internal

// The class  manages an optional contained value, i.e. a value that may or may
// not be present.
//
// It is similar to std::optional, except that it provides a default value for
// the case where the value is not set.
//
// The default value is provided via a template parameter to a
// const pointer to ensure that the default value is a statically known value at
// compile time.
//
// Example:
//
// static constexpr float kMyFloatDefault = 10.0f;
// using MyFloat = OptionalWithDefault<float, &kMyFloatDefault>;
//
// This class includes some specialization for the type std::string for
// proper compatibility with std::string_view.
//
// Example:
//
// static constexpr std::string_view kMyStringDefault = "Hello";
// using MyString = OptionalWithDefault<std::string, &kMyStringDefault>;
//
// In this case, Value() and operator* will return std::string_view instead of
// const std::string&. operator-> will return const std::string_view* instead of
// const std::string*.
//
// Note: The default value must outlive the OptionalWithDefault object. Given
// that the default value is expected to be statically known, this is rarely an
// issue.
template <typename T, const auto* DefaultValuePointer>
class OptionalWithDefault {
 public:
  // const T& or std::string_view if T is std::string.
  using AccessRefT =
      typename imp_internal::OptionalWithDefaultAccessTypes<T>::RefT;
  // const T* or const std::string_view* if T is std::string.
  using AccessPtrT =
      typename imp_internal::OptionalWithDefaultAccessTypes<T>::PtrT;

  // Constructs an an empty optional, i.e. the value is not set and accessing
  // the optional will return the default value.
  constexpr OptionalWithDefault() noexcept = default;

  // Constructs an optional with the given value that can be converted to type
  // T.
  //
  // Note: Follows the same implicit conversion rules as std::optional to allow
  // for transparently wrapping values that can be assigned to the contained
  // type, making this type usable for aggregate types and other APIs.
  template <typename U = std::remove_cv_t<T>,
            std::enable_if_t<std::is_convertible_v<U, T>, bool> = true>
  constexpr OptionalWithDefault(U&& value);

  // Constructs an empty optional, i.e. the value is not set and accessing the
  // optional will return the default value.
  //
  // Note: Follows the same implicit conversion rules as std::optional to allow
  // for transparently wrapping values that can be assigned to the contained
  // type, making this type usable for aggregate types and other APIs.
  constexpr OptionalWithDefault(std::nullopt_t) noexcept;

  // Assigns the optional with the given value that can be converted to type T.
  //
  // Note: Follows the same implicit conversion rules as std::optional to allow
  // for transparently wrapping values that can be assigned to the contained
  // type, making this type usable for aggregate types and other APIs.
  template <typename U = std::remove_cv_t<T>,
            std::enable_if_t<std::is_convertible_v<U, T>, bool> = true>
  OptionalWithDefault& operator=(U&& value);

  // Assigns empty to the optional, i.e. the value is not set and accessing the
  // optional will return the default value.
  //
  // Similar to calling Reset().
  OptionalWithDefault& operator=(std::nullopt_t) noexcept;

  // Returns true if a value is assigned to the optional.
  //
  // If false, then accessing the optional will return the default value.
  constexpr explicit operator bool() const noexcept;

  // Returns true if a value is assigned to the optional.
  //
  // If false, then accessing the optional will return the default value.
  constexpr bool HasValue() const noexcept;

  // Returns the value of the optional, or the default value if the value is
  // not set.
  //
  // Unlike std::optional, this is always safe to call because in the case
  // where the value is not set, the default value is returned. It is like the
  // equivalent of calling value_or(kDefaultValue) in std::optional.
  //
  // Note: The returned value is never mutable because it is not allowed to
  // mutate the default value. Use GetMutable() to modify the assigned value.
  AccessRefT Value() const;

  // Clears the assigned value if there is one, making the optional empty and
  // causing it to return the default value when accessed.
  void Reset();

  // Returns a reference to the value of the optional.
  //
  // If the value is not set, then the default value is returned.
  //
  // This is always safe to call even if HasValue is false, because it will
  // return the default value in that case.
  //
  // Note: The returned value is never mutable because it is not allowed to
  // mutate the default value. Use GetMutable() to modify the assigned value.
  AccessRefT operator*() const;

  // Returns a pointer to the value of the optional.
  //
  // If the value is not set, then the default value is returned.
  //
  // This is always safe to call even if HasValue is false, because it will
  // return the default value in that case.
  //
  // Note: This pointer may be invalidated if the optional is reassigned,
  // mutated, or if Reset is called. Do not store this pointer for later use.
  AccessPtrT operator->() const;

  // Gets a mutable pointer to the value of the optional. This allows the value
  // to be modified in place.
  //
  // If the value is not set, then nullptr is returned.
  T* GetMutable();

  // Returns a reference to the default value.
  static constexpr AccessRefT GetDefaultValue();

 private:
  // Note: default copy/move semantics are used so that the behavior matches the
  // default behavior of std::optional.
  std::optional<T> value_;

  // Used for std::string specialization.
  imp_internal::AdditionalFields<T> additional_fields_;
};

template <typename T, const auto* DefaultValuePointer>
template <typename U, std::enable_if_t<std::is_convertible_v<U, T>, bool>>
constexpr OptionalWithDefault<T, DefaultValuePointer>::OptionalWithDefault(
    U&& value)
    : value_(std::forward<U>(value)) {}

template <typename T, const auto* DefaultValuePointer>
constexpr OptionalWithDefault<T, DefaultValuePointer>::OptionalWithDefault(
    std::nullopt_t) noexcept
    : value_(std::nullopt) {}

template <typename T, const auto* DefaultValuePointer>
template <typename U, std::enable_if_t<std::is_convertible_v<U, T>, bool>>
OptionalWithDefault<T, DefaultValuePointer>&
OptionalWithDefault<T, DefaultValuePointer>::operator=(U&& value) {
  value_ = std::forward<U>(value);
  return *this;
}

template <typename T, const auto* DefaultValuePointer>
OptionalWithDefault<T, DefaultValuePointer>&
OptionalWithDefault<T, DefaultValuePointer>::operator=(
    std::nullopt_t) noexcept {
  value_ = std::nullopt;
  return *this;
}

template <typename T, const auto* DefaultValuePointer>
constexpr OptionalWithDefault<T, DefaultValuePointer>::operator bool()
    const noexcept {
  return HasValue();
}

template <typename T, const auto* DefaultValuePointer>
constexpr bool OptionalWithDefault<T, DefaultValuePointer>::HasValue()
    const noexcept {
  return value_.has_value();
}

template <typename T, const auto* DefaultValuePointer>
OptionalWithDefault<T, DefaultValuePointer>::AccessRefT
OptionalWithDefault<T, DefaultValuePointer>::Value() const {
  return HasValue() ? value_.value() : GetDefaultValue();
}

template <typename T, const auto* DefaultValuePointer>
void OptionalWithDefault<T, DefaultValuePointer>::Reset() {
  value_.reset();
}

template <typename T, const auto* DefaultValuePointer>
OptionalWithDefault<T, DefaultValuePointer>::AccessRefT
OptionalWithDefault<T, DefaultValuePointer>::operator*() const {
  return Value();
}

template <typename T, const auto* DefaultValuePointer>
OptionalWithDefault<T, DefaultValuePointer>::AccessPtrT
OptionalWithDefault<T, DefaultValuePointer>::operator->() const {
  if constexpr (std::is_same_v<T, std::string>) {
    if (HasValue()) {
      // Update the cached_view so that it points to the current value.
      // This is necessary so that this method doesn't return a temporary.
      //
      // Note, we intentionally do this just in time, and don't keep it up to
      // date for each operation. Instead, we just document that this pointer
      // can be invalidated. Even if we did update this in assignment operators
      // and constructors, we couldn't keep it up to date for modifications done
      // via GetMutable().
      additional_fields_.cached_view = value_.value();
      return &additional_fields_.cached_view;
    } else {
      return DefaultValuePointer;
    }
  } else {
    return &Value();
  }
}

template <typename T, const auto* DefaultValuePointer>
T* OptionalWithDefault<T, DefaultValuePointer>::GetMutable() {
  return value_.has_value() ? &value_.value() : nullptr;
}

template <typename T, const auto* DefaultValuePointer>
constexpr OptionalWithDefault<T, DefaultValuePointer>::AccessRefT
OptionalWithDefault<T, DefaultValuePointer>::GetDefaultValue() {
  return *DefaultValuePointer;
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_OPTIONAL_WITH_DEFAULT_H_
