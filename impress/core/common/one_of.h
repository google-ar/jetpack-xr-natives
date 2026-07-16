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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_ONE_OF_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_ONE_OF_H_

#include <string>
#include <type_traits>
#include <utility>
#include <variant>

#include "core/common/optional_with_default.h"

namespace imp {

namespace imp_internal {

// Uses SFINAE to default HasDefaultConstant to false.
template <typename Tag, typename = void>
struct HasDefaultConstant : std::false_type {};

// HasDefaultConstant is true if the Tag has a kDefault constant.
template <typename Tag>
struct HasDefaultConstant<Tag, std::void_t<decltype(Tag::kDefault)>>
    : std::true_type {};

}  // namespace imp_internal

// A class that holds a value of one of the the specified type, or no value. In
// the case that no value is held, a default value will be returned for each
// type. OneOf can be thought of as std::variant that is always safe to access.
//
// OneOf is intended to match the semantics of the `oneof` type in Protobuf:
// (broken link).
//
// Example:
//
// struct Email {
//   using Type = std::string;
//   static constexpr std::string_view kDefault = "hello@world.com";
// };
//
// struct PhoneNumber {
//   using Type = std::string;
//   // If kDefault is not specified, then the default-initialized value for the
//   // Type will be used.
// };
//
// OneOf<Email, PhoneNumber> contact_info;
//
// // Returns the default value of "hello@world.com" since no value is held:
// std::string_view email = contact_info.Value<Email>();
// // Returns the default-initialized value of "" since no value is held:
// std::string_view phone = contact_info.Value<PhoneNumber>();
//
// 
// 
//
// contact_info.Emplace<PhoneNumber>("123-456-7890");
//
// // contact_info now holds a PhoneNumber.
// 
// 
//
// // Value<Email>() is still safe to access and returns its default value.
// email = contact_info.Value<Email>();               // "hello@world.com"
// phone = contact_info.Value<PhoneNumber>();         // "123-456-7890"
//
// TODO: Add visitor functionality to OneOf.
template <typename... Tags>
class OneOf {
 private:
  // Wrapper to hold the value of a Tag.
  template <typename Tag>
  struct Wrapper {
    // Add a constructor to facilitate in-place initialization.
    template <typename... U>
    explicit Wrapper(U&&... args);

    constexpr bool operator==(const Wrapper& other) const;

    constexpr bool operator!=(const Wrapper& other) const;

    typename Tag::Type value;
  };

  // The underlying variant that holds the value of one of the Tags, or
  // std::monostate if no value is held. Wrapper<Tags>... expands to
  // Wrapper<Tag1>, Wrapper<Tag2>, etc.
  std::variant<std::monostate, Wrapper<Tags>...> variant_;

 public:
  // Constructs an empty OneOf: one where HasValue() will return
  // false and default values will be returned for each Tag.
  constexpr OneOf() noexcept = default;

  // Allows for in-place initialization of the OneOf.
  template <typename Tag, typename... Args>
  explicit constexpr OneOf(std::in_place_type_t<Tag>, Args&&... args);

  constexpr bool operator==(const OneOf& other) const;

  constexpr bool operator!=(const OneOf& other) const;

  // Returns true if the OneOf holds a value for any Tag.
  // Equivalent to checking if Holds<Tag>() is true for any Tag.
  constexpr bool HasValue() const noexcept;

  // Returns true if the OneOf holds a value of the specified Tag.
  template <typename Tag>
  constexpr bool Holds() const noexcept;

  // Clears the assigned value if there is one. After this call, HasValue()
  // will return false.
  void Reset() noexcept;

  // Returns a const reference to the value of the OneOf for the
  // specified Tag. If the OneOf does not hold a value of the
  // specified Tag, then the default value for the specified Tag will be
  // returned.
  template <typename Tag>
  constexpr typename imp_internal::OptionalWithDefaultAccessTypes<
      typename Tag::Type>::RefT
  Value() const;

  // Returns a mutable reference to the value of the OneOf for
  // the specified Tag. If the OneOf does not hold a value of the
  // specified Tag, then it will be set to the default value for the specified
  // Tag.
  template <typename Tag>
  typename Tag::Type& MutableValue();

  // Emplaces a value of the specified Tag into the OneOf.
  template <typename Tag, typename... Args>
  void Emplace(Args&&... args);
};

template <typename... Tags>
template <typename Tag>
template <typename... U>
OneOf<Tags...>::Wrapper<Tag>::Wrapper(U&&... args)
    : value(std::forward<U>(args)...) {}

template <typename... Tags>
template <typename Tag>
constexpr bool OneOf<Tags...>::Wrapper<Tag>::operator==(
    const Wrapper& other) const {
  return value == other.value;
}

template <typename... Tags>
template <typename Tag>
constexpr bool OneOf<Tags...>::Wrapper<Tag>::operator!=(
    const Wrapper& other) const {
  return value != other.value;
}

template <typename... Tags>
template <typename Tag, typename... Args>
constexpr OneOf<Tags...>::OneOf(std::in_place_type_t<Tag>, Args&&... args)
    : variant_(std::in_place_type<Wrapper<Tag>>, std::forward<Args>(args)...) {}

template <typename... Tags>
constexpr bool OneOf<Tags...>::operator==(const OneOf& other) const {
  return variant_ == other.variant_;
}

template <typename... Tags>
constexpr bool OneOf<Tags...>::operator!=(const OneOf& other) const {
  return variant_ != other.variant_;
}

template <typename... Tags>
constexpr bool OneOf<Tags...>::HasValue() const noexcept {
  return !std::holds_alternative<std::monostate>(variant_);
}

template <typename... Tags>
template <typename Tag>
constexpr bool OneOf<Tags...>::Holds() const noexcept {
  return std::holds_alternative<Wrapper<Tag>>(variant_);
}

template <typename... Tags>
void OneOf<Tags...>::Reset() noexcept {
  variant_ = std::monostate{};
}

template <typename... Tags>
template <typename Tag>
constexpr typename imp_internal::OptionalWithDefaultAccessTypes<
    typename Tag::Type>::RefT
OneOf<Tags...>::Value() const {
  // Attempt to get the assigned value.
  if (Holds<Tag>()) {
    return std::get<Wrapper<Tag>>(variant_).value;
  }
  // Attempt to get the default value.
  if constexpr (imp_internal::HasDefaultConstant<Tag>::value) {
    return Tag::kDefault;
  }
  // Attempt to get the constexpr default value.
  if constexpr (imp_internal::IsConstexprDefaultConstructible<
                    typename Tag::Type>::value ||
                std::is_same_v<typename Tag::Type, std::string>) {
    return imp_internal::kUnspecifiedDefault<typename Tag::Type>;
  }
  // Otherwise, get the runtime default value.
  return *imp_internal::RuntimeDefaultValuePointerProvider<
      typename Tag::Type>::GetDefaultValuePointer();
}

template <typename... Tags>
template <typename Tag>
typename Tag::Type& OneOf<Tags...>::MutableValue() {
  // Emplace a default value if there is not one assigned.
  if (!Holds<Tag>()) {
    if constexpr (imp_internal::HasDefaultConstant<Tag>::value) {
      // Emplace the specified default value.
      variant_.template emplace<Wrapper<Tag>>(Tag::kDefault);
    } else if constexpr (imp_internal::IsConstexprDefaultConstructible<
                             typename Tag::Type>::value ||
                         std::is_same_v<typename Tag::Type, std::string>) {
      // Emplace the constexpr default value.
      variant_.template emplace<Wrapper<Tag>>(
          imp_internal::kUnspecifiedDefault<typename Tag::Type>);
    } else {
      // Emplace the runtime default value.
      variant_.template emplace<Wrapper<Tag>>(
          *imp_internal::RuntimeDefaultValuePointerProvider<
              typename Tag::Type>::GetDefaultValuePointer());
    }
  }
  // Return the value.
  return std::get<Wrapper<Tag>>(variant_).value;
}

template <typename... Tags>
template <typename Tag, typename... Args>
void OneOf<Tags...>::Emplace(Args&&... args) {
  variant_.template emplace<Wrapper<Tag>>(std::forward<Args>(args)...);
}

}  // namespace imp

#endif
