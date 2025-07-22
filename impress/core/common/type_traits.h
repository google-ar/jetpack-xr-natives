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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_TYPE_TRAITS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_TYPE_TRAITS_H_

#include <cstddef>
#include <string_view>
#include <type_traits>

#include "absl/strings/string_view.h"
#include "core/common/hash.h"

namespace imp {
namespace type_traits {

// A constexpr function that provides type names without requiring rtti.
// The trick is `__PRETTY_FUNCTION__` has a predictable format that, for this
// particular method, terminates with the substring "<T = ActualTypeName>".
template <typename T>
constexpr absl::string_view GetTypeName() {
  constexpr absl::string_view func_name = __PRETTY_FUNCTION__;
  constexpr absl::string_view kPrefix =
      "absl::string_view imp::type_traits::GetTypeName() [T = ";
  return func_name.substr(kPrefix.length(),
                          func_name.size() - kPrefix.length() - 1);
}

// Gets a unique hash of a type evaluated at compile time.
template <typename T>
constexpr HashValue GetTypeHash() {
  return Hash(GetTypeName<T>());
}

// A constexpr function that provides field names without requiring rtti.
// The trick is `__PRETTY_FUNCTION__` has a predictable format that, for this
// method, terminates with the substring "<field = &ActualFieldName>".
template <auto field>
constexpr absl::string_view GetFieldName() {
  constexpr absl::string_view func_name = __PRETTY_FUNCTION__;
  constexpr absl::string_view kPrefix =
      "absl::string_view imp::type_traits::GetFieldName() [field = &";
  return func_name.substr(kPrefix.length(),
                          func_name.size() - kPrefix.length() - 1);
}

namespace internal {
// Internal helper class used by GetTypeList.
template <typename... T>
class TypeListHelper {};
}  // namespace internal

// A constexpr function that converts a template type list to a string without
// rtti. See comment on GetTypeName for how this works.
//
// For example, GetTypeList<int, float>() returns "<int, float>".
template <typename... T>
constexpr absl::string_view GetTypeList() {
  constexpr absl::string_view kPrefix =
      "imp::type_traits::internal::TypeListHelper";
  constexpr absl::string_view raw_type_name =
      GetTypeName<internal::TypeListHelper<T...>>();

  return raw_type_name.substr(kPrefix.length(),
                              raw_type_name.size() - kPrefix.length());
}

// Convenience alias for GetTypeName.
template <typename T>
constexpr absl::string_view kTypeName = GetTypeName<T>();

// Convenience alias for GetTypeHash.
template <typename T>
constexpr HashValue kTypeHash = GetTypeHash<T>();

// Convenience alias for GetFieldName.
template <auto field>
constexpr absl::string_view kFieldName = GetFieldName<field>();

// Checks if a type is an expansion of a given template.
// Note, this doesn't work for templates that use non-type parameters.
template <typename, template <typename...> typename>
struct IsTemplateType : public std::false_type {};
template <template <typename...> typename U, typename... Ts>
struct IsTemplateType<U<Ts...>, U> : public std::true_type {};

// Note: This function uses std::string_view rather than absl::string_view,
// because the former has constexpr versions of e.g. find_last_of.
inline constexpr std::string_view TryRemoveTemplateArgs(
    std::string_view pretty) {
  // subtract 1 to strip the space that precedes the open bracket.
  return (pretty.empty() || pretty.back() != ']')
             ? pretty
             : pretty.substr(0, pretty.find_last_of('[') - 1);
}

// Note: This function uses std::string_view rather than absl::string_view,
// because the former has constexpr versions of e.g. find_last_of.
inline constexpr std::string_view RemoveFunctionArgs(std::string_view pretty) {
  int counter = 0;
  size_t cursor = pretty.size();
  do {
    size_t next_cursor = pretty.find_last_of("()", cursor - 1);
    if (next_cursor == std::string_view::npos) return "ERROR";
    if (pretty.at(next_cursor) == ')')
      ++counter;
    else
      --counter;
    cursor = next_cursor;
  } while (counter);
  return pretty.substr(0, cursor);
}

// Note: This function consumes std::string_view rather than absl::string_view,
// because the former has constexpr versions of e.g. find_first_of.
inline constexpr absl::string_view GetFunctionName(std::string_view pretty) {
  auto first_dash_or_plus = pretty.find_first_of("+-");
  if (first_dash_or_plus == 0) {
    // Objective-C names do not need cleanup.
    return absl::string_view(pretty.data(), pretty.size());
  }
  auto no_args = RemoveFunctionArgs(TryRemoveTemplateArgs(pretty));
  auto last_scope = no_args.find_last_of(':');
  if (last_scope != std::string_view::npos) {
    auto next_to_last_scope =
        no_args.substr(0, last_scope - 1).find_last_of(':');
    if (next_to_last_scope != std::string_view::npos) {
      auto name_subsection = no_args.substr(
          next_to_last_scope + 1, no_args.size() - next_to_last_scope - 1);
      return absl::string_view(name_subsection.data(), name_subsection.size());
    }
  }
  return absl::string_view(no_args.data(), no_args.size());
}

#define IMP_FUNCTION_NAME() \
  imp::type_traits::GetFunctionName(__PRETTY_FUNCTION__)

template <typename T, typename = void>
struct HasGetNameMethod : std::false_type {};

template <typename T>
struct HasGetNameMethod<
    T, std::void_t<decltype(std::declval<T>().GetName() == "")>>
    : std::true_type {};

template <typename T>
constexpr bool kHasGetNameMethod = HasGetNameMethod<T>::value;

}  // namespace type_traits
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_TYPE_TRAITS_H_
