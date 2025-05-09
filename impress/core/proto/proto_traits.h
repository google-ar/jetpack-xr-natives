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

#ifndef THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_TRAITS_H_
#define THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_TRAITS_H_

#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"

namespace imp::proto_traits {

namespace internal {
// Uses SFINAE to detect if Visitor has Visit function that accepts the given
// Arg.
template <typename Visitor, typename Cursor, typename Arg,
          typename FieldTypeIds,
          std::enable_if_t<
              std::is_same_v<
                  Cursor, decltype(std::declval<Visitor>().VisitVariant(
                              std::declval<Cursor>(), std::declval<Arg*>(),
                              std::declval<Arg*>(), std::declval<std::string>(),
                              std::declval<FieldTypeIds>(),
                              std::declval<std::vector<int>>()))>,
              int> = 0>
static constexpr bool HasVisitVariantFunction(int) {
  return true;
}

template <typename Visitor, typename Cursor, typename Arg,
          typename FieldTypeIds>
static constexpr bool HasVisitVariantFunction(...) {
  return false;
}

template <
    typename T,
    std::enable_if_t<
        std::is_same<bool, decltype(std::declval<T>().ParseFromString(
                               std::declval<absl::string_view>()))>::value,
        int> = 0>
static constexpr bool HasParseFromStringImpl(int) {
  return true;
}

template <typename T>
static constexpr bool HasParseFromStringImpl(...) {
  return false;
}

// Uses SFINAE to detect if Visitor has a Visit function.
template <typename Message, typename Visitor, typename Cursor,
          std::enable_if_t<
              std::is_same_v<Cursor, decltype(std::declval<Message>().Visit(
                                         std::declval<Visitor&>(),
                                         std::declval<Cursor>(),
                                         std::declval<Message*>()))>,
              int> = 0>
static constexpr bool HasVisitFunction(int) {
  return true;
}

template <typename Message, typename Visitor, typename Cursor>
static constexpr bool HasVisitFunction(...) {
  return false;
}

// Uses SFINAE to detect if Visitor has a VisitField function.
template <
    typename Message, typename Visitor, typename Cursor,
    std::enable_if_t<
        std::is_same_v<Cursor,
                       decltype(std::declval<Message>().VisitField(
                           std::declval<int>(), std::declval<Visitor&>(),
                           std::declval<Cursor>(), std::declval<Message*>()))>,
        int> = 0>
static constexpr bool HasVisitFieldFunction(int) {
  return true;
}

template <typename Message, typename Visitor, typename Cursor>
static constexpr bool HasVisitFieldFunction(...) {
  return false;
}

}  // namespace internal

template <typename Visitor, typename Cursor, typename Arg,
          typename FieldTypeIds>
constexpr bool kHasVisitVariantFunction =
    internal::HasVisitVariantFunction<Visitor, Cursor, Arg, FieldTypeIds>(0);

template <typename Message, typename Visitor, typename Cursor>
constexpr bool kHasVisitFunction =
    internal::HasVisitFunction<Message, Visitor, Cursor>(0);

template <typename Message, typename Visitor, typename Cursor>
constexpr bool kHasVisitFieldFunction =
    internal::HasVisitFieldFunction<Message, Visitor, Cursor>(0);

// Helper for detecting during compilation if a type has a ParseFromString
// method defined. This is used to differentiate between standard C++ protos and
// Impress protos so they can each be parsed correctly.
template <typename T>
static constexpr bool kIsStandardProto = internal::HasParseFromStringImpl<T>(0);

}  // namespace imp::proto_traits

#endif  // THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_TRAITS_H_
