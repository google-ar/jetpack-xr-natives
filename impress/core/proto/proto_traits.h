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

#include <cstddef>
#include <map>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/async/future.h"
#include "core/common/copyable_ptr.h"
#include "core/common/one_of.h"
#include "core/common/optional_with_default.h"

namespace imp {
class BaseView;

}  // namespace imp

namespace imp::proto_traits {

namespace internal {
template <typename... Ts>
struct TypeList;
}

template <typename T, typename Visited = internal::TypeList<>>
struct StateRequiresAsyncPreload;

namespace internal {

// Uses SFINAE to detect if Visitor has Visit function that accepts the given
// Arg.
template <
    typename Visitor, typename Cursor, typename Arg, typename FieldTypeIds,
    std::enable_if_t<std::is_convertible_v<
                         decltype(std::declval<Visitor>().VisitVariant(
                             std::declval<Cursor>(), std::declval<Arg*>(),
                             std::declval<Arg*>(), std::declval<std::string>(),
                             std::declval<FieldTypeIds>(),
                             std::declval<std::vector<int>>())),
                         Cursor>,
                     int> = 0>
static constexpr bool HasVisitVariantFunction(int) {
  return true;
}

template <typename Visitor, typename Cursor, typename Arg,
          typename FieldTypeIds>
static constexpr bool HasVisitVariantFunction(...) {
  return false;
}

// Uses SFINAE to detect if Visitor has a valid VisitOneOf function, for
// example:
//
// template <typename... Tags, typename FieldType, FieldType... field_types,
// size_t... I>
// Cursor* VisitOneOf(
//     Cursor* cursor,
//     imp::OneOf<Tags...>* field,
//     imp::OneOf<Tags...>* other,
//     absl::string_view field_name,
//     std::integer_sequence<FieldType, field_types...> field_type_ids,
//     const std::vector<int>& variant_field_ids,
//     std::index_sequence<I...> index_sequence
// );
//
// Where field_type_ids is an integer sequence of the field types of the
// oneof, variant_field_ids is the field ids of the OneOf, and index_sequence
// is a compile time sequence of the indices of the OneOf.
template <
    typename Visitor, typename Cursor, typename Arg, typename FieldTypeIds,
    typename IndexSequence,
    std::enable_if_t<
        std::is_convertible_v<
            decltype(std::declval<Visitor>().VisitOneOf(
                std::declval<Cursor>(), std::declval<Arg*>(),
                std::declval<Arg*>(), std::declval<std::string>(),
                std::declval<FieldTypeIds>(), std::declval<std::vector<int>>(),
                std::declval<IndexSequence>())),
            Cursor>,
        int> = 0>
static constexpr bool HasVisitOneOfFunction(int) {
  return true;
}

template <typename Visitor, typename Cursor, typename Arg,
          typename FieldTypeIds, typename IndexSequence>
static constexpr bool HasVisitOneOfFunction(...) {
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

template <typename Visitor, typename Cursor, typename Arg,
          typename FieldTypeIds, typename IndexSequence>
constexpr bool kHasVisitOneOfFunction =
    internal::HasVisitOneOfFunction<Visitor, Cursor, Arg, FieldTypeIds,
                                    IndexSequence>(0);

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

namespace internal {

// Helper trait to detect if a type T is an Impress generated proto message
// by checking for the existence of a static constexpr kFieldsCount member.
template <typename T, typename = void>
struct HasFields : std::false_type {};

template <typename T>
struct HasFields<T, std::void_t<decltype(T::kFieldsCount)>> : std::true_type {};

// Uses SFINAE to detect if T has an AsyncPreload function that takes a
// BaseView& and returns a Future<absl::Status>.
template <
    typename T,
    std::enable_if_t<std::is_same_v<imp::Future<absl::Status>,
                                    decltype(std::declval<T>().AsyncPreload(
                                        std::declval<imp::BaseView&>()))>,
                     int> = 0>
static constexpr bool StateHasAsyncPreloadFuncInternal(int) {
  return true;
}

template <typename T>
static constexpr bool StateHasAsyncPreloadFuncInternal(...) {
  return false;
}

// Compile-time type list used to keep track of visited types during
// recursive traversal of proto fields. This is used to prevent infinite
// recursion in case of self-referential or cyclic proto definitions.
template <typename... Ts>
struct TypeList {};

// Helper to check if a TypeList contains a specific type T.
template <typename T, typename List>
struct ListContains;

template <typename T>
struct ListContains<T, TypeList<>> {
  static constexpr bool value = false;
};

template <typename T, typename... Ts>
struct ListContains<T, TypeList<T, Ts...>> {
  static constexpr bool value = true;
};

template <typename T, typename Head, typename... Tail>
struct ListContains<T, TypeList<Head, Tail...>> {
  static constexpr bool value = ListContains<T, TypeList<Tail...>>::value;
};

// Helper to append a type T to a TypeList.
template <typename T, typename List>
struct AppendToTypeListT;

template <typename T, typename... Ts>
struct AppendToTypeListT<T, TypeList<Ts...>> {
  using Type = TypeList<Ts..., T>;
};

template <typename T, typename List>
using AppendToTypeList = typename AppendToTypeListT<T, List>::Type;

// Helper to check if any of the types in a list require async preload.
template <typename Visited, typename... Ts>
static constexpr bool AnyRequireAsyncPreload =
    (imp::proto_traits::StateRequiresAsyncPreload<Ts, Visited>::value || ...);

// Helper to check all fields of a proto message T.
// Iterates over all fields using index sequence and checks if any of them
// require async preload, while keeping track of visited types.
template <typename T, typename Visited, size_t... Is>
static constexpr bool CheckFields(std::index_sequence<Is...>) {
  return AnyRequireAsyncPreload<Visited,
                                typename T::template FieldType<Is>::Type...>;
}

}  // namespace internal

// Main trait to determine at compile-time if a state type T (or any of its
// nested fields recursively) requires asynchronous preloading.
// A state requires preloading if:
// 1. It has an `AsyncPreload` member function.
// 2. It is a proto message and at least one of its fields requires preloading.
//
// The `Visited` template parameter is used internally to detect and break
// circular dependencies during compile-time traversal.
template <typename T, typename Visited>
struct StateRequiresAsyncPreload {
  static constexpr bool value = []() {
    if constexpr (internal::ListContains<T, Visited>::value) {
      // Break circular dependency.
      return false;
    } else if constexpr (internal::StateHasAsyncPreloadFuncInternal<T>(0)) {
      // Type T itself has AsyncPreload.
      return true;
    } else if constexpr (internal::HasFields<T>::value) {
      // T is a proto message, recursively check all its fields.
      using NewVisited = internal::AppendToTypeList<T, Visited>;
      return internal::CheckFields<T, NewVisited>(
          std::make_index_sequence<T::kFieldsCount>{});
    }
    return false;
  }();
};

// Specialization for std::vector. A vector requires preloading if its
// element type requires preloading.
template <typename T, typename Visited>
struct StateRequiresAsyncPreload<std::vector<T>, Visited> {
  static constexpr bool value = StateRequiresAsyncPreload<T, Visited>::value;
};

// Specialization for CopyablePtr.
template <typename T, typename Visited>
struct StateRequiresAsyncPreload<CopyablePtr<T>, Visited> {
  static constexpr bool value = StateRequiresAsyncPreload<T, Visited>::value;
};

// Specialization for absl::optional / std::optional (when they are alias).
template <typename T, typename Visited>
struct StateRequiresAsyncPreload<absl::optional<T>, Visited> {
  static constexpr bool value = StateRequiresAsyncPreload<T, Visited>::value;
};

// Specialization for std::map. A map requires preloading if its value type
// requires preloading.
template <typename K, typename V, typename Visited>
struct StateRequiresAsyncPreload<std::map<K, V>, Visited> {
  static constexpr bool value = StateRequiresAsyncPreload<V, Visited>::value;
};

// Specialization for imp::OptionalWithDefault.
template <typename T, const auto* DefaultValuePointer, typename Visited>
struct StateRequiresAsyncPreload<
    imp::OptionalWithDefault<T, DefaultValuePointer>, Visited> {
  static constexpr bool value = StateRequiresAsyncPreload<T, Visited>::value;
};

// Specialization for imp::OneOf.
template <typename... Tags, typename Visited>
struct StateRequiresAsyncPreload<imp::OneOf<Tags...>, Visited> {
  static constexpr bool value =
      (StateRequiresAsyncPreload<typename Tags::Type, Visited>::value || ...);
};

}  // namespace imp::proto_traits

#endif  // THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_TRAITS_H_
