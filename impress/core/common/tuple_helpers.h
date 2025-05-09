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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_TUPLE_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_TUPLE_HELPERS_H_

#include <cstddef>
#include <tuple>

namespace imp {
// Iterates over a tuple executes a callback for each tuple element. Passes
// each tuple element into the callback 'func'. The func method should be a
// templated method or operator().
//
// For example:
//
// A helper functor to print some
// data. struct PrintWidgets {
//   template <typename WIDGET_TYPE>
//   constexpr void operator()(WIDGET_TYPE&& widget) {
//      widget.print();
//   }
// };
template <typename TUPLE, typename Fn, size_t IDX = 0>
constexpr void ForEachTupleElement(TUPLE& tuple, Fn&& func) {
  func(std::get<IDX>(tuple));

  constexpr size_t kNext = IDX + 1;
  constexpr size_t kMax = std::tuple_size<TUPLE>::value;
  if constexpr (kNext < kMax) {
    ForEachTupleElement<TUPLE, Fn, kNext>(tuple, func);
  }
}

// Returns true if type is in a tuple.
template <typename T, typename TUPLE, size_t IDX = 0>
constexpr bool HasTypeInTuple() {
  // Checks if T is matches the element type at IDX.
  if constexpr (std::is_same<
                    T, typename std::tuple_element<IDX, TUPLE>::type>::value) {
    return true;
  }
  // At Compile time, recurses to the next tuple type.
  constexpr size_t kNext = IDX + 1;
  constexpr size_t kMax = std::tuple_size<TUPLE>::value;
  if constexpr (kNext < kMax) {
    return HasTypeInTuple<T, TUPLE, kNext>();
  }
  return false;
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_TUPLE_HELPERS_H_
