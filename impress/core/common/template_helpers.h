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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_TEMPLATE_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_TEMPLATE_HELPERS_H_

#include <cstddef>
#include <type_traits>
#include <utility>

namespace imp {

// Helper for executing a for loop as a constant expression at compile time.
// Function Fn is evaluated for each index less than Size starting from I.
template <std::size_t I, std::size_t Size, typename Fn>
void ForConstexpr(Fn fn) {
  if constexpr (I < Size) {
    fn(std::integral_constant<std::size_t, I>());
    ForConstexpr<I + 1, Size, Fn>(std::move(fn));
  }
}

namespace internal {
template <std::size_t Index, std::size_t Cursor, class T>
constexpr T GetAtInternal(std::integer_sequence<T>) {
  return 0;
}

template <std::size_t Index, std::size_t Cursor, class T, T Head, T... Tail>
constexpr T GetAtInternal(std::integer_sequence<T, Head, Tail...>) {
  if (Index == Cursor) {
    return Head;
  }
  return GetAtInternal<Index, Cursor + 1>(std::integer_sequence<T, Tail...>{});
}
}  // namespace internal

// Returns the entry in a std::integer_sequence at index Index.
template <std::size_t Index, class S>
constexpr auto GetAt(S s) {
  return internal::GetAtInternal<Index, 0>(s);
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_TEMPLATE_HELPERS_H_
