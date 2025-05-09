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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_TYPE_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_TYPE_HELPERS_H_

#include <type_traits>

namespace imp {

// Takes a type T and a list of Us, if T matches any of the Us the 'value'
// is set to true. This struct is useful in conjunction with std::enable_if_t.
template <typename T, typename... Us>
class IsAnyOf {
 public:
  // Logical OR each of the Us. Any matching T and U results in a true value.
  // See fold expressions for details.
  // (https://en.cppreference.com/w/cpp/language/fold)
  constexpr static bool value = (... || std::is_same_v<T, Us>);
};

template <typename T, typename... Us>
constexpr bool kIsAnyOf = IsAnyOf<T, Us...>::value;

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_TYPE_HELPERS_H_
