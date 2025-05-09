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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_VISIT_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_VISIT_H_

#include "absl/types/variant.h"

namespace imp {

template <typename... Ts>
struct MakeOverload : Ts... {
  using Ts::operator()...;
};
template <typename... Ts>
MakeOverload(Ts...) -> MakeOverload<Ts...>;

// Based on example 4 in the documentation for absl::visit()
// (see https://en.cppreference.com/w/cpp/utility/variant/visit)
//
// Example:
//
// using VariantType = absl::variant<int, long, double, std::string>;
// VariantType vt = ...;
// imp::Visit(vt,
//     [](int curr){ ... },   // <-- runs if vt holds int
//     [](long curr){ ... },  // <-- runs if vt holds long
//     [](auto curr){ ... }); // <-- runs if vt holds double or std::string
template <typename Variant, typename... Alternatives>
decltype(auto) Visit(Variant&& variant, Alternatives&&... alternatives) {
  return absl::visit(MakeOverload{std::forward<Alternatives>(alternatives)...},
                     std::forward<Variant>(variant));
}

// A version that takes two variants; the visit method takes both arguments
template <typename Variant, typename... Alternatives>
decltype(auto) Visit(Variant&& a, Variant&& b, Alternatives&&... alternatives) {
  return absl::visit(MakeOverload{std::forward<Alternatives>(alternatives)...},
                     std::forward<Variant>(a), std::forward<Variant>(b));
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_VISIT_H_
