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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_VARIANT_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_VARIANT_H_

#include "absl/status/status.h"
#include "absl/status/statusor.h"

namespace imp {
namespace imp_internal {

// Helper visitor for converting a variant<Ts...>` into another type that can be
// constructed from the contained variant type, or returns an error otherwise.
//
// For example, if you have a variant<int, float, std::string> and want to
// convert it to a variant<float, std::string, bool>, you can do so with:
//   absl::StatusOr<std::variant<float, std::string, bool>> converted =
//       TryConvertVariantTo<std::variant<float, std::string, bool>>(
//           std::move(variant));
template <typename To>
struct VariantVisitor {
  template <typename T>
  absl::StatusOr<To> operator()(T&& v) const {
    if constexpr (std::is_constructible_v<To, T>) {
      return To(std::forward<T>(v));
    } else {
      return absl::InvalidArgumentError("Cannot convert variant type");
    }
  }
};
}  // namespace imp_internal

template <typename To, typename Variant>
absl::StatusOr<To> TryConvertVariantTo(Variant&& variant) {
  return absl::visit(imp_internal::VariantVisitor<To>{},
                     std::forward<Variant>(variant));
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_VARIANT_H_
