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

#include "core/split_engine/renderable_validator.h"

#include <cstddef>
#include <cstdint>
#include <limits>

#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"

namespace imp::split_engine {

absl::Status RenderableValidator::ValidatePrimitiveType(
    filament::backend::PrimitiveType primitive_type) noexcept {
  switch (primitive_type) {
    case filament::backend::PrimitiveType::POINTS:
    case filament::backend::PrimitiveType::LINES:
    case filament::backend::PrimitiveType::LINE_STRIP:
    case filament::backend::PrimitiveType::TRIANGLES:
    case filament::backend::PrimitiveType::TRIANGLE_STRIP:
      return absl::OkStatus();
  }

  // No default case to get a compiler error if new enum is added.
  // Return is reachable if user input is invalid.
  return absl::InvalidArgumentError(absl::StrFormat(
      "Invalid primitive type: %u", static_cast<uint32_t>(primitive_type)));
}

absl::Status RenderableValidator::ValidateIndexOffsetCount(
    uint32_t offset, uint32_t count, size_t index_count) noexcept {
  const uint64_t offset_64 = offset;
  const uint64_t count_64 = count;
  const uint64_t sum_64 = offset_64 + count_64;
  // `size_t` is 32 bits on WASM.
  const uint64_t index_count_64 = index_count;

  const bool valid_arguments =
      (sum_64 <= std::numeric_limits<uint32_t>::max()) &&
      (sum_64 <= index_count_64);

  if (!valid_arguments) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Index offset %u and count %u are out of range "
                        "for index count %u",
                        offset, count, index_count));
  }

  return absl::OkStatus();
}

}  // namespace imp::split_engine
