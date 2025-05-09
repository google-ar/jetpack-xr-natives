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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_RENDERABLE_VALIDATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_RENDERABLE_VALIDATOR_H_

#include <cstddef>
#include <cstdint>

#include "absl/status/status.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"

namespace imp::split_engine {

class RenderableValidator {
 public:
  static absl::Status ValidateIndexOffsetCount(uint32_t offset, uint32_t count,
                                               size_t index_count) noexcept;

  static absl::Status ValidatePrimitiveType(
      filament::backend::PrimitiveType primitive_type) noexcept;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_RENDERABLE_VALIDATOR_H_
