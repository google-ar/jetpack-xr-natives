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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_DETAILS_BUNDLE_RESOURCE_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_DETAILS_BUNDLE_RESOURCE_HELPERS_H_

#include <cstddef>
#include <optional>

#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "core/common/schemas/render_generated.h"
#include "core/common/typed_span.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"

namespace imp::loader::details {

template <typename T>
inline bool VerifyEnum(T value) {
  return value >= T::MIN && value <= T::MAX;
}

template <typename T>
inline bool VerifyFlags(T value) {
  return (value | T::ANY) == T::ANY;
}

// Info metadata helpers.

size_t GetAttributeTypeSize(filament::backend::ElementType type);
size_t GetAttributeTypeSize(schemas::AttributeType type);
size_t GetIndexElementSize(const schemas::IndexBufferInfo* info);
size_t GetIndexCount(const schemas::IndexBufferInfo* info);

}  // namespace imp::loader::details

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_DETAILS_BUNDLE_RESOURCE_HELPERS_H_
