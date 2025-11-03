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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_MATERIAL_CREATOR_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_MATERIAL_CREATOR_HELPER_H_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/future.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_renderer.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {
flatbuffers::Offset<void> CreateBuiltInMaterialSpecWithDefaultParameters(
    flatbuffers::FlatBufferBuilder& builder,
    android_xr::schemas::BuiltInMaterialSpec spec);

// Size of the BuiltInMaterialSpec enum, excluding NONE.
constexpr size_t kBuiltInMaterialSpecSize =
    static_cast<size_t>(android_xr::schemas::BuiltInMaterialSpec::MAX) - 1;

// Returns a list of all built-in material specs from the first valid enum value
// (1) to the last one (MAX). This list includes GenericMaterialSpec.
constexpr std::array<android_xr::schemas::BuiltInMaterialSpec,
                     kBuiltInMaterialSpecSize>
CreateBuiltInMaterialSpecList() {
  std::array<android_xr::schemas::BuiltInMaterialSpec, kBuiltInMaterialSpecSize>
      result{};

  for (size_t i = 0; i < result.size(); ++i) {
    result[i] = static_cast<android_xr::schemas::BuiltInMaterialSpec>(i + 1);
  }

  return result;
}

// Size of the BuiltInMaterialSpec enum, excluding NONE and GenericMaterialSpec.
constexpr size_t kCustomBuiltInMaterialSpecSize = kBuiltInMaterialSpecSize - 1;

// Returns a list of all built-in material specs from the first valid enum value
// (2) to the last one (MAX), excluding GenericMaterialSpec. (1) is for
// GenericMaterialSpec.
constexpr std::array<android_xr::schemas::BuiltInMaterialSpec,
                     kCustomBuiltInMaterialSpecSize>
CreateBuiltInCustomMaterialSpecList() {
  std::array<android_xr::schemas::BuiltInMaterialSpec,
             kCustomBuiltInMaterialSpecSize>
      result{};

  // Skip the first element, which is GenericMaterialSpec.
  for (size_t i = 0; i < result.size(); ++i) {
    result[i] = static_cast<android_xr::schemas::BuiltInMaterialSpec>(i + 2);
  }

  return result;
}

constexpr size_t GetBuiltInMaterialParameterSize(
    android_xr::schemas::BuiltInMaterialSpec spec) {
  switch (spec) {
    case android_xr::schemas::BuiltInMaterialSpec::NONE:
      return 0;
    case android_xr::schemas::BuiltInMaterialSpec::GenericMaterialSpec:
      return sizeof(android_xr::schemas::GenericMaterialParameters);
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterial5cf26af8:
      return sizeof(android_xr::schemas::BuiltInMaterial5cf26af8Parameters);
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialE3ca0ab9:
      return sizeof(android_xr::schemas::BuiltInMaterialE3ca0ab9Parameters);
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialD1750064:
      return sizeof(android_xr::schemas::BuiltInMaterialD1750064Parameters);
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialEb117dd9:
      return sizeof(android_xr::schemas::BuiltInMaterialEb117dd9Parameters);
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterial1b616c8a:
      return sizeof(android_xr::schemas::BuiltInMaterial1b616c8aParameters);
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterial0d0cb9aa:
      return sizeof(android_xr::schemas::BuiltInMaterial0d0cb9aaParameters);
    case android_xr::schemas::BuiltInMaterialSpec::
        BuiltInMaterialTextureExternal:
      return sizeof(
          android_xr::schemas::BuiltInMaterialTextureExternalParameters);
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialbd7fe08c:
      return sizeof(android_xr::schemas::BuiltInMaterialbd7fe08cParameters);
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialGsplatSpec:
      return sizeof(android_xr::schemas::BuiltInMaterialGsplatParameters);
    case android_xr::schemas::BuiltInMaterialSpec::
        BuiltInMaterialGsplatBackgroundSpec:
      return sizeof(
          android_xr::schemas::BuiltInMaterialGsplatBackgroundParameters);
      // default case is omitted to get free compiler error if new enum value is
      // added.
  }

  
  return 0;
}

constexpr size_t GetMaxBuiltInMaterialParameterSize() {
  std::array<size_t,
             static_cast<size_t>(android_xr::schemas::BuiltInMaterialSpec::MAX)>
      sizes{};

  const auto built_in_material_spec_list = CreateBuiltInMaterialSpecList();
  std::transform(built_in_material_spec_list.begin(),
                 built_in_material_spec_list.end(), sizes.begin(),
                 GetBuiltInMaterialParameterSize);

  // Return the maximum size of all parameters.
  return *std::max_element(sizes.begin(), sizes.end());
}

// Shortcut to create a built-in material with default parameters during test
// setup.
Future<absl::Status> CreateBuiltInMaterialWithDefaultParameters(
    SplitEngineRenderer* split_engine_renderer,
    android_xr::schemas::BuiltInMaterialSpec spec, BridgeId bridge_id,
    uint64_t material_instance_id);

};  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_MATERIAL_CREATOR_HELPER_H_
