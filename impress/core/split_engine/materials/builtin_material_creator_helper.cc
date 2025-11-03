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

#include "core/split_engine/materials/builtin_material_creator_helper.h"

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
    android_xr::schemas::BuiltInMaterialSpec spec) {
  switch (spec) {
    case android_xr::schemas::BuiltInMaterialSpec::NONE:
      
      return {};
    case android_xr::schemas::BuiltInMaterialSpec::GenericMaterialSpec:
      return android_xr::schemas::CreateGenericMaterialSpec(builder).Union();
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterial5cf26af8:
      return android_xr::schemas::CreateBuiltInMaterial5cf26af8(builder)
          .Union();
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialE3ca0ab9:
      return android_xr::schemas::CreateBuiltInMaterialE3ca0ab9(builder)
          .Union();
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialD1750064:
      return android_xr::schemas::CreateBuiltInMaterialD1750064(builder)
          .Union();
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialEb117dd9:
      return android_xr::schemas::CreateBuiltInMaterialEb117dd9(builder)
          .Union();
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterial1b616c8a:
      return android_xr::schemas::CreateBuiltInMaterial1b616c8a(builder)
          .Union();
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterial0d0cb9aa:
      return android_xr::schemas::CreateBuiltInMaterial0d0cb9aa(builder)
          .Union();
    case android_xr::schemas::BuiltInMaterialSpec::
        BuiltInMaterialTextureExternal:
      return android_xr::schemas::CreateBuiltInMaterialTextureExternal(builder)
          .Union();
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialbd7fe08c:
      return android_xr::schemas::CreateBuiltInMaterialbd7fe08c(builder)
          .Union();
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialGsplatSpec:
      return android_xr::schemas::CreateBuiltInMaterialGsplatSpec(builder)
          .Union();
    case android_xr::schemas::BuiltInMaterialSpec::
        BuiltInMaterialGsplatBackgroundSpec:
      return android_xr::schemas::CreateBuiltInMaterialGsplatBackgroundSpec(
                 builder)
          .Union();
      // Default case is omitted to get free compiler error when new enum
      // value is added.
  }

  
  return {};
}

// Shortcut to create a built-in material with default parameters.
Future<absl::Status> CreateBuiltInMaterialWithDefaultParameters(
    SplitEngineRenderer* split_engine_renderer,
    android_xr::schemas::BuiltInMaterialSpec spec, const BridgeId bridge_id,
    const uint64_t material_instance_id) {
  flatbuffers::FlatBufferBuilder builder;
  flatbuffers::Offset<android_xr::schemas::BuiltInMaterialRequest> fbb_request =
      android_xr::schemas::CreateBuiltInMaterialRequest(
          builder, material_instance_id, spec,
          CreateBuiltInMaterialSpecWithDefaultParameters(builder, spec));
  builder.Finish(fbb_request);

  const android_xr::schemas::BuiltInMaterialRequest* request =
      flatbuffers::GetRoot<android_xr::schemas::BuiltInMaterialRequest>(
          builder.GetBufferPointer());

  return split_engine_renderer->CreateBuiltInMaterial(bridge_id, *request);
}

};  // namespace imp::split_engine
