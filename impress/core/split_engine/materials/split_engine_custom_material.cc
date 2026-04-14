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

#include "core/split_engine/materials/split_engine_custom_material.h"

#include <cstdint>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Material.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/future.h"
#include "core/materials/material.h"
#include "core/split_engine/android/split_engine_android_bridge.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/materials/split_engine_material.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

Future<absl::Status> SplitEngineCustomMaterial::RequestCustomFilamentMaterial(
    BaseView& view, absl::string_view material_source,
    filament::Material* filament_material,
    const MaterialPreCompileOptions& precompile_options) {
  const uint64_t material_id = SplitEngineSerializer::GetId(filament_material);

  flatbuffers::FlatBufferBuilder fbb;
  flatbuffers::Offset<android_xr::schemas::FilamentMaterialSpec>
      material_spec_offset = android_xr::schemas::CreateFilamentMaterialSpec(
          fbb, fbb.CreateString(material_source),
          Pack(fbb, precompile_options));
  flatbuffers::Offset<android_xr::schemas::AddCustomMaterialRequest> request =
      android_xr::schemas::CreateAddCustomMaterialRequest(
          fbb, material_id,
          android_xr::schemas::CustomMaterialSpec::FilamentMaterialSpec,
          material_spec_offset.Union());

  return SendRequest<android_xr::schemas::AddCustomMaterialRequest,
                     absl::Status>(view.GetSplitEngineSerializer()->GetBridge(),
                                   fbb, request);
}

SplitEngineCustomMaterial::SplitEngineCustomMaterial(BaseView& view,
                                                     OwnedMaterialPtr material)
    : SplitEngineMaterial(view, std::move(material)) {}

SplitEngineCustomMaterial::~SplitEngineCustomMaterial() {
  SplitEngineSerializer* serializer = view_.GetSplitEngineSerializer();
  if (serializer && material_) {
    serializer->RemoveMaterialInstance(GetFilamentMaterialInstance());
  }
}

bool SplitEngineCustomMaterial::HasParameter(absl::string_view name) {
  return material_->HasParameter(name);
}

}  // namespace imp::split_engine
