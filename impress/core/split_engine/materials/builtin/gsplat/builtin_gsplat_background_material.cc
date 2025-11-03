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
#include "core/split_engine/materials/builtin/gsplat/builtin_gsplat_background_material.h"

#include <utility>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "flatbuffers/verifier.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/material/material_asset.h"
#include "core/async/future.h"
#include "core/materials/material.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/builtin/builtin_custom_material.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/materials/builtin/gsplat/builtin_gsplat_material_assets.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

Future<BuiltInMaterialPtr> BuiltInGSplatBackgroundMaterial::Create(
    BaseView& view, BridgeId bridge_id,
    const android_xr::schemas::BuiltInMaterialGsplatBackgroundSpec& spec) {
  return view.GetAssetManager()
      .LoadMaterial(kBuiltinGsplatBackgroundMatCmat)
      .Then([&view, bridge_id](
                AssetPtr<MaterialAsset> material_asset) -> BuiltInMaterialPtr {
        return absl::WrapUnique(new BuiltInGSplatBackgroundMaterial(
            view, bridge_id,
            view.GetMaterialFactory().CreateMaterial(material_asset)));
      });
}
BuiltInGSplatBackgroundMaterial::BuiltInGSplatBackgroundMaterial(
    BaseView& view, BridgeId bridge_id, OwnedMaterialPtr material)
    : BuiltInCustomMaterial(bridge_id, std::move(material)), view_(view) {}

absl::Status BuiltInGSplatBackgroundMaterial::SetParameters(
    flatbuffers::Verifier& verifier,
    const android_xr::schemas::BuiltInMaterialInstanceParameters& parameters,
    const TextureBorrower& texture_borrower) {
  if (parameters.data_type() != android_xr::schemas::BuiltInMaterialParameters::
                                    BuiltInMaterialGsplatBackgroundParameters) {
    return absl::InvalidArgumentError(
        "This material requires BuiltInMaterialGsplatBackgroundParameters");
  }

  if (!VerifyBuiltInMaterialParameters(
          verifier, parameters.data(),
          android_xr::schemas::BuiltInMaterialParameters::
              BuiltInMaterialGsplatBackgroundParameters)) {
    return absl::InvalidArgumentError("Invalid parameters");
  }

  // No parameters to set.

  return absl::OkStatus();
}
split_engine::BuiltInMaterialPtr BuiltInGSplatBackgroundMaterial::Duplicate()
    const {
  return absl::WrapUnique(new BuiltInGSplatBackgroundMaterial(
      view_, GetBridgeId(),
      view_.GetMaterialFactory().WrapMaterial(
          filament::MaterialInstance::duplicate(
              GetMaterial()->GetFilamentMaterialInstance()))));
}

}  // namespace imp::split_engine
