// Copyright 2024 Google LLC
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

#include "core/split_engine/materials/builtin/builtin_svxr_footprint_material.h"

#include <utility>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "flatbuffers/verifier.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/material/material_asset.h"
#include "core/async/future.h"
#include "core/material_library/material_param_value.h"
#include "core/materials/material.h"
#include "core/render/texture.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/materials/builtin/builtin_custom_material.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/materials/builtin/builtin_svxr_footprint_material_assets.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

Future<BuiltInMaterialPtr> BuiltInSVXRFootprintMaterial::Create(
    BaseView& view, const android_xr::schemas::BuiltInMaterial0d0cb9aa& spec) {
  return view.GetAssetManager()
      .LoadMaterial(kBuiltinSvxrFootprintMatCmat)
      .Then([&view](
                AssetPtr<MaterialAsset> material_asset) -> BuiltInMaterialPtr {
        return absl::WrapUnique(new BuiltInSVXRFootprintMaterial(
            view, view.GetMaterialFactory().CreateMaterial(material_asset)));
      });
}

BuiltInMaterialPtr BuiltInSVXRFootprintMaterial::Duplicate() const {
  return absl::WrapUnique(new BuiltInSVXRFootprintMaterial(
      view_, view_.GetMaterialFactory().WrapMaterial(
                 filament::MaterialInstance::duplicate(
                     GetMaterial()->GetFilamentMaterialInstance()))));
}

BuiltInSVXRFootprintMaterial::BuiltInSVXRFootprintMaterial(
    BaseView& view, OwnedMaterialPtr material)
    : BuiltInCustomMaterial(std::move(material)), view_(view) {}

absl::Status BuiltInSVXRFootprintMaterial::SetParameters(
    flatbuffers::Verifier& verifier,
    const android_xr::schemas::BuiltInMaterialInstanceParameters& parameters,
    const TextureBorrower& texture_borrower) {
  if (parameters.data_type() != android_xr::schemas::BuiltInMaterialParameters::
                                    BuiltInMaterial0d0cb9aaParameters) {
    return absl::InvalidArgumentError(
        "This material requires BuiltInMaterial0d0cb9aaParameters");
  }
  if (!VerifyBuiltInMaterialParameters(
          verifier, parameters.data(),
          android_xr::schemas::BuiltInMaterialParameters::
              BuiltInMaterial0d0cb9aaParameters)) {
    return absl::InvalidArgumentError("Invalid parameters");
  }

  // TODO add unit tests.
  const android_xr::schemas::BuiltInMaterial0d0cb9aaParameters*
      svxr_footprint_material_parameters = parameters.data_as<
          android_xr::schemas::BuiltInMaterial0d0cb9aaParameters>();

  if (auto* primary_touch_point =
          svxr_footprint_material_parameters->primary_touch_point()) {
    GetMaterial()->SetParameter("primary_touch_point",
                                UnPack(*primary_touch_point));
  }
  if (auto* touch_control =
          svxr_footprint_material_parameters->touch_control()) {
    GetMaterial()->SetParameter("touch_control", UnPack(*touch_control));
  }
  if (auto* touch_response =
          svxr_footprint_material_parameters->touch_response()) {
    GetMaterial()->SetParameter("touch_response", UnPack(*touch_response));
  }
  if (auto* secondary_touch_point =
          svxr_footprint_material_parameters->secondary_touch_point()) {
    GetMaterial()->SetParameter("secondary_touch_point",
                                UnPack(*secondary_touch_point));
  }
  if (auto* falloff_color =
          svxr_footprint_material_parameters->falloff_color()) {
    GetMaterial()->SetParameter("falloff_color", UnPack(*falloff_color));
  }
  if (auto* cutoff_color = svxr_footprint_material_parameters->cutoff_color()) {
    GetMaterial()->SetParameter("cutoff_color", UnPack(*cutoff_color));
  }

  return absl::OkStatus();
}

}  // namespace imp::split_engine
