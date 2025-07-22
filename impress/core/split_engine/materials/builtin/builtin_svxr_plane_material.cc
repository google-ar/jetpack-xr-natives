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

#include "core/split_engine/materials/builtin/builtin_svxr_plane_material.h"

#include <utility>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "flatbuffers/verifier.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/material/material_asset.h"
#include "core/async/future.h"
#include "core/material_library/flatbuffer_utils.h"
#include "core/materials/material.h"
#include "core/render/texture.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/materials/builtin/builtin_custom_material.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/materials/builtin/builtin_svxr_plane_material_assets.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

Future<BuiltInMaterialPtr> BuiltInSVXRPlaneMaterial::Create(
    BaseView& view, BridgeId bridge_id,
    const android_xr::schemas::BuiltInMaterialbd7fe08c& spec) {
  return view.GetAssetManager()
      .LoadMaterial(kBuiltinSvxrPlaneMatCmat)
      .Then([&view, bridge_id](
                AssetPtr<MaterialAsset> material_asset) -> BuiltInMaterialPtr {
        return absl::WrapUnique(new BuiltInSVXRPlaneMaterial(
            view, bridge_id,
            view.GetMaterialFactory().CreateMaterial(material_asset)));
      });
}

BuiltInMaterialPtr BuiltInSVXRPlaneMaterial::Duplicate() const {
  return absl::WrapUnique(new BuiltInSVXRPlaneMaterial(
      view_, GetBridgeId(),
      view_.GetMaterialFactory().WrapMaterial(
          filament::MaterialInstance::duplicate(
              GetMaterial()->GetFilamentMaterialInstance()))));
}

BuiltInSVXRPlaneMaterial::BuiltInSVXRPlaneMaterial(BaseView& view,
                                                   BridgeId bridge_id,
                                                   OwnedMaterialPtr material)
    : BuiltInCustomMaterial(bridge_id, std::move(material)), view_(view) {}

absl::Status BuiltInSVXRPlaneMaterial::SetParameters(
    flatbuffers::Verifier& verifier,
    const android_xr::schemas::BuiltInMaterialInstanceParameters& parameters,
    const TextureBorrower& texture_borrower) {
  if (parameters.data_type() != android_xr::schemas::BuiltInMaterialParameters::
                                    BuiltInMaterialbd7fe08cParameters) {
    return absl::InvalidArgumentError(
        "This material requires BuiltInMaterialbd7fe08cParameters");
  }
  if (!VerifyBuiltInMaterialParameters(
          verifier, parameters.data(),
          android_xr::schemas::BuiltInMaterialParameters::
              BuiltInMaterialbd7fe08cParameters)) {
    return absl::InvalidArgumentError("Invalid parameters");
  }

  // TODO add unit tests.
  const android_xr::schemas::BuiltInMaterialbd7fe08cParameters*
      svxr_plane_material_parameters = parameters.data_as<
          android_xr::schemas::BuiltInMaterialbd7fe08cParameters>();

  if (auto* highlight_point =
          svxr_plane_material_parameters->highlight_point()) {
    GetMaterial()->SetParameter("highlight_point", UnPack(*highlight_point));
  }
  if (auto* dot_pattern = svxr_plane_material_parameters->dot_pattern()) {
    const BorrowedTexturePtr texture =
        texture_borrower(dot_pattern->texture_id());
    if (!texture) {
      return absl::NotFoundError(
          absl::StrFormat("Texture not found: %d", dot_pattern->texture_id()));
    }
    GetMaterial()->SetParameter("dot_pattern", texture,
                                ConvertSampler(dot_pattern->sampler()));
  }
  if (auto* plane_control = svxr_plane_material_parameters->plane_control()) {
    GetMaterial()->SetParameter("plane_control", UnPack(*plane_control));
  }
  if (auto* falloff_color = svxr_plane_material_parameters->falloff_color()) {
    GetMaterial()->SetParameter("falloff_color", UnPack(*falloff_color));
  }
  if (auto* cutoff_color = svxr_plane_material_parameters->cutoff_color()) {
    GetMaterial()->SetParameter("cutoff_color", UnPack(*cutoff_color));
  }
  if (auto* uv_scale = svxr_plane_material_parameters->uv_scale()) {
    GetMaterial()->SetParameter("uv_scale", UnPack(*uv_scale));
  }

  return absl::OkStatus();
}

}  // namespace imp::split_engine
