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

#include "core/split_engine/materials/builtin/builtin_water_material.h"

#include <utility>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "flatbuffers/verifier.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/material/material_asset.h"
#include "core/async/future.h"
#include "core/lighting/environment_light.h"
#include "core/material_library/flatbuffer_utils.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/builtin/builtin_custom_material.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/materials/builtin/builtin_water_material_assets.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/lighting/light_manager.h"
#include "core/view/view_events.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

namespace {

constexpr absl::string_view kReflectionCubeParameterName = "reflectionCube";
constexpr absl::string_view kReflectionRotationParameterName =
    "reflectionRotation";
constexpr absl::string_view kNormalMapParameterName = "normalMap";
constexpr absl::string_view kAlphaMapParameterName = "alphaMap";
constexpr absl::string_view kNormalTilingParameterName = "normalTiling";
constexpr absl::string_view kNormalSpeedParameterName = "normalSpeed";
constexpr absl::string_view kAlphaStepMultiplierParameterName =
    "alphaStepMultiplier";
constexpr absl::string_view kNormalZParameterName = "normalZ";
constexpr absl::string_view kNormalBoundaryParameterName = "normalBoundary";

}  // namespace

Future<BuiltInMaterialPtr> BuiltInWaterMaterial::Create(
    BaseView& view, const android_xr::schemas::BuiltInMaterial5cf26af8& spec) {
  Future<AssetPtr<MaterialAsset>> future = view.GetAssetManager().LoadMaterial(
      spec.transparent() ? kBuiltinWaterTransparentMatCmat
                         : kBuiltinWaterMatCmat);
  return future.Then(
      [&view](AssetPtr<MaterialAsset> material_asset) -> BuiltInMaterialPtr {
        return absl::WrapUnique(new BuiltInWaterMaterial(
            view, view.GetMaterialFactory().CreateMaterial(material_asset)));
      });
}

BuiltInWaterMaterial::BuiltInWaterMaterial(BaseView& view,
                                           OwnedMaterialPtr material)
    : BuiltInCustomMaterial(std::move(material)), view_(view) {
  // All samplers must have valid textures so set the placeholder texture.
  GetMaterial()->SetParameter(
      kReflectionCubeParameterName,
      view.GetTextureFactory().BorrowPlaceholderTexture());
  GetMaterial()->SetParameter(
      kNormalMapParameterName,
      view.GetTextureFactory().BorrowPlaceholderTexture());
  GetMaterial()->TrySetParameter(
      kAlphaMapParameterName,
      view.GetTextureFactory().BorrowPlaceholderTexture());
}

BuiltInMaterialPtr BuiltInWaterMaterial::Duplicate() const {
  return absl::WrapUnique(new BuiltInWaterMaterial(
      view_, view_.GetMaterialFactory().WrapMaterial(
                 filament::MaterialInstance::duplicate(
                     GetMaterial()->GetFilamentMaterialInstance()))));
}

absl::Status BuiltInWaterMaterial::SetParameters(
    flatbuffers::Verifier& verifier,
    const android_xr::schemas::BuiltInMaterialInstanceParameters& parameters,
    const TextureBorrower& texture_borrower) {
  if (parameters.data_type() != android_xr::schemas::BuiltInMaterialParameters::
                                    BuiltInMaterial5cf26af8Parameters) {
    return absl::InvalidArgumentError(
        "This material requires BuiltInMaterial5cf26af8Parameters");
  }

  if (!VerifyBuiltInMaterialParameters(
          verifier, parameters.data(),
          android_xr::schemas::BuiltInMaterialParameters::
              BuiltInMaterial5cf26af8Parameters)) {
    return absl::InvalidArgumentError("Invalid parameters");
  }

  const android_xr::schemas::BuiltInMaterial5cf26af8Parameters* schema =
      parameters
          .data_as<android_xr::schemas::BuiltInMaterial5cf26af8Parameters>();

  if (schema->reflection_cube()) {
    const BorrowedTexturePtr texture =
        texture_borrower(schema->reflection_cube()->texture_id());
    if (!texture) {
      return absl::NotFoundError(absl::StrFormat(
          "Texture not found: %d", schema->reflection_cube()->texture_id()));
    }
    filament::TextureSampler sampler =
        ConvertSampler(*schema->reflection_cube()->sampler());
    GetMaterial()->SetParameter(kReflectionCubeParameterName, texture, sampler);
  }
  if (schema->normal_map()) {
    const BorrowedTexturePtr texture =
        texture_borrower(schema->normal_map()->texture_id());
    if (!texture) {
      return absl::NotFoundError(absl::StrFormat(
          "Texture not found: %d", schema->normal_map()->texture_id()));
    }
    filament::TextureSampler sampler =
        ConvertSampler(*schema->normal_map()->sampler());
    GetMaterial()->SetParameter(kNormalMapParameterName, texture, sampler);
  }
  if (schema->normal_tiling()) {
    GetMaterial()->SetParameter(kNormalTilingParameterName,
                                FromFloatFlatbuffer(*schema->normal_tiling()));
  }
  if (schema->normal_speed()) {
    GetMaterial()->SetParameter(kNormalSpeedParameterName,
                                FromFloatFlatbuffer(*schema->normal_speed()));
  }

  if (schema->alpha_map()) {
    const BorrowedTexturePtr texture =
        texture_borrower(schema->alpha_map()->texture_id());
    if (!texture) {
      return absl::NotFoundError(absl::StrFormat(
          "Texture not found: %d", schema->alpha_map()->texture_id()));
    }
    filament::TextureSampler sampler =
        ConvertSampler(*schema->alpha_map()->sampler());
    // Opaque water material does not have alpha map.
    GetMaterial()->TrySetParameter(kAlphaMapParameterName, texture, sampler);
  }

  if (schema->alpha_step_multiplier()) {
    GetMaterial()->TrySetParameter(
        kAlphaStepMultiplierParameterName,
        FromFloatFlatbuffer(*schema->alpha_step_multiplier()));
  }
  if (schema->normal_z()) {
    GetMaterial()->TrySetParameter(kNormalZParameterName,
                                   FromFloatFlatbuffer(*schema->normal_z()));
  }
  if (schema->normal_boundary()) {
    GetMaterial()->TrySetParameter(
        kNormalBoundaryParameterName,
        FromFloatFlatbuffer(*schema->normal_boundary()));
  }
  return absl::OkStatus();
}

}  // namespace imp::split_engine
