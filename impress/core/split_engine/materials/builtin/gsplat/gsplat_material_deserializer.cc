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

#include "core/split_engine/materials/builtin/gsplat/gsplat_material_deserializer.h"

#include <utility>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "flatbuffers/verifier.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/material/material_asset.h"
#include "core/async/future.h"
#include "core/common/small_source_location.h"
#include "core/material_library/flatbuffer_utils.h"
#include "core/materials/material.h"
#include "core/render/texture.h"
#include "core/resources/resource_definition.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/materials/builtin/builtin_custom_material.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/materials/builtin/gsplat/gsplat_material_deserializer_assets.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::split_engine {
namespace {
constexpr char kOpacityScaleParameter[] = "opacityScale";
constexpr char kMinScreenSizeParameter[] = "minScreenSize";
constexpr char kMaxScreenSizeParameter[] = "maxScreenSize";
constexpr char kWindowDimensionInMagicWindowParameter[] =
    "windowDimensionInMagicWindow";
constexpr char kMagicWindowFromUserWorldMatrixParameter[] =
    "magicWindowFromUserWorldMatrix";
constexpr char kPositionDataTexture[] = "splatDataPosition";
constexpr char kCov3dDataTexture[] = "splatDataCov3d";
constexpr char kColorDataTexture[] = "splatDataColor";
constexpr char kSortedIndicesTexture[] = "sortedIndices";
constexpr char kSplatScaleParameter[] = "splatScale";
constexpr char kVisualizeChunksParameter[] = "visualizeChunks";

// Helper function to read texture from flatbuffer and set it on a material.
absl::Status SetMaterialParameterFromFlatbuffer(
    const TextureBorrower& texture_borrower,
    const BorrowedMaterialPtr& material,
    const android_xr::schemas::BuiltInTextureParameter* serialized_texture,
    absl::string_view material_parameter_name) {
  if (!serialized_texture) {
    return absl::OkStatus();
  }
  if (!material->HasParameter(material_parameter_name)) {
    return absl::OkStatus();
  }

  const BorrowedTexturePtr texture =
      texture_borrower(serialized_texture->texture_id());
  if (!texture) {
    return absl::NotFoundError(absl::StrFormat(
        "Texture not found: %d", serialized_texture->texture_id()));
  }
  material->SetParameter(material_parameter_name, texture,
                         ConvertSampler(serialized_texture->sampler()));
  return absl::OkStatus();
}

resources::ResourceDefinition GetResourceDefinition(
    android_xr::schemas::GsplatMode material_mode) {
  switch (material_mode) {
    case android_xr::schemas::GsplatMode::UNSPECIFIED:
    case android_xr::schemas::GsplatMode::GSPLAT:
      return kBuiltinGsplatMatCmat;
    case android_xr::schemas::GsplatMode::MAGIC_WINDOW:
      return kBuiltinMagicWindowMatCmat;
  }
}

absl::Status HasParameter(const BorrowedMaterialPtr& material,
                          absl::string_view parameter_name) {
  if (material->HasParameter(parameter_name)) {
    return absl::OkStatus();
  }
  return absl::UnimplementedError(
      absl::StrFormat("%s is not supported in this material.", parameter_name));
}
}  // namespace

Future<BuiltInMaterialPtr> GsplatMaterialDeserializer::Create(
    BaseView& view, BridgeId bridge_id,
    const android_xr::schemas::BuiltInMaterialGsplatSpec& spec) {
  ::imp::resources::ResourceDefinition source =
      GetResourceDefinition(spec.material_mode());

  return view.GetAssetManager().LoadMaterial(source).Then(
      [&view, bridge_id, material_mode = spec.material_mode()](
          AssetPtr<MaterialAsset> material_asset) -> BuiltInMaterialPtr {
        return absl::WrapUnique(new GsplatMaterialDeserializer(
            view, bridge_id, material_mode,
            view.GetMaterialFactory().CreateMaterial(material_asset)));
      });
}

GsplatMaterialDeserializer::GsplatMaterialDeserializer(
    BaseView& view, BridgeId bridge_id,
    android_xr::schemas::GsplatMode material_mode, OwnedMaterialPtr material)
    : BuiltInCustomMaterial(bridge_id, std::move(material)),
      view_(view),
      material_mode_(material_mode) {}

absl::Status GsplatMaterialDeserializer::SetParameters(
    flatbuffers::Verifier& verifier,
    const android_xr::schemas::BuiltInMaterialInstanceParameters& parameters,
    const TextureBorrower& texture_borrower) {
  if (parameters.data_type() != android_xr::schemas::BuiltInMaterialParameters::
                                    BuiltInMaterialGsplatParameters) {
    return absl::InvalidArgumentError(
        "This material requires BuiltInMaterialGsplatParameters");
  }

  if (!VerifyBuiltInMaterialParameters(
          verifier, parameters.data(),
          android_xr::schemas::BuiltInMaterialParameters::
              BuiltInMaterialGsplatParameters)) {
    return absl::InvalidArgumentError("Invalid parameters");
  }

  const android_xr::schemas::BuiltInMaterialGsplatParameters*
      serialized_parameters =
          parameters
              .data_as<android_xr::schemas::BuiltInMaterialGsplatParameters>();

  MP_RETURN_IF_ERROR(
      SetCommonMaterialParameters(texture_borrower, *serialized_parameters));

  switch (material_mode_) {
    case android_xr::schemas::GsplatMode::UNSPECIFIED:
    case android_xr::schemas::GsplatMode::GSPLAT:
      return SetGsplatMaterialParameters(texture_borrower,
                                         *serialized_parameters);
      break;
    case android_xr::schemas::GsplatMode::MAGIC_WINDOW:
      return SetMagicWindowMaterialParameters(texture_borrower,
                                              *serialized_parameters);
      break;
  }
}

split_engine::BuiltInMaterialPtr GsplatMaterialDeserializer::Duplicate() const {
  return absl::WrapUnique(new GsplatMaterialDeserializer(
      view_, GetBridgeId(), material_mode_,
      view_.GetMaterialFactory().WrapMaterial(
          filament::MaterialInstance::duplicate(
              GetMaterial()->GetFilamentMaterialInstance()))));
}

absl::Status GsplatMaterialDeserializer::SetCommonMaterialParameters(
    const TextureBorrower& texture_borrower,
    const android_xr::schemas::BuiltInMaterialGsplatParameters&
        serialized_parameters) {
  BorrowedMaterialPtr render_material = GetRenderMaterial();

  if (const android_xr::schemas::Float* splat_scale =
          serialized_parameters.splat_scale()) {
    MP_RETURN_IF_ERROR(HasParameter(render_material, kSplatScaleParameter));
    render_material->SetParameter(kSplatScaleParameter, UnPack(*splat_scale));
  }

  if (const android_xr::schemas::Float* opacity_scale =
          serialized_parameters.opacity_scale()) {
    MP_RETURN_IF_ERROR(HasParameter(render_material, kOpacityScaleParameter));
    render_material->SetParameter(kOpacityScaleParameter,
                                  UnPack(*opacity_scale));
  }

  if (const android_xr::schemas::Float2* min_screen_size =
          serialized_parameters.min_screen_size()) {
    MP_RETURN_IF_ERROR(HasParameter(render_material, kMinScreenSizeParameter));
    render_material->SetParameter(kMinScreenSizeParameter,
                                  UnPack(*min_screen_size));
  }
  if (const android_xr::schemas::Float2* max_screen_size =
          serialized_parameters.max_screen_size()) {
    MP_RETURN_IF_ERROR(HasParameter(render_material, kMaxScreenSizeParameter));
    render_material->SetParameter(kMaxScreenSizeParameter,
                                  UnPack(*max_screen_size));
  }
  if (const android_xr::schemas::BuiltInTextureParameter*
          sorted_indices_texture =
              serialized_parameters.sorted_indices_texture()) {
    MP_RETURN_IF_ERROR(HasParameter(render_material, kSortedIndicesTexture));
    MP_RETURN_IF_ERROR(SetMaterialParameterFromFlatbuffer(
        texture_borrower, render_material, sorted_indices_texture,
        kSortedIndicesTexture));
  }
  return absl::OkStatus();
}

absl::Status GsplatMaterialDeserializer::SetMagicWindowMaterialParameters(
    const TextureBorrower& texture_borrower,
    const android_xr::schemas::BuiltInMaterialGsplatParameters&
        serialized_parameters) {
  BorrowedMaterialPtr magic_window_material = GetRenderMaterial();
  if (const android_xr::schemas::Float2* window_dimension_in_magic_window =
          serialized_parameters.window_dimension_in_magic_window()) {
    MP_RETURN_IF_ERROR(HasParameter(magic_window_material,
                                 kWindowDimensionInMagicWindowParameter));
    magic_window_material->SetParameter(
        kWindowDimensionInMagicWindowParameter,
        UnPack(*window_dimension_in_magic_window));
  }
  if (const android_xr::schemas::Mat4f* magic_window_from_user_world_matrix =
          serialized_parameters.magic_window_from_user_world_matrix()) {
    MP_RETURN_IF_ERROR(HasParameter(magic_window_material,
                                 kMagicWindowFromUserWorldMatrixParameter));
    magic_window_material->SetParameter(
        kMagicWindowFromUserWorldMatrixParameter,
        UnPack(*magic_window_from_user_world_matrix));
  }

  return SetGsplatMaterialParameters(texture_borrower, serialized_parameters);
}

absl::Status GsplatMaterialDeserializer::SetGsplatMaterialParameters(
    const TextureBorrower& texture_borrower,
    const android_xr::schemas::BuiltInMaterialGsplatParameters&
        serialized_parameters) {
  BorrowedMaterialPtr precompute_material = GetPrecomputeMaterial();
  MP_RETURN_IF_ERROR(SetMaterialParameterFromFlatbuffer(
      texture_borrower, precompute_material,
      serialized_parameters.position_data_texture(), kPositionDataTexture));
  MP_RETURN_IF_ERROR(SetMaterialParameterFromFlatbuffer(
      texture_borrower, precompute_material,
      serialized_parameters.cov3d_data_texture(), kCov3dDataTexture));
  MP_RETURN_IF_ERROR(SetMaterialParameterFromFlatbuffer(
      texture_borrower, precompute_material,
      serialized_parameters.color_data_texture(), kColorDataTexture));

  if (const android_xr::schemas::Bool* visualize_chunks =
          serialized_parameters.visualize_chunks()) {
    precompute_material->SetParameter(kVisualizeChunksParameter,
                                      UnPack(*visualize_chunks));
  }
  return absl::OkStatus();
}

BorrowedMaterialPtr GsplatMaterialDeserializer::GetPrecomputeMaterial(
    SmallSourceLocation loc) const {
  return GetMaterial(loc);
}

BorrowedMaterialPtr GsplatMaterialDeserializer::GetRenderMaterial(
    SmallSourceLocation loc) const {
  return GetMaterial(loc);
}
}  // namespace imp::split_engine
