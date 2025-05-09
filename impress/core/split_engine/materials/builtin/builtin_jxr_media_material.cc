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

#include "core/split_engine/materials/builtin/builtin_jxr_media_material.h"

#include <utility>

#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "flatbuffers/verifier.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/material/material_asset.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/future.h"
#include "core/config.h"
#include "core/material_library/flatbuffer_utils.h"
#include "core/materials/material.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/resources/resource_definition.h"
#include "core/split_engine/materials/builtin/builtin_custom_material.h"
#include "core/split_engine/materials/builtin/builtin_jxr_media_material_assets.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "split_engine/schemas/split_engine_material_generated.h"

#if IMP_PLATFORM(ANDROID)
#include <sys/system_properties.h>
#endif  // IMP_PLATFORM(ANDROID)

namespace imp::split_engine {

namespace {

// Main texture used for single view media types as well as the primary view
// of multiview media types.
static constexpr absl::string_view kTextureParameter = "mediaTexture";

// Auxiliary textures used for the secondary view of multiview media types and
// the depth textures.
static constexpr absl::string_view kAuxiliaryTextureParameter =
    "mediaAuxiliaryTexture";
static constexpr absl::string_view kPrimaryDepthTextureParameter =
    "mediaPrimaryDepthTexture";
static constexpr absl::string_view kSecondaryDepthTextureParameter =
    "mediaSecondaryDepthTexture";
static constexpr absl::string_view kPrimaryAlphaMaskTextureParameter =
    "primaryAlphaMaskTexture";
static constexpr absl::string_view kAuxiliaryAlphaMaskTextureParameter =
    "auxiliaryAlphaMaskTexture";

// Parameter used to pass the stereo type information to the material.
static constexpr absl::string_view kStereoTypeParameter = "stereoType";

// Parameter used to pass the (UV) feather radius information to the material.
static constexpr absl::string_view kFeatherRadiusParameter = "featherRadius";

// Parameter used to determine use of super sampling in the material.
static constexpr absl::string_view kUseSuperSamplingParameter =
    "useSuperSampling";

#if IMP_PLATFORM(ANDROID)
static constexpr const char* kUseSuperSamplingProperty =
    "jxr.surface_entity.use_super_sampling";
#endif  // IMP_PLATFORM(ANDROID)

resources::ResourceDefinition GetMaterialResource(
    android_xr::schemas::BuiltInMaterial1b616c8aShapeType shape_type) {
  switch (shape_type) {
    case android_xr::schemas::BuiltInMaterial1b616c8aShapeType::DEFAULT_FLAT:
      return kBuiltinJxrMediaMatCmat;
    default:
      IMP_LOG(imp::ERROR) << "Unsupported shape type!";
      return kBuiltinJxrMediaMatCmat;
  }
}

}  // namespace

Future<split_engine::BuiltInMaterialPtr> BuiltInJxrMediaMaterial::Create(
    BaseView& view, const android_xr::schemas::BuiltInMaterial1b616c8a& spec) {
  MaterialPreCompileOptions material_pre_compile_options;
  MaterialPreCompileConstant use_super_sampling_constant;
  use_super_sampling_constant.name = kUseSuperSamplingParameter;
  use_super_sampling_constant.value = false;
#if IMP_PLATFORM(ANDROID)
  char propValue[PROP_VALUE_MAX];
  int len = __system_property_get(kUseSuperSamplingProperty, propValue);
  if (len > 0) {
    std::string value(propValue, len);
    if (value == "true") {
      use_super_sampling_constant.value = true;
    }
  }
#endif  // IMP_PLATFORM(ANDROID)
  if (std::get<bool>(use_super_sampling_constant.value)) {
    IMP_LOG(imp::INFO) << "Super sampling enabled in JXR media material";
  } else {
    IMP_LOG(imp::INFO) << "Super sampling disabled in JXR media material";
  }
  material_pre_compile_options.constants.push_back(use_super_sampling_constant);

  Future<AssetPtr<MaterialAsset>> future = view.GetAssetManager().LoadMaterial(
      GetMaterialResource(spec.shape()), material_pre_compile_options);
  return future.Then([&view](AssetPtr<MaterialAsset> material_asset)
                         -> split_engine::BuiltInMaterialPtr {
    return absl::WrapUnique(new BuiltInJxrMediaMaterial(
        view, view.GetMaterialFactory().CreateMaterial(material_asset)));
  });
}

BuiltInJxrMediaMaterial::BuiltInJxrMediaMaterial(BaseView& view,
                                                 OwnedMaterialPtr material)
    : BuiltInCustomMaterial(std::move(material)), view_(view) {
  // All samplers must have valid textures so set the placeholder texture.
  if (GetMaterial()->HasParameter(kTextureParameter)) {
    GetMaterial()->SetParameter(
        kTextureParameter, view.GetTextureFactory().BorrowPlaceholderTexture());
  }
  if (GetMaterial()->HasParameter(kAuxiliaryTextureParameter)) {
    GetMaterial()->SetParameter(
        kAuxiliaryTextureParameter,
        view.GetTextureFactory().BorrowPlaceholderTexture());
  }
  if (GetMaterial()->HasParameter(kPrimaryDepthTextureParameter)) {
    GetMaterial()->SetParameter(
        kPrimaryDepthTextureParameter,
        view.GetTextureFactory().BorrowPlaceholderTexture());
  }
  if (GetMaterial()->HasParameter(kSecondaryDepthTextureParameter)) {
    GetMaterial()->SetParameter(
        kSecondaryDepthTextureParameter,
        view.GetTextureFactory().BorrowPlaceholderTexture());
  }
  if (GetMaterial()->HasParameter(kPrimaryAlphaMaskTextureParameter)) {
    GetMaterial()->SetParameter(
        kPrimaryAlphaMaskTextureParameter,
        view.GetTextureFactory().BorrowPlaceholderTexture());
  }
  if (GetMaterial()->HasParameter(kAuxiliaryAlphaMaskTextureParameter)) {
    GetMaterial()->SetParameter(
        kAuxiliaryAlphaMaskTextureParameter,
        view.GetTextureFactory().BorrowPlaceholderTexture());
  }
}

split_engine::BuiltInMaterialPtr BuiltInJxrMediaMaterial::Duplicate() const {
  return absl::WrapUnique(new BuiltInJxrMediaMaterial(
      view_, view_.GetMaterialFactory().WrapMaterial(
                 filament::MaterialInstance::duplicate(
                     GetMaterial()->GetFilamentMaterialInstance()))));
}

absl::Status BuiltInJxrMediaMaterial::SetParameters(
    flatbuffers::Verifier& verifier,
    const android_xr::schemas::BuiltInMaterialInstanceParameters& parameters,
    const TextureBorrower& texture_borrower) {
  if (parameters.data_type() != android_xr::schemas::BuiltInMaterialParameters::
                                    BuiltInMaterial1b616c8aParameters) {
    return absl::InvalidArgumentError(
        "This material requires BuiltInMaterial1b616c8aParameters");
  }

  if (!VerifyBuiltInMaterialParameters(
          verifier, parameters.data(),
          android_xr::schemas::BuiltInMaterialParameters::
              BuiltInMaterial1b616c8aParameters)) {
    return absl::InvalidArgumentError("Invalid parameters");
  }

  const android_xr::schemas::BuiltInMaterial1b616c8aParameters*
      xr_media_parameters = parameters.data_as<
          android_xr::schemas::BuiltInMaterial1b616c8aParameters>();
  if (xr_media_parameters->primary_texture() &&
      GetMaterial()->HasParameter(kTextureParameter)) {
    const BorrowedTexturePtr texture =
        texture_borrower(xr_media_parameters->primary_texture()->texture_id());
    if (!texture) {
      return absl::NotFoundError(absl::StrFormat(
          "Texture not found: %d",
          xr_media_parameters->primary_texture()->texture_id()));
    }
    filament::TextureSampler sampler =
        ConvertSampler(*xr_media_parameters->primary_texture()->sampler());
    GetMaterial()->SetParameter(kTextureParameter, texture, sampler);
  }
  if (xr_media_parameters->auxiliary_texture() &&
      GetMaterial()->HasParameter(kAuxiliaryTextureParameter)) {
    const BorrowedTexturePtr texture = texture_borrower(
        xr_media_parameters->auxiliary_texture()->texture_id());
    if (!texture) {
      return absl::NotFoundError(absl::StrFormat(
          "Texture not found: %d",
          xr_media_parameters->auxiliary_texture()->texture_id()));
    }
    filament::TextureSampler sampler =
        ConvertSampler(*xr_media_parameters->auxiliary_texture()->sampler());
    GetMaterial()->SetParameter(kAuxiliaryTextureParameter, texture, sampler);
  }
  if (xr_media_parameters->primary_alpha_mask() &&
      GetMaterial()->HasParameter(kPrimaryAlphaMaskTextureParameter)) {
    const BorrowedTexturePtr texture = texture_borrower(
        xr_media_parameters->primary_alpha_mask()->texture_id());
    if (!texture) {
      return absl::NotFoundError(absl::StrFormat(
          "Texture not found: %d",
          xr_media_parameters->primary_alpha_mask()->texture_id()));
    }
    filament::TextureSampler sampler =
        ConvertSampler(*xr_media_parameters->primary_alpha_mask()->sampler());
    GetMaterial()->SetParameter(kPrimaryAlphaMaskTextureParameter, texture,
                                sampler);
  }

  if (xr_media_parameters->auxiliary_alpha_mask() &&
      GetMaterial()->HasParameter(kAuxiliaryAlphaMaskTextureParameter)) {
    const BorrowedTexturePtr texture = texture_borrower(
        xr_media_parameters->auxiliary_alpha_mask()->texture_id());
    if (!texture) {
      return absl::NotFoundError(absl::StrFormat(
          "Texture not found: %d",
          xr_media_parameters->auxiliary_alpha_mask()->texture_id()));
    }
    filament::TextureSampler sampler =
        ConvertSampler(*xr_media_parameters->auxiliary_alpha_mask()->sampler());
    GetMaterial()->SetParameter(kAuxiliaryAlphaMaskTextureParameter, texture,
                                sampler);
  }

  GetMaterial()->SetParameter(
      kStereoTypeParameter,
      static_cast<int>(xr_media_parameters->stereo_type()));
  if (xr_media_parameters->feather_radius()) {
    GetMaterial()->SetParameter(
        kFeatherRadiusParameter,
        imp::float2(xr_media_parameters->feather_radius()->x(),
                    xr_media_parameters->feather_radius()->y()));
  }

  return absl::OkStatus();
}

}  // namespace imp::split_engine
