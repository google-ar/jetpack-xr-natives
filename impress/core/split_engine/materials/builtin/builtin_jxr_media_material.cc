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

#include <sys/stat.h>
#include <sys/types.h>

#include <memory>
#include <utility>

#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/variant.h"
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
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/media/media_color_space.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/render/display_color_space.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/resources/resource_definition.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/materials/builtin/builtin_custom_material.h"
#include "core/split_engine/materials/builtin/builtin_jxr_media_material_assets.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/view_events.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"

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
static constexpr absl::string_view kMaterialConstantUseSuperSampling =
    "useSuperSampling";

constexpr absl::string_view kMaterialConstantEnableColorCorrection =
    "enableColorCorrection";

#if IMP_PLATFORM(ANDROID)
// Use `adb shell setprop jxr.surface_entity.enable_color_correction
// Unset/ForceOn/ForceOff` to set the color correction mode.
static constexpr const char* kEnableColorCorrectionProperty =
    "jxr.surface_entity.enable_color_correction";
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
    BaseView& view, BridgeId bridge_id,
    const android_xr::schemas::BuiltInMaterial1b616c8a& spec) {
  MaterialPreCompileOptions material_pre_compile_options;
  MaterialPreCompileConstant use_super_sampling_constant;
  use_super_sampling_constant.name = kMaterialConstantUseSuperSampling;
  use_super_sampling_constant.value = true;

  const android_xr::schemas::Bool* use_super_sampling_spec =
      spec.use_super_sampling();
  if (use_super_sampling_spec) {
    use_super_sampling_constant.value = UnPack(*use_super_sampling_spec);
  }

  MaterialPreCompileConstant enable_color_correction_constant;
  enable_color_correction_constant.name =
      kMaterialConstantEnableColorCorrection;
  enable_color_correction_constant.value = true;

#if IMP_PLATFORM(ANDROID)
  char propValue[PROP_VALUE_MAX];
  int len = __system_property_get(kEnableColorCorrectionProperty, propValue);
  if (len > 0) {
    std::string value(propValue, len);
    if (value == "ForceOff") {
      enable_color_correction_constant.value = false;
    }
  }
#endif  // IMP_PLATFORM(ANDROID)

  if (std::get<bool>(use_super_sampling_constant.value)) {
    IMP_LOG(imp::INFO) << "Super sampling enabled in JXR media material";
  } else {
    IMP_LOG(imp::INFO) << "Super sampling disabled in JXR media material";
  }
  material_pre_compile_options.constants.push_back(use_super_sampling_constant);

  if (std::get<bool>(enable_color_correction_constant.value)) {
    IMP_LOG(imp::INFO) << "Color correction enabled in JXR media material";
  } else {
    IMP_LOG(imp::INFO) << "Color correction disabled in JXR media material";
  }
  material_pre_compile_options.constants.push_back(
      enable_color_correction_constant);

  Future<AssetPtr<MaterialAsset>> future = view.GetAssetManager().LoadMaterial(
      GetMaterialResource(spec.shape()), material_pre_compile_options);
  return future.Then([&view, bridge_id, enable_color_correction_constant](
                         AssetPtr<MaterialAsset> material_asset)
                         -> split_engine::BuiltInMaterialPtr {
    std::unique_ptr<BuiltInJxrMediaMaterial> material =
        absl::WrapUnique(new BuiltInJxrMediaMaterial(
            view, bridge_id,
            view.GetMaterialFactory().CreateMaterial(material_asset)));
    material->SetColorCorrectionMode(
        absl::get<bool>(enable_color_correction_constant.value));
    return std::move(material);
  });
}

void BuiltInJxrMediaMaterial::SetColorCorrectionMode(
    bool enable_color_correction) {
  if (enable_color_correction) {
    color_correction_mode_ = ColorCorrectionMode::kSystemBestEffort;
    IMP_LOG(imp::INFO) << "JXR media material: Color correction enabled, mode set to "
                 "best effort.";
  } else {
    color_correction_mode_ = ColorCorrectionMode::kDisabled;
    IMP_LOG(imp::INFO) << "JXR media material: Color correction disabled.";
  }
}

BuiltInJxrMediaMaterial::BuiltInJxrMediaMaterial(BaseView& view,
                                                 BridgeId bridge_id,
                                                 OwnedMaterialPtr material)
    : BuiltInCustomMaterial(bridge_id, std::move(material)), view_(view) {
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
      view_, GetBridgeId(),
      view_.GetMaterialFactory().WrapMaterial(
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
    GetMaterial()->SetParameter(
        kTextureParameter, texture,
        ConvertSampler(xr_media_parameters->primary_texture()->sampler()));
    // Store the primary texture id to update the color space parameters later
    // in the post frame update event. We assume the auxiliary texture, if
    // present, has the same color space as the primary texture.
    primary_media_texture_id_ =
        xr_media_parameters->primary_texture()->texture_id();
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
    GetMaterial()->SetParameter(
        kAuxiliaryTextureParameter, texture,
        ConvertSampler(xr_media_parameters->auxiliary_texture()->sampler()));
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
    GetMaterial()->SetParameter(
        kPrimaryAlphaMaskTextureParameter, texture,
        ConvertSampler(xr_media_parameters->primary_alpha_mask()->sampler()));
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
    GetMaterial()->SetParameter(
        kAuxiliaryAlphaMaskTextureParameter, texture,
        ConvertSampler(xr_media_parameters->auxiliary_alpha_mask()->sampler()));
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

  return SetupColorCorrectionParameters(xr_media_parameters);
}

// This function handles setting up color correction based on user preferences
// and available information.
absl::Status BuiltInJxrMediaMaterial::SetupColorCorrectionParameters(
    const android_xr::schemas::BuiltInMaterial1b616c8aParameters*
        xr_media_parameters) {
  // If color correction is disabled, just return.
  if (color_correction_mode_ == ColorCorrectionMode::kDisabled) {
    // If disabled, disconnect any existing post frame update connection.
    if (post_frame_update_connected_) {
      post_frame_update_connection_.Disconnect();
      post_frame_update_connected_ = false;
    }
    IMP_LOG(imp::INFO) << "JXR media material: color correction is disabled, ignoring "
                 "media color space parameters.";
    return absl::OkStatus();
  }

  // Initialize the color space with unknown parameters.
  MediaColorSpace color_space;
  // Check if the app (user) has provided valid surface color space parameters.
  if (xr_media_parameters->media_color_space()) {
    // If provided, construct a MediaColorSpace object from the parameters.
    color_space = MediaColorSpace(
        static_cast<MediaColorSpace::Standard>(
            xr_media_parameters->media_color_space()->color_standard()),
        static_cast<MediaColorSpace::Transfer>(
            xr_media_parameters->media_color_space()->color_transfer()),
        static_cast<MediaColorSpace::Range>(
            xr_media_parameters->media_color_space()->color_range()),
        xr_media_parameters->media_color_space()->max_luminance());
    IMP_LOG(imp::INFO)
        << "JXR media material: media color space is provided by the app: "
        << color_space.ToString();
  } else {
    IMP_LOG(imp::ERROR)
        << "JXR media material: media color space is not provided by the app.";
  }

  // If the obtained color space is valid/known, set the color correction mode
  // to user override and override the color space parameters in the material.
  if (color_space.IsKnown()) {
    color_correction_mode_ = ColorCorrectionMode::kUserOverride;
    // Disconnect the post frame update connection as automated updates are not
    // needed in this mode.
    post_frame_update_connection_.Disconnect();
    post_frame_update_connected_ = false;
    // Apply the provided color space parameters directly.
    OverrideColorSpaceParameters(view_, color_space);
    IMP_LOG(imp::INFO) << "JXR media material: Color correction mode is user override, "
                 "disconnected from post frame update event.";
  } else {
    // If the color space is unknown (not provided or invalid), fall back to the
    // system's best effort color correction mode.
    IMP_LOG(imp::INFO) << "JXR media material: user provided color space is unknown, "
                 "falling back to best effort mode.";
    color_correction_mode_ = ColorCorrectionMode::kSystemBestEffort;
    // If not already connected, connect to the post frame update event.
    // This allows the system to update color space parameters dynamically.
    if (!post_frame_update_connected_) {
      post_frame_update_connection_ = view_.GetDispatcher().Connect(
          [this](const imp::ViewPostFrameUpdateEvent& event) {
            // The lambda function updates color space parameters on each post
            // frame update.
            UpdateColorSpaceParameters(view_, primary_media_texture_id_);
          });
      post_frame_update_connected_ = true;
      IMP_LOG(imp::INFO) << "JXR media material: color correction mode is best effort, "
                   "connected to post frame update event.";
    }
  }

  // Return status indicating successful setup.
  return absl::OkStatus();
}

DisplayColorSpace BuiltInJxrMediaMaterial::GetRequiredDisplayColorSpace()
    const {
  // By default, use P3 as the display color space for JXR Media material.
  return DisplayColorSpace::kP3;
}

}  // namespace imp::split_engine
