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

#include "core/split_engine/materials/builtin/builtin_youtube_stereo_player_material.h"

#include <functional>
#include <utility>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "flatbuffers/verifier.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/future.h"
#include "core/common/registry.h"
#include "core/material_library/flatbuffer_utils.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/render/android/platform_android_external_texture_surface.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/builtin/builtin_custom_material.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/materials/builtin/builtin_youtube_stereo_player_material_assets.h"
#include "core/split_engine/split_engine_renderer.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/view_events.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

namespace {
constexpr absl::string_view kMaterialConstantStereoType = "stereoType";
constexpr absl::string_view kMaterialConstantEnableColorCorrection =
    "enableColorCorrection";

constexpr absl::string_view kColorConversionMatrixParameter =
    "colorConversionMatrix";
constexpr absl::string_view kTransferFunctionParameter = "transferFunction";
constexpr absl::string_view kMaxContentLightLevelParameter =
    "maxContentLightLevel";
}  // namespace

Future<BuiltInMaterialPtr> BuiltInYouTubeStereoPlayerMaterial::Create(
    BaseView& view, const android_xr::schemas::BuiltInMaterialEb117dd9& spec) {
  MaterialPreCompileOptions material_precompile_options;
  MaterialPreCompileConstant stereo_type_constant;
  stereo_type_constant.name = kMaterialConstantStereoType;
  stereo_type_constant.value = 0;
  switch (spec.stereo_type()) {
    case android_xr::schemas::BuiltInMaterialEb117dd9StereoType::MONO:
      stereo_type_constant.value = 0;
      break;
    case android_xr::schemas::BuiltInMaterialEb117dd9StereoType::TOP_BOTTOM:
      stereo_type_constant.value = 1;
      break;
    case android_xr::schemas::BuiltInMaterialEb117dd9StereoType::LEFT_RIGHT:
      stereo_type_constant.value = 2;
      break;
    default:
      break;
  }
  material_precompile_options.constants.push_back(stereo_type_constant);

  MaterialPreCompileConstant enable_color_correction_constant;
  enable_color_correction_constant.name =
      kMaterialConstantEnableColorCorrection;
  enable_color_correction_constant.value = true;
  material_precompile_options.constants.push_back(
      enable_color_correction_constant);

  return view.GetAssetManager()
      .LoadMaterial(kBuiltinYoutubeStereoPlayerMatCmat,
                    material_precompile_options)
      .Then([&view](AssetPtr<imp::MaterialAsset> material_asset)
                -> BuiltInMaterialPtr {
        return absl::WrapUnique(new BuiltInYouTubeStereoPlayerMaterial(
            view, view.GetMaterialFactory().CreateMaterial(material_asset)));
      });
}

BuiltInYouTubeStereoPlayerMaterial::BuiltInYouTubeStereoPlayerMaterial(
    BaseView& view, OwnedMaterialPtr material)
    : BuiltInCustomMaterial(std::move(material)), view_(view) {
  // All samplers must have valid textures so set the placeholder texture.
  GetMaterial()->SetParameter(
      "videoTexture", view.GetTextureFactory().BorrowPlaceholderTexture());

  // Listens for the post frame update event to update the color space
  // parameters.
  post_frame_update_connection_ = view_.GetDispatcher().Connect(
      [this](const imp::ViewPostFrameUpdateEvent& event) {
        UpdateColorSpaceParameters();
      });
}

BuiltInMaterialPtr BuiltInYouTubeStereoPlayerMaterial::Duplicate() const {
  return absl::WrapUnique(new BuiltInYouTubeStereoPlayerMaterial(
      view_, view_.GetMaterialFactory().WrapMaterial(
                 filament::MaterialInstance::duplicate(
                     GetMaterial()->GetFilamentMaterialInstance()))));
}

absl::Status BuiltInYouTubeStereoPlayerMaterial::SetParameters(
    flatbuffers::Verifier& verifier,
    const android_xr::schemas::BuiltInMaterialInstanceParameters& parameters,
    const TextureBorrower& texture_borrower) {
  if (parameters.data_type() != android_xr::schemas::BuiltInMaterialParameters::
                                    BuiltInMaterialEb117dd9Parameters) {
    return absl::InvalidArgumentError(
        "This material requires BuiltInMaterialEb117dd9Parameters");
  }

  if (!VerifyBuiltInMaterialParameters(
          verifier, parameters.data(),
          android_xr::schemas::BuiltInMaterialParameters::
              BuiltInMaterialEb117dd9Parameters)) {
    return absl::InvalidArgumentError("Invalid parameters");
  }

  const android_xr::schemas::BuiltInMaterialEb117dd9Parameters*
      youtube_stereo_player_material_parameters = parameters.data_as<
          android_xr::schemas::BuiltInMaterialEb117dd9Parameters>();

  if (youtube_stereo_player_material_parameters->video_texture()) {
    const BorrowedTexturePtr texture = texture_borrower(
        youtube_stereo_player_material_parameters->video_texture()
            ->texture_id());
    if (!texture) {
      return absl::NotFoundError(absl::StrFormat(
          "Texture not found: %d",
          youtube_stereo_player_material_parameters->video_texture()
              ->texture_id()));
    }
    filament::TextureSampler sampler = ConvertSampler(
        *youtube_stereo_player_material_parameters->video_texture()->sampler());
    GetMaterial()->SetParameter("videoTexture", texture, sampler);
    // Store the texture id for later color space update.
    texture_id_ = youtube_stereo_player_material_parameters->video_texture()
                      ->texture_id();
  }
  if (youtube_stereo_player_material_parameters->flip_vertically()) {
    GetMaterial()->SetParameter(
        "flipVertically",
        youtube_stereo_player_material_parameters->flip_vertically()->value());
  }
  if (youtube_stereo_player_material_parameters->flip_horizontally()) {
    GetMaterial()->SetParameter(
        "flipHorizontally",
        youtube_stereo_player_material_parameters->flip_horizontally()
            ->value());
  }
  if (youtube_stereo_player_material_parameters->is_stereo()) {
    GetMaterial()->SetParameter(
        "isStereo",
        youtube_stereo_player_material_parameters->is_stereo()->value());
  }
  if (youtube_stereo_player_material_parameters->is_left_right_stereo()) {
    GetMaterial()->SetParameter(
        "isLeftRightStereo",
        youtube_stereo_player_material_parameters->is_left_right_stereo()
            ->value());
  }
  if (youtube_stereo_player_material_parameters->eye_mode()) {
    GetMaterial()->SetParameter(
        "eyeMode",
        youtube_stereo_player_material_parameters->eye_mode()->value());
  }
  return absl::OkStatus();
}

void BuiltInYouTubeStereoPlayerMaterial::UpdateColorSpaceParameters() {
  // Set the parameters related to the color space. SplitEngineRenderer has
  // the information about the color space of the surface texture.
  absl::StatusOr<std::reference_wrapper<SplitEngineRenderer>> renderer =
      view_.GetRegistry().Get<SplitEngineRenderer>();

  // When the color space is not available, we assume the color space is
  // BT709/sRGB, which is a valid assumption for SDR videos. For HDR videos,
  // it will be Rec.2020(HLG/PQ), but we don't expect this to happen for HDR
  // videos.
  SurfaceColorSpace default_color_space = SurfaceColorSpace();
  default_color_space.SetStandard(SurfaceColorSpace::Standard::kBT709);
  default_color_space.SetTransfer(SurfaceColorSpace::Transfer::kSRGB);
  default_color_space.SetRange(SurfaceColorSpace::Range::kFull);

  // colorConversionMatrix must be set to identity matrix otherwise the output
  // color will be black. Other parameters have default values defined in the
  // shader.
  if (!renderer.ok() || !texture_id_.has_value()) {
    GetMaterial()->SetParameter(
        kColorConversionMatrixParameter,
        default_color_space.GetColorTransformMatrixDisplayP3().value_or(
            imp::kIdentityMat3f));
    return;
  }
  absl::StatusOr<SurfaceColorSpace> source_texture_color_space =
      renderer->get().GetTextureColorSpace(texture_id_.value());
  if (!source_texture_color_space.ok()) {
    GetMaterial()->SetParameter(
        kColorConversionMatrixParameter,
        default_color_space.GetColorTransformMatrixDisplayP3().value_or(
            imp::kIdentityMat3f));
    return;
  }
  if (source_texture_color_space->GetStandard() ==
      SurfaceColorSpace::Standard::kUnknown) {
    GetMaterial()->SetParameter(
        kColorConversionMatrixParameter,
        default_color_space.GetColorTransformMatrixDisplayP3().value_or(
            imp::kIdentityMat3f));
    return;
  }

  // Set the color space parameters.
  GetMaterial()->SetParameter(kColorConversionMatrixParameter,
                              source_texture_color_space.value()
                                  .GetColorTransformMatrixDisplayP3()
                                  .value_or(imp::kIdentityMat3f));
  GetMaterial()->SetParameter(
      kTransferFunctionParameter,
      static_cast<int>(source_texture_color_space.value().GetTransfer()));
  GetMaterial()->SetParameter(
      kMaxContentLightLevelParameter,
      source_texture_color_space.value().GetMaxContentLightLevel());
}
}  // namespace imp::split_engine
