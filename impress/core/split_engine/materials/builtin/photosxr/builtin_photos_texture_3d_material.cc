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

#include "core/split_engine/materials/builtin/photosxr/builtin_photos_texture_3d_material.h"

#include <utility>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "flatbuffers/verifier.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/material/material_asset.h"
#include "core/async/future.h"
#include "core/material_library/flatbuffer_utils.h"
#include "core/materials/material.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/builtin/builtin_custom_material.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/materials/builtin/photosxr/builtin_photos_texture_3d_material_assets.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

Future<BuiltInMaterialPtr> BuiltInPhotosTexture3dMaterial::Create(
    BaseView& view, const android_xr::schemas::BuiltInMaterialD1750064& spec) {
  return view.GetAssetManager()
      .LoadMaterial(kBuiltinPhotosTexture3dMatCmat)
      .Then([&view](
                AssetPtr<MaterialAsset> material_asset) -> BuiltInMaterialPtr {
        return absl::WrapUnique(new BuiltInPhotosTexture3dMaterial(
            view, view.GetMaterialFactory().CreateMaterial(material_asset)));
      });
}

BuiltInMaterialPtr BuiltInPhotosTexture3dMaterial::Duplicate() const {
  return absl::WrapUnique(new BuiltInPhotosTexture3dMaterial(
      view_, view_.GetMaterialFactory().WrapMaterial(
                 filament::MaterialInstance::duplicate(
                     GetMaterial()->GetFilamentMaterialInstance()))));
}

BuiltInPhotosTexture3dMaterial::BuiltInPhotosTexture3dMaterial(
    BaseView& view, OwnedMaterialPtr material)
    : BuiltInCustomMaterial(std::move(material)), view_(view) {
  // All samplers must have valid textures so set the placeholder texture.
  GetMaterial()->SetParameter(
      "imageTexture", view.GetTextureFactory().BorrowPlaceholderTexture());
  GetMaterial()->SetParameter(
      "videoTexture", view.GetTextureFactory().BorrowPlaceholderTexture());
  GetMaterial()->SetParameter(
      "auxiliaryVideoTexture",
      view.GetTextureFactory().BorrowPlaceholderTexture());
  GetMaterial()->SetParameter(
      "thumbnailTexture", view.GetTextureFactory().BorrowPlaceholderTexture());
  GetMaterial()->SetParameter(
      "blurTexture", view.GetTextureFactory().BorrowPlaceholderTexture());

  // Set the default tint color to white.
  GetMaterial()->SetParameter("tintColor", imp::float3(1.0f, 1.0f, 1.0f));
}

absl::Status BuiltInPhotosTexture3dMaterial::SetParameters(
    flatbuffers::Verifier& verifier,
    const android_xr::schemas::BuiltInMaterialInstanceParameters& parameters,
    const TextureBorrower& texture_borrower) {
  if (parameters.data_type() != android_xr::schemas::BuiltInMaterialParameters::
                                    BuiltInMaterialD1750064Parameters) {
    return absl::InvalidArgumentError(
        "This material requires BuiltInMaterialD1750064Parameters");
  }
  if (!VerifyBuiltInMaterialParameters(
          verifier, parameters.data(),
          android_xr::schemas::BuiltInMaterialParameters::
              BuiltInMaterialD1750064Parameters)) {
    return absl::InvalidArgumentError("Invalid parameters");
  }

  const android_xr::schemas::BuiltInMaterialD1750064Parameters* p =
      parameters
          .data_as<android_xr::schemas::BuiltInMaterialD1750064Parameters>();

  if (p->image_texture()) {
    const BorrowedTexturePtr texture =
        texture_borrower(p->image_texture()->texture_id());
    if (!texture) {
      return absl::NotFoundError(absl::StrFormat(
          "Texture not found: %d", p->image_texture()->texture_id()));
    }
    filament::TextureSampler sampler =
        ConvertSampler(*p->image_texture()->sampler());
    GetMaterial()->SetParameter("imageTexture", texture, sampler);
  }

  if (p->video_texture()) {
    const BorrowedTexturePtr texture =
        texture_borrower(p->video_texture()->texture_id());
    if (!texture) {
      return absl::NotFoundError(absl::StrFormat(
          "Texture not found: %d", p->video_texture()->texture_id()));
    }
    filament::TextureSampler sampler =
        ConvertSampler(*p->video_texture()->sampler());
    GetMaterial()->SetParameter("videoTexture", texture, sampler);
  }

  if (p->auxiliary_video_texture()) {
    const BorrowedTexturePtr texture =
        texture_borrower(p->auxiliary_video_texture()->texture_id());
    if (!texture) {
      return absl::NotFoundError(absl::StrFormat(
          "Texture not found: %d", p->auxiliary_video_texture()->texture_id()));
    }
    filament::TextureSampler sampler =
        ConvertSampler(*p->auxiliary_video_texture()->sampler());
    GetMaterial()->SetParameter("auxiliaryVideoTexture", texture, sampler);
  }

  if (p->thumbnail_texture()) {
    const BorrowedTexturePtr texture =
        texture_borrower(p->thumbnail_texture()->texture_id());
    if (!texture) {
      return absl::NotFoundError(absl::StrFormat(
          "Texture not found: %d", p->thumbnail_texture()->texture_id()));
    }
    filament::TextureSampler sampler =
        ConvertSampler(*p->thumbnail_texture()->sampler());
    GetMaterial()->SetParameter("thumbnailTexture", texture, sampler);
  }

  if (p->blur_texture()) {
    const BorrowedTexturePtr texture =
        texture_borrower(p->blur_texture()->texture_id());
    if (!texture) {
      return absl::NotFoundError(absl::StrFormat(
          "Texture not found: %d", p->blur_texture()->texture_id()));
    }
    filament::TextureSampler sampler =
        ConvertSampler(*p->blur_texture()->sampler());
    GetMaterial()->SetParameter("blurTexture", texture, sampler);
  }

  if (p->show_video()) {
    GetMaterial()->SetParameter("showVideo", p->show_video()->value());
  }

  if (p->thumbnail_mix()) {
    GetMaterial()->SetParameter("thumbnailMix", p->thumbnail_mix()->value());
  }

  if (p->blur_mix()) {
    GetMaterial()->SetParameter("blurMix", p->blur_mix()->value());
  }

  if (p->blur_fallback_color()) {
    GetMaterial()->SetParameter(
        "blurFallbackColor", FromFloat3Flatbuffer(*p->blur_fallback_color()));
  }

  if (p->is_stereo()) {
    GetMaterial()->SetParameter("isStereo", p->is_stereo()->value());
  }

  if (p->is_stereo_using_auxiliary_texture()) {
    GetMaterial()->SetParameter(
        "isStereoUsingAuxiliaryTexture",
        p->is_stereo_using_auxiliary_texture()->value());
  }

  if (p->stereo_axis()) {
    GetMaterial()->SetParameter("stereoAxis", p->stereo_axis()->value());
  }

  if (p->stereo_disparity_adjustment()) {
    GetMaterial()->SetParameter("stereoDisparityAdjustment",
                                p->stereo_disparity_adjustment()->value());
  }

  if (p->uv_transform()) {
    GetMaterial()->SetParameter("uvTransform",
                                FromMat3fFlatbuffer(*p->uv_transform()));
  }
  if (p->parallax_amount()) {
    GetMaterial()->SetParameter("parallaxAmount",
                                p->parallax_amount()->value());
  }

  if (p->extra_zoom_after_parallax()) {
    GetMaterial()->SetParameter("extraZoomAfterParallax",
                                p->extra_zoom_after_parallax()->value());
  }

  if (p->max_window_pop_distance()) {
    GetMaterial()->SetParameter("maxWindowPopDistance",
                                p->max_window_pop_distance()->value());
  }

  if (p->max_media_inset_distance()) {
    GetMaterial()->SetParameter("maxMediaInsetDistance",
                                p->max_media_inset_distance()->value());
  }

  if (p->edge_fade_amount()) {
    GetMaterial()->SetParameter("edgeFadeAmount",
                                p->edge_fade_amount()->value());
  }

  if (p->window_edge_fade_thickness()) {
    GetMaterial()->SetParameter("windowEdgeFadeThickness",
                                p->window_edge_fade_thickness()->value());
  }

  if (p->inset_media_edge_fade_thickness()) {
    GetMaterial()->SetParameter("insetMediaEdgeFadeThickness",
                                p->inset_media_edge_fade_thickness()->value());
  }

  if (p->corner_radius()) {
    GetMaterial()->SetParameter("cornerRadius", p->corner_radius()->value());
  }

  if (p->curve_params()) {
    GetMaterial()->SetParameter("curveParams",
                                FromFloat3Flatbuffer(*p->curve_params()));
  }

  if (p->blur_center_clear_amount()) {
    GetMaterial()->SetParameter("blurCenterClearAmount",
                                p->blur_center_clear_amount()->value());
  }

  if (p->blur_center_edge_fade_thickness()) {
    GetMaterial()->SetParameter("blurCenterEdgeFadeThickness",
                                p->blur_center_edge_fade_thickness()->value());
  }

  if (p->blur_center_mask_flip()) {
    GetMaterial()->SetParameter("blurCenterMaskFlip",
                                p->blur_center_mask_flip()->value());
  }

  if (p->noise_in_blur_mix()) {
    GetMaterial()->SetParameter("noiseInBlurMix",
                                p->noise_in_blur_mix()->value());
  }

  if (p->frame_time_seconds()) {
    GetMaterial()->SetParameter("frameTimeSeconds",
                                p->frame_time_seconds()->value());
  }

  if (p->time_scale()) {
    GetMaterial()->SetParameter("timeScale", p->time_scale()->value());
  }

  if (p->pulse_xy_scale()) {
    GetMaterial()->SetParameter("pulseXyScale", p->pulse_xy_scale()->value());
  }

  if (p->pulse_exponent()) {
    GetMaterial()->SetParameter("pulseExponent",
                                FromFloat4Flatbuffer(*p->pulse_exponent()));
  }

  if (p->pulse_multiplier()) {
    GetMaterial()->SetParameter("pulseMultiplier",
                                FromFloat4Flatbuffer(*p->pulse_multiplier()));
  }

  if (p->grain_xy_scale()) {
    GetMaterial()->SetParameter("grainXyScale", p->grain_xy_scale()->value());
  }

  if (p->grain_mask_exponent_and_multiplier()) {
    GetMaterial()->SetParameter(
        "grainMaskExpAndMultiplier",
        FromFloat2Flatbuffer(*p->grain_mask_exponent_and_multiplier()));
  }

  if (p->grain_dim_amount()) {
    GetMaterial()->SetParameter("grainDimAmount",
                                p->grain_dim_amount()->value());
  }
  if (p->grain_exponent_and_multiplier()) {
    GetMaterial()->SetParameter(
        "grainExpAndMultiplier",
        FromFloat2Flatbuffer(*p->grain_exponent_and_multiplier()));
  }

  if (p->is_pano()) {
    GetMaterial()->SetParameter("isPano", p->is_pano()->value());
  }

  if (p->pano_fov_degrees()) {
    GetMaterial()->SetParameter("panoFovDegrees",
                                FromFloat3Flatbuffer(*p->pano_fov_degrees()));
  }

  if (p->pano_spherical_amount()) {
    GetMaterial()->SetParameter("panoSphericalAmount",
                                p->pano_spherical_amount()->value());
  }
  if (p->pano_sphere_radius()) {
    GetMaterial()->SetParameter("panoSphereRadius",
                                p->pano_sphere_radius()->value());
  }

  if (p->pano_blur_fade_to_gray_exponent()) {
    GetMaterial()->SetParameter(
        "panoBlurFadeToGrayExponent",
        FromFloat2Flatbuffer(*p->pano_blur_fade_to_gray_exponent()));
  }

  if (p->pano_blur_fade_to_gray_subtractor()) {
    GetMaterial()->SetParameter(
        "panoBlurFadeToGraySubtractor",
        FromFloat2Flatbuffer(*p->pano_blur_fade_to_gray_subtractor()));
  }

  if (p->pano_media_edge_fade_exponent()) {
    GetMaterial()->SetParameter("panoMediaEdgeFadeExponent",
                                p->pano_media_edge_fade_exponent()->value());
  }

  if (p->pano_corner_radius_decrease_exponent()) {
    GetMaterial()->SetParameter(
        "panoCornerRadiusDecreaseExponent",
        p->pano_corner_radius_decrease_exponent()->value());
  }

  if (p->gamma_to_srgb()) {
    GetMaterial()->SetParameter("gammaToSrgb", p->gamma_to_srgb()->value());
  }

  if (p->tint_color()) {
    GetMaterial()->SetParameter("tintColor",
                                FromFloat3Flatbuffer(*p->tint_color()));
  }

  if (p->opacity()) {
    GetMaterial()->SetParameter("opacity", p->opacity()->value());
  }

  return absl::OkStatus();
}

}  // namespace imp::split_engine
