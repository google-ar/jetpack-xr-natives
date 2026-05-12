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

#include "split_engine/materials/photos_texture_3d_material.h"

#include <memory>
#include <utility>

#include "absl/memory/memory.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/future.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_builtin_material.h"
#include "core/view/base_view.h"
#include "split_engine/materials/photos_texture_3d_material_params.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace android_xr {

imp::Future<std::unique_ptr<PhotosTexture3DMaterial>>
PhotosTexture3DMaterial::Create(imp::BaseView& view,
                                const PhotosTexture3DMaterialParams& params) {
  auto fbb = std::make_unique<flatbuffers::FlatBufferBuilder>();
  flatbuffers::Offset<android_xr::schemas::BuiltInMaterialD1750064>
      spec_offset = android_xr::schemas::CreateBuiltInMaterialD1750064(*fbb);
  return imp::split_engine::SplitEngineBuiltinMaterial::RequestBuiltInMaterial(
             view, std::move(fbb),
             android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialD1750064,
             spec_offset.Union())
      .Then([&view, &params](
                imp::split_engine::PlaceholderOrBuiltInMaterialPtr material) {
        return absl::WrapUnique(
            new PhotosTexture3DMaterial(view, params, std::move(material)));
      });
}

PhotosTexture3DMaterial::PhotosTexture3DMaterial(
    imp::BaseView& view, const PhotosTexture3DMaterialParams& params,
    imp::split_engine::PlaceholderOrBuiltInMaterialPtr material)
    : SplitEngineBuiltinMaterial(
          view,
          android_xr::schemas::BuiltInMaterialParameters::
              BuiltInMaterialD1750064Parameters,
          std::move(material)),
      params_(params) {}

PhotosTexture3DMaterial::~PhotosTexture3DMaterial() { Cleanup(); }

flatbuffers::Offset<void> PhotosTexture3DMaterial::SerializeParameters(
    flatbuffers::FlatBufferBuilder& fbb,
    imp::split_engine::BuiltInTextureParameterCreator&
        texture_parameter_creator) const {
  using BuiltInTextureParameter = android_xr::schemas::BuiltInTextureParameter;
  namespace params = photos_params;

  flatbuffers::Offset<BuiltInTextureParameter> image_texture;
  WriteTexture<params::ImageTexture>(image_texture, fbb,
                                     texture_parameter_creator);
  flatbuffers::Offset<BuiltInTextureParameter> video_texture;
  WriteTexture<params::VideoTexture>(video_texture, fbb,
                                     texture_parameter_creator);
  flatbuffers::Offset<BuiltInTextureParameter> auxiliary_video_texture;
  WriteTexture<params::AuxiliaryVideoTexture>(auxiliary_video_texture, fbb,
                                              texture_parameter_creator);
  flatbuffers::Offset<BuiltInTextureParameter> thumbnail_texture;
  WriteTexture<params::ThumbnailTexture>(thumbnail_texture, fbb,
                                         texture_parameter_creator);
  flatbuffers::Offset<BuiltInTextureParameter> blur_texture;
  WriteTexture<params::BlurTexture>(blur_texture, fbb,
                                    texture_parameter_creator);

  // TODO: (broken link) - Change the parameters to all be optional and only
  // serialize the ones that are set / changed.
  return android_xr::schemas::CreateBuiltInMaterialD1750064Parameters(
             fbb, image_texture, video_texture, auxiliary_video_texture,
             thumbnail_texture, blur_texture,
             params_.GetPacked<params::ShowVideo>(),
             params_.GetPacked<params::ThumbnailMix>(),
             params_.GetPacked<params::BlurMix>(),
             params_.GetPacked<params::BlurFallbackColor>(),
             params_.GetPacked<params::IsStereo>(),
             params_.GetPacked<params::IsStereoUsingAuxiliaryTexture>(),
             params_.GetPacked<params::StereoAxis>(),
             params_.GetPacked<params::StereoDisparityAdjustment>(),
             params_.GetPacked<params::UvTransform>(),
             params_.GetPacked<params::ParallaxAmount>(),
             params_.GetPacked<params::ExtraZoomAfterParallax>(),
             params_.GetPacked<params::MaxWindowPopDistance>(),
             params_.GetPacked<params::MaxMediaInsetDistance>(),
             params_.GetPacked<params::EdgeFadeAmount>(),
             params_.GetPacked<params::WindowEdgeFadeThickness>(),
             params_.GetPacked<params::InsetMediaEdgeFadeThickness>(),
             params_.GetPacked<params::CornerRadius>(),
             params_.GetPacked<params::CurveParams>(),
             params_.GetPacked<params::BlurCenterClearAmount>(),
             params_.GetPacked<params::BlurCenterEdgeFadeThickness>(),
             params_.GetPacked<params::BlurCenterMaskFlip>(),
             params_.GetPacked<params::NoiseInBlurMix>(),
             params_.GetPacked<params::FrameTimeSeconds>(),
             params_.GetPacked<params::TimeScale>(),
             params_.GetPacked<params::PulseXYScale>(),
             params_.GetPacked<params::PulseExponent>(),
             params_.GetPacked<params::PulseMultiplier>(),
             params_.GetPacked<params::GrainXYScale>(),
             params_.GetPacked<params::GrainMaskExponentAndMultiplier>(),
             params_.GetPacked<params::GrainDimAmount>(),
             params_.GetPacked<params::GrainExponentAndMultiplier>(),
             params_.GetPacked<params::IsPano>(),
             params_.GetPacked<params::PanoFovDegrees>(),
             params_.GetPacked<params::PanoSphericalAmount>(),
             params_.GetPacked<params::PanoSphereRadius>(),
             params_.GetPacked<params::PanoBlurFadeToGrayExponent>(),
             params_.GetPacked<params::PanoBlurFadeToGraySubtractor>(),
             params_.GetPacked<params::PanoMediaEdgeFadeExponent>(),
             params_.GetPacked<params::PanoCornerRadiusDecreaseExponent>(),
             params_.GetPacked<params::GammaToSrgb>(),
             params_.GetPacked<params::TintColor>(),
             params_.GetPacked<params::Opacity>())
      .Union();
}

template <typename Param>
void PhotosTexture3DMaterial::WriteTexture(
    flatbuffers::Offset<android_xr::schemas::BuiltInTextureParameter>& offset,
    flatbuffers::FlatBufferBuilder& fbb,
    imp::split_engine::BuiltInTextureParameterCreator&
        texture_parameter_creator) const {
  auto texture = params_.Get<Param>();
  if (texture.has_value() && texture.value() != nullptr) {
    offset = texture_parameter_creator.Create(fbb, texture.value());
  }
}

}  // namespace android_xr
