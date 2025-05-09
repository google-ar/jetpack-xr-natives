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

#include "split_engine/materials/youtube_stereo_player_material.h"

#include <memory>
#include <optional>
#include <utility>

#include "absl/memory/memory.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/future.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_material.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"

namespace android_xr {

imp::Future<std::unique_ptr<YouTubeStereoPlayerMaterial>>
YouTubeStereoPlayerMaterial::Create(imp::BaseView& view) {
  auto fbb = std::make_unique<flatbuffers::FlatBufferBuilder>();
  flatbuffers::Offset<android_xr::schemas::BuiltInMaterialEb117dd9>
      spec_offset = android_xr::schemas::CreateBuiltInMaterialEb117dd9(*fbb);
  return imp::split_engine::SplitEngineMaterial::RequestBuiltInMaterial(
             view, std::move(fbb),
             android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialEb117dd9,
             spec_offset.Union())
      .Then(
          [&view](imp::split_engine::PlaceholderOrBuiltInMaterialPtr material) {
            return absl::WrapUnique(
                new YouTubeStereoPlayerMaterial(view, std::move(material)));
          });
}

YouTubeStereoPlayerMaterial::YouTubeStereoPlayerMaterial(
    imp::BaseView& view,
    imp::split_engine::PlaceholderOrBuiltInMaterialPtr material)
    : SplitEngineMaterial(view,
                          android_xr::schemas::BuiltInMaterialParameters::
                              BuiltInMaterialEb117dd9Parameters,
                          std::move(material)) {}

flatbuffers::Offset<void> YouTubeStereoPlayerMaterial::SerializeParameters(
    flatbuffers::FlatBufferBuilder& fbb,
    imp::split_engine::BuiltInTextureParameterCreator&
        texture_parameter_creator) const {
  flatbuffers::Offset<android_xr::schemas::BuiltInTextureParameter>
      video_texture;
  if (video_texture_) {
    filament::TextureSampler texture_sampler(
        filament::TextureSampler::MinFilter::LINEAR_MIPMAP_NEAREST,
        filament::TextureSampler::MagFilter::LINEAR);
    video_texture = texture_parameter_creator.Create(
        fbb, video_texture_.Borrow(), texture_sampler);
  }
  android_xr::schemas::Bool flip_horizontally;
  if (flip_horizontally_.has_value()) {
    flip_horizontally = android_xr::schemas::Bool(*flip_horizontally_);
  }
  android_xr::schemas::Bool flip_vertically;
  if (flip_vertically_.has_value()) {
    flip_vertically = android_xr::schemas::Bool(*flip_vertically_);
  }
  android_xr::schemas::Bool is_stereo;
  if (is_stereo_.has_value()) {
    is_stereo = android_xr::schemas::Bool(*is_stereo_);
  }
  android_xr::schemas::Bool is_left_right_stereo;
  if (is_left_right_stereo_.has_value()) {
    is_left_right_stereo = android_xr::schemas::Bool(*is_left_right_stereo_);
  }
  // TODO: Remove these two fields from the material.
  android_xr::schemas::Bool is_visible = android_xr::schemas::Bool(true);
  android_xr::schemas::Bool should_show_video = android_xr::schemas::Bool(true);

  android_xr::schemas::Int eye_mode;
  if (eye_mode_.has_value()) {
    eye_mode = android_xr::schemas::Int(*eye_mode_);
  }
  return android_xr::schemas::CreateBuiltInMaterialEb117dd9Parameters(
             fbb, video_texture, &flip_horizontally, &flip_vertically,
             &is_stereo, &is_left_right_stereo, &is_visible, &should_show_video,
             &eye_mode)
      .Union();
}

void YouTubeStereoPlayerMaterial::SetVideoTexture(
    imp::OwnedOrBorrowedTexturePtr video_texture) {
  video_texture_ = std::move(video_texture);
  MarkParametersDirty();
}

void YouTubeStereoPlayerMaterial::SetFlipHorizontally(bool flip_horizontally) {
  flip_horizontally_ = flip_horizontally;
  MarkParametersDirty();
}

void YouTubeStereoPlayerMaterial::SetFlipVertically(bool flip_vertically) {
  flip_vertically_ = flip_vertically;
  MarkParametersDirty();
}

void YouTubeStereoPlayerMaterial::SetIsStereo(bool is_stereo) {
  is_stereo_ = is_stereo;
  MarkParametersDirty();
}

void YouTubeStereoPlayerMaterial::SetIsLeftRightStereo(
    bool is_left_right_stereo) {
  is_left_right_stereo_ = is_left_right_stereo;
  MarkParametersDirty();
}

void YouTubeStereoPlayerMaterial::SetEyeMode(EyeMode eye_mode) {
  switch (eye_mode) {
    case EyeMode::kLeftEye:
      eye_mode_ = 0;
      break;
    case EyeMode::kRightEye:
      eye_mode_ = 1;
      break;
    case EyeMode::kBothEyes:
      eye_mode_ = 2;
      break;
  }
  MarkParametersDirty();
}

}  // namespace android_xr
