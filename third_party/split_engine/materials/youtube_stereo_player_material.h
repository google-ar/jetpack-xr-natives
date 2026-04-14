/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_SPLIT_ENGINE_MATERIALS_YOUTUBE_STEREO_PLAYER_MATERIAL_H_
#define THIRD_PARTY_SPLIT_ENGINE_MATERIALS_YOUTUBE_STEREO_PLAYER_MATERIAL_H_

#include <memory>
#include <optional>

#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_builtin_material.h"
#include "imp.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"

namespace android_xr {

// Displays a YouTube VR video in stereo mode.
class YouTubeStereoPlayerMaterial
    : public imp::split_engine::SplitEngineBuiltinMaterial {
 public:
  static imp::Future<std::unique_ptr<YouTubeStereoPlayerMaterial>> Create(
      imp::BaseView& view);

  ~YouTubeStereoPlayerMaterial() override;

  // Defines which eye the material should be rendered for. The same material
  // is used for rendering each eye, and both eyes.
  enum class EyeMode {
    kLeftEye = 0,
    kRightEye = 1,
    kBothEyes = 2,
  };

  // Sets the video texture to be used for rendering.
  void SetVideoTexture(imp::OwnedOrBorrowedTexturePtr video_texture);
  // Sets whether the video should be flipped horizontally. This flips the uv
  // horizontally.
  void SetFlipHorizontally(bool flip_horizontally);
  // Sets whether the video should be flipped vertically. This flips the uv
  // vertically.
  void SetFlipVertically(bool flip_vertically);
  // Sets whether the video should be rendered in stereo mode. Otherwise it
  // renders in mono mode.
  void SetIsStereo(bool is_stereo);
  // Sets whether the video should be rendered in left-right stereo mode.
  // Otherwise it renders in top-bottom stereo mode.
  void SetIsLeftRightStereo(bool is_left_right_stereo);
  // Sets the eye mode that indicates which eye mesh it's rendering to.
  void SetEyeMode(EyeMode eye_mode);

 protected:
  flatbuffers::Offset<void> SerializeParameters(
      flatbuffers::FlatBufferBuilder& fbb,
      imp::split_engine::BuiltInTextureParameterCreator&
          texture_parameter_creator) const override;

 private:
  YouTubeStereoPlayerMaterial(imp::BaseView& view,
                              imp::OwnedMaterialPtr material);

  imp::OwnedOrBorrowedTexturePtr video_texture_;
  std::optional<android_xr::schemas::Bool> flip_horizontally_;
  std::optional<android_xr::schemas::Bool> flip_vertically_;
  std::optional<android_xr::schemas::Bool> is_stereo_;
  std::optional<android_xr::schemas::Bool> is_left_right_stereo_;
  std::optional<android_xr::schemas::Int> eye_mode_;
};

}  // namespace android_xr

#endif  // THIRD_PARTY_SPLIT_ENGINE_MATERIALS_YOUTUBE_STEREO_PLAYER_MATERIAL_H_
