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

#ifndef THIRD_PARTY_SPLIT_ENGINE_MATERIALS_JXR_MEDIA_MATERIAL_H_
#define THIRD_PARTY_SPLIT_ENGINE_MATERIALS_JXR_MEDIA_MATERIAL_H_

#include <sys/types.h>

#include <memory>
#include <optional>

#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/future.h"
#include "core/math/vec.h"
#include "core/media/media_color_space.h"
#include "core/media/media_type.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_builtin_material.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"

namespace android_xr {

// A material wrapper for the built-in JxrMediaMaterial to support Split Engine.
//
// Note: This is the split-engine app side of BuiltInJxrMediaMaterial.
class JxrMediaMaterial : public imp::split_engine::SplitEngineBuiltinMaterial {
 public:
  // Creates a new JxrMediaMaterial.
  static imp::Future<std::unique_ptr<JxrMediaMaterial>> Create(
      imp::BaseView& view,
      imp::MediaShapeType shape_type = imp::MediaShapeType::kDefaultFlat,
      bool use_super_sampling = false,
      imp::RenderEyeTarget render_eye_target = imp::RenderEyeTarget::kBoth,
      imp::MediaBlendingMode blending_mode =
          imp::MediaBlendingMode::kTransparent);

  ~JxrMediaMaterial() override;

  flatbuffers::Offset<void> SerializeParameters(
      flatbuffers::FlatBufferBuilder& fbb,
      imp::split_engine::BuiltInTextureParameterCreator&
          texture_parameter_creator) const override;

  void SetPrimaryTexture(imp::OwnedOrBorrowedTexturePtr texture);
  void SetAuxiliaryTexture(imp::OwnedOrBorrowedTexturePtr texture);
  void SetPrimaryDepthTexture(imp::OwnedOrBorrowedTexturePtr texture);
  void SetAuxiliaryDepthTexture(imp::OwnedOrBorrowedTexturePtr texture);
  void SetPrimaryAlphaMask(imp::OwnedOrBorrowedTexturePtr alpha_mask);
  void SetAuxiliaryAlphaMask(imp::OwnedOrBorrowedTexturePtr alpha_mask);
  void SetStereoType(imp::MediaStereoMode stereo_type);
  void SetContentColorMetadata(imp::MediaColorSpace color_space);
  void ResetContentColorMetadata();

  void SetFeatherRadius(imp::float2 feather_radius);
  void SetCornerRadius(imp::float2 corner_radius);
  void SetSubViewConfig(imp::float4 sub_view_rect_left,
                        imp::float4 sub_view_rect_right);

 private:
  JxrMediaMaterial(imp::BaseView& view,
                   imp::split_engine::PlaceholderOrBuiltInMaterialPtr material);

  // This tracks the "Spatial API level" of the system image; we're depending on
  // the Serializer reporting the same value as XrExtensions.GetApiLevel().
  int spatial_api_level_;

  imp::OwnedOrBorrowedTexturePtr primary_texture_;
  imp::OwnedOrBorrowedTexturePtr auxiliary_texture_;
  imp::OwnedOrBorrowedTexturePtr primary_alpha_mask_;
  imp::OwnedOrBorrowedTexturePtr auxiliary_alpha_mask_;
  imp::MediaStereoMode stereo_type_ = imp::MediaStereoMode::kMonoscopic;
  imp::MediaColorSpace color_space_;
  std::optional<android_xr::schemas::Float2> feather_radius_;
  std::optional<android_xr::schemas::Float2> corner_radius_;
  std::optional<android_xr::schemas::Float4> sub_view_rect_left_;
  std::optional<android_xr::schemas::Float4> sub_view_rect_right_;
};

}  // namespace android_xr

#endif  // THIRD_PARTY_SPLIT_ENGINE_MATERIALS_JXR_MEDIA_MATERIAL_H_
