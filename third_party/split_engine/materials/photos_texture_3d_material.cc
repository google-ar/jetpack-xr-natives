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
#include <string>
#include <utility>
#include <variant>

#include "absl/memory/memory.h"
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

imp::Future<std::unique_ptr<PhotosTexture3DMaterial>>
PhotosTexture3DMaterial::Create(imp::BaseView& view) {
  auto fbb = std::make_unique<flatbuffers::FlatBufferBuilder>();
  flatbuffers::Offset<android_xr::schemas::BuiltInMaterialD1750064>
      spec_offset = android_xr::schemas::CreateBuiltInMaterialD1750064(*fbb);
  return imp::split_engine::SplitEngineMaterial::RequestBuiltInMaterial(
             view, std::move(fbb),
             android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialD1750064,
             spec_offset.Union())
      .Then(
          [&view](imp::split_engine::PlaceholderOrBuiltInMaterialPtr material) {
            return absl::WrapUnique(
                new PhotosTexture3DMaterial(view, std::move(material)));
          });
}

PhotosTexture3DMaterial::PhotosTexture3DMaterial(
    imp::BaseView& view,
    imp::split_engine::PlaceholderOrBuiltInMaterialPtr material)
    : SplitEngineMaterial(view,
                          android_xr::schemas::BuiltInMaterialParameters::
                              BuiltInMaterialD1750064Parameters,
                          std::move(material)) {}

flatbuffers::Offset<void> PhotosTexture3DMaterial::SerializeParameters(
    flatbuffers::FlatBufferBuilder& fbb,
    imp::split_engine::BuiltInTextureParameterCreator&
        texture_parameter_creator) const {
  using BuiltInTextureParameter = android_xr::schemas::BuiltInTextureParameter;
  using Bool = android_xr::schemas::Bool;
  using Float = android_xr::schemas::Float;
  using Float2 = android_xr::schemas::Float2;
  using Float3 = android_xr::schemas::Float3;
  using Float4 = android_xr::schemas::Float4;
  using Mat3f = android_xr::schemas::Mat3f;

  flatbuffers::Offset<BuiltInTextureParameter> image_texture;
  WriteTexture("image_texture", image_texture, fbb, texture_parameter_creator);
  flatbuffers::Offset<BuiltInTextureParameter> video_texture;
  WriteTexture("video_texture", video_texture, fbb, texture_parameter_creator);
  flatbuffers::Offset<BuiltInTextureParameter> auxiliary_video_texture;
  WriteTexture("auxiliary_video_texture", auxiliary_video_texture, fbb,
               texture_parameter_creator);
  flatbuffers::Offset<BuiltInTextureParameter> thumbnail_texture;
  WriteTexture("thumbnail_texture", thumbnail_texture, fbb,
               texture_parameter_creator);
  flatbuffers::Offset<BuiltInTextureParameter> blur_texture;
  WriteTexture("blur_texture", blur_texture, fbb, texture_parameter_creator);

  // TODO: (broken link) - Change the parameters to all be optional and only
  // serialize the ones that are set / changed.
  return android_xr::schemas::CreateBuiltInMaterialD1750064Parameters(
             fbb, image_texture, video_texture, auxiliary_video_texture,
             thumbnail_texture, blur_texture, ParamOrNull<Bool>("show_video"),
             ParamOrNull<Float>("thumbnail_mix"),
             ParamOrNull<Float>("blur_mix"),
             ParamOrNull<Float3>("blur_fallback_color"),
             ParamOrNull<Bool>("is_stereo"),
             ParamOrNull<Bool>("is_stereo_using_auxiliary_texture"),
             ParamOrNull<Float>("stereo_axis"),
             ParamOrNull<Float>("stereo_disparity_adjustment"),
             ParamOrNull<Mat3f>("uv_transform"),
             ParamOrNull<Float>("parallax_amount"),
             ParamOrNull<Float>("extra_zoom_after_parallax"),
             ParamOrNull<Float>("max_window_pop_distance"),
             ParamOrNull<Float>("max_media_inset_distance"),
             ParamOrNull<Float>("edge_fade_amount"),
             ParamOrNull<Float>("window_edge_fade_thickness"),
             ParamOrNull<Float>("inset_media_edge_fade_thickness"),
             ParamOrNull<Float>("corner_radius"),
             ParamOrNull<Float3>("curve_params"),
             ParamOrNull<Float>("blur_center_clear_amount"),
             ParamOrNull<Float>("blur_center_edge_fade_thickness"),
             ParamOrNull<Bool>("blur_center_mask_flip"),
             ParamOrNull<Float>("noise_in_blur_mix"),
             ParamOrNull<Float>("frame_time_seconds"),
             ParamOrNull<Float>("time_scale"),
             ParamOrNull<Float>("pulse_xy_scale"),
             ParamOrNull<Float4>("pulse_exponent"),
             ParamOrNull<Float4>("pulse_multiplier"),
             ParamOrNull<Float>("grain_xy_scale"),
             ParamOrNull<Float2>("grain_mask_exponent_and_multiplier"),
             ParamOrNull<Float>("grain_dim_amount"),
             ParamOrNull<Float2>("grain_exponent_and_multiplier"),
             ParamOrNull<Bool>("is_pano"),
             ParamOrNull<Float3>("pano_fov_degrees"),
             ParamOrNull<Float>("pano_spherical_amount"),
             ParamOrNull<Float>("pano_sphere_radius"),
             ParamOrNull<Float2>("pano_blur_fade_to_gray_exponent"),
             ParamOrNull<Float2>("pano_blur_fade_to_gray_subtractor"),
             ParamOrNull<Float>("pano_media_edge_fade_exponent"),
             ParamOrNull<Float>("pano_corner_radius_decrease_exponent"),
             ParamOrNull<Float>("gamma_to_srgb"),
             ParamOrNull<Float3>("tint_color"), ParamOrNull<Float>("opacity"))
      .Union();
}

void PhotosTexture3DMaterial::WriteTexture(
    const std::string& name,
    flatbuffers::Offset<android_xr::schemas::BuiltInTextureParameter>& offset,
    flatbuffers::FlatBufferBuilder& fbb,
    imp::split_engine::BuiltInTextureParameterCreator&
        texture_parameter_creator) const {
  auto itr = parameters_.find(name);
  if (itr != parameters_.end()) {
    if (std::holds_alternative<imp::OwnedOrBorrowedTexturePtr>(itr->second)) {
      auto& texture = std::get<imp::OwnedOrBorrowedTexturePtr>(itr->second);
      offset = texture_parameter_creator.Create(fbb, texture.Borrow());
    }
  }
}

}  // namespace android_xr