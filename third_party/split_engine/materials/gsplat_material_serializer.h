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
#ifndef THIRD_PARTY_SPLIT_ENGINE_MATERIALS_GSPLAT_MATERIAL_SERIALIZER_H_
#define THIRD_PARTY_SPLIT_ENGINE_MATERIALS_GSPLAT_MATERIAL_SERIALIZER_H_
#include <sys/types.h>

#include <memory>
#include <optional>
#include <utility>

#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/gsplat/gsplat_asset.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_material.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"

namespace android_xr {

// A material wrapper for the built-in GsplatMaterialSerializer to support Split
// Engine.
//
// Note: This is the split-engine app side of GsplatMaterialDeserializer.
class GsplatMaterialSerializer : public imp::split_engine::SplitEngineMaterial {
 public:
  static imp::Future<std::unique_ptr<GsplatMaterialSerializer>> Create(
      imp::BaseView& view, imp::AssetPtr<imp::GSplatAsset> gsplat_asset,
      android_xr::schemas::GsplatMode material_mode);

  ~GsplatMaterialSerializer() override;

  flatbuffers::Offset<void> SerializeParameters(
      flatbuffers::FlatBufferBuilder& fbb,
      imp::split_engine::BuiltInTextureParameterCreator&
          texture_parameter_creator) const override;

  void SetOpacityScale(float opacity_scale) {
    opacity_scale_ = imp::split_engine::Pack(opacity_scale);
    MarkParametersDirty();
  }

  void SetMinScreenSize(imp::float2 min_screen_size) {
    min_screen_size_ = imp::split_engine::Pack(min_screen_size);
    MarkParametersDirty();
  }

  void SetMaxScreenSize(imp::float2 max_screen_size) {
    min_screen_size_ = imp::split_engine::Pack(max_screen_size);
    MarkParametersDirty();
  }

  void SetWindowDimensionInMagicWindow(
      imp::float2 window_dimension_in_magic_window) {
    window_dimension_in_magic_window_ =
        imp::split_engine::Pack(window_dimension_in_magic_window);
    MarkParametersDirty();
  }

  void SetMagicWindowFromUserWorldMatrix(
      imp::mat4f magic_window_from_user_world_matrix) {
    magic_window_from_user_world_matrix_ =
        imp::split_engine::Pack(magic_window_from_user_world_matrix);
    MarkParametersDirty();
  }

  void SetPrecomputedDataTexture(imp::OwnedOrBorrowedTexturePtr texture) {
    LOG(FATAL) << "(broken link): Precomputed data texture is not supported yet.";
    //  precomputed_data_texture_ = std::move(texture);
    //  MarkParametersDirty();
  }

  void SetPrecomputeTextures(imp::AssetPtr<imp::GSplatAsset> gsplat_asset) {
    position_data_texture_ = gsplat_asset->position_data_texture();
    cov3d_data_texture_ = gsplat_asset->cov3d_data_texture();
    color_data_texture_ = gsplat_asset->color_data_texture();
    MarkParametersDirty();
  }

  void SetSortedIndicesTexture(imp::OwnedOrBorrowedTexturePtr texture) {
    condemned_texture_ = std::move(sorted_indices_texture_);
    sorted_indices_texture_ = std::move(texture);
    MarkParametersDirty();
  }

  void SetSplatScale(float splat_scale) {
    splat_scale_ = imp::split_engine::Pack(splat_scale);
    MarkParametersDirty();
  }

  void SetVisualizeChunks(bool visualize_chunks) {
    visualize_chunks_ = imp::split_engine::Pack(visualize_chunks);
    MarkParametersDirty();
  }

 private:
  GsplatMaterialSerializer(
      imp::BaseView& view,
      imp::split_engine::PlaceholderOrBuiltInMaterialPtr material,
      imp::AssetPtr<imp::GSplatAsset> gsplat_asset);
  imp::BaseView& view_;
  imp::AssetPtr<imp::GSplatAsset> gsplat_asset_;

  std::optional<android_xr::schemas::GsplatMode> material_mode_;
  std::optional<android_xr::schemas::Float> opacity_scale_;
  std::optional<android_xr::schemas::Float2> min_screen_size_;
  std::optional<android_xr::schemas::Float2> max_screen_size_;
  std::optional<android_xr::schemas::Float2> window_dimension_in_magic_window_;
  std::optional<android_xr::schemas::Mat4f>
      magic_window_from_user_world_matrix_;
  std::optional<android_xr::schemas::Mat4f> gsplat_from_user_world_matrix_;

  imp::OwnedOrBorrowedTexturePtr precomputed_data_texture_;
  imp::OwnedOrBorrowedTexturePtr position_data_texture_;
  imp::OwnedOrBorrowedTexturePtr cov3d_data_texture_;
  imp::OwnedOrBorrowedTexturePtr color_data_texture_;
  imp::OwnedOrBorrowedTexturePtr sorted_indices_texture_;
  // When a texture is replaced, it will first be
  // moved here, then later deleted. It is not deleted immediately because the
  // built-in material may still be using it.
  imp::OwnedOrBorrowedTexturePtr condemned_texture_;
  std::optional<android_xr::schemas::Bool> visualize_chunks_;
  std::optional<android_xr::schemas::Float> splat_scale_;
};

}  // namespace android_xr

#endif  // THIRD_PARTY_SPLIT_ENGINE_MATERIALS_GSPLAT_MATERIAL_SERIALIZER_H_
