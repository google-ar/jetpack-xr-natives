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

#include <optional>
#include <utility>

#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Color.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "filament/libs/math/include/math/mathfwd.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/common/owned_ptr.h"
#include "core/gsplat/gsplat_asset.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/ncsb/node.h"
#include "core/render/texture.h"
#include "core/render_passes/texture_pipeline_renderer_projection_quad.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_builtin_material.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"

namespace android_xr {

// A material wrapper for the built-in GsplatMaterialSerializer to support Split
// Engine.
//
// Callers must ensure all textures borrowed by the GsplatMaterialSerializer
// outlive the class.  Additionally, if a texture is ever changed the caller
// must ensure the old texture is not destroyed before the new texture is
// asynchronously uploaded.
//
// Note: This is the split-engine app side of GsplatMaterialDeserializer.
class GsplatMaterialSerializer
    : public imp::split_engine::SplitEngineBuiltinMaterial {
 public:
  // Creates a Gsplat material with a specified Rendering mode that uses a given
  // precomputed data texture to render the Gsplat scene.
  static imp::Future<imp::OwnedPtr<GsplatMaterialSerializer>> Create(
      imp::NodeHandle gsplat_node, imp::AssetPtr<imp::GSplatAsset> gsplat_asset,
      android_xr::schemas::GsplatMode material_mode,
      bool use_triangles_for_splats,
      imp::BorrowedTexturePtr precomputed_data_texture =
          imp::BorrowedTexturePtr(),
      absl::string_view render_group = imp::Node::kMainGroupName,
      std::optional<imp::uint2> magic_window_offscreen_resolution =
          std::nullopt);

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
    max_screen_size_ = imp::split_engine::Pack(max_screen_size);
    MarkParametersDirty();
  }


  void SetPrecomputedDataTexture(imp::OwnedOrBorrowedTexturePtr texture) {
    precomputed_data_texture_ = std::move(texture);
    MarkParametersDirty();
  }

  void SetPrecomputeTextures(imp::AssetPtr<imp::GSplatAsset> gsplat_asset) {
    if (!gsplat_asset) {
      return;
    }

    position_data_texture_ = gsplat_asset->position_data_texture();
    cov3d_data_texture_ = gsplat_asset->cov3d_data_texture();
    color_data_texture_ = gsplat_asset->color_data_texture();
    MarkParametersDirty();
  }

  void SetSplatScale(float splat_scale) {
    splat_scale_ = imp::split_engine::Pack(splat_scale);
    MarkParametersDirty();
  }

  // The `projection_quad` is specified in the local space of the `gsplat_node`.
  // See (broken link)
  void SetMagicWindowProjectionQuad(
      const std::optional<imp::TexturePipelineRendererProjectionQuad>&
          projection_quad) {
    magic_window_projection_quad_ = projection_quad;
    MarkParametersDirty();
  }

  void SetMagicWindowOffscreenResolution(
      const std::optional<imp::uint2>& magic_window_offscreen_resolution) {
    if (magic_window_offscreen_resolution.has_value()) {
      magic_window_offscreen_resolution_ =
          imp::split_engine::Pack(magic_window_offscreen_resolution.value());
    } else {
      magic_window_offscreen_resolution_ = std::nullopt;
    }
    MarkParametersDirty();
  }

  void SetViewResolution(imp::float2 view_resolution) {
    view_resolution_ = imp::split_engine::Pack(view_resolution);
    MarkParametersDirty();
  }

  /**
   * Material parameter setters inherited from Material.h
   *
   * Parameter name and type must be valid for serialization.
   *
   * Unsupported parameters will terminate the program with an error. There is
   * an explicit check for parameter name/type mismatches to prevent triggering
   * undefined behavior.
   */
  void SetParameter(absl::string_view parameter_name, bool value) override;
  void SetParameter(absl::string_view parameter_name,
                    imp::bool2 value) override;
  void SetParameter(absl::string_view parameter_name,
                    imp::bool3 value) override;
  void SetParameter(absl::string_view parameter_name,
                    imp::bool4 value) override;
  void SetParameter(absl::string_view parameter_name, float value) override;
  void SetParameter(absl::string_view parameter_name,
                    imp::float2 value) override;
  void SetParameter(absl::string_view parameter_name,
                    imp::float3 value) override;
  void SetParameter(absl::string_view parameter_name,
                    imp::float4 value) override;
  void SetParameter(absl::string_view parameter_name, int value) override;
  void SetParameter(absl::string_view parameter_name, imp::int2 value) override;
  void SetParameter(absl::string_view parameter_name, imp::int3 value) override;
  void SetParameter(absl::string_view parameter_name, imp::int4 value) override;
  void SetParameter(absl::string_view parameter_name, uint value) override;
  void SetParameter(absl::string_view parameter_name,
                    imp::uint2 value) override;
  void SetParameter(absl::string_view parameter_name,
                    imp::uint3 value) override;
  void SetParameter(absl::string_view parameter_name,
                    imp::uint4 value) override;
  void SetParameter(absl::string_view parameter_name,
                    imp::mat3f value) override;
  void SetParameter(absl::string_view parameter_name,
                    imp::mat4f value) override;

  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const bool> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const imp::bool2> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const imp::bool3> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const imp::bool4> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const float> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const imp::float2> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const imp::float3> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const imp::float4> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const int> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const imp::int2> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const imp::int3> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const imp::int4> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const uint> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const imp::uint2> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const imp::uint3> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const imp::uint4> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const imp::mat3f> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const imp::mat4f> value) override;

  void SetParameter(absl::string_view parameter_name, filament::RgbaType type,
                    filament::math::float4 color) override;

  void SetParameter(absl::string_view parameter_name, filament::RgbType type,
                    filament::math::float3 color) override;

  void SetParameter(
      absl::string_view parameter_name, const imp::Texture* texture,
      std::optional<filament::TextureSampler> sampler_override) override;

  void SetParameter(
      absl::string_view parameter_name, imp::TexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override) override;

  void SetParameter(
      absl::string_view parameter_name, imp::OwnedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override) override;

  void SetParameter(
      absl::string_view parameter_name, imp::BorrowedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override) override;

  bool HasParameter(absl::string_view parameter_name) override;

 private:
  friend class GsplatMaterialSerializerTest;

  GsplatMaterialSerializer(imp::BaseView& view, imp::OwnedMaterialPtr material,
                           imp::AssetPtr<imp::GSplatAsset> gsplat_asset,
                           android_xr::schemas::GsplatMode material_mode,
                           imp::BorrowedTexturePtr precomputed_data_texture,
                           std::optional<android_xr::schemas::Uint2>
                               magic_window_offscreen_resolution);

  void SetSortedIndicesTexture(imp::BorrowedTexturePtr texture) {
    sorted_indices_texture_ = texture;
    MarkParametersDirty();
  }

  imp::AssetPtr<imp::GSplatAsset> gsplat_asset_;

  std::optional<android_xr::schemas::GsplatMode> material_mode_;
  std::optional<android_xr::schemas::Uint> gsplat_node_id_;
  std::optional<android_xr::schemas::Float> opacity_scale_;
  std::optional<android_xr::schemas::Float2> min_screen_size_;
  std::optional<android_xr::schemas::Float2> max_screen_size_;

  imp::OwnedOrBorrowedTexturePtr precomputed_data_texture_;
  imp::OwnedOrBorrowedTexturePtr position_data_texture_;
  imp::OwnedOrBorrowedTexturePtr cov3d_data_texture_;
  imp::OwnedOrBorrowedTexturePtr color_data_texture_;
  imp::BorrowedTexturePtr sorted_indices_texture_;
  std::optional<android_xr::schemas::Float> splat_scale_;
  std::optional<imp::TexturePipelineRendererProjectionQuad>
      magic_window_projection_quad_;
  std::optional<android_xr::schemas::Uint2> magic_window_offscreen_resolution_;
  // The resolution of the view the gsplat is being rendered to.
  std::optional<android_xr::schemas::Float2> view_resolution_;
};

}  // namespace android_xr

#endif  // THIRD_PARTY_SPLIT_ENGINE_MATERIALS_GSPLAT_MATERIAL_SERIALIZER_H_
