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

#ifndef THIRD_PARTY_SPLIT_ENGINE_MATERIALS_GSPLAT_GSPLAT_MATERIAL_DESERIALIZER_H_
#define THIRD_PARTY_SPLIT_ENGINE_MATERIALS_GSPLAT_GSPLAT_MATERIAL_DESERIALIZER_H_

#include <optional>

#include "absl/status/status.h"
#include "flatbuffers/verifier.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/common/rememberer.h"
#include "core/common/small_source_location.h"
#include "core/materials/material.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/update_system.h"
#include "core/render/texture.h"
#include "core/render_passes/group_to_projection_quad_texture_renderer/group_to_projection_quad_texture_renderer.h"
#include "core/split_engine/materials/builtin/builtin_custom_material.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/materials/builtin/gsplat/precompute_texture_pipeline.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

// The built-in material for Gsplat visualization.
// This material has several distinctive modes of operation:
//
// 1) Standard Gsplat rendering (when material_mode in spec is GSPLAT)
//    For efficiency, Gsplat rendering is done in 2-passes: the first pass
//    processes the raw data (positions, covariances, etc.) into a texture
//    (precomputed data texture), and the second pass uses that texture to
//    render the splats. Depending on the parameters given, we have 2 ways to
//    produce this texture:
//
//    A) With an internal precompute data pipeline
//       This is when the has_precomputed_data_texture in spec is false.
//       In this mode, an internal texture pipeline is created to generate this
//       texture, and then the texture is used to render the actual splats.
//
//    B) With client provided precompute data texture
//       This is when the has_precomputed_data_texture in spec is true.
//       In this mode, the client generate the precomputed data texture itself,
//       and provide that to the material. In this mode, no precompute data
//       pipeline is created, and the material will use the given texture
//       instead.
//
// 2) Magic Window rendering (when material_mode in spec is MAGIC_WINDOW)
//    In this mode, a special shader is used to do a similar Gsplat rendering
//    like (1), but with additional fragment discard logic in the shader to cull
//    the splats that are outside of the given window, to create a "magic
//    window" look.
//    TODO: (broken link) - Due to an issue with TexturePipelineRenderer, this
//    mode does not use precomputed data texture but instead renders the splats
//    directly from the raw data.
//    TODO: (broken link) - This Magic Window rendering mode is too inefficient,
//    and will be replaced by a new method that renders an offscreen texture
//    that contains the rendered splat scene to create the same magic window
//    look.
//
class GsplatMaterialDeserializer : public BuiltInCustomMaterial,
                                   public imp::Rememberer {
 public:
  static Future<BuiltInMaterialPtr> Create(
      BaseView& view, BridgeId bridge_id,
      const android_xr::schemas::BuiltInMaterialGsplatSpec& spec);

  ~GsplatMaterialDeserializer() override;

  split_engine::BuiltInMaterialPtr Duplicate() const override;

  absl::Status SetParameters(
      flatbuffers::Verifier& verifier,
      const android_xr::schemas::BuiltInMaterialInstanceParameters& parameters,
      const TextureBorrower& texture_borrower) override;

 private:
  static BuiltInMaterialPtr Create(
      NodeHandle gsplat_node, BridgeId bridge_id,
      android_xr::schemas::GsplatMode material_mode,
      AssetPtr<MaterialAsset> material_asset,
      std::optional<imp::uint2> offscreen_texture_resolution,
      ComponentHandle<PrecomputeTexturePipeline> precompute_texture_pipeline,
      ComponentHandle<GroupToProjectionQuadTextureRenderer>
          group_to_projection_quad_texture_renderer);

  GsplatMaterialDeserializer(
      NodeHandle gsplat_node, BridgeId bridge_id,
      android_xr::schemas::GsplatMode material_mode, OwnedMaterialPtr material,
      std::optional<imp::uint2> offscreen_texture_resolution,
      ComponentHandle<PrecomputeTexturePipeline> precompute_texture_pipeline,
      ComponentHandle<GroupToProjectionQuadTextureRenderer>
          group_to_projection_quad_texture_renderer);

  // Update precompute specific parameters.
  absl::Status UpdatePrecomputeTexturePipeline(
      const TextureBorrower& texture_borrower,
      const android_xr::schemas::GsplatParameters& serialized_parameters);

  // The material used to precompute texture pipeline, using splat positions,
  // colors, etc.
  BorrowedMaterialPtr GetPrecomputedDataMaterial(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;

  // The material used for rendering.
  BorrowedMaterialPtr GetRenderMaterial(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;

  // Updates parameters on the render material.
  absl::Status SetRenderMaterialParameters(
      const TextureBorrower& texture_borrower,
      const android_xr::schemas::GsplatParameters& serialized_parameters);

  // Updates the raw gsplat data parameters (e.g., splat positions, colors) on
  // the given material.
  absl::Status SetRawGsplatDataParameters(
      const TextureBorrower& texture_borrower,
      const android_xr::schemas::GsplatParameters& serialized_parameters,
      BorrowedMaterialPtr material);

  // Updates parameters for the magic window mode (mode 2).
  absl::Status SetMagicWindowMaterialParameters(
      const TextureBorrower& texture_borrower,
      const android_xr::schemas::MagicWindowParameters& serialized_parameters);

  BaseView& view_;
  NodeHandle gsplat_node_;
  android_xr::schemas::GsplatMode material_mode_;
  ComponentHandle<PrecomputeTexturePipeline> precompute_texture_pipeline_;
  ComponentHandle<GroupToProjectionQuadTextureRenderer>
      group_to_projection_quad_texture_renderer_;
  std::optional<imp::uint2> offscreen_texture_resolution_;
};

}  // namespace imp::split_engine
#endif  // THIRD_PARTY_SPLIT_ENGINE_MATERIALS_GSPLAT_GSPLAT_MATERIAL_DESERIALIZER_H_
