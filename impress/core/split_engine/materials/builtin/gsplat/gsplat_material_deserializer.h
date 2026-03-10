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
#include "core/math/mat.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/update_system.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/builtin/builtin_custom_material.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/materials/builtin/gsplat/precompute_texture_pipeline.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

// The built-in material for GSplat visualization.
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
      ComponentHandle<PrecomputeTexturePipeline> pipeline);

  GsplatMaterialDeserializer(
      NodeHandle gsplat_node, BridgeId bridge_id,
      android_xr::schemas::GsplatMode material_mode, OwnedMaterialPtr material,
      ComponentHandle<PrecomputeTexturePipeline> precompute_texture_pipeline);

  // Update precompute specific parameters.
  absl::Status UpdatePrecomputeTexturePipeline(
      const TextureBorrower& texture_borrower,
      const android_xr::schemas::BuiltInMaterialGsplatParameters&
          serialized_parameters);

  // The material used to precompute splat positions, colors, etc.
  // For some modes, this is the same as the render material.
  BorrowedMaterialPtr GetPrecomputedDataMaterial(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;

  // The material used for rendering parameters
  BorrowedMaterialPtr GetRenderMaterial(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;

  // Updates parameters on the render material.
  absl::Status SetRenderMaterialParameters(
      const TextureBorrower& texture_borrower,
      const android_xr::schemas::BuiltInMaterialGsplatParameters&
          serialized_parameters);

  // Updates parameters on the precompute material.
  absl::Status SetPrecomputedDataParameters(
      const TextureBorrower& texture_borrower,
      const android_xr::schemas::BuiltInMaterialGsplatParameters&
          serialized_parameters,
      BorrowedMaterialPtr precompute_material);

  // Updates parameters on the magic window material.
  absl::Status SetMagicWindowMaterialParameters(
      const TextureBorrower& texture_borrower,
      const android_xr::schemas::BuiltInMaterialGsplatParameters&
          serialized_parameters);

  BaseView& view_;
  NodeHandle gsplat_node_;
  android_xr::schemas::GsplatMode material_mode_;
  ComponentHandle<PrecomputeTexturePipeline> precompute_texture_pipeline_;
};

}  // namespace imp::split_engine
#endif  // THIRD_PARTY_SPLIT_ENGINE_MATERIALS_GSPLAT_GSPLAT_MATERIAL_DESERIALIZER_H_
