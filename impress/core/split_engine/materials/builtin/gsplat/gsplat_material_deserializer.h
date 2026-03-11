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

#include "absl/status/status.h"
#include "filament/filament/include/filament/Texture.h"
#include "flatbuffers/verifier.h"
#include "core/async/future.h"
#include "core/material_library/material_param_value.h"
#include "core/materials/material.h"
#include "core/ncsb/update_system.h"
#include "core/render/texture.h"
#include "core/resources/resource_definition.h"
#include "core/split_engine/materials/builtin/builtin_custom_material.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

// The built-in material for GSplat visualization.
class GsplatMaterialDeserializer : public BuiltInCustomMaterial {
 public:
  static Future<BuiltInMaterialPtr> Create(
      BaseView& view, BridgeId bridge_id,
      const android_xr::schemas::BuiltInMaterialGsplatSpec& spec);

  split_engine::BuiltInMaterialPtr Duplicate() const override;

  absl::Status SetParameters(
      flatbuffers::Verifier& verifier,
      const android_xr::schemas::BuiltInMaterialInstanceParameters& parameters,
      const TextureBorrower& texture_borrower) override;

 private:
  GsplatMaterialDeserializer(BaseView& view, BridgeId bridge_id,
                             android_xr::schemas::GsplatMode material_mode,
                             OwnedMaterialPtr material);
  // The material used to precompute splat positions, colors, etc.
  BorrowedMaterialPtr GetPrecomputeMaterial(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;

  // The material used for rendering parameters
  BorrowedMaterialPtr GetRenderMaterial(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;

  absl::Status SetPrecomputeMaterialParameters(
      const TextureBorrower& texture_borrower,
      const android_xr::schemas::BuiltInMaterialGsplatParameters&
          serialized_parameters);

  absl::Status SetCommonMaterialParameters(
      const TextureBorrower& texture_borrower,
      const android_xr::schemas::BuiltInMaterialGsplatParameters&
          serialized_parameters);
  absl::Status SetGsplatMaterialParameters(
      const TextureBorrower& texture_borrower,
      const android_xr::schemas::BuiltInMaterialGsplatParameters&
          serialized_parameters);
  absl::Status SetMagicWindowMaterialParameters(
      const TextureBorrower& texture_borrower,
      const android_xr::schemas::BuiltInMaterialGsplatParameters&
          serialized_parameters);
  absl::Status SetPrecomputedSplatDataParameters(
      const TextureBorrower& texture_borrower,
      const android_xr::schemas::BuiltInMaterialGsplatParameters&
          serialized_parameters);
  absl::Status SetRawSplatDataParameters(
      const TextureBorrower& texture_borrower,
      const android_xr::schemas::BuiltInMaterialGsplatParameters&
          serialized_parameters);

  BaseView& view_;
  android_xr::schemas::GsplatMode material_mode_;
};

}  // namespace imp::split_engine
#endif  // THIRD_PARTY_SPLIT_ENGINE_MATERIALS_GSPLAT_GSPLAT_MATERIAL_DESERIALIZER_H_
