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
#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_GSPLAT_BUILTIN_GSPLAT_BACKGROUND_MATERIAL_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_GSPLAT_BUILTIN_GSPLAT_BACKGROUND_MATERIAL_H_

#include "absl/status/status.h"
#include "filament/filament/include/filament/Texture.h"
#include "flatbuffers/verifier.h"
#include "core/async/future.h"
#include "core/material_library/material_param_value.h"
#include "core/materials/material.h"
#include "core/ncsb/update_system.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/builtin/builtin_custom_material.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

// De-serialized gsplat background material, an internal wrapper around the
// filament material.
//
// Access via GsplatBackgroundMaterial instead of making changes here directly.
class BuiltInGSplatBackgroundMaterial : public BuiltInCustomMaterial {
 public:
  static Future<BuiltInMaterialPtr> Create(
      BaseView& view, BridgeId bridge_id,
      const android_xr::schemas::BuiltInMaterialGsplatBackgroundSpec& spec);

  split_engine::BuiltInMaterialPtr Duplicate() const override;

  absl::Status SetParameters(
      flatbuffers::Verifier& verifier,
      const android_xr::schemas::BuiltInMaterialInstanceParameters& parameters,
      const TextureBorrower& texture_borrower) override;

 private:
  BuiltInGSplatBackgroundMaterial(BaseView& view, BridgeId bridge_id,
                                  OwnedMaterialPtr material);
  BaseView& view_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_GSPLAT_BUILTIN_GSPLAT_BACKGROUND_MATERIAL_H_
