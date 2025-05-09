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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_JXR_MEDIA_MATERIAL_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_JXR_MEDIA_MATERIAL_H_

#include "absl/status/status.h"
#include "flatbuffers/verifier.h"
#include "core/async/future.h"
#include "core/material_library/material_param_value.h"
#include "core/materials/material.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/builtin/builtin_custom_material.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

// The built-in material for displaying 3D textures and videos in Jetpack XR.
//
// Note: This is the split-engine renderer side of JxrMediaMaterial.
class BuiltInJxrMediaMaterial : public split_engine::BuiltInCustomMaterial {
 public:
  // Creates a built-in 3D texture material based on the given spec.
  static Future<split_engine::BuiltInMaterialPtr> Create(
      BaseView& view, const android_xr::schemas::BuiltInMaterial1b616c8a& spec);

  split_engine::BuiltInMaterialPtr Duplicate() const override;

  absl::Status SetParameters(
      flatbuffers::Verifier& verifier,
      const android_xr::schemas::BuiltInMaterialInstanceParameters& parameters,
      const TextureBorrower& texture_borrower) override;

 private:
  BuiltInJxrMediaMaterial(BaseView& view, OwnedMaterialPtr material);

  BaseView& view_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_BUILTIN_JXR_MEDIA_MATERIAL_H_
