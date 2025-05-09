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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_PHOTOS_TEXTURE_3D_MATERIAL_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_PHOTOS_TEXTURE_3D_MATERIAL_H_

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "flatbuffers/verifier.h"
#include "core/async/future.h"
#include "core/material_library/material_param_value.h"
#include "core/materials/material.h"
#include "core/ncsb/update_system.h"
#include "core/split_engine/materials/builtin/builtin_custom_material.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

// The built-in material for PhotosXR 3D textures.
class BuiltInPhotosTexture3dMaterial : public BuiltInCustomMaterial {
 public:
  // Creates a built-in photos 3D texture material based on the given spec.
  static Future<BuiltInMaterialPtr> Create(
      BaseView& view, const android_xr::schemas::BuiltInMaterialD1750064& spec);

  BuiltInMaterialPtr Duplicate() const override;

  absl::Status SetParameters(
      flatbuffers::Verifier& verifier,
      const android_xr::schemas::BuiltInMaterialInstanceParameters& parameters,
      const TextureBorrower& texture_borrower) override;

 private:
  BuiltInPhotosTexture3dMaterial(BaseView& view, OwnedMaterialPtr material);

  BaseView& view_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_PHOTOS_TEXTURE_3D_MATERIAL_H_
