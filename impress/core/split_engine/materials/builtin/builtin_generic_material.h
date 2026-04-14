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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_GENERIC_MATERIAL_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_GENERIC_MATERIAL_H_

#include "absl/status/status.h"
#include "flatbuffers/verifier.h"
#include "core/async/future.h"
#include "core/material_library/generic_material.h"
#include "core/material_library/generic_material_spec.h"
#include "core/material_library/material_package.h"
#include "core/material_library/material_param_value.h"
#include "core/render/display_color_space.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/materials/builtin/builtin_material_wrapper.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

// The built-in material for all glTF materials. This material is on the backend
// side, and is requested by SplitEngineGenericMaterial on the app side.
class BuiltInGenericMaterial : public BuiltInMaterialWrapper<GenericMaterial> {
 public:
  // Creates a built-in generic material based on the given spec, which is used
  // to lookup the material in the material package.
  static Future<BuiltInMaterialPtr> Create(
      BaseView& view, const GenericMaterialSpec& spec,
      const MaterialPackage::MaterialCache& materials);

  BuiltInMaterialPtr Duplicate() const override;

  absl::Status SetParameters(
      flatbuffers::Verifier& verifier,
      const android_xr::schemas::BuiltInMaterialInstanceParameters& parameters,
      const TextureBorrower& texture_borrower) override;

  DisplayColorSpace GetRequiredDisplayColorSpace() const override {
    return DisplayColorSpace::kBT709;
  }

 private:
  explicit BuiltInGenericMaterial(GenericMaterialPtr generic_material);
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_GENERIC_MATERIAL_H_
