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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_CUSTOM_MATERIAL_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_CUSTOM_MATERIAL_H_

#include "core/common/small_source_location.h"
#include "core/materials/material.h"
#include "core/render/display_color_space.h"
#include "core/split_engine/materials/builtin/builtin_material.h"

namespace imp::split_engine {

// The base class for all built-in materials that are not generic materials.
// This class holds the OwnedMaterialPtr and implements GetMaterialInternal.
class BuiltInCustomMaterial : public BuiltInMaterial {
 public:
  BuiltInCustomMaterial(OwnedMaterialPtr material);

  DisplayColorSpace GetRequiredDisplayColorSpace() const override {
    return DisplayColorSpace::kBT709;
  }

 protected:
  BorrowedMaterialPtr GetMaterialInternal(
      SmallSourceLocation loc) const override;

 private:
  OwnedMaterialPtr material_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_CUSTOM_MATERIAL_H_
