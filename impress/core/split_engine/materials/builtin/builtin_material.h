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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_MATERIAL_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_MATERIAL_H_

#include <memory>

#include "absl/status/status.h"
#include "flatbuffers/verifier.h"
#include "core/common/small_source_location.h"
#include "core/materials/material.h"
#include "core/render/display_color_space.h"
#include "core/render/texture.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

class BuiltInMaterial;
using BuiltInMaterialPtr = std::unique_ptr<BuiltInMaterial>;

// The interface for all Split Engine built-in materials, namely to get the
// backing filament material instance and set parameters by flatbuffer schema.
//
// Built-in materials are on the Split Engine renderer side, which includes
// generic glTF materials. Each built-in material has a front-end C++ class to
// handle communicating parameters to these built-in materials via schema.
class BuiltInMaterial {
 public:
  virtual ~BuiltInMaterial() = default;

  // Duplicates this built-in material, duplicating the underlying filament
  // material instance and allowing future schema updates to be applied to the
  // new material instance.
  virtual BuiltInMaterialPtr Duplicate() const = 0;

  // Sets the parameters for this material. The parameters must match the type
  // expected by the concrete built-in material type.
  virtual absl::Status SetParameters(
      flatbuffers::Verifier& verifier,
      const android_xr::schemas::BuiltInMaterialInstanceParameters& parameters,
      const TextureBorrower& texture_borrower) = 0;

  BorrowedMaterialPtr GetMaterial(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const {
    return GetMaterialInternal(loc);
  }

  // Indicates what display color space this built-in material requires.
  virtual DisplayColorSpace GetRequiredDisplayColorSpace() const = 0;

 protected:
  // Returns the backing filament material instance, which can be assigned to
  // renderables normally.
  virtual BorrowedMaterialPtr GetMaterialInternal(
      SmallSourceLocation loc) const = 0;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_MATERIAL_H_
