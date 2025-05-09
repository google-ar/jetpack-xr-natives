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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_SPEC_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_SPEC_H_

#include <string>

#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/material_library/schemas/generic_material_generated.h"

namespace imp {

// GenericMaterialSpec defines all the permutation axes for a glTF material.
// This is used to load and cache materials.
class GenericMaterialSpec {
 public:
  GenericMaterialSpec(
      schemas::GenericMaterialLightingModel lighting_model,
      schemas::GenericMaterialBlendMode blend_mode,
      schemas::GenericMaterialDoubleSidedMode double_sided_mode,
      schemas::GenericMaterialDepthClearMaterial depth_clear_material)
      : lighting_model_(lighting_model),
        blend_mode_(blend_mode),
        double_sided_mode_(double_sided_mode),
        depth_clear_material_(depth_clear_material) {}

  schemas::GenericMaterialLightingModel GetLightingModel() const;
  schemas::GenericMaterialBlendMode GetBlendMode() const;
  schemas::GenericMaterialDoubleSidedMode GetDoubleSidedMode() const;
  schemas::GenericMaterialDepthClearMaterial GetDepthClearMaterial() const;

  const char* DescribeLightingModel() const;
  const char* DescribeBlendMode() const;
  const char* DescribeDoubleSidedMode() const;
  std::string Describe() const;

  static GenericMaterialSpec FromFlatbuffer(
      const schemas::GenericMaterialSpec& flatbuffer);
  flatbuffers::Offset<schemas::GenericMaterialSpec> ToFlatbuffer(
      flatbuffers::FlatBufferBuilder& builder) const;

  bool operator==(const GenericMaterialSpec& other) const;

 private:
  schemas::GenericMaterialLightingModel lighting_model_;
  schemas::GenericMaterialBlendMode blend_mode_;
  schemas::GenericMaterialDoubleSidedMode double_sided_mode_;
  schemas::GenericMaterialDepthClearMaterial depth_clear_material_;

  template <typename H>
  friend H AbslHashValue(H h, const GenericMaterialSpec& spec) {
    return H::combine(std::move(h), spec.GetLightingModel(),
                      spec.GetBlendMode(), spec.GetDoubleSidedMode(),
                      spec.GetDepthClearMaterial());
  }
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_SPEC_H_
