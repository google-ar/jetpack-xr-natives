// Copyright 2024 Google LLC
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

#include "core/material_library/generic_material_spec.h"

#include <cstddef>
#include <string>

#include "absl/strings/str_format.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/material_library/schemas/generic_material_generated.h"

namespace imp {

schemas::GenericMaterialLightingModel GenericMaterialSpec::GetLightingModel()
    const {
  return lighting_model_;
}

schemas::GenericMaterialBlendMode GenericMaterialSpec::GetBlendMode() const {
  return blend_mode_;
}

schemas::GenericMaterialDoubleSidedMode
GenericMaterialSpec::GetDoubleSidedMode() const {
  return double_sided_mode_;
}

schemas::GenericMaterialDepthClearMaterial
GenericMaterialSpec::GetDepthClearMaterial() const {
  return depth_clear_material_;
}

bool GenericMaterialSpec::operator==(const GenericMaterialSpec& other) const {
  return GetLightingModel() == other.GetLightingModel() &&
         GetBlendMode() == other.GetBlendMode() &&
         GetDoubleSidedMode() == other.GetDoubleSidedMode() &&
         GetDepthClearMaterial() == other.GetDepthClearMaterial();
}

const char* GenericMaterialSpec::DescribeLightingModel() const {
  static constexpr char const* kLookup[]{"lit", "unlit"};
  return kLookup[static_cast<size_t>(lighting_model_)];
}

const char* GenericMaterialSpec::DescribeBlendMode() const {
  static constexpr char const* kLookup[]{"opaque", "masked", "transparent",
                                         "refractive"};
  return kLookup[static_cast<size_t>(blend_mode_)];
}

const char* GenericMaterialSpec::DescribeDoubleSidedMode() const {
  static constexpr char const* kLookup[]{"single_sided", "double_sided"};
  return kLookup[static_cast<size_t>(double_sided_mode_)];
}

std::string GenericMaterialSpec::Describe() const {
  const char* lm = DescribeLightingModel();
  const char* bm = DescribeBlendMode();
  const char* dm = DescribeDoubleSidedMode();
  return absl::StrFormat("Generic-%s-%s-%s", lm, bm, dm);
}

GenericMaterialSpec GenericMaterialSpec::FromFlatbuffer(
    const schemas::GenericMaterialSpec& flatbuffer) {
  auto dcm = flatbuffer.depth_clear_material();
  if (dcm == schemas::GenericMaterialDepthClearMaterial::Enabled) {
    dcm = schemas::GenericMaterialDepthClearMaterial::Enabled;
  }
  return GenericMaterialSpec(
      flatbuffer.lighting_model(), flatbuffer.blend_mode(),
      flatbuffer.double_sided_mode(), flatbuffer.depth_clear_material());
}

flatbuffers::Offset<schemas::GenericMaterialSpec>
GenericMaterialSpec::ToFlatbuffer(
    flatbuffers::FlatBufferBuilder& builder) const {
  return schemas::CreateGenericMaterialSpec(builder, lighting_model_,
                                            blend_mode_, double_sided_mode_,
                                            depth_clear_material_);
}

}  // namespace imp
