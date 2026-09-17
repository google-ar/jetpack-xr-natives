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

#include "core/gsplat/conversion.h"

#include <sys/types.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/libs/math/include/math/TQuatHelpers.h"
#include "filament/libs/math/include/math/scalar.h"
#include "core/common/trace.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "miniply.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {
namespace {
// The position vector stored in the ply file as {XYZ}.
const std::vector<std::string> kPositionPropertyNames = {"x", "y", "z"};
// The rotations are stored in the ply file as {WXYZ} but we want {XYZW}.
// Swizzle the real (w) component of the quaternion to the last element.
const std::vector<std::string> kRotationPropertyNames = {"rot_1", "rot_2",
                                                         "rot_3", "rot_0"};
// The scale vector stored in the ply file as {XYZ}.
const std::vector<std::string> kScalePropertyNames = {"scale_0", "scale_1",
                                                      "scale_2"};
// The base color is stored in the ply file as Band 0 spherical harmonic
// coefficients; and separate opacity.
const std::vector<std::string> kColorPropertyNames = {"f_dc_0", "f_dc_1",
                                                      "f_dc_2", "opacity"};
// The 15 additional spherical harmonic coefficients (for rendering SH up to
// Band 3) stored in the ply file.
const std::vector<std::string> kSHCoefficientPropertyNames = {
    "f_rest_0",  "f_rest_1",  "f_rest_2",  "f_rest_3",  "f_rest_4",
    "f_rest_5",  "f_rest_6",  "f_rest_7",  "f_rest_8",  "f_rest_9",
    "f_rest_10", "f_rest_11", "f_rest_12", "f_rest_13", "f_rest_14",
    "f_rest_15", "f_rest_16", "f_rest_17", "f_rest_18", "f_rest_19",
    "f_rest_20", "f_rest_21", "f_rest_22", "f_rest_23", "f_rest_24",
    "f_rest_25", "f_rest_26", "f_rest_27", "f_rest_28", "f_rest_29",
    "f_rest_30", "f_rest_31", "f_rest_32", "f_rest_33", "f_rest_34",
    "f_rest_35", "f_rest_36", "f_rest_37", "f_rest_38", "f_rest_39",
    "f_rest_40", "f_rest_41", "f_rest_42", "f_rest_43", "f_rest_44"};

// Load the properties specified in property_names from the ply file into the
// returned data_buffer vector. All properties will be converted to the provided
// dest_type.
template <typename T>
absl::StatusOr<std::vector<T>> LoadProperties(
    const miniply::PLYReader& ply_data,
    const miniply::PLYPropertyType dest_type,
    const absl::Span<const std::string> property_names) {
  std::vector<uint32_t> property_indices;
  for (const std::string& property_name : property_names) {
    const uint32_t property_index =
        ply_data.find_property(property_name.c_str());
    if (property_index == miniply::kInvalidIndex) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "Failed to find property %s from PLY file.", property_name));
    }
    property_indices.push_back(property_index);
  }
  std::vector<T> data_buffer;
  data_buffer.resize(ply_data.element()->count);
  if (!ply_data.extract_properties(property_indices.data(),
                                   property_indices.size(), dest_type,
                                   data_buffer.data())) {
    std::string property_names_string;
    for (const absl::string_view property_name : property_names) {
      absl::StrAppend(&property_names_string, property_name, ", ");
    }
    return absl::InternalError(
        absl::StrFormat("Failed to load at least one of properties: %s",
                        property_names_string));
  }
  return data_buffer;
}

// Load all needed properties for gsplat rendering from the PLY file into the
// provided vectors.
absl::Status LoadAllProperties(
    const miniply::PLYReader& reader, std::vector<float3>& positions,
    std::vector<quatf>& rotations, std::vector<float3>& scales,
    std::vector<float4>& colors, const bool should_load_optional_data,
    std::vector<GSplatRuntimeData::SHCoefficients>& spherical_harmonics) {
  IMP_TRACE();
  MP_ASSIGN_OR_RETURN(
      positions, LoadProperties<float3>(reader, miniply::PLYPropertyType::Float,
                                        kPositionPropertyNames));
  MP_ASSIGN_OR_RETURN(
      rotations, LoadProperties<quatf>(reader, miniply::PLYPropertyType::Float,
                                       kRotationPropertyNames));
  MP_ASSIGN_OR_RETURN(
      scales, LoadProperties<float3>(reader, miniply::PLYPropertyType::Float,
                                     kScalePropertyNames));
  MP_ASSIGN_OR_RETURN(
      colors, LoadProperties<float4>(reader, miniply::PLYPropertyType::Float,
                                     kColorPropertyNames));
  if (should_load_optional_data) {
    MP_ASSIGN_OR_RETURN(spherical_harmonics,
                     LoadProperties<GSplatRuntimeData::SHCoefficients>(
                         reader, miniply::PLYPropertyType::Float,
                         kSHCoefficientPropertyNames));
  }
  return absl::OkStatus();
}

// Based on (broken link)/
// Insert two 0 bits after each of the 21 low bits of x
uint64_t MortonPart1By2(uint64_t x) {
  // Comments show the interleaving affecting the lower 32 bits of the encoding.
  x &= 0x1fffff;
  //  x = ---- ---- ---k jihg fedc ba98 7654 3210
  x = (x ^ (x << 32)) & 0x1f00000000ffffUL;
  //  x = ---- ---- ---- ---- fedc ba98 7654 3210
  x = (x ^ (x << 16)) & 0x1f0000ff0000ffUL;
  //  x = fedc ba98 ---- ---- ---- ---- 7654 3210
  x = (x ^ (x << 8)) & 0x100f00f00f00f00fUL;
  //  x = ---- ba98 ---- ---- 7654 ---- ---- 3210
  x = (x ^ (x << 4)) & 0x10c30c30c30c30c3UL;
  //  x = ba-- --98 ---- 76-- --54 ---- 32-- --10
  x = (x ^ (x << 2)) & 0x1249249249249249UL;
  //  x = -a-- 9--8 --7- -6-- 5--4 --3- -2-- 1--0
  return x;
}

// Encode three 21-bit unsigned integers into 3D Morton order.
// Unsigned integers larger than 2^21 - 1 will suffer from data loss due to
// encoding masking only to the 21 least significant bits.
uint64_t MortonEncode(uint3 v) {
  return (MortonPart1By2(v.z) << 2) | (MortonPart1By2(v.y) << 1) |
         MortonPart1By2(v.x);
}

// Sort the positions using the morton encoding of the position. Morton Encoding
// is an operation applied to integers, so this function converts the position
// vector into a single 64 bit integer which can then be used for the encoding
// and sorting.
std::vector<size_t> SortedMortonPositions(const std::vector<float3>& positions,
                                          const float3& min,
                                          const float3& max) {
  const float3 range = max - min;
  // Since we are trying to pack a 3 element vector into a 64 bit integer, we
  // want to evenly distribute the bits amongst the elements of the vector. This
  // means we have 21 bits of precision per element, so we scale the range to
  // fit in that.
  constexpr uint32_t kRangeScaler = ((1 << 21) - 1);
  std::vector<size_t> morton_order;
  std::vector<uint64_t> morton_encodings;
  morton_order.reserve(positions.size());
  morton_encodings.reserve(positions.size());
  for (int i = 0; i < positions.size(); ++i) {
    uint3 position_int =
        static_cast<uint3>((positions[i] - min) / range * kRangeScaler);
    morton_order.push_back(i);
    morton_encodings.push_back(MortonEncode(position_int));
  }
  std::sort(morton_order.begin(), morton_order.end(),
            [&morton_encodings](const uint32_t a, const uint32_t b) {
              return morton_encodings[a] < morton_encodings[b];
            });
  return morton_order;
}

// Sort the required data in the GSplatRuntimeData struct in place based on the
// order of indices in the given index.
void SortRequiredDataInPlace(const std::vector<std::size_t>& index,
                             GSplatRuntimeData& data) {
  IMP_TRACE();
  std::vector<bool> done(data.positions.size());
  for (std::size_t i = 0; i < data.positions.size(); ++i) {
    if (done[i]) {
      continue;
    }
    done[i] = true;
    std::size_t prev_j = i;
    std::size_t j = index[i];
    while (i != j) {
      std::swap(data.positions[prev_j], data.positions[j]);
      std::swap(data.rotations[prev_j], data.rotations[j]);
      std::swap(data.scales[prev_j], data.scales[j]);
      std::swap(data.colors[prev_j], data.colors[j]);
      done[j] = true;
      prev_j = j;
      j = index[j];
    }
  }
}

// Sort the optional data in the GSplatRuntimeData struct in place based on the
// order of indices in the given index.
void SortOptionalDataInPlace(const std::vector<std::size_t>& index,
                             GSplatRuntimeData& data) {
  IMP_TRACE();
  std::vector<bool> done(data.spherical_harmonics.size());
  for (std::size_t i = 0; i < data.spherical_harmonics.size(); ++i) {
    if (done[i]) {
      continue;
    }
    done[i] = true;
    std::size_t prev_j = i;
    std::size_t j = index[i];
    while (i != j) {
      std::swap(data.spherical_harmonics[prev_j], data.spherical_harmonics[j]);
      done[j] = true;
      prev_j = j;
      j = index[j];
    }
  }
}

// Convert a color in sRGB space to linear space.
// (broken link)
float SrgbToLinear(float color) {
  return color <= 0.04045f ? color / 12.92f
                           : pow((color + 0.055f) / 1.055f, 2.4f);
}

void LinearizeColor(float4& color) {
  // Convert splat band 0 spherical harmonic color to linear color.
  constexpr float kSHBand0Coefficient = 0.2820948f;
  color.rgb = color.rgb * kSHBand0Coefficient + 0.5f;
  color.rgb = {SrgbToLinear(color.r), SrgbToLinear(color.g),
               SrgbToLinear(color.b)};

  // Convert from splat exponential opacity to linear opacity.
  color.a = 1.0f / (1.0f + exp(-color.a));
  color = filament::math::saturate(color);
}
}  // namespace

absl::StatusOr<GSplatRuntimeData> GSplatPlyToRuntimeData(
    absl::string_view ply_data, bool load_optional_data) {
  IMP_TRACE();
  miniply::PLYReader reader(const_cast<char*>(ply_data.data()),
                            ply_data.size());
  if (!reader.valid()) {
    return absl::InternalError("Failed to parse PLY file.");
  }
  while (reader.has_element()) {
    if (!reader.element_is(miniply::kPLYVertexElement)) {
      reader.next_element();
      continue;
    }

    if (reader.element()->count == 0) {
      return absl::InternalError("No vertices in PLY file.");
    }
    reader.load_element();

    GSplatRuntimeData result;
    MP_RETURN_IF_ERROR(LoadAllProperties(
        reader, result.positions, result.rotations, result.scales,
        result.colors, load_optional_data, result.spherical_harmonics));
    float3 bounds_min = float3(std::numeric_limits<float>::max());
    float3 bounds_max = float3(std::numeric_limits<float>::lowest());
    // Linearize and normalize the loaded PLY data as needed while computing the
    // min and max bounds of the data.
    for (int i = 0; i < result.positions.size(); ++i) {
      LinearizeColor(result.colors[i]);
      result.scales[i] = abs(exp(result.scales[i]));
      result.rotations[i] = normalize(result.rotations[i]);
      bounds_min = min(bounds_min, result.positions[i]);
      bounds_max = max(bounds_max, result.positions[i]);
    }

    const std::vector<size_t> morton_order =
        SortedMortonPositions(result.positions, bounds_min, bounds_max);
    SortRequiredDataInPlace(morton_order, result);
    if (load_optional_data) {
      SortOptionalDataInPlace(morton_order, result);
    }
    return result;
  }

  return absl::InternalError("No gaussian splat data found in PLY file.");
}
}  // namespace imp
