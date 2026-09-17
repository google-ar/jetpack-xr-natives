/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_GSPLAT_CONVERSION_H_
#define THIRD_PARTY_IMPRESS_CORE_GSPLAT_CONVERSION_H_

#include <array>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/math/quat.h"
#include "core/math/vec.h"

namespace imp {

// A struct containing all the data from a ply file needed to render a gsplat
// scene in a c++ friendly format. This is a structure of arrays, and there is a
// 1:1 mapping of entries in each vector to a splat in the scene; i.e. index 0
// in each vector corresponds to data pertaining to the first splat in the
// scene.
struct GSplatRuntimeData {
  // The positions of each splat in world space.
  std::vector<float3> positions;
  // The rotations of each splat in world space.
  std::vector<quatf> rotations;
  // The scales of each splat in world space.
  std::vector<float3> scales;
  // The Linear RGB color and Opacity for each splat.
  std::vector<float4> colors;

  // Each splat has 15 spherical harmonic coefficients.
  static constexpr int kSphericalHarmonicCount = 15;
  using SHCoefficients = std::array<float3, kSphericalHarmonicCount>;
  // The spherical harmonic coefficients of each splat. Size will be equal to te
  // number of splats, or 0 if spherical harmonics were not loaded because
  // optional data was not requested.
  std::vector<SHCoefficients> spherical_harmonics;
};

// Converts a PLY file loaded in memory into a GSplatRuntimeData struct.
// If should_load_optional_data is true, optional data not necessary for basic
// gaussian splat rendering will be loaded (currently this is only spherical
// harmonics).
absl::StatusOr<GSplatRuntimeData> GSplatPlyToRuntimeData(
    absl::string_view ply_data, bool should_load_optional_data);
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_GSPLAT_CONVERSION_H_
