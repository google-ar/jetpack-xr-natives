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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_AR_POINT_CLOUD_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_AR_POINT_CLOUD_H_

#include <vector>

#include "absl/types/optional.h"
#include "core/math/vec.h"

namespace imp {
namespace ar {

// Individual cross-platform implementation for a 3D point within a point cloud.
struct ArPointCloudPoint {
  // (x, y, z) coordinates for the point in world space.
  float3 position;
  // Identifier for the point; valid across different frames.
  uint64_t id;
  // Confidence for the point estimation, from 0 to 1.
  absl::optional<float> confidence;
};

// Set of points representing the point cloud.
using ArPointCloud = std::vector<ArPointCloudPoint>;

}  // namespace ar
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_AR_POINT_CLOUD_H_
