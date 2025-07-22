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

#include "core/collision/bvh_collision_accelerator.h"

#include <cstdlib>
#include <optional>
#include <vector>

#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "core/collision/bvh.h"
#include "core/collision/collision_helpers.h"
#include "core/collision/ray.h"
#include "core/geometry/shapes/box.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"

namespace imp {

std::optional<collision::MultiPrimitiveMeshIntersection<float>>
BvhCollisionAccelerator::Intersect(const Ray& ray) const {
  // Find the closest intersection among all the bvhs.
  std::optional<Bvh::RayIntersection> min_ray_intersection;
  size_t hit_primitive_id = 0;
  for (size_t primitive_id = 0; primitive_id < bvhs_.size(); ++primitive_id) {
    std::optional<Bvh::RayIntersection> ray_intersection =
        bvhs_[primitive_id].IntersectRay(ray);
    if (ray_intersection &&
        (!min_ray_intersection ||
         ray_intersection->distance < min_ray_intersection->distance)) {
      min_ray_intersection = ray_intersection;
      hit_primitive_id = primitive_id;
    }
  }
  if (min_ray_intersection) {
    collision::MultiPrimitiveMeshIntersection<float> result;
    result.collision_point =
        min_ray_intersection->distance * normalize(ray.direction) + ray.origin;
    result.collision_normal = min_ray_intersection->normal;
    result.intersection_dist = min_ray_intersection->distance;
    result.primitive_id = hit_primitive_id;
    result.triangle_id = min_ray_intersection->triangle_id;
    return result;
  }
  return std::nullopt;
}

// Create a bvh for each primitive in the (gltf) mesh.
void BvhCollisionAccelerator::Create(
    absl::Span<const MeshVertexAndIndexData> mesh_data, Box aabb) {
  bvhs_.reserve(mesh_data.size());
  for (const MeshVertexAndIndexData& mesh_data : mesh_data) {
    bvhs_.emplace_back(Bvh(mesh_data, options_));
  }
}
}  // namespace imp
