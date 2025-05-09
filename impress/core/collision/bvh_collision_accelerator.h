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

#ifndef THIRD_PARTY_IMPRESS_CORE_COLLISION_BVH_COLLISION_ACCELERATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_COLLISION_BVH_COLLISION_ACCELERATOR_H_

#include <optional>
#include <vector>

#include "absl/types/span.h"
#include "core/collision/bvh.h"
#include "core/collision/collision_helpers.h"
#include "core/collision/mesh_collision_accelerator.h"
#include "core/collision/ray.h"
#include "core/geometry/shapes/box.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"

namespace imp {

// A collider class to accelerate ray/GltfMesh intersections.
class BvhCollisionAccelerator : public MeshCollisionAccelerator{
 public:
  explicit BvhCollisionAccelerator(Bvh::Options options = {})
      : options_(options) {}
  void Create(absl::Span<const MeshVertexAndIndexData> mesh_data,
                      Box aabb) override;

  // Return the closest intersection between a ray and the GltfMesh.
  // Note that the returned normal is the normal of the intersected triangle
  // and not the normal by averaging the normals at the vertices of the triangle
  // since that is simpler and the averaged normal is not needed for our ray
  // tracing applications at the moment.
  std::optional<collision::MultiPrimitiveMeshIntersection<float>>
  Intersect(const Ray& ray) const override;

 private:
  // TODO: Add a option for backface intersections.
  Bvh::Options options_;
  std::vector<Bvh> bvhs_;
};

}  // namespace imp

#endif  // #ifndef THIRD_PARTY_IMPRESS_CORE_COLLISION_BVH_COLLISION_ACCELERATOR_H_
