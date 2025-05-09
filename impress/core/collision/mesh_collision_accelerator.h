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

#ifndef THIRD_PARTY_IMPRESS_CORE_COLLISION_MESH_COLLISION_ACCELERATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_COLLISION_MESH_COLLISION_ACCELERATOR_H_

#include <optional>

#include "absl/types/span.h"
#include "core/collision/collision_helpers.h"
#include "core/collision/ray.h"
#include "core/geometry/shapes/box.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"

namespace imp {

// An interface for a collision accelerator for a mesh.
class MeshCollisionAccelerator {
 public:
  virtual ~MeshCollisionAccelerator() = default;

  // Builds the collision accelerator.
  virtual void Create(absl::Span<const MeshVertexAndIndexData> mesh_data,
                      Box aabb) = 0;

  // Uses the collision accelerator to intersect a ray with the mesh.
  virtual std::optional<collision::MultiPrimitiveMeshIntersection<float>>
  Intersect(const Ray& ray) const = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COLLISION_MESH_COLLISION_ACCELERATOR_H_
