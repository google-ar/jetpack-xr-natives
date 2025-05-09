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

#ifndef THIRD_PARTY_IMPRESS_CORE_COLLISION_COLLISION_ACCELERATOR_PROVIDER_H_
#define THIRD_PARTY_IMPRESS_CORE_COLLISION_COLLISION_ACCELERATOR_PROVIDER_H_

#include <functional>
#include <memory>
#include <optional>

#include "absl/types/span.h"
#include "core/collision/mesh_collision_accelerator.h"
#include "core/geometry/shapes/box.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"

namespace imp {

// Provides default or customized collision accelerators on demand.
//
// Example usage for customized collision accelerators:
//
// CollisionAcceleratorProvider& provider =
// view->GetRegistry().GetOrCreate<CollisionAcceleratorProvider>();
// provider.SetMeshCollisionAcceleratorFactory(
//     [](absl::Span<const MeshVertexAndIndexData> mesh_data,
//         Box aabb) -> std::unique_ptr<MeshCollisionAccelerator> {
//       auto custom_mesh_bvh =
//       std::make_unique<CustomMeshCollisionAccelerator>();
//       custom_mesh_bvh->Create(mesh_data, aabb);
//       return custom_mesh_bvh;
//     });
//
// After this, all calls to provider.GetMeshCollisionAccelerator() will return
// the custom collision accelerator.
class CollisionAcceleratorProvider {
 public:
  CollisionAcceleratorProvider();
  // Sets the factory function for creating mesh collision accelerators.
  void SetMeshCollisionAcceleratorFactory(
      std::function<std::unique_ptr<MeshCollisionAccelerator>(
          absl::Span<const MeshVertexAndIndexData> mesh_data, Box aabb)>
          mesh_collision_accelerator_factory);

  // Returns a collision accelerator for the given mesh data and aabb, using the
  // stored factory function.
  std::unique_ptr<MeshCollisionAccelerator> GetMeshCollisionAccelerator(
      absl::Span<const MeshVertexAndIndexData> mesh_data, Box aabb);

 private:
  std::optional<std::function<std::unique_ptr<MeshCollisionAccelerator>(
      absl::Span<const MeshVertexAndIndexData> mesh_data, Box aabb)>>
      mesh_collision_accelerator_factory_;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COLLISION_COLLISION_ACCELERATOR_PROVIDER_H_
