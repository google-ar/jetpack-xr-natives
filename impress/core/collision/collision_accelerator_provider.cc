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

#include "core/collision/collision_accelerator_provider.h"

#include <functional>
#include <memory>
#include <optional>
#include <utility>

#include "absl/types/span.h"
#include "core/collision/bvh_collision_accelerator.h"
#include "core/collision/mesh_collision_accelerator.h"
#include "core/geometry/shapes/box.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"

namespace imp {

CollisionAcceleratorProvider::CollisionAcceleratorProvider() {
  SetMeshCollisionAcceleratorFactory(
      [](absl::Span<const MeshVertexAndIndexData> mesh_data,
         Box aabb) -> std::unique_ptr<MeshCollisionAccelerator> {
        auto mesh_bvh = std::make_unique<BvhCollisionAccelerator>();
        mesh_bvh->Create(mesh_data, aabb);
        return mesh_bvh;
      });
}

void CollisionAcceleratorProvider::SetMeshCollisionAcceleratorFactory(
    std::function<std::unique_ptr<MeshCollisionAccelerator>(
        absl::Span<const MeshVertexAndIndexData> mesh_data, Box aabb)>
        mesh_collision_accelerator_factory) {
  mesh_collision_accelerator_factory_ =
      std::move(mesh_collision_accelerator_factory);
}

std::unique_ptr<MeshCollisionAccelerator>
CollisionAcceleratorProvider::GetMeshCollisionAccelerator(
    absl::Span<const MeshVertexAndIndexData> mesh_data, Box aabb) {
  if (!mesh_collision_accelerator_factory_) {
    return nullptr;
  }
  return (*mesh_collision_accelerator_factory_)(mesh_data, aabb);
}

}  // namespace imp
