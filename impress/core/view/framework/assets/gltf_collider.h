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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_GLTF_COLLIDER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_GLTF_COLLIDER_H_

#include "core/collision/collider_mask_helpers.h"
#include "core/collision/collision_flags.h"
#include "core/collision/ray.h"
#include "core/config.h"
#include "core/ncsb/component.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/collision/ray_hit.h"

namespace imp {

// Enables box collision testing on a Node.
class GltfCollider : public Component,
                     public ColliderMaskHelpers<GltfCollider> {
 public:
  // CollisionMode decides the type of collider to use.
  enum class CollisionMode {
    // By default, use AABB collider.
    kBounds,
    // This enables mesh collider.
    kTriangles,
  };

  void Setup(ComponentHandle<GltfMesh> gltf_mesh,
             CollisionMode mode = CollisionMode::kBounds);
  void Cleanup();

  absl::optional<RayHit> Intersect(const Ray& world_ray);
  // If bvh collision accelerator is used, this function will be able to deal
  // with large positions. However, intersection with mesh boundaries will not
  // be guaranteed.
  absl::optional<DoubleRayHit> IntersectPrecise(const DoubleRay& world_ray);

  // Returns true if this collider is configured to use per-triangle collision
  // and per-triangle data is available.
  bool UsingPerTriangleCollision() const;

  void Visualize(VisualizationStyle visualization_style =
                     VisualizationStyle::kNotSelected) const;

  void OnActiveStatusChanged(bool is_active);

 private:
  friend class ColliderMaskHelpers<GltfCollider>;
  Flags<CollisionMask> CollisionFlags() const { return collision_flags_; }

  ComponentHandle<GltfMesh> gltf_mesh_;
  CollisionMode mode_;
  Flags<CollisionMask> collision_flags_{CollisionMask::kDefault};
#if IMP_RUNTIME(DEV)
  bool visualize_mesh_data_ = false;
#endif
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_GLTF_COLLIDER_H_
