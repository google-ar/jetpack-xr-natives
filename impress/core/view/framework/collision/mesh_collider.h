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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_COLLISION_MESH_COLLIDER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_COLLISION_MESH_COLLIDER_H_

#include "absl/status/status.h"
#include "absl/types/optional.h"
#include "core/collision/collider_mask_helpers.h"
#include "core/collision/collision_flags.h"
#include "core/collision/ray.h"
#include "core/common/enum_flags.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/isf_info.h"
#include "core/view/framework/collision/collider_state.proto.imp.h"
#include "core/view/framework/collision/ray_hit.h"
#include "core/view/framework/render/mesh_renderer.h"

namespace imp {

/* Enables mesh colliders for meshes created dynamically. To add this, the
 * node must have a MeshRenderer that holds the mesh data.
 */
class MeshCollider : public Component,
                     public ColliderMaskHelpers<MeshCollider> {
 public:
  absl::Status Setup();
  absl::Status Setup(MeshColliderState::ColliderMode mode);
  absl::Status SetupWithState();
  void Cleanup();

  absl::optional<RayHit> Intersect(const Ray& world_ray);
  absl::optional<DoubleRayHit> IntersectPrecise(const DoubleRay& world_ray);

  void Visualize(VisualizationStyle visualization_style =
                     VisualizationStyle::kNotSelected) const;

  void OnActiveStatusChanged(bool is_active);

 private:
  friend class ColliderMaskHelpers<MeshCollider>;
  Flags<CollisionMask> CollisionFlags() const { return collision_flags_; }

  ComponentHandle<MeshRenderer> mesh_renderer_;
  MeshColliderState state_;
  Flags<CollisionMask> collision_flags_{CollisionMask::kDefault};

 public:
  using IsfInfo = IsfInfo<&MeshCollider::state_>;
  static constexpr bool kRunInEditMode = true;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_COLLISION_MESH_COLLIDER_H_
