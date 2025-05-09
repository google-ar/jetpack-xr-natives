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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPRITE_SPRITE_COLLIDER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPRITE_SPRITE_COLLIDER_H_

#include <optional>

#include "absl/status/status.h"
#include "core/collision/collider_mask_helpers.h"
#include "core/collision/collision_flags.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_system.h"
#include "core/ncsb/isf_info.h"
#include "core/sprite/sprite_collider_state.proto.imp.h"
#include "core/sprite/sprite_renderer.h"
#include "core/view/framework/collision/ray_hit.h"

namespace imp {

// A collider for the SpriteRenderer. The SpriteCollider must be added after the
// SpriteRenderer on the same node, and the intersection tests are done with
// only that SpriteRenderer.
class SpriteCollider : public Component,
                       public ColliderMaskHelpers<SpriteCollider> {
 public:
  class System : public ComponentSystem<SpriteCollider> {
   public:
    explicit System(BaseView* view);

    void BeforeFirstComponentAdded() override;
    void AfterLastComponentRemoved() override;
  };

  // Sets up the SpriteCollider. There must be a SpriteRenderer on the same node
  // as this collider, or the Setup will fail.
  absl::Status Setup();

  // Determines if this collider is intersecting with a screen position.
  // If an intersection is found, returns a RayHit containing the node this
  // component is attached to. Otherwise, returns nullopt.
  std::optional<RayHit> Intersect(float2 screen_pos);

  // Determines if this collider is intersecting with a screen position.
  // If an intersection is found, returns a DoubleRayHit containing the node
  // this component is attached to. Otherwise, returns nullopt.
  std::optional<DoubleRayHit> IntersectPrecise(float2 screen_pos);

  std::optional<Rect> GetScreenBounds() const;

  void Visualize(VisualizationStyle visualization_style =
                     VisualizationStyle::kNotSelected);

 private:
  friend class ColliderMaskHelpers<SpriteCollider>;

  SpriteColliderState state_;
  ComponentHandle<SpriteRenderer> sprite_renderer_;
  imp::Flags<imp::CollisionMask> collision_flags_;

 public:
  using IsfInfo =
      IsfInfo<&SpriteCollider::state_, IsfDependencies<SpriteRenderer>>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_SPRITE_SPRITE_COLLIDER_H_
