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

#ifndef THIRD_PARTY_IMPRESS_CORE_TEXT_TEXT_COLLIDER_H_
#define THIRD_PARTY_IMPRESS_CORE_TEXT_TEXT_COLLIDER_H_

#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/collision/collider_mask_helpers.h"
#include "core/collision/collision_flags.h"
#include "core/collision/ray.h"
#include "core/common/enum_flags.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_system.h"
#include "core/ncsb/isf_info.h"
#include "core/text/text_collider_state.proto.imp.h"
#include "core/text/text_renderer.h"
#include "core/view/framework/collision/ray_hit.h"

namespace imp {

// A collider for the TextRenderer. The TextCollider must be added after the
// TextRenderer on the same node, and the intersection tests are done with
// only that TextRenderer.
class TextCollider : public Component,
                     public ColliderMaskHelpers<TextCollider> {
 public:
  // Sets up the TextCollider. There must be a TextRenderer on the same node
  // as this collider, or the Setup will fail.
  absl::Status Setup();

  // Determines if this collider is intersecting with a screen position.
  // If an intersection is found, returns a RayHit containing the node this
  // component is attached to. Otherwise, returns nullopt.
  std::optional<RayHit> Intersect(float2 screen_pos);

  // Determines if this collider is intersecting with a world ray.
  // If an intersection is found, returns a RayHit containing the node this
  // component is attached to. Otherwise, returns nullopt.
  std::optional<RayHit> Intersect(const Ray& ray);

  // Unsupported, throws an error.
  std::optional<DoubleRayHit> IntersectPrecise(float2 screen_pos);

  // Determines if this collider is intersecting with a world ray.
  // If an intersection is found, returns a RayHit containing the node this
  // component is attached to. Otherwise, returns nullopt.
  std::optional<DoubleRayHit> IntersectPrecise(const DoubleRay& ray);

  // Update padding ratio, must be greater than -1.
  void UpdatePaddingRatio(float padding_ratio) {
    // It is assumed padding ratio is always greater than -1.0f.
    state_.padding_ratio = padding_ratio;
  }

  bool IsInScreenSpace() const;

  absl::StatusOr<Rect> GetScreenBounds() const;
  Box GetLocalBounds() const;
  Box GetWorldBounds() const;
  void Visualize(VisualizationStyle visualization_style =
                     VisualizationStyle::kNotSelected);

  Flags<CollisionMask> CollisionFlags() const { return collision_flags_; }

  // ComponentSystem for registering the TextCollider to the relevant
  // CollisionSystem
  class System : public ComponentSystem<TextCollider> {
   public:
    explicit System(BaseView* view) : ComponentSystem<TextCollider>(view) {}

    void BeforeFirstComponentAdded() override;
    void AfterLastComponentRemoved() override;

    // Value indicating how much to pad the bounds of the text when doing
    // intersection tests in VertexDomain::DEVICE (ie. screenspace); this
    // padding is applied after `state_.padding_ratio`, if both are present. For
    // example, if this is set to 5, then the size of the screen-bounds will be
    // expanded by 5px in all directions for intersection tests.
    void SetScreenPadding(float screen_padding);

    // Returns the screen padding for all TextColliders.
    float GetScreenPadding() const;

   private:
    float screen_padding_ = 0.0f;
  };

 private:
  TextColliderState state_;
  ComponentHandle<TextRenderer> text_renderer_;
  friend class imp::ColliderMaskHelpers<TextCollider>;
  imp::Flags<imp::CollisionMask> collision_flags_;
  std::vector<float2> path_text_collider_outline_;
  absl::StatusOr<std::vector<float2>> GetScaledScreenPath() const;

 public:
  using IsfInfo = IsfInfo<&TextCollider::state_, IsfDependencies<TextRenderer>>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_TEXT_TEXT_COLLIDER_H_
