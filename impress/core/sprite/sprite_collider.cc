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

#include "core/sprite/sprite_collider.h"

#include <optional>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "core/collision/collision_flags.h"
#include "core/collision/collision_helpers.h"
#include "core/common/debug_draw.h"
#include "core/common/enum_flags.h"
#include "core/common/platform_helpers.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_system.h"
#include "core/sprite/sprite_renderer.h"
#include "core/sprite/sprite_renderer_state.proto.imp.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/framework/collision/collision_manager.h"
#include "core/view/framework/collision/collision_system.h"
#include "core/view/framework/collision/ray_hit.h"

namespace imp {

absl::Status SpriteCollider::Setup() {
  ComponentHandle<SpriteRenderer> sprite_renderer =
      GetNode()->GetComponent<SpriteRenderer>();
  if (!sprite_renderer) {
    return absl::FailedPreconditionError(
        "Node does not contain a SpriteRenderer.");
  }

  sprite_renderer_ = sprite_renderer;
  collision_flags_ = ToFlags(CollisionMask::kSpriteCollider);
  return absl::OkStatus();
}

std::optional<RayHit> SpriteCollider::Intersect(float2 screen_pos) {
  // Cannot intersect with an inactive collider.
  if (!IsActive()) {
    return std::nullopt;
  }

  if (!sprite_renderer_) {
    IMP_LOG(imp::ERROR) << "SpriteCollider::Intersect called without a SpriteRenderer";
    return std::nullopt;
  }

  const float3 screen_position = float3(screen_pos, 0);

  float2 intersect_position;
  Rect collision_bounds;

  if (sprite_renderer_->GetRenderSpace() ==
      SpriteRendererState::RenderSpace::SCREEN) {
    // Get the hit point in the texture space.
    const mat4f screen_to_texture_matrix =
        inverse(sprite_renderer_->GetTextureToScreenMatrix());
    intersect_position = (screen_to_texture_matrix * screen_position).xy;
    collision_bounds =
        Rect{.center = {0.5f, 0.5f}, .half_extent = {0.5f, 0.5f}};
  } else {
    intersect_position = screen_pos;
    std::optional<Rect> screen_bounds = GetScreenBounds();
    if (!screen_bounds.has_value()) {
      return std::nullopt;
    }

    collision_bounds = *screen_bounds;
  }

  // Modify the bounds based on the padding ratio;
  collision_bounds.half_extent *= (1 + state_.padding_ratio);

  // TODO Consider adding a helper function to compare the 2D AABB
  // with the point, instead of making 3D AABB checking.
  if (collision::RectContainsPoint(collision_bounds, intersect_position) !=
      collision::Result::kDoesIntersect) {
    return std::nullopt;
  }

  return RayHit(0.0f, kIdentityQuatf, screen_position, GetNode());
}

std::optional<DoubleRayHit> SpriteCollider::IntersectPrecise(
    float2 screen_pos) {
  // Final result is in screen coordinates so there isn't a meaningful loss in
  // precision to go from float to double.
  std::optional<RayHit> result = Intersect(screen_pos);
  if (!result) {
    return std::nullopt;
  }

  return DoubleRayHit(*result);
}

std::optional<Rect> SpriteCollider::GetScreenBounds() const {
  if (sprite_renderer_->GetRenderSpace() ==
      SpriteRendererState::RenderSpace::WORLD_TO_SCREEN) {
    std::optional<float2> screen_point =
        GetView().GetCameraManager().GetCamera()->PixelFromWorldPointPrecise(
            GetNode()->GetWorldPositionPrecise());

    if (!screen_point.has_value()) {
      return std::nullopt;
    }

    Rect bounds = sprite_renderer_->GetLocalBounds();
    bounds.center += *screen_point;
    return bounds;
  } else {
    IMP_LOG(imp::FATAL) << "SpriteCollider::GetScreenBounds not implemented for this "
                  "renderspace";
    return Rect();
  }
}

void SpriteCollider::Visualize(VisualizationStyle visualization_style) {
  debug_draw::Color color =
      debug_draw::DefaultColorFromVisualizationStyle(visualization_style);
  if (sprite_renderer_->GetRenderSpace() ==
      SpriteRendererState::RenderSpace::WORLD_TO_SCREEN) {
    std::optional<Rect> screen_bounds = GetScreenBounds();
    if (!screen_bounds.has_value()) {
      return;
    }
    debug_draw::NormalizedScreen().RectLines(
        debug_draw::ScreenToClipSpace(GetView().GetSize(), *screen_bounds),
        color);
  }
}

SpriteCollider::System::System(BaseView* view) : ComponentSystem(view) {}

void SpriteCollider::System::BeforeFirstComponentAdded() {
  GetView()
      .GetCollisionManager()
      .AddCollisionSystem<SpriteCollider, CollisionSystem>(&GetView());
}

void SpriteCollider::System::AfterLastComponentRemoved() {
  GetView()
      .GetCollisionManager()
      .RemoveCollisionSystem<SpriteCollider, CollisionSystem>();
}

}  // namespace imp
