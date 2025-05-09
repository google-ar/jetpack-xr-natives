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

#include "core/view/framework/collision/box_collider.h"

#include <optional>

#include "absl/types/optional.h"
#include "core/collision/collision_flags.h"
#include "core/collision/collision_helpers.h"
#include "core/collision/ray.h"
#include "core/common/debug_draw.h"
#include "core/common/filament_helpers.h"
#include "core/math/math.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/framework/collision/ray_hit.h"

namespace imp {

BoxCollider::BoxCollider() : collision_flags_(CollisionMask::kColliderBox) {}
absl::optional<RayHit> BoxCollider::Intersect(const Ray& world_ray) {
  if (!IsActive()) {
    return {};
  }

  mat4f local_from_world = inverse(GetNode()->GetWorldTrs());
  Ray local_ray = world_ray.GetTransformed(local_from_world);
  std::optional<collision::RayIntersection<float>> intersection =
      collision::AABBIntersectsRay(state_.box, local_ray);
  if (intersection.has_value()) {
    mat4f world_from_local = GetNode()->GetWorldTrs();
    return RayHit(
        intersection->distance,
        // TODO Output the correct surface normal.
        quatf{0.0},
        // Transform the collision point back into world space.
        GetNode()->WorldFromLocalPoint(intersection->collision_point),
        GetNode(),
        TransformedSurfaceNormal(intersection->normal, world_from_local));
  }
  // No collision (empty).
  return {};
}

absl::optional<DoubleRayHit> BoxCollider::IntersectPrecise(
    const DoubleRay& world_ray) {
  if (!IsActive()) {
    return {};
  }

  mat4 local_from_world = inverse(GetNode()->GetWorldTrsPrecise());
  DoubleRay local_ray = world_ray.GetTransformed(local_from_world);
  std::optional<collision::RayIntersection<double>> intersection =
      collision::AABBIntersectsRay(state_.box, local_ray);
  if (intersection.has_value()) {
    mat4 world_from_local = GetNode()->GetWorldTrsPrecise();
    return DoubleRayHit(
        intersection->distance,
        // TODO Output the correct surface normal.
        quat{0.0},
        // Transform the collision point back into world space.
        GetNode()->WorldFromLocalPointPrecise(intersection->collision_point),
        GetNode(),
        TransformedSurfaceNormal(intersection->normal, world_from_local));
  }
  // No collision (empty).
  return {};
}

Box BoxCollider::GetLocalBox() const { return state_.box; }

Box BoxCollider::GetWorldBox() const {
  return imp::TransformBounds(state_.box, GetNode()->GetWorldTrs());
}

void BoxCollider::Visualize(VisualizationStyle visualization_style) const {
  debug_draw::Color color =
      debug_draw::DefaultColorFromVisualizationStyle(visualization_style);
  debug_draw::Local(GetNode().GetEntity()).BoxLines(GetLocalBox(), color);
}

void BoxCollider::Setup() { UpdateSplitEngine(); }

void BoxCollider::SetupWithState() { Setup(state_.box); }

void BoxCollider::Setup(Box box) {
  state_.box = box;
  UpdateSplitEngine();
}

void BoxCollider::SetBox(Box box) {
  state_.box = box;
  UpdateSplitEngine();
}

void BoxCollider::Cleanup() {
  if (split_engine::SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    serializer->ClearCollider(
        GetEntity(),
        split_engine::SplitEngineSerializer::ColliderType::kBoxCollider);
  }
}

void BoxCollider::OnActiveStatusChanged(bool is_active) {
  if (split_engine::SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    serializer->SetBoxCollider(GetEntity(), state_.box, is_active);
  }
}

void BoxCollider::UpdateSplitEngine() {
  if (split_engine::SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    serializer->SetBoxCollider(GetEntity(), state_.box, IsActive());
  }
}

}  // namespace imp
