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

#include "core/view/framework/collision/cone_collider.h"

#include <optional>

#include "core/common/log.h"
#include "absl/types/optional.h"
#include "core/collision/collision_flags.h"
#include "core/collision/collision_helpers.h"
#include "core/collision/ray.h"
#include "core/common/debug_draw.h"
#include "core/geometry/shapes/cone.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/framework/collision/collision_manager.h"
#include "core/view/framework/collision/collision_system.h"
#include "core/view/framework/collision/ray_hit.h"

namespace imp {
using split_engine::SplitEngineSerializer;

void ConeCollider::Setup() { SetCone(kZero3, 0.5f, 1.0f); }

void ConeCollider::Setup(const Cone& cone) { SetCone(cone); }

void ConeCollider::SetupWithState() {
  SetCone(state_.base, state_.radius, state_.height);
}

void ConeCollider::SetCone(const Cone& cone) {
  SetCone(cone.base, cone.radius, cone.height);
}

void ConeCollider::SetCone(float3 base, float radius, float height) {
  if (radius < 0.0f || height < 0.0f) {
    IMP_LOG(imp::ERROR) << "Cannot set negative radius or height on a ConeCollider.";
  }
  state_.base = base;
  state_.radius = radius;
  state_.height = height;

  if (SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    // TODO: Re-enable this once the bug is fixed.
    // serializer->SetConeCollider(GetEntity(), GetCone(), IsActive());
  }
}

absl::optional<RayHit> ConeCollider::Intersect(const Ray& world_ray) {
  if (!IsActive()) {
    return std::nullopt;
  }

  const mat4f local_from_world = inverse(GetNode()->GetWorldTrs());
  const Ray local_ray = world_ray.GetTransformed(local_from_world);

  std::optional<collision::RayIntersection<float>> intersection =
      collision::ConeIntersectsRay<float>(GetCone(), local_ray);
  if (!intersection.has_value()) {
    return std::nullopt;
  }

  mat4f world_from_local = GetNode()->GetWorldTrs();
  return RayHit(
      intersection->distance, quatf{0.0f},
      // Transform the collision point back into world space.
      GetNode()->WorldFromLocalPoint(intersection->collision_point), GetNode(),
      TransformedSurfaceNormal(intersection->normal, world_from_local));
}

absl::optional<DoubleRayHit> ConeCollider::IntersectPrecise(
    const DoubleRay& world_ray) {
  if (!IsActive()) {
    return std::nullopt;
  }

  const mat4 local_from_world = inverse(GetNode()->GetWorldTrsPrecise());
  const DoubleRay local_ray = world_ray.GetTransformed(local_from_world);

  std::optional<collision::RayIntersection<double>> intersection =
      collision::ConeIntersectsRay<double>(GetCone(), local_ray);
  if (!intersection.has_value()) {
    return std::nullopt;
  }

  mat4 world_from_local = GetNode()->GetWorldTrsPrecise();
  return DoubleRayHit(
      intersection->distance, quat{0.0f},
      // Transform the collision point back into world space.
      GetNode()->WorldFromLocalPointPrecise(intersection->collision_point),
      GetNode(),
      TransformedSurfaceNormal(intersection->normal, world_from_local));
}

Cone ConeCollider::GetCone() const {
  return Cone{state_.base, state_.radius, state_.height};
}

void ConeCollider::Visualize(VisualizationStyle visualization_style) const {
  debug_draw::Color color =
      debug_draw::DefaultColorFromVisualizationStyle(visualization_style);
  debug_draw::Local(GetNode().GetEntity())
      .ConeLines(state_.base, state_.radius, state_.height, color);
}

void ConeCollider::System::BeforeFirstComponentAdded() {
  GetView()
      .GetCollisionManager()
      .AddCollisionSystem<ConeCollider, CollisionSystem>(&GetView());
}

void ConeCollider::Cleanup() {
  if (SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    // TODO: Re-enable this once the bug is fixed.
    // serializer->ClearCollider(
    //     GetEntity(),
    //     split_engine::SplitEngineSerializer::ColliderType::kConeCollider);
  }
}

void ConeCollider::OnActiveStatusChanged(bool is_active) {
  if (SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    // TODO: Re-enable this once the bug is fixed.
    // serializer->SetConeCollider(GetEntity(), GetCone(), is_active);
  }
}

}  // namespace imp
