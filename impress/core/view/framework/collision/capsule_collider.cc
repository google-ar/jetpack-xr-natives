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

#include "core/view/framework/collision/capsule_collider.h"

#include <optional>

#include "core/common/log.h"
#include "absl/types/optional.h"
#include "filament/libs/math/include/math/TMatHelpers.h"
#include "core/collision/collision_flags.h"
#include "core/collision/collision_helpers.h"
#include "core/collision/ray.h"
#include "core/common/debug_draw.h"
#include "core/config.h"
#include "core/geometry/capsule_helper.h"
#include "core/geometry/shapes/capsule.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/framework/collision/collision_manager.h"
#include "core/view/framework/collision/collision_system.h"
#include "core/view/framework/collision/ray_hit.h"

#if IMP_RUNTIME(DEV)
#include "core/ncsb/path_manager.h"
#include "core/view/framework/collision/compound_collider.h"
#endif

namespace imp {
using split_engine::SplitEngineSerializer;

void CapsuleCollider::Setup() { SetCapsule(float3{0.0f}, 1.0f, 0.5f); }

void CapsuleCollider::Setup(const Capsule& capsule) { SetCapsule(capsule); }

void CapsuleCollider::SetupWithState() {
  SetCapsule(state_.center, state_.height, state_.radius);
}

void CapsuleCollider::SetCapsule(const Capsule& capsule) {
  SetCapsule(capsule.center, capsule.height, capsule.radius);
}

void CapsuleCollider::SetCapsule(float3 center, float height, float radius) {
  if (radius < 0.0f || height < 0.0f) {
    IMP_LOG(imp::ERROR) << "Cannot set negative height or radius on a CapsuleCollider.";
  }
  state_.center = center;
  state_.height = height;
  state_.radius = radius;
  hit_node_ = GetNode();

  if (SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    serializer->SetCapsuleCollider(GetEntity(), GetCapsule(), IsActive());
  }
}

void CapsuleCollider::SetHitNode(NodeHandle hit_node) { hit_node_ = hit_node; };

NodeHandle CapsuleCollider::GetHitNode() const {
#if IMP_RUNTIME(DEV)
  if (editor::IsInEditMode(GetView().GetRegistry())) {
    if (auto compound_collider =
            GetView()
                .GetPathManager()
                .GetComponentFromAncestorOrSelf<CompoundCollider>(GetNode())) {
      return compound_collider->GetNode();
    }
  }
#endif
  return hit_node_;
}

absl::optional<RayHit> CapsuleCollider::Intersect(const Ray& world_ray) {
  if (!IsActive()) {
    return {};
  }

  const mat4f local_from_world = inverse(GetNode()->GetWorldTrs());
  const Ray local_ray = world_ray.GetTransformed(local_from_world);
  std::optional<collision::RayIntersection<float>> result =
      CapsuleIntersectsRay<float>(
          Capsule{state_.center, state_.height, state_.radius}, local_ray);
  if (!result.has_value()) {
    return {};
  }

  // Transform the collision point back into world space.
  const float3 world_collision_point =
      GetNode()->WorldFromLocalPoint(result->collision_point);
  const float3 world_surface_normal =
      GetNode()->WorldFromLocalVector(result->normal);
  return RayHit(norm(world_collision_point - world_ray.origin), quatf{0.0f},
                world_collision_point, GetHitNode(), world_surface_normal);
}

absl::optional<DoubleRayHit> CapsuleCollider::IntersectPrecise(
    const DoubleRay& world_ray) {
  if (!IsActive()) {
    return {};
  }

  const mat4 local_from_world = inverse(GetNode()->GetWorldTrsPrecise());
  const DoubleRay local_ray = world_ray.GetTransformed(local_from_world);
  std::optional<collision::RayIntersection<double>> result =
      CapsuleIntersectsRay<double>(
          Capsule{state_.center, state_.height, state_.radius}, local_ray);
  if (!result.has_value()) {
    return {};
  }

  // Transform the collision point back into world space.
  const double3 world_collision_point =
      GetNode()->WorldFromLocalPointPrecise(result->collision_point);
  const double3 world_surface_normal =
      GetNode()->WorldFromLocalVectorPrecise(result->normal);
  return DoubleRayHit(norm(world_collision_point - world_ray.origin),
                      quat{0.0f}, world_collision_point, GetHitNode(),
                      world_surface_normal);
}

Capsule CapsuleCollider::GetCapsule() const {
  return Capsule{state_.center, state_.height, state_.radius};
}

void CapsuleCollider::Visualize(VisualizationStyle visualization_style) const {
  debug_draw::Color color =
      debug_draw::DefaultColorFromVisualizationStyle(visualization_style);
  Capsule capsule = GetCapsule();
  debug_draw::Local(GetNode().GetEntity())
      .CapsuleLines(capsule.center, capsule.height, capsule.radius, color);
}

void CapsuleCollider::System::BeforeFirstComponentAdded() {
  GetView()
      .GetCollisionManager()
      .AddCollisionSystem<CapsuleCollider, CollisionSystem>(&GetView());
}

void CapsuleCollider::Cleanup() {
  if (SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    serializer->ClearCollider(
        GetEntity(),
        split_engine::SplitEngineSerializer::ColliderType::kCapsuleCollider);
  }
}

void CapsuleCollider::OnActiveStatusChanged(bool is_active) {
  if (SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    serializer->SetCapsuleCollider(GetEntity(), GetCapsule(), is_active);
  }
}

}  // namespace imp
