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

#include "core/view/framework/collision/sphere_collider.h"

#include "core/common/log.h"
#include "absl/types/optional.h"
#include "filament/libs/math/include/math/TMatHelpers.h"
#include "core/collision/collision_flags.h"
#include "core/collision/collision_helpers.h"
#include "core/collision/ray.h"
#include "core/common/debug_draw.h"
#include "core/config.h"
#include "core/geometry/shapes/sphere.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/framework/collision/ray_hit.h"

#if IMP_RUNTIME(DEV)
#include "core/ncsb/path_manager.h"
#include "core/view/framework/collision/compound_collider.h"
#endif

namespace imp {
using split_engine::SplitEngineSerializer;

SphereCollider::SphereCollider()
    : collision_flags_(CollisionMask::kColliderSphere) {}

void SphereCollider::Setup() { SetSphere(float3{0.0f}, 1.0f); }

void SphereCollider::Setup(const Sphere& sphere) { SetSphere(sphere); }

void SphereCollider::Setup(float3 center, float radius) {
  SetSphere(center, radius);
}

void SphereCollider::SetupWithState() {
  SetSphere(state_.center, state_.radius);
}

void SphereCollider::SetSphere(const Sphere& sphere) {
  SetSphere(sphere.center, sphere.radius);
}

void SphereCollider::SetSphere(float3 center, float radius) {
  if (radius < 0.0f) {
    radius = 0.0f;
    IMP_LOG(imp::ERROR) << "Cannot set negative radius on a SphereCollider.";
  }
  state_.center = center;
  state_.radius = radius;
  hit_node_ = GetNode();
  UpdateSplitEngine();
}

void SphereCollider::SetHitNode(NodeHandle hit_node) { hit_node_ = hit_node; };

NodeHandle SphereCollider::GetHitNode() const {
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

absl::optional<RayHit> SphereCollider::Intersect(const Ray& world_ray) {
  if (!IsActive()) {
    return {};
  }
  float3 collision_point;
  float distance;
  const mat4f local_from_world = inverse(GetNode()->GetWorldTrs());
  const Ray local_ray = world_ray.GetTransformed(local_from_world);
  if (collision::SphereIntersectsRay(Sphere{state_.center, state_.radius},
                                     local_ray, &distance, &collision_point) ==
      collision::Result::kDoesIntersect) {
    const float3 local_normal = collision_point - state_.center;
    // Transform the collision point back into world space.
    const float3 world_collision_point =
        GetNode()->WorldFromLocalPoint(collision_point);
    const float3 transformed_surface_normal =
        TransformedSurfaceNormal(local_normal, GetNode()->GetWorldTrs());
    return RayHit(distance, quatf{0.0f}, world_collision_point, GetHitNode(),
                  transformed_surface_normal);
  }
  // No collision (empty).
  return {};
}

absl::optional<DoubleRayHit> SphereCollider::IntersectPrecise(
    const DoubleRay& world_ray) {
  if (!IsActive()) {
    return {};
  }
  double3 collision_point;
  double distance;
  const mat4 local_from_world = inverse(GetNode()->GetWorldTrsPrecise());
  const DoubleRay local_ray = world_ray.GetTransformed(local_from_world);
  if (collision::SphereIntersectsRay(Sphere{state_.center, state_.radius},
                                     local_ray, &distance, &collision_point) ==
      collision::Result::kDoesIntersect) {
    const double3 local_normal = collision_point - state_.center;
    // Transform the collision point back into world space.
    const double3 world_collision_point =
        GetNode()->WorldFromLocalPointPrecise(collision_point);
    const double3 transformed_surface_normal =
        TransformedSurfaceNormal(local_normal, GetNode()->GetWorldTrsPrecise());
    return DoubleRayHit(distance, quat{0.0}, world_collision_point,
                        GetHitNode(), transformed_surface_normal);
  }
  // No collision (empty).
  return {};
}

Sphere SphereCollider::GetSphere() const {
  return Sphere{state_.center, state_.radius};
}

Sphere SphereCollider::GetWorldSphere() const {
  return Sphere{GetNode()->WorldFromLocalPoint(state_.center),
                GetNode()->GetWorldScale().x * state_.radius};
}

void SphereCollider::Visualize(VisualizationStyle visualization_style) const {
  debug_draw::Color color =
      debug_draw::DefaultColorFromVisualizationStyle(visualization_style);
  Sphere sphere = GetSphere();
  debug_draw::Local(GetNode().GetEntity())
      .SphereLines(sphere.center, sphere.radius, color);
}

void SphereCollider::UpdateSplitEngine() {
  if (SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    serializer->SetSphereCollider(GetEntity(), GetSphere(), IsActive());
  }
}

void SphereCollider::Cleanup() {
  if (SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    serializer->ClearCollider(
        GetEntity(),
        split_engine::SplitEngineSerializer::ColliderType::kSphereCollider);
  }
}

void SphereCollider::OnActiveStatusChanged(bool is_active) {
  if (SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    serializer->SetSphereCollider(GetEntity(), GetSphere(), is_active);
  }
}

}  // namespace imp
