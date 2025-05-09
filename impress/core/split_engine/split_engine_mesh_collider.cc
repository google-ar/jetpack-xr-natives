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

#include "core/split_engine/split_engine_mesh_collider.h"

#include <vector>

#include "core/common/log.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "filament/libs/math/include/math/TMatHelpers.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/collision/collision_flags.h"
#include "core/collision/collision_helpers.h"
#include "core/collision/ray.h"
#include "core/common/debug_draw.h"
#include "core/geometry/shapes/box.h"
#include "core/math/almost_equal.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"
#include "core/render/base_renderable_manager.h"
#include "core/split_engine/split_engine_renderable_info.h"
#include "core/view/framework/collision/collision_manager.h"
#include "core/view/framework/collision/collision_system.h"
#include "core/view/framework/collision/ray_hit.h"

namespace imp {
namespace {

Box GetLocalBounds(const BaseRenderableManager& renderable_manager,
                   const utils::Entity& entity) {
  return renderable_manager.GetAxisAlignedBoundingBox(
      renderable_manager.GetInstance(entity));
}

}  // namespace

absl::optional<RayHit> SplitEngineMeshCollider::Intersect(
    const Ray& world_ray) {
  if (!IsActive()) {
    return {};
  }

  if (RoughlyEqual(GetNode()->GetWorldScale(), kZero3)) {
    return {};
  }

  const mat4f local_from_world = inverse(GetNode()->GetWorldTrs());
  const Ray local_ray = world_ray.GetTransformed(local_from_world);

  const Box local_bounds =
      GetLocalBounds(GetView().GetRenderableManager(), GetNode().GetEntity());

  const std::optional<collision::RayIntersection<float>> aabb_intersection =
      collision::AABBIntersectsRay(local_bounds, local_ray);

  if (!aabb_intersection.has_value()) {
    // No collision (empty).
    return {};
  }

  auto renderable_info =
      GetNode()->GetComponent<split_engine::SplitEngineRenderableInfo>();
  if (!renderable_info) {
    // This should not be possible, all split engine renderables should have
    // this component.
    IMP_LOG(imp::ERROR) << "All split engine renderables should have a "
                  "SplitEngineRenderableInfo component.";
    return {};
  }

  absl::Span<const MeshVertexAndIndexData> primitive_mesh_data =
      renderable_info->GetAllMeshData();

  // Mesh data is not available, fallback to aabb.
  if (primitive_mesh_data.empty()) {
    // Transform the collision point back into world space.
    const float3 world_point =
        GetNode()->WorldFromLocalPoint(aabb_intersection->collision_point);
    const float3 world_normal =
        GetNode()->WorldFromLocalVector(aabb_intersection->normal);
    return RayHit(norm(world_ray.origin - world_point), quatf{0.0f},
                  world_point, GetNode(), world_normal);
  }

  const std::optional<collision::MultiPrimitiveMeshIntersection<float>> result =
      collision::MultiPrimitiveMeshIntersectsRay(primitive_mesh_data, local_ray,
                                                 true);
  if (!result.has_value()) {
    // No collision (empty).
    return {};
  }

  // Construct RayHit from mesh collision.
  const collision::CollidedTriangle col_triangle{
      .primitive_id = result->primitive_id, .triangle_id = result->triangle_id};
  const float3 world_point =
      GetNode()->WorldFromLocalPoint(result->collision_point);
  const float3 collision_normal =
      GetNode()->WorldFromLocalVector(result->collision_normal);
  return RayHit(norm(world_ray.origin - world_point), quatf{0.0f},
                // Transform the collision point back into world space.
                world_point, GetNode(), collision_normal, col_triangle);
}

absl::optional<DoubleRayHit> SplitEngineMeshCollider::IntersectPrecise(
    const DoubleRay& world_ray) {
  if (!IsActive()) {
    return {};
  }

  if (RoughlyEqual(GetNode()->GetWorldScale(), kZero3)) {
    return {};
  }

  const mat4 local_from_world = inverse(GetNode()->GetWorldTrsPrecise());
  const DoubleRay local_ray = world_ray.GetTransformed(local_from_world);

  const Box local_bounds =
      GetLocalBounds(GetView().GetRenderableManager(), GetNode().GetEntity());

  const std::optional<collision::RayIntersection<double>> aabb_intersection =
      collision::AABBIntersectsRay(local_bounds, local_ray);

  if (!aabb_intersection.has_value()) {
    // No collision (empty).
    return {};
  }

  auto renderable_info =
      GetNode()->GetComponent<split_engine::SplitEngineRenderableInfo>();
  if (!renderable_info) {
    // This should not be possible, all split engine renderables should have
    // this component.
    IMP_LOG(imp::ERROR) << "All split engine renderables should have a "
                  "SplitEngineRenderableInfo component.";
    return {};
  }

  absl::Span<const MeshVertexAndIndexData> primitive_mesh_data =
      renderable_info->GetAllMeshData();

  // Mesh data is not available, fallback to aabb.
  if (!primitive_mesh_data.empty()) {
    // Transform the collision point back into world space.
    const double3 world_point = GetNode()->WorldFromLocalPointPrecise(
        aabb_intersection->collision_point);
    const double3 world_normal =
        GetNode()->WorldFromLocalVectorPrecise(aabb_intersection->normal);
    return DoubleRayHit(norm(world_ray.origin - world_point), quat{0.0f},
                        world_point, GetNode(), world_normal);
  }

  const std::optional<collision::MultiPrimitiveMeshIntersection<double>>
      result = collision::MultiPrimitiveMeshIntersectsRay(primitive_mesh_data,
                                                          local_ray, true);
  if (!result.has_value()) {
    // No collision (empty).
    return {};
  }

  // Construct RayHit from mesh collision.
  const collision::CollidedTriangle col_triangle{
      .primitive_id = result->primitive_id, .triangle_id = result->triangle_id};
  const double3 world_point =
      GetNode()->WorldFromLocalPoint(result->collision_point);
  const double3 collision_normal =
      GetNode()->WorldFromLocalVector(result->collision_normal);
  return DoubleRayHit(norm(world_ray.origin - world_point), quat{0.0f},
                      // Transform the collision point back into world space.
                      world_point, GetNode(), collision_normal, col_triangle);
}

void SplitEngineMeshCollider::Visualize(
    VisualizationStyle visualization_style) const {
  debug_draw::Color color =
      debug_draw::DefaultColorFromVisualizationStyle(visualization_style);

  auto renderable_info =
      GetNode()->GetComponent<split_engine::SplitEngineRenderableInfo>();
  if (!renderable_info) {
    return;
  }

  absl::Span<const MeshVertexAndIndexData> primitive_mesh_data =
      renderable_info->GetAllMeshData();

  if (primitive_mesh_data.empty()) {
    const Box local_bounds =
        GetLocalBounds(GetView().GetRenderableManager(), GetNode().GetEntity());
    debug_draw::Local(GetNode().GetEntity()).BoxLines(local_bounds, color);
  } else {
    debug_draw::Local(GetNode().GetEntity())
        .MeshLines(primitive_mesh_data, color);
  }
}

void SplitEngineMeshCollider::System::BeforeFirstComponentAdded() {
  GetView()
      .GetCollisionManager()
      .AddCollisionSystem<SplitEngineMeshCollider, CollisionSystem>(&GetView());
}

void SplitEngineMeshCollider::System::AfterLastComponentRemoved() {
  GetView()
      .GetCollisionManager()
      .RemoveCollisionSystem<SplitEngineMeshCollider, CollisionSystem>();
}

}  // namespace imp
