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

#include "core/view/framework/collision/mesh_collider.h"

#include <cstddef>
#include <optional>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/types/optional.h"
#include "filament/libs/math/include/math/TMatHelpers.h"
#include "filament/libs/math/include/math/TVecHelpers.h"
#include "core/collision/bvh.h"
#include "core/collision/collision_flags.h"
#include "core/collision/collision_helpers.h"
#include "core/collision/ray.h"
#include "core/common/debug_draw.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh.h"
#include "core/model/mesh/mesh_data.h"
#include "core/ncsb/component_handle.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/framework/collision/collider_state.proto.imp.h"
#include "core/view/framework/collision/ray_hit.h"
#include "core/view/framework/render/mesh_renderer.h"

namespace imp {

absl::Status MeshCollider::Setup() {
  mesh_renderer_ = GetNode()->GetComponent<MeshRenderer>();
  if (!mesh_renderer_)
    return absl::FailedPreconditionError(
        "The node has no mesh renderer component.");
  if (split_engine::SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    serializer->SetMeshCollider(GetEntity(), IsActive());
  }
  return absl::OkStatus();
}

absl::Status MeshCollider::Setup(MeshColliderState::ColliderMode mode) {
  state_.collider_mode = mode;
  return Setup();
}

absl::Status MeshCollider::SetupWithState() { return Setup(); }

void MeshCollider::Cleanup() {
  if (split_engine::SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    serializer->ClearCollider(
        GetEntity(),
        split_engine::SplitEngineSerializer::ColliderType::kMeshCollider);
  }
}

absl::optional<RayHit> MeshCollider::Intersect(const Ray& world_ray) {
  if (!IsActive()) {
    return {};
  }

  mat4f local_from_world = inverse(GetNode()->GetWorldTrs());
  Ray local_ray = world_ray.GetTransformed(local_from_world);

  for (size_t i = 0; i < mesh_renderer_->GetPrimitiveCount(); ++i) {
    Mesh* mesh = mesh_renderer_->GetMesh(i);
    if (!mesh) continue;
    MeshData* mesh_data = mesh->GetMeshData();
    if (mesh_data || state_.collider_mode ==
                         MeshColliderState::ColliderMode::
                             COLLIDE_WITH_MESH_OR_AABB_SUBSTITUTE) {
      std::optional<collision::RayIntersection<float>> aabb_intersection =
          collision::AABBIntersectsRay(mesh->GetAabb(), local_ray);

      if (aabb_intersection.has_value()) {
        if (state_.collider_mode == MeshColliderState::ColliderMode::
                                        COLLIDE_WITH_MESH_OR_AABB_SUBSTITUTE) {
          float3 world_point = GetNode()->WorldFromLocalPoint(
              aabb_intersection->collision_point);
          return RayHit(norm(world_ray.origin - world_point),
                        // TODO Output the correct surface normal.
                        quatf{0.0f},
                        // Transform the collision point back into world space.
                        world_point, GetNode(), std::nullopt);
        }

        if (Bvh* bvh = mesh->GetCollisionAccelerationStructure()) {
          std::optional<Bvh::RayIntersection> result =
              bvh->IntersectRay(local_ray);
          if (result.has_value()) {
            float3 world_point = GetNode()->WorldFromLocalPoint(
                local_ray.origin + result->distance * local_ray.direction);
            float3 world_normal =
                GetNode()->WorldFromLocalVector(result->normal);
            return RayHit(
                norm(world_ray.origin - world_point),
                // TODO Output the correct surface normal.
                quatf{0.0f},
                // Transform the collision point back into world space.
                world_point, GetNode(), world_normal,
                collision::CollidedTriangle{
                    .primitive_id = i,
                    .triangle_id = static_cast<size_t>(result->triangle_id)});
          }
          return std::nullopt;
        }

        std::optional<collision::MeshIntersection<float>> mesh_intersection =
            collision::MeshIntersectsRay(mesh_data, local_ray, true,
                                         mesh->GetMeshRange());
        if (mesh_intersection.has_value()) {
          float3 world_point = GetNode()->WorldFromLocalPoint(
              mesh_intersection->collision_point);
          float3 world_normal = GetNode()->WorldFromLocalVector(
              mesh_intersection->collision_normal);
          return RayHit(norm(world_ray.origin - world_point),
                        // TODO Output the correct surface normal.
                        quatf{0.0f},
                        // Transform the collision point back into world space.
                        world_point, GetNode(), world_normal,
                        collision::CollidedTriangle{
                            .primitive_id = i,
                            .triangle_id = mesh_intersection->triangle_id});
        }
      }
    }
  }

  // No collision (empty).
  return {};
}

absl::optional<DoubleRayHit> MeshCollider::IntersectPrecise(
    const DoubleRay& world_ray) {
  if (!IsActive()) {
    return {};
  }

  mat4 local_from_world = inverse(GetNode()->GetWorldTrsPrecise());
  DoubleRay local_ray = world_ray.GetTransformed(local_from_world);

  for (size_t i = 0; i < mesh_renderer_->GetPrimitiveCount(); ++i) {
    Mesh* mesh = mesh_renderer_->GetMesh(i);
    if (!mesh) continue;
    MeshData* mesh_data = mesh->GetMeshData();
    if (mesh_data || state_.collider_mode ==
                         MeshColliderState::ColliderMode::
                             COLLIDE_WITH_MESH_OR_AABB_SUBSTITUTE) {
      std::optional<collision::RayIntersection<double>> aabb_intersection =
          collision::AABBIntersectsRay(mesh->GetAabb(), local_ray);

      if (aabb_intersection.has_value()) {
        if (state_.collider_mode == MeshColliderState::ColliderMode::
                                        COLLIDE_WITH_MESH_OR_AABB_SUBSTITUTE) {
          double3 world_point = GetNode()->WorldFromLocalPointPrecise(
              aabb_intersection->collision_point);
          return DoubleRayHit(
              norm(world_ray.origin - world_point),
              // TODO Output the correct surface normal.
              quat{0.0f},
              // Transform the collision point back into world space.
              world_point, GetNode(), std::nullopt);
        }

        if (Bvh* bvh = mesh->GetCollisionAccelerationStructure()) {
          std::optional<Bvh::RayIntersection> result =
              bvh->IntersectRay(Ray(local_ray));
          if (result.has_value()) {
            double3 world_point = GetNode()->WorldFromLocalPointPrecise(
                local_ray.origin + result->distance * local_ray.direction);
            float3 world_normal =
                GetNode()->WorldFromLocalVectorPrecise(result->normal);
            return DoubleRayHit(
                norm(world_ray.origin - world_point),
                // TODO Output the correct surface normal.
                quat{0.0f},
                // Transform the collision point back into world space.
                world_point, GetNode(), world_normal,
                collision::CollidedTriangle{
                    .primitive_id = i,
                    .triangle_id = static_cast<size_t>(result->triangle_id)});
          }
          return std::nullopt;
        }

        std::optional<collision::MeshIntersection<double>> mesh_intersection =
            collision::MeshIntersectsRay(mesh_data, local_ray, true,
                                         mesh->GetMeshRange());
        if (mesh_intersection.has_value()) {
          double3 world_point = GetNode()->WorldFromLocalPointPrecise(
              mesh_intersection->collision_point);
          double3 world_normal = GetNode()->WorldFromLocalVectorPrecise(
              mesh_intersection->collision_normal);
          return DoubleRayHit(
              norm(world_ray.origin - world_point),
              // TODO Output the correct surface normal.
              quat{0.0f},
              // Transform the collision point back into world space.
              world_point, GetNode(), world_normal,
              collision::CollidedTriangle{
                  .primitive_id = i,
                  .triangle_id = mesh_intersection->triangle_id});
        }
      }
    }
  }

  // No collision (empty).
  return {};
}

void MeshCollider::Visualize(VisualizationStyle visualization_style) const {
  debug_draw::Color color =
      debug_draw::DefaultColorFromVisualizationStyle(visualization_style);
  for (size_t i = 0; i < mesh_renderer_->GetPrimitiveCount(); ++i) {
    auto mesh = mesh_renderer_->GetMesh(i);
    if (!mesh) {
      IMP_LOG(imp::WARNING) << "Caution: MeshRenderer has invalid mesh.";
    } else {
      debug_draw::Local(GetNode().GetEntity()).BoxLines(mesh->GetAabb(), color);
    }
  }
}

void MeshCollider::OnActiveStatusChanged(bool is_active) {
  if (split_engine::SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    serializer->SetMeshCollider(GetEntity(), is_active);
  }
}

}  // namespace imp
