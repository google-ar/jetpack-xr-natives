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

#include "core/view/framework/assets/gltf_collider.h"

#include <optional>

#include "absl/types/optional.h"
#include "filament/libs/math/include/math/TVecHelpers.h"
#include "core/collision/collision_flags.h"
#include "core/collision/collision_helpers.h"
#include "core/collision/mesh_collision_accelerator.h"
#include "core/collision/ray.h"
#include "core/common/debug_draw.h"
#include "core/config.h"
#include "core/math/almost_equal.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/collision/ray_hit.h"

namespace imp {
void GltfCollider::Setup(ComponentHandle<GltfMesh> gltf_mesh,
                         CollisionMode mode) {
  gltf_mesh_ = gltf_mesh;
  mode_ = mode;
  if (split_engine::SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    serializer->SetMeshCollider(GetEntity(), IsActive());
  }
#if IMP_RUNTIME(DEV)
  visualize_mesh_data_ = !GetNode()->GetComponent<GltfMesh>()->IsSkinned();
#endif
}

void GltfCollider::Cleanup() {
  if (split_engine::SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    serializer->ClearCollider(
        GetEntity(),
        split_engine::SplitEngineSerializer::ColliderType::kMeshCollider);
  }
}

absl::optional<RayHit> GltfCollider::Intersect(const Ray& world_ray) {
  if (!IsActive()) {
    return {};
  }

  if (RoughlyEqual(GetNode()->GetWorldScale(), kZero3)) {
    return {};
  }

  mat4f local_from_world = inverse(GetNode()->GetWorldTrs());
  Ray local_ray = world_ray.GetTransformed(local_from_world);

  // TODO: Make GltfCollider work with skinned meshes
  // and morph target.
  std::optional<collision::RayIntersection<float>> intersection_aabb =
      collision::AABBIntersectsRay(gltf_mesh_->GetLocalBounds(), local_ray);
  if (intersection_aabb.has_value()) {
    if (!UsingPerTriangleCollision()) {
      float3 world_point =
          GetNode()->WorldFromLocalPoint(intersection_aabb->collision_point);
      return RayHit(norm(world_ray.origin - world_point),
                    // TODO Output the correct surface normal.
                    quatf{0.0f},
                    // Transform the collision point back into world space.
                    world_point, GetNode());
    } else {
      std::optional<collision::MultiPrimitiveMeshIntersection<float>> result;
      if (const MeshCollisionAccelerator* accelerator =
              gltf_mesh_->GetMeshCollisionAccelerator()) {
        result = accelerator->Intersect(local_ray);
      } else {
        gltf_mesh_->UpdateSkinnedMesh();
        result = collision::MultiPrimitiveMeshIntersectsRay(
            gltf_mesh_->GetMeshData(), local_ray, true);
      }

      if (result.has_value()) {
        float3 world_point =
            GetNode()->WorldFromLocalPoint(result->collision_point);
        float3 collision_normal =
            GetNode()->WorldFromLocalVector(result->collision_normal);
        return RayHit(
            norm(world_ray.origin - world_point), quatf{0.0f},
            // Transform the collision point back into world space.
            world_point, GetNode(), collision_normal,
            collision::CollidedTriangle{.primitive_id = result->primitive_id,
                                        .triangle_id = result->triangle_id});
      }
    }
  }

  // No collision (empty).
  return {};
}

absl::optional<DoubleRayHit> GltfCollider::IntersectPrecise(
    const DoubleRay& world_ray) {
  if (!IsActive()) {
    return {};
  }

  if (AlmostEqual(GetNode()->GetWorldScale(), kZero3)) {
    return {};
  }
  mat4 local_from_world = inverse(GetNode()->GetWorldTrsPrecise());
  DoubleRay local_ray = world_ray.GetTransformed(local_from_world);

  // TODO: Make GltfCollider work with skinned meshes
  // and morph target.
  std::optional<collision::RayIntersection<double>> intersection_aabb =
      collision::AABBIntersectsRay(gltf_mesh_->GetLocalBounds(), local_ray);
  if (intersection_aabb.has_value()) {
    if (!UsingPerTriangleCollision()) {
      double3 world_point = GetNode()->WorldFromLocalPointPrecise(
          intersection_aabb->collision_point);
      return DoubleRayHit(
          norm(world_ray.origin - world_point),
          // TODO Output the correct surface normal.
          quat{0.0},
          // Transform the collision point back into world space.
          world_point, GetNode());
    } else {
      std::optional<collision::MultiPrimitiveMeshIntersection<double>>
          precise_result;
      if (const MeshCollisionAccelerator* accelerator =
              gltf_mesh_->GetMeshCollisionAccelerator()) {
        std::optional<collision::MultiPrimitiveMeshIntersection<float>> result =
            accelerator->Intersect(Ray(local_ray));
        if (result.has_value()) {
          precise_result = collision::MultiPrimitiveMeshIntersection<double>();
          precise_result->primitive_id = result->primitive_id;
          precise_result->triangle_id = result->triangle_id;
          precise_result->collision_point = result->collision_point;
          precise_result->collision_normal = result->collision_normal;
          precise_result->intersection_dist = result->intersection_dist;
        }
      } else {
        gltf_mesh_->UpdateSkinnedMesh();
        precise_result = collision::MultiPrimitiveMeshIntersectsRay(
            gltf_mesh_->GetMeshData(), local_ray, true);
      }
      if (precise_result.has_value()) {
        collision::CollidedTriangle col_triangle;
        col_triangle.triangle_id = precise_result->triangle_id;
        col_triangle.primitive_id = precise_result->primitive_id;
        // Transform the collision info back into world space.
        double3 world_point = GetNode()->WorldFromLocalPointPrecise(
            precise_result->collision_point);
        double3 collision_normal = GetNode()->WorldFromLocalVectorPrecise(
            precise_result->collision_normal);
        return DoubleRayHit(
            norm(world_ray.origin - world_point), quat{0.0f},
            world_point, GetNode(), collision_normal, col_triangle);
      }
    }
  }

  // No collision (empty).
  return {};
}

bool GltfCollider::UsingPerTriangleCollision() const {
  return mode_ == CollisionMode::kTriangles &&
         !gltf_mesh_->GetMeshData().empty();
}

void GltfCollider::Visualize(VisualizationStyle visualization_style) const {
  debug_draw::Color color =
      debug_draw::DefaultColorFromVisualizationStyle(visualization_style);

  if (gltf_mesh_->GetMeshData().empty()) {
    debug_draw::Local(GetNode().GetEntity())
        .BoxLines(gltf_mesh_->GetLocalBounds(), color);
  } else {
#if IMP_RUNTIME(DEV)
    // Skip visualizing mesh data if it is skinned, as the data may not be
    // accurate.
    if (!visualize_mesh_data_) return;
#endif
    // Displays mesh lines to indicate that mesh data is available on CPU.
    debug_draw::Local(GetNode().GetEntity())
        .MeshLines(gltf_mesh_->GetMeshData(), color);
  }
}

void GltfCollider::OnActiveStatusChanged(bool is_active) {
  if (split_engine::SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    serializer->SetMeshCollider(GetEntity(), is_active);
  }
}

}  // namespace imp
