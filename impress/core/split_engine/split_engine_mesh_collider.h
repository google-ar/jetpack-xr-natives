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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_MESH_COLLIDER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_MESH_COLLIDER_H_

#include "absl/types/optional.h"
#include "core/collision/collision_flags.h"
#include "core/collision/ray.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_system.h"
#include "core/view/framework/collision/ray_hit.h"

namespace imp {

// A collider that tests for collisions using mesh data from∂∂
// SplitEngineRenderer.
class SplitEngineMeshCollider : public Component {
 public:
  absl::optional<RayHit> Intersect(const Ray& world_ray);
  absl::optional<DoubleRayHit> IntersectPrecise(const DoubleRay& world_ray);
  void Visualize(VisualizationStyle visualization_style) const;

  // ComponentSystem for registering the SplitEngineMeshCollider to the relevant
  // CollisionSystem
  class System : public ComponentSystem<SplitEngineMeshCollider> {
   public:
    explicit System(BaseView* view)
        : ComponentSystem<SplitEngineMeshCollider>(view) {}

    void BeforeFirstComponentAdded() override;
    void AfterLastComponentRemoved() override;
  };
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_MESH_COLLIDER_H_
