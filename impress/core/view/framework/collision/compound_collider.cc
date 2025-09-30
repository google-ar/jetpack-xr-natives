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

#include "core/view/framework/collision/compound_collider.h"

#include <optional>
#include <utility>

#include "absl/types/optional.h"
#include "core/collision/ray.h"
#include "core/geometry/shapes/compound_shape.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/path_manager.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/collision/capsule_collider.h"
#include "core/view/framework/collision/collider_state.proto.imp.h"
#include "core/view/framework/collision/collision_manager.h"
#include "core/view/framework/collision/cone_collider.h"
#include "core/view/framework/collision/cylinder_collider.h"
#include "core/view/framework/collision/ray_hit.h"
#include "core/view/framework/collision/sphere_collider.h"
// #include "split_engine/schemas/split_engine_data_generated.h"

namespace imp {
using split_engine::SplitEngineSerializer;

void CompoundCollider::Setup() { SetCompoundShape({}); }

void CompoundCollider::Setup(const CompoundShape& compound_shape) {
  SetCompoundShape(compound_shape);
}

void CompoundCollider::SetupWithState() {
  if (SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    // TODO: Re-enable this once the bug is fixed.
    // serializer->SetCompoundCollider(GetEntity(), GetCompoundShape(),
    // IsActive());
  }
}

void CompoundCollider::SetCompoundShape(const CompoundShape& compound_shape) {
  state_.center = compound_shape.center;

  SetupWithState();
}

CompoundShape CompoundCollider::GetCompoundShape() const {
  return CompoundShape{.center = state_.center};
}

absl::optional<RayHit> CompoundCollider::Intersect(const Ray& ray) {
  std::optional<RayHit> result;
  auto check_hit = [&](auto collider) {
    auto hit = collider->Intersect(ray);
    if (hit.has_value() &&
        (!result.has_value() || hit->distance < result->distance)) {
      result = std::move(hit);
    }
  };
  for (auto& child : GetView().GetPathManager().GetDescendants(GetNode())) {
    if (auto collider = child->GetComponent<SphereCollider>()) {
      check_hit(collider);
    } else if (auto collider = child->GetComponent<BoxCollider>()) {
      check_hit(collider);
    } else if (auto collider = child->GetComponent<CylinderCollider>()) {
      check_hit(collider);
    } else if (auto collider = child->GetComponent<ConeCollider>()) {
      check_hit(collider);
    } else if (auto collider = child->GetComponent<CapsuleCollider>()) {
      check_hit(collider);
    }
  }
  if (result.has_value()) {
    result->node = GetNode();
  }
  return result;
}

absl::optional<DoubleRayHit> CompoundCollider::IntersectPrecise(
    const DoubleRay& ray) {
  std::optional<DoubleRayHit> result;
  auto check_hit = [&](auto collider) {
    auto hit = collider->IntersectPrecise(ray);
    if (hit.has_value() &&
        (!result.has_value() || hit->distance < result->distance)) {
      result = std::move(hit);
    }
  };
  for (auto& child : GetView().GetPathManager().GetDescendants(GetNode())) {
    if (auto collider = child->GetComponent<SphereCollider>()) {
      check_hit(collider);
    } else if (auto collider = child->GetComponent<BoxCollider>()) {
      check_hit(collider);
    } else if (auto collider = child->GetComponent<CylinderCollider>()) {
      check_hit(collider);
    } else if (auto collider = child->GetComponent<ConeCollider>()) {
      check_hit(collider);
    } else if (auto collider = child->GetComponent<CapsuleCollider>()) {
      check_hit(collider);
    }
  }
  if (result.has_value()) {
    result->node = GetNode();
  }
  return result;
}

void CompoundCollider::Cleanup() {
  if (SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    // TODO: Re-enable this once the bug is fixed.
    // serializer->ClearCollider(
    //     GetEntity(),
    //     split_engine::SplitEngineSerializer::ColliderType::kCompoundCollider);
  }
}

void CompoundCollider::OnActiveStatusChanged(bool is_active) {
  if (SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    // TODO: Re-enable this once the bug is fixed.
    // serializer->SetCompoundCollider(GetEntity(), GetCompoundShape(),
    // is_active);
  }
}

}  // namespace imp
