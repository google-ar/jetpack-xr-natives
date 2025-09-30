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

#include "core/physics/collidable_shapes/collidable_shape.h"

#include <cmath>
#include <limits>

#include "bullet/src/LinearMath/btTransform.h"
#include "core/math/almost_equal.h"
#include "core/math/vec.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/physics/physics_helper.h"

namespace imp {

CollidableShape::CollidableShape(NodeHandle node) : node_(node) {}

NodeHandle CollidableShape::GetNode() const { return node_; }

btTransform CollidableShape::GetNodeBtTransform() {
  float3 center_offset = node_->GetWorldRotation() * GetCollidableCenter();
  return ToBtTransform(
      node_->GetWorldPosition() + center_offset * node_->GetWorldScale(),
      node_->GetWorldRotation());
}

bool CollidableShape::UpdatedEvenScale(float3& prev_scale,
                                       const float3& new_scale) {
  const bool x_changed = !RoughlyEqual(new_scale.x, prev_scale.x);
  const bool y_changed = !RoughlyEqual(new_scale.y, prev_scale.y);
  const bool z_changed = !RoughlyEqual(new_scale.z, prev_scale.z);

  if (!x_changed && !y_changed && !z_changed) {
    return false;
  }
  float new_fscale = std::numeric_limits<float>::min();
  if (x_changed) {
    new_fscale = fmax(new_scale.x, new_fscale);
  }
  if (y_changed) {
    new_fscale = fmax(new_scale.y, new_fscale);
  }
  if (z_changed) {
    new_fscale = fmax(new_scale.z, new_fscale);
  }
  prev_scale = new_fscale;
  return true;
}

}  // namespace imp
