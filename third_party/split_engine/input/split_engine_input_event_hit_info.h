/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_SPLIT_ENGINE_INPUT_SPLIT_ENGINE_INPUT_EVENT_HIT_INFO_H_
#define THIRD_PARTY_SPLIT_ENGINE_INPUT_SPLIT_ENGINE_INPUT_EVENT_HIT_INFO_H_

#include <optional>
#include <string>

#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"

namespace android_xr {

// Represents where an input event made contact with objects in the 3D space.
// Should be kept in sync with its Java counterpart, the hit info for the
// Android XR input event.
struct SplitEngineInputEventHitInfo {
 public:
  explicit SplitEngineInputEventHitInfo() = default;
  std::string ToString() const;

  // The Impress Node that was hit. In case the node doesn't belong to a
  // subspace, the IsValid() method on this member will return false.
  imp::NodeHandle target;

  // Whether the hit position is valid.
  bool hit_position_is_valid = false;

  // The ray hit position, in the receiver's task coordinate space.
  imp::float3 hit_position;

  // The ray hit position, in the receiver's world coordinate space.
  std::optional<imp::float3> world_hit_position;

  // The matrix transforming task node coordinates into CPM node coordinates.
  imp::mat4f transform;
};

}  // namespace android_xr

#endif  // THIRD_PARTY_SPLIT_ENGINE_INPUT_SPLIT_ENGINE_INPUT_EVENT_HIT_INFO_H_
