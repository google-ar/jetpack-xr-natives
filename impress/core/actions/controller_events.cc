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

#include "core/actions/controller_events.h"

#include <optional>
#include <vector>

#include "core/actions/input_action_event.h"
#include "core/math/transform.h"

namespace imp {

std::optional<RayHit> ControllerHitEvent::GetHit() const {
  if (hits_.empty()) {
    return std::nullopt;
  }
  return hits_.front();
}

NodeHandle ControllerHitEvent::GetHitNode() const {
  return hits_.empty() ? NodeHandle() : hits_.front().node;
}

const InputActionState<Transform<float>>&
ControllerHitEvent::GetControllerInputActionState() const {
  return GetInputActionState<Transform<float>>(raycast_action_name_);
}

const Transform<float>& ControllerHitEvent::GetControllerTransform() const {
  return GetControllerInputActionState().current_state;
}

Ray ControllerHitEvent::GetControllerRay() const {
  Transform<float> transform = GetControllerTransform();
  return Ray{transform.translation, transform.rotation * kForward};
}

std::vector<NodeHandle> ControllerHitEvent::GetAllIntersectingNodes() const {
  std::vector<NodeHandle> hit_nodes(hits_.size());
  absl::c_transform(hits_, hit_nodes.data(),
                    [](const RayHit& hit) { return hit.node; });
  return hit_nodes;
}

const ControllerHitEvent::Hand& ControllerHitEvent::GetHand() const {
  return hand_;
}
}  // namespace imp
