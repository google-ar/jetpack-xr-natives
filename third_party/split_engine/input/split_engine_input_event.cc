// Copyright 2025 Google LLC
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

#include "split_engine/input/split_engine_input_event.h"

#include <memory>
#include <string>

#include "absl/strings/str_cat.h"
#include "core/math/vec.h"
#include "split_engine/input/split_engine_input_event_hit_info.h"

namespace android_xr {

SplitEngineInputEvent& SplitEngineInputEvent::operator=(
    const SplitEngineInputEvent& other) {
  dispatch_flag = other.dispatch_flag;
  device_type = other.device_type;
  pointer_type = other.pointer_type;
  timestamp_ms = other.timestamp_ms;
  origin = other.origin;
  direction = other.direction;
  button_state = other.button_state;

  // Create deep copies of the complex members.
  if (other.hit_node) {
    hit_node = std::make_unique<SplitEngineInputEventHitInfo>();
    *hit_node = *other.hit_node;
  } else {
    hit_node = nullptr;
  }
  if (other.secondary_hit_node) {
    secondary_hit_node = std::make_unique<SplitEngineInputEventHitInfo>();
    *secondary_hit_node = *other.secondary_hit_node;
  } else {
    secondary_hit_node = nullptr;
  }

  return *this;
}

std::string SplitEngineInputEvent::ToString() const {
  return absl::StrCat(
      "SplitEngineInputEvent{dispatch_flag=", dispatch_flag,
      ", device_type=", device_type, ", pointer_type=", pointer_type,
      ", timestamp_ms=", timestamp_ms, ", origin=", imp::ToString(origin),
      ", direction=", imp::ToString(direction), ", button_state=", button_state,
      ", hit_node=", hit_node ? hit_node->ToString() : "null",
      ", secondary_hit_node=",
      secondary_hit_node ? secondary_hit_node->ToString() : "null", "}");
}

}  // namespace android_xr
