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

#include "split_engine/input/split_engine_input_event_hit_info.h"

#include <string>

#include "absl/strings/str_cat.h"
#include "core/math/mat.h"

namespace android_xr {

std::string SplitEngineInputEventHitInfo::ToString() const {
  return absl::StrCat("InputEventHitInfo{subspace_impress_node_id=", target,
                      ", hit_position_is_valid=", hit_position_is_valid,
                      ", hit_position=", hit_position.x, ",", hit_position.y,
                      ",", hit_position.z,
                      ", transform=", imp::ToString(transform), "}");
}

}  // namespace android_xr
