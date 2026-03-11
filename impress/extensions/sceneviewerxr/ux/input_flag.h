/*
 * Copyright 2025 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INPUT_FLAG_H_
#define THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INPUT_FLAG_H_

#include <cstdint>

#include "split_engine/input/split_engine_input_event.h"

namespace svxr {

// Input flags that are computed from incoming input events.
enum class InputFlag : uint32_t {
  kIsRight = (1 << 0),
  kIsDown = (1 << 1),
  kIsGaze = (1 << 2),
  kIsDownStarting = (1 << 3),
  kIsDownStopping = (1 << 4),
  kIsHover = (1 << 5),
  kIsHoverStarting = (1 << 6),
  kIsHoverStopping = (1 << 7),
};

imp::Flags<InputFlag> GenerateInputFlags(
    const ::android_xr::SplitEngineInputEvent& current_input,
    const ::android_xr::SplitEngineInputEvent& prev_input,
    bool is_current_ray_hovering, bool& is_previous_ray_hovering);

}  // namespace svxr

#endif  // THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INPUT_FLAG_H_
