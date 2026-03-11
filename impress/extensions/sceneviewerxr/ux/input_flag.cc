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

#include "extensions/sceneviewerxr/ux/input_flag.h"

#include "core/common/enum_flags.h"
#include "split_engine/input/split_engine_input_event.h"

namespace svxr {

using ::android_xr::SplitEngineInputEvent;

imp::Flags<InputFlag> GenerateInputFlags(
    const SplitEngineInputEvent& current_input,
    const SplitEngineInputEvent& prev_input, bool is_current_ray_hovering,
    bool& is_previous_ray_hovering) {
  imp::Flags<InputFlag> input_flags;

  bool is_mouse =
      current_input.device_type == SplitEngineInputEvent::DeviceType::MOUSE;
  bool is_right =
      current_input.pointer_type == SplitEngineInputEvent::PointerType::RIGHT;
  bool is_left =
      current_input.pointer_type == SplitEngineInputEvent::PointerType::LEFT;
  if (!is_mouse && !is_right && !is_left) {
    return input_flags;
  }

  bool is_gaze = current_input.device_type ==
                 SplitEngineInputEvent::DeviceType::GAZE_AND_GESTURE;

  input_flags.Set(InputFlag::kIsRight, is_right || is_mouse);
  input_flags.Set(InputFlag::kIsGaze, is_gaze);
  if (current_input.button_state == 1) {
    input_flags.Set(InputFlag::kIsDown);
  }
  if (current_input.button_state != prev_input.button_state) {
    input_flags.Set(current_input.button_state ? InputFlag::kIsDownStarting
                                               : InputFlag::kIsDownStopping);
  }

  bool current_input_is_hovering = is_current_ray_hovering;
  bool previous_input_is_hovering = is_previous_ray_hovering;
  is_previous_ray_hovering = current_input_is_hovering;

  bool hover_started = current_input_is_hovering && !previous_input_is_hovering;
  bool hover_stopped = !current_input_is_hovering && previous_input_is_hovering;

  if (current_input_is_hovering) {
    input_flags.Set(InputFlag::kIsHover);
  }

  if (hover_started) {
    input_flags.Set(InputFlag::kIsHoverStarting);
  } else if (hover_stopped) {
    input_flags.Set(InputFlag::kIsHoverStopping);
  }

  return input_flags;
}

}  // namespace svxr
