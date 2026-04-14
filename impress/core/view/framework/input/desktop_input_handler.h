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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_INPUT_DESKTOP_INPUT_HANDLER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_INPUT_DESKTOP_INPUT_HANDLER_H_

#include <vector>

#include "core/common/enum_flags.h"
#include "core/input/input_manager.h"
#include "core/input/key_codes.h"
#include "core/input/pointer_event.h"
#include "core/input/wheel_event.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/view/base_view.h"

namespace imp {

// Contains all input events on desktop, allows for association between pointer
// and keyboard events.
struct DesktopCombinedInputEvent : public Event {
  DesktopCombinedInputEvent() = default;
  std::vector<KeyboardEvent> keyboard_events;
  std::vector<PointerEvent> pointer_events;
  std::vector<WheelEvent> wheel_events;
};

// A simplified input handler that skips gesture detection and hit detection.
class DesktopInputHandler : public InputHandlerBase {
 public:
  explicit DesktopInputHandler(BaseView* view);
  void Update(InputManager* input_manager) override;

 private:
  BaseView* view_;
  PointerEvent pointer_event_;
  Flags<KeyModifier> modifier_;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_INPUT_DESKTOP_INPUT_HANDLER_H_
