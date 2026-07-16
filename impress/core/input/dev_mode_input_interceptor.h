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

#ifndef THIRD_PARTY_IMPRESS_CORE_INPUT_DEV_MODE_INPUT_INTERCEPTOR_H_
#define THIRD_PARTY_IMPRESS_CORE_INPUT_DEV_MODE_INPUT_INTERCEPTOR_H_

#include <memory>
#include <vector>

#include "absl/types/optional.h"
#include "core/actions/input_action_event.h"
#include "core/input/input_manager.h"
#include "core/input/keyboard_controller.h"
#include "core/input/keyboard_event.h"
#include "core/input/pointer_event.h"
#include "core/input/wheel_event.h"
#include "core/view/base_view.h"

namespace imp {

class BaseView;

// Intercepts and filters input that is targeting ImGui widgets.
class DevModeInputInterceptor : public InputInterceptor {
 public:
  explicit DevModeInputInterceptor(BaseView* view);
  ~DevModeInputInterceptor() override = default;
  // The interceptor should process and filter out any pointer events that it
  // does not want to propagate to the rest of the view.
  // When UI is overlaid:
  // 1. Sends input event to ImGui.
  // 2. Prevents captured input from getting to the app.
  // When using remote editor:
  // 1. Sends input event to ImGui.
  void FilterPointerEvents(std::vector<PointerEvent>& pointer_events) override;
  // The interceptor should process and filter out any keyboard or text input
  // events that it does not want to propagate to the rest of the view.
  void FilterKeyboardEvents(
      std::vector<KeyboardEvent>& keyboard_events,
      std::vector<TextInputEvent>& text_input_events) override;
  // The interceptor should process and filter out any wheel events that it does
  // not want to propagate to the rest of the view.
  void FilterWheelEvents(std::vector<WheelEvent>& wheel_events) override;
  // The interceptor should process and filter out any input action events that
  // it does not want to propagate to the rest of the view.
  void FilterInputActionEvents(
      std::vector<InputActionEvent>& input_action_events) override;

 private:
  // For platforms with soft keyboards, e.g. Android and iOS.
  std::unique_ptr<KeyboardController> soft_keyboard_controller_;
  absl::optional<Pointer::Id> captured_id_;
  BaseView* view_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_INPUT_DEV_MODE_INPUT_INTERCEPTOR_H_
