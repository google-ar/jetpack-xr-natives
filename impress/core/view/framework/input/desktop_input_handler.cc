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

#include "core/view/framework/input/desktop_input_handler.h"

#include "core/ncsb/dispatcher/dispatcher.h"

namespace imp {

DesktopInputHandler::DesktopInputHandler(BaseView* view)
    : view_(view), modifier_(KeyModifier::NONE) {}

void DesktopInputHandler::Update(InputManager* input_manager) {
  while (input_manager->HasKeyboardEvent() ||
         input_manager->HasPointerEvent() || input_manager->HasWheelEvent()) {
    DesktopCombinedInputEvent event;
    if (input_manager->HasKeyboardEvent()) {
      event.keyboard_events = input_manager->PopKeyboardEvents();
    }
    if (input_manager->HasPointerEvent()) {
      event.pointer_events = input_manager->PopPointerEvents();
    }
    if (input_manager->HasWheelEvent()) {
      event.wheel_event = input_manager->PopWheelEvent();
    }
    view_->GetDispatcher().Send(DesktopCombinedInputEvent(event));
  }
}

}  // namespace imp
