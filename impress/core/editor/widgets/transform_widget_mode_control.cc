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

#include "core/editor/widgets/transform_widget_mode_control.h"

#include "dear_imgui/imgui.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/input/key_codes.h"
#include "core/input/keyboard_event.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/view/framework/gestures/tap_gesture.h"

namespace imp::editor {

void TransformWidgetModeControl::Setup() {
  Editor& editor = GetView().GetRegistry().Get<Editor>()->get();
  editor.GetDispatcher().Connect(
      GetNode(),
      [this](const imp::TapGesture::TapEvent& event) mutable {
        CycleMode();
        return imp::Dispatcher::kAccept;
      },
      this);

  auto handle_keyboard = [this](const imp::KeyboardEvent& event) {
    // Don't handle key presses if ImGui is using the keyboard.
    if (ImGui::GetIO().WantTextInput) return;

    if (event.type != KeyboardEventType::kOnDown) return;

    switch (event.key.code) {
      case VirtualKeyCode::VK_w:
        SetMode(Mode::kTranslate);
        break;
      case VirtualKeyCode::VK_e:
        SetMode(Mode::kRotate);
        break;
      case VirtualKeyCode::VK_r:
        SetMode(Mode::kScale);
        break;
      default:
        break;
    }
  };

  editor.GetDispatcher().Connect(handle_keyboard, this);
}

void TransformWidgetModeControl::CycleMode() {
  if (!IsActive()) return;

  switch (mode_) {
    case Mode::kTranslate:
      SetMode(Mode::kRotate);
      break;
    case Mode::kRotate:
      SetMode(Mode::kScale);
      break;
    case Mode::kScale:
      SetMode(Mode::kTranslate);
      break;
  }
}

void TransformWidgetModeControl::SetMode(Mode mode) {
  if (!IsActive()) return;

  mode_ = mode;
  state_.translate->SetEnabled(mode_ == Mode::kTranslate);
  state_.rotate->SetEnabled(mode_ == Mode::kRotate);
  state_.scale->SetEnabled(mode_ == Mode::kScale);
}

}  // namespace imp::editor
