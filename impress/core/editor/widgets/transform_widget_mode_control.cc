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

#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/events.h"
#include "core/view/framework/gestures/tap_gesture.h"

namespace imp::editor {

void TransformWidgetModeControl::Setup() {
  GetView().GetRegistry().Get<Editor>()->get().GetDispatcher().Connect(
      GetNode(),
      [this](const imp::TapGesture::TapEvent& event) mutable {
        CycleMode();
        return imp::Dispatcher::kAccept;
      },
      this);
}

void TransformWidgetModeControl::CycleMode() {
  if (IsActive()) {
    switch (mode_) {
      case Mode::kTranslate:
        mode_ = Mode::kRotate;
        state_.translate->SetEnabled(false);
        state_.rotate->SetEnabled(true);
        state_.scale->SetEnabled(false);
        break;
      case Mode::kRotate:
        mode_ = Mode::kScale;
        state_.translate->SetEnabled(false);
        state_.rotate->SetEnabled(false);
        state_.scale->SetEnabled(true);
        break;
      case Mode::kScale:
        mode_ = Mode::kTranslate;
        state_.translate->SetEnabled(true);
        state_.rotate->SetEnabled(false);
        state_.scale->SetEnabled(false);
        break;
    }
  }
}

}  // namespace imp::editor
