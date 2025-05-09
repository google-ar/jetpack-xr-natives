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

#include "core/editor/widgets/toggle_camera.h"

#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/events.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/view/base_view.h"

namespace imp::editor {

namespace {
constexpr absl::string_view kSwitchCameraButtonLabel = "Switch camera";
}

ToggleCamera::ToggleCamera(BaseView& view) : view_(view) {}

void ToggleCamera::DrawImGui() {
  if (ImGui::Button(kSwitchCameraButtonLabel.data())) {
    Editor& editor = view_.GetRegistry().Get<Editor>()->get();
    editor.GetDispatcher().Send(ToggleCameraEvent());
  }
}

}  // namespace imp::editor
