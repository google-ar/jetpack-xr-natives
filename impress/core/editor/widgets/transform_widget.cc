// Copyright 2026 Google LLC
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

#include "core/editor/widgets/transform_widget.h"

#include <cmath>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "dear_imgui/imgui.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/events.h"
#include "core/input/key_codes.h"
#include "core/input/keyboard_event.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/update_system.h"
#include "core/view/framework/gestures/tap_gesture.h"
#include "core/view/utils/frame_time.h"

namespace imp::editor {

namespace {

// Minimum distance from the camera to the transform widget.
const float kMinCameraDistance = 1e-5f;

// The base scale for the transform widget.
// This should match the scale in transform_widget.textproto.
const float kBaseScale = 0.02f;

}  // namespace

void TransformWidget::Setup() {
  Editor& editor = GetView().GetRegistry().Get<Editor>()->get();
  Dispatcher& editor_dispatcher = editor.GetDispatcher();

  // Catch the TapEvent to prevent the editor from doing anything else.
  editor_dispatcher.Connect(
      GetNode(),
      [this](const imp::TapGesture::TapEvent& event) mutable {
        CycleMode();
        return imp::Dispatcher::kAccept;
      },
      this);

  editor_dispatcher.Connect(
      [this](const editor::NodeSelectionChangedEvent& event) mutable {
        const absl::flat_hash_set<NodeHandle>& selected_nodes =
            GetView().GetRegistry().Get<Editor>()->get().GetSelectedNodes();
        active_nodes_.assign(selected_nodes.begin(), selected_nodes.end());
        GetNode()->SetEnabled(!active_nodes_.empty());
      },
      this);

  editor_dispatcher.Connect(
      [this](const imp::KeyboardEvent& event) {
        if (ImGui::GetIO().WantTextInput) return;

        if (event.type == KeyboardEventType::kOnUp) {
          switch (event.key.code) {
            case VirtualKeyCode::VK_LEFTBRACKET:
              scale_level_--;
              scale_ = pow(1.1f, scale_level_);
              GetView().GetDispatcher().Send(
                  TransformWidgetScaleChangedEvent(scale_));
              break;
            case VirtualKeyCode::VK_RIGHTBRACKET:
              scale_level_++;
              scale_ = pow(1.1f, scale_level_);
              GetView().GetDispatcher().Send(
                  TransformWidgetScaleChangedEvent(scale_));
              break;
            default:
              break;
          }
        } else if (event.type == KeyboardEventType::kOnDown) {
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
        }
      },
      this);
}

void TransformWidget::CycleMode() {
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

void TransformWidget::SetMode(Mode mode) {
  if (!IsActive()) return;

  mode_ = mode;
  if (state_.translate) state_.translate->SetEnabled(mode_ == Mode::kTranslate);
  if (state_.rotate) state_.rotate->SetEnabled(mode_ == Mode::kRotate);
  if (state_.scale) state_.scale->SetEnabled(mode_ == Mode::kScale);
}

void TransformWidget::Update(const FrameTime& frame_time) {
  Editor& editor = GetView().GetRegistry().Get<Editor>()->get();
  if (!editor.GetEditorRoot()->IsActive()) return;
  if (active_nodes_.empty()) {
    // TODO: Remove this once we have a better way to hide the
    // transform widget.
    GetNode()->SetEnabled(false);
    return;
  }

  // Compute the centroid of the active nodes.
  float3 sum_positions = {0.0f, 0.0f, 0.0f};
  int valid_nodes_count = 0;
  for (const NodeHandle& node : active_nodes_) {
    if (!node.IsValid()) continue;
    sum_positions += node->GetWorldPosition();
    valid_nodes_count++;
  }

  // If there are no valid nodes, disable the transform widget and return.
  if (valid_nodes_count == 0) {
    GetNode()->SetEnabled(false);
    return;
  }

  const float3 centroid = sum_positions / static_cast<float>(valid_nodes_count);
  const float3 camera_position =
      editor.GetCamera()->GetNode()->GetWorldPosition();
  const float3 camera_to_target = centroid - camera_position;

  // Avoid divide by zero when normalizing a zero vector.
  float3 dif;
  if (length(camera_to_target) > kMinCameraDistance) {
    dif = normalize(camera_to_target);
  } else {
    // If the camera is at the centroid, place the widget slightly in front.
    dif = float3{0.0f, 0.0f, -1.0f};
  }

  GetNode()->SetWorldPosition(camera_position + dif);
  GetNode()->SetLocalScale(float3(kBaseScale * scale_));
  // TODO: Enable for local mode.
  // GetNode()->SetWorldRotation(active_nodes_[0]->GetWorldRotation());
}

}  // namespace imp::editor
