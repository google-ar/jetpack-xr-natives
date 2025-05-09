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

#include "core/editor/components/camera_zoom.h"

#include <algorithm>

#include "core/common/log.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/editor_plugin.h"
#include "core/editor/events.h"
#include "core/input/wheel_event.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/view/framework/gestures/pinch_gesture.h"
#include "core/view/framework/input/pointer_input_handler.h"

namespace imp::editor {

constexpr float kScrollZoomSensitivity = 0.01f;
constexpr float kScrollZoomMinDistance = kScrollZoomSensitivity * 2.0f;
constexpr float kZoomMaxDistance = 200.0f;
constexpr float kPinchZoomSensitivity = 0.05f;
constexpr float kPinchZoomMinDistance = kPinchZoomSensitivity * 2.0f;

void CameraZoom::Setup(NodeHandle pivot) {
  if (!pivot) {
    IMP_LOG(imp::FATAL) << "CameraZoomComponent needs a pivot to work properly";
  }

  pivot_ = pivot;

  Editor& editor = GetView().GetRegistry().Get<Editor>()->get();
  Dispatcher& editor_dispatcher = editor.GetDispatcher();

  auto wheel_scroll_event_listener =
      [this](const imp::WheelScrollEvent& event) mutable {
        UpdateCameraZoom(
            event.event.GetDelta() * (invert_scroll_enabled_ ? 1.0f : -1.0f),
            kScrollZoomSensitivity, kScrollZoomMinDistance);
      };

  auto pinch_gesture_event_listener =
      [this](const imp::PinchGesture::UpdateEvent& event) mutable {
        UpdateCameraZoom(event.gap_delta, kPinchZoomSensitivity,
                         kPinchZoomMinDistance);
      };

  auto invert_scroll_event_listener =
      [this](const InvertMouseScrollEvent& event) mutable {
        invert_scroll_enabled_ = event.enabled;
      };

  editor_dispatcher.Connect(wheel_scroll_event_listener, this);
  editor_dispatcher.Connect(pinch_gesture_event_listener, this);
  editor_dispatcher.Connect(invert_scroll_event_listener, this);

  // When the app camera is ignored, the editor camera is always active and can
  // be controlled by the app dispatcher.
  if (editor.GetCameraConfiguration() ==
      EditorPlugin::CameraConfiguration::kEditorCameraOnly) {
    Dispatcher& app_dispatcher = GetView().GetDispatcher();
    app_dispatcher.Connect(wheel_scroll_event_listener, this);
    app_dispatcher.Connect(pinch_gesture_event_listener, this);
    app_dispatcher.Connect(invert_scroll_event_listener, this);
  }
}

void CameraZoom::UpdateCameraZoom(float zoom_delta, float zoom_sensitivity,
                                  float zoom_min_distance) {
  float adjusted_zoom_delta = zoom_delta * zoom_sensitivity;
  float3 pivot_position = pivot_->GetWorldPosition();
  float3 node_to_pivot = pivot_position - GetNode()->GetWorldPosition();
  float distance = length(node_to_pivot) - adjusted_zoom_delta;
  distance = std::clamp(distance, zoom_min_distance, kZoomMaxDistance);
  node_to_pivot = normalize(node_to_pivot);
  float3 target_position = pivot_position - (node_to_pivot * distance);
  GetNode()->SetWorldPosition(target_position);
}

}  // namespace imp::editor
