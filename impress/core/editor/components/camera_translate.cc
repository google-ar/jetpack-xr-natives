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

#include "core/editor/components/camera_translate.h"

#include <cstdint>
#include <optional>

#include "core/common/log.h"
#include "core/camera/camera_component.h"
#include "core/collision/ray.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/editor_plugin.h"
#include "core/input/input_manager.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/gestures/drag_gesture.h"
#include "core/view/framework/gestures/multi_drag_gesture.h"

namespace imp::editor {

// 1 / kDragSmoothingFactor is th seconds to slerp to the desired rotation.
constexpr int kCameraPanPointerCount = 2;

void CameraTranslate::Setup(NodeHandle pivot) {
  if (!pivot) {
    IMP_LOG(imp::FATAL) << "CameraTranslateComponent needs a pivot to work properly";
  }

  pivot_ = pivot;

  Editor& editor = GetView().GetRegistry().Get<Editor>()->get();
  Dispatcher& editor_dispatcher = editor.GetDispatcher();

  // Camera transformations
  // Detect right click for camera panning on desktop/web
  // TODO: Enable arrow keys for drag gesture movement
  auto drag_gesture_start_event_listener =
      [this](const DragGesture::StartEvent& event) mutable {
        Editor& editor = GetView().GetRegistry().Get<Editor>()->get();
        uint32_t active_pointer = editor.UseLegacyCameraControls()
                                      ? kMousePointerIdRight
                                      : kMousePointerIdMiddle;
        if (event.pointer != active_pointer) {
          return;
        }
        intersection_prev_ =
            GetPointerIntersectionWithHorizontalPivotPlane(event.position);
      };

  auto drag_gesture_update_event_listener =
      [this](const DragGesture::UpdateEvent& event) mutable {
        Editor& editor = GetView().GetRegistry().Get<Editor>()->get();
        uint32_t active_pointer = editor.UseLegacyCameraControls()
                                      ? kMousePointerIdRight
                                      : kMousePointerIdMiddle;
        if (event.pointer != active_pointer ||
            !intersection_prev_.has_value()) {
          return;
        }
        MaybeUpdateCameraPivot(event.position);
      };

  // Detect multi finger drag for camera panning on mobile
  auto multi_drag_gesture_start_event_listener =
      [this](const MultiDragGesture::StartEvent& event) mutable {
        if (event.pointer_count != kCameraPanPointerCount) {
          return;
        }
        intersection_prev_ = GetPointerIntersectionWithHorizontalPivotPlane(
            event.centroid_position);
      };

  auto multi_drag_gesture_update_event_listener =
      [this](const MultiDragGesture::UpdateEvent& event) mutable {
        if (event.pointer_count != kCameraPanPointerCount ||
            !intersection_prev_.has_value()) {
          return;
        }
        MaybeUpdateCameraPivot(event.centroid_position);
      };

  editor_dispatcher.Connect(drag_gesture_start_event_listener, this);
  editor_dispatcher.Connect(drag_gesture_update_event_listener, this);
  editor_dispatcher.Connect(multi_drag_gesture_start_event_listener, this);
  editor_dispatcher.Connect(multi_drag_gesture_update_event_listener, this);

  // When the app camera is ignored, the editor camera is always active and can
  // be controlled by the app dispatcher.
  if (editor.GetCameraConfiguration() ==
      EditorPlugin::CameraConfiguration::kEditorCameraOnly) {
    Dispatcher& app_dispatcher = GetView().GetDispatcher();
    app_dispatcher.Connect(drag_gesture_start_event_listener, this);
    app_dispatcher.Connect(drag_gesture_update_event_listener, this);
    app_dispatcher.Connect(multi_drag_gesture_start_event_listener, this);
    app_dispatcher.Connect(multi_drag_gesture_update_event_listener, this);
  }
}

std::optional<float3>
CameraTranslate::GetPointerIntersectionWithHorizontalPivotPlane(
    float2 pointer) {
  Editor& editor = GetView().GetRegistry().Get<Editor>()->get();
  ComponentHandle<CameraComponent> editor_camera = editor.GetCamera();
  Ray ray = editor_camera->WorldRayFromPixelPoint(pointer);
  float denom = dot(kUp, ray.direction);
  if (abs(denom) < 0.0001f) {
    return std::nullopt;
  }
  float t = dot(pivot_->GetWorldPosition() - ray.origin, kUp) / denom;
  if (t < 0) {
    return std::nullopt;
  }
  return ray.origin + ray.direction * t;
}

void CameraTranslate::MaybeUpdateCameraPivot(float2 position) {
  std::optional<float3> intersection =
      GetPointerIntersectionWithHorizontalPivotPlane(position);
  if (!intersection.has_value()) {
    return;
  }
  pivot_->SetLocalPosition(pivot_->GetLocalPosition() +
                           (intersection_prev_.value() - intersection.value()));
  // Get the new intersection since moving the pivot will change the ray.
  intersection_prev_ = GetPointerIntersectionWithHorizontalPivotPlane(position);
}

}  // namespace imp::editor
