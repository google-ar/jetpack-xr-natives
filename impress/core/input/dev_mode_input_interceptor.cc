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

#include "core/input/dev_mode_input_interceptor.h"

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "absl/strings/match.h"
#include "absl/types/variant.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/imgui_internal.h"
#include "core/actions/action_config.h"
#include "core/actions/controller_events.h"
#include "core/actions/input_action_event.h"
#include "core/common/registry.h"
#include "core/common/trace.h"
#include "core/editor/editor.h"
#include "core/editor/editor_info.h"
#include "core/editor/widgets/viewport/viewport_widget.h"
#include "core/editor/xr/xr_editor_ui.h"
#include "core/geometry/shapes/rect.h"
#include "core/input/input_manager.h"
#include "core/input/key_codes.h"
#include "core/input/keyboard_controller.h"
#include "core/input/keyboard_event.h"
#include "core/input/pointer_event.h"
#include "core/input/wheel_event.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/path_manager.h"
#include "core/view/framework/collision/collision_manager.h"
#include "core/view/framework/collision/ray_hit.h"
#include "core/window/filament_host.h"

namespace imp {

namespace {
std::optional<int> PointerToImGuiMouseIndex(Pointer::Id pointer_id) {
  if (pointer_id == kMousePointerIdLeft) {
    return ImGuiMouseButton_Left;
  } else if (pointer_id == kMousePointerIdRight) {
    return ImGuiMouseButton_Right;
  } else if (pointer_id == kMousePointerIdMiddle) {
    return ImGuiMouseButton_Middle;
  }
  return std::nullopt;
}

bool IsMouseOverViewport(const std::optional<Rect>& viewport_rect,
                         const float2& point) {
  if (!viewport_rect) return false;

  const bool point_over_viewport = viewport_rect->Contains(point);

  const ImGuiContext* g = ImGui::GetCurrentContext();
  return point_over_viewport && g && g->HoveredWindow &&
         absl::StrContains(g->HoveredWindow->Name,
                           editor::ViewportWidget::kViewportWindowName);
}

// Constructs a ControllerHitEvent, if controller input is identified in the
// given InputActionEvent.
std::optional<ControllerHitEvent> BuildControllerHitEvent(
    BaseView& view, const InputActionEvent& input_action_event) {
  // Look for either "user/hand/left or a "user/hand/right" subaction paths.
  // Note that these constants match OpenXR standards.
  ControllerHitEvent::Hand hand = ControllerHitEvent::Hand::kUnknown;
  if (input_action_event.subaction_path == kDefaultEyeSubactionPath) {
    // Ignore eye gaze.
    return std::nullopt;
  } else if (input_action_event.subaction_path ==
             kDefaultLeftHandSubactionPath) {
    hand = ControllerHitEvent::Hand::kLeft;
  } else if (input_action_event.subaction_path ==
             kDefaultRightHandSubactionPath) {
    hand = ControllerHitEvent::Hand::kRight;
  }

  // Look for an "aim" ray.
  auto iter = input_action_event.action_name_to_input_action_state.find(
      kDefaultAimActionName);
  if (iter == input_action_event.action_name_to_input_action_state.end()) {
    return std::nullopt;
  }

  // Perform a ray cast and construct a ControllerHitEvent
  Transform<float> transform =
      absl::get<InputActionState<Transform<float>>>(iter->second).current_state;
  std::vector<RayHit> ray_hits = view.GetCollisionManager().IntersectAll(
      Ray{transform.translation, transform.rotation * kForward});
  return ControllerHitEvent(hand, kDefaultAimActionName, ray_hits,
                            input_action_event);
}

// Selectively captures input that may be interacting with the XR Editor.
void InterceptXrEditorInput(BaseView& view,
                            InputActionEvent& input_action_event) {
  // TODO Implement intercompatibility with PointerHitEvent
  // Check if there's controller input in this InputActionEvent.
  std::optional<ControllerHitEvent> controller_hit_event =
      BuildControllerHitEvent(view, input_action_event);
  if (!controller_hit_event.has_value()) {
    return;
  }

  absl::StatusOr<std::reference_wrapper<editor::Editor>> editor_or =
      view.GetRegistry().Get<editor::Editor>();
  if (!editor_or.ok()) {
    IMP_LOG(imp::FATAL) << "Editor could not be retrieved from the registry.";
  }
  // Send a ControllerHitEvent on in the Editor dispatcher
  Dispatcher& editor_dispatcher = editor_or->get().GetDispatcher();
  editor_dispatcher.Send(*controller_hit_event);

  // If hitting the XR Editor, strip away button inputs.
  NodeHandle hit_node = controller_hit_event->GetHitNode();
  if (hit_node &&
      view.GetPathManager().GetComponentFromAncestorOrSelf<editor::XrEditorUi>(
          hit_node)) {
    InputActionState<Transform<float>> raycast_input_action_state =
        input_action_event.GetInputActionState<Transform<float>>(
            kDefaultAimActionName);
    input_action_event.action_name_to_input_action_state.clear();
    input_action_event.action_name_to_input_action_state.insert_or_assign(
        std::string(kDefaultAimActionName), raycast_input_action_state);
  }
}

}  // namespace

DevModeInputInterceptor::DevModeInputInterceptor(BaseView* view)
    : captured_id_(), view_(view) {
  soft_keyboard_controller_ =
      KeyboardController::Create(view->GetContext(), view);
}

void DevModeInputInterceptor::FilterPointerEvents(
    std::vector<PointerEvent>& pointer_events) {
  IMP_TRACE();
  if (!ImGui::GetCurrentContext()) return;

  bool use_remote_screen = false;
  absl::StatusOr<std::reference_wrapper<editor::Editor>> editor =
      view_->GetRegistry().Get<editor::Editor>();
  if (editor.ok()) {
    if (editor->get().GetDisplayMode() ==
        editor::EditorInfo::DisplayMode::kRemoteScreen) {
      use_remote_screen = true;
    }
  }

  window::FilamentHost::DevModeExtension* dev_mode_extension =
      view_->GetHost()->TryGetExtension();
  if (dev_mode_extension && dev_mode_extension->HasRenderTarget() &&
      !use_remote_screen) {
    // In this case, UI is in 3D floating panel, so the 2D pointer is ignored.
    return;
  }

  ImGuiIO& io = ImGui::GetIO();

  // If we have no pointer capture yet, see if we can acquire one.
  if (!captured_id_) {
    std::vector<Pointer::Id> down_pointers;
    std::vector<float2> down_pointer_positions;

    // Examine any down events, so we can check imgui for captures
    for (auto& event : pointer_events) {
      if (event.Type() != PointerEventType::kDown) continue;
      for (int down_index = 0; down_index < event.ChangedPointerCount();
           ++down_index) {
        const Pointer& pointer = event.GetPointer(down_index);
        std::optional<int> mouse_button = PointerToImGuiMouseIndex(pointer.id);
        if (mouse_button.has_value()) {
          down_pointers.push_back(pointer.id);
          down_pointer_positions.push_back(pointer.point);
          io.MouseDown[*mouse_button] = true;
          io.MouseClicked[*mouse_button] = true;
        }
      }
    }

    if (size_t down_count = down_pointer_positions.size()) {
      size_t down_index;
      std::optional<Rect> viewport_rect = GetViewportRect();
      for (down_index = 0; down_index < down_count; ++down_index) {
        const float2 point = down_pointer_positions[down_index];
        io.MousePos = ImVec2(point.x, point.y);
        ImGui::UpdateHoveredWindowAndCaptureFlags(io.MousePos);

        const bool is_over_viewport = IsMouseOverViewport(viewport_rect, point);
        if (io.WantCaptureMouse) {
          if (is_over_viewport) {
            // Don't let ImGui capture the pointer if it's over the 3D viewport
            // and is not obscured by another ImGui window.
            // This allows ImGui to overlay the viewport and still get inputs.
            // e.g. Right-click Hierarchy context menu overlapping the viewport.
            continue;
          }
          break;
        }
        auto& editor = view_->GetRegistry().Get<editor::Editor>()->get();
        if (editor.IsEnabled() && !is_over_viewport) {
          // If the editor is enabled and we are NOT over a viewport,
          // then the editor should capture the input to prevent it
          // from reaching the app.
          break;
        }
      }

      if (down_index != down_count) {
        // Imgui claimed a pointer.
        captured_id_.emplace(down_pointers[down_index]);
      } else {
        // Window should lose focus because we touched outside of ImGui.
        ImGui::SetWindowFocus(nullptr);
        // Unset mouse button left, right and middle when losing focus.
        io.MouseClicked[ImGuiMouseButton_Left] =
            io.MouseClicked[ImGuiMouseButton_Right] =
                io.MouseClicked[ImGuiMouseButton_Middle] = false;
        io.MouseDown[ImGuiMouseButton_Left] =
            io.MouseDown[ImGuiMouseButton_Right] =
                io.MouseDown[ImGuiMouseButton_Middle] = false;
      }
    } else if (!pointer_events.empty()) {
      // No capture, no down events; update mouse pos with first found pointer
      // on the last event.  This helps hovering work.
      auto point = pointer_events.back().GetPointer().point;
      io.MousePos = ImVec2(point.x, point.y);
    }
  }

  if (captured_id_) {
    // Remember ahead of time if our captured pointer will release this tick.
    bool releasing_or_cancelling_capture = false;

    for (const PointerEvent& event : pointer_events) {
      if (event.Type() != PointerEventType::kUp &&
          event.Type() != PointerEventType::kCancel)
        continue;

      int up_index;
      for (up_index = 0; up_index < event.ChangedPointerCount(); ++up_index) {
        const Pointer& pointer = event.GetPointer(up_index);
        if (pointer.id == *captured_id_) break;
      }

      if (up_index == event.ChangedPointerCount()) continue;

      releasing_or_cancelling_capture = true;
    }

    // Filter any messages using the captured pointer
    for (int event_index = 0; event_index < pointer_events.size();
         ++event_index) {
      const PointerEvent& event = pointer_events[event_index];
      const int pointer_index = event.GetIndexForPointerId(*captured_id_);

      if (pointer_index < 0 || pointer_index >= event.PointerCount()) continue;

      const Pointer& pointer = event.GetPointer(pointer_index);
      io.MousePos = ImVec2(pointer.point.x, pointer.point.y);

      if (use_remote_screen) continue;

      if (event.PointerCount() == 1 ||
          (pointer_index == 0 && event.ChangedPointerCount() == 1)) {
        // The removal of this pointer makes the event irrelevant; drop it.
        pointer_events.erase(pointer_events.begin() + event_index--);
      } else {
        std::vector<Pointer> new_pointers(event.GetPointers().begin(),
                                          event.GetPointers().end());
        new_pointers.erase(new_pointers.begin() + pointer_index);
        pointer_events[event_index] = PointerEvent(
            event.Type(), new_pointers,
            event.ChangedPointerCount() -
                (pointer_index < event.ChangedPointerCount() ? 1 : 0),
            event.ElapsedTime());
      }
    }

    if (releasing_or_cancelling_capture) {
      std::optional<int> mouse_button = PointerToImGuiMouseIndex(*captured_id_);

      if (mouse_button.has_value()) {
        io.MouseDown[*mouse_button] = false;
      }

      captured_id_.reset();
    }
  }

  // Transform remaining events to viewport space
  if (!editor.ok() || !editor->get().IsEnabled()) return;

  const std::optional<Rect> viewport_rect = editor->get().GetViewportRect();

  if (!viewport_rect) return;

  for (PointerEvent& event : pointer_events) {
    std::vector<Pointer> transformed_pointers;

    for (const Pointer& p : event.GetPointers()) {
      Pointer transformed_p = p;
      transformed_p.point -= viewport_rect->GetMin();
      transformed_pointers.push_back(transformed_p);
    }

    event = PointerEvent(event.Type(), transformed_pointers,
                         event.ChangedPointerCount(), event.ElapsedTime());
  }
}

void DevModeInputInterceptor::FilterKeyboardEvents(
    std::vector<KeyboardEvent>& keyboard_events,
    std::vector<TextInputEvent>& text_input_events) {
  IMP_TRACE();
  ImGuiIO& io = ImGui::GetIO();

  bool use_remote_screen = false;
  absl::StatusOr<std::reference_wrapper<editor::Editor>> editor =
      view_->GetRegistry().Get<editor::Editor>();
  if (editor.ok()) {
    if (editor->get().GetDisplayMode() ==
        editor::EditorInfo::DisplayMode::kRemoteScreen) {
      use_remote_screen = true;
    }
  }

  // To show/hide soft keyboard. Do nothing if the platform doesn't support
  // soft keyboard.
  if (soft_keyboard_controller_ && !use_remote_screen) {
    soft_keyboard_controller_->SetKeyboardShown(io.WantTextInput);
  }

  for (auto& event : keyboard_events) {
    switch (event.type) {
      case KeyboardEventType::kOnDown:
      case KeyboardEventType::kOnUp: {
        ImGuiKey key = static_cast<ImGuiKey>(event.key.code);
        bool down = (event.type == KeyboardEventType::kOnDown);
        io.AddKeyEvent(key, down);
        if (key == ImGuiKey_LeftShift || key == ImGuiKey_RightShift ||
            HasKeyModifier(KeyModifier::SHIFT, event.key.modifiers)) {
          io.AddKeyEvent(ImGuiMod_Shift, down);
        }
        if (key == ImGuiKey_LeftCtrl || key == ImGuiKey_RightCtrl ||
            HasKeyModifier(KeyModifier::CTRL, event.key.modifiers)) {
          io.AddKeyEvent(ImGuiMod_Ctrl, down);
        }
        if (key == ImGuiKey_LeftAlt || key == ImGuiKey_RightAlt ||
            HasKeyModifier(KeyModifier::ALT, event.key.modifiers)) {
          io.AddKeyEvent(ImGuiMod_Alt, down);
        }
        if (key == ImGuiKey_LeftSuper || key == ImGuiKey_RightSuper ||
            HasKeyModifier(KeyModifier::GUI, event.key.modifiers)) {
          io.AddKeyEvent(ImGuiMod_Super, down);
        }
        break;
      }
      case KeyboardEventType::kNone:
      case KeyboardEventType::kMax:
        break;
    }
  }

  if (io.WantCaptureKeyboard) {
    keyboard_events.clear();
  }

  for (auto& event : text_input_events) {
    io.AddInputCharactersUTF8(event.text.c_str());
  }
}

void DevModeInputInterceptor::FilterWheelEvents(
    std::vector<WheelEvent>& wheel_events) {
  auto wheel_itr = wheel_events.begin();
  while (wheel_itr != wheel_events.end()) {
    if (TryConsumeWheelEvent(*wheel_itr)) {
      wheel_itr = wheel_events.erase(wheel_itr);
    } else {
      ++wheel_itr;
    }
  }

  // Transform remaining events to viewport space
  absl::StatusOr<std::reference_wrapper<editor::Editor>> editor =
      view_->GetRegistry().Get<editor::Editor>();
  if (!editor.ok() || !editor->get().IsEnabled()) return;

  const std::optional<Rect> viewport_rect = editor->get().GetViewportRect();

  if (!viewport_rect) return;

  for (WheelEvent& event : wheel_events) {
    event =
        WheelEvent(event.GetDelta(), event.GetPoint() - viewport_rect->GetMin(),
                   event.GetElapsedTime());
  }
}

void DevModeInputInterceptor::FilterInputActionEvents(
    std::vector<InputActionEvent>& input_action_events) {
  IMP_TRACE();
  window::FilamentHost::DevModeExtension* dev_mode_extension =
      view_->GetHost()->TryGetExtension();
  if (!dev_mode_extension || !dev_mode_extension->IsEnabled()) {
    return;
  }

  // TODO Use this intercepted set of input actions to
  // capture input for "Editor camera mode". For example, here we can:
  //   - Take over the "menu", "select" buttons for Editor-specific
  //   actions.
  //   - Prioritize the transform widget in raycasts.
  //   - Select Impress nodes using controller ray.
  for (auto& input_action_event : input_action_events) {
    InterceptXrEditorInput(*view_, input_action_event);
  }

  absl::StatusOr<std::reference_wrapper<editor::Editor>> editor =
      view_->GetRegistry().Get<editor::Editor>();
  if (editor.ok() && editor->get().IsEnabled()) {
    input_action_events.clear();
  }
}

bool DevModeInputInterceptor::TryConsumeWheelEvent(
    const WheelEvent& wheel_event) {
  constexpr float kWheelSensitivity = 0.2f;

  ImGuiIO& io = ImGui::GetIO();

  absl::StatusOr<std::reference_wrapper<editor::Editor>> editor =
      view_->GetRegistry().Get<editor::Editor>();

  if (!editor.ok()) return false;

  const bool is_over_viewport = IsMouseOverViewport(
      editor->get().GetViewportRect(), float2(io.MousePos.x, io.MousePos.y));

  // Don't consume the wheel event if it's over the viewport.
  if (is_over_viewport) return false;

  if (ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow)) {
    io.MouseWheel += wheel_event.GetDelta().y * kWheelSensitivity;
    return true;
  }

  return editor->get().IsEnabled();
}

std::optional<Rect> DevModeInputInterceptor::GetViewportRect() const {
  auto& editor = view_->GetRegistry().Get<editor::Editor>()->get();
  return editor.GetViewportRect();
}
}  // namespace imp
