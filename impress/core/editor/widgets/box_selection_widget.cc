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

#include "core/editor/widgets/box_selection_widget.h"

#include <algorithm>
#include <optional>
#include <utility>

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_set.h"
#include "dear_imgui/imgui.h"
#include "core/camera/camera_component.h"
#include "core/editor/editor.h"
#include "core/editor/selection_controller.h"
#include "core/geometry/shapes/box.h"
#include "core/geometry/shapes/rect.h"
#include "core/input/input_manager.h"
#include "core/input/key_codes.h"
#include "core/input/keyboard_event.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/path_manager.h"
#include "core/render/mesh_renderer.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/gestures/drag_gesture.h"

namespace imp::editor {

namespace {

// Selection box border color.
constexpr ImU32 kBorderColor = IM_COL32(255, 255, 255, 255);

// Selection box fill color.
constexpr ImU32 kBoxFillColor = IM_COL32(255, 255, 255, 64);

// Selection box border thickness.
constexpr float kBorderThickness = 2.0f;

// Amount of corner rounding on the selection box.
constexpr float kBoxRounding = 0.0f;

// The key codes that enable adding to selection.
constexpr VirtualKeyCode kAddSelectKeyCodes[] = {
    VirtualKeyCode::VK_LEFT_SHIFT,
    VirtualKeyCode::VK_RIGHT_SHIFT,
};

// The key codes that enable removing from selection.
constexpr VirtualKeyCode kRemoveSelectKeyCodes[] = {
    VirtualKeyCode::VK_LEFT_SUPER,
    VirtualKeyCode::VK_RIGHT_SUPER,
    VirtualKeyCode::VK_LEFT_CTRL,
    VirtualKeyCode::VK_RIGHT_CTRL,
};

// Returns true if the bounding box satisfies the selection mode.
bool IsBoundsInSelectionBox(const Box& world_bounds,
                            const CameraComponent& camera,
                            const Rect& selection_rect,
                            BoxSelectionWidget::SelectionMode mode) {
  if (!camera.IntersectsFrustum(world_bounds)) return false;

  const float3 b_min = world_bounds.getMin();
  const float3 b_max = world_bounds.getMax();

  // Get the 8 corners of the bounding box.
  const float3 corners[8] = {
      {b_min.x, b_min.y, b_min.z}, {b_max.x, b_min.y, b_min.z},
      {b_min.x, b_max.y, b_min.z}, {b_max.x, b_max.y, b_min.z},
      {b_min.x, b_min.y, b_max.z}, {b_max.x, b_min.y, b_max.z},
      {b_min.x, b_max.y, b_max.z}, {b_max.x, b_max.y, b_max.z},
  };

  // Min/max bounds in screen space we'll set as we iterate through the corners.
  float2 min_corner(1e9f, 1e9f);
  float2 max_corner(-1e9f, -1e9f);
  const float2 viewport_size = camera.PixelFromUVPoint({1.0f, 1.0f});

  // Convert the world space corners to screen space.
  for (const float3& corner : corners) {
    std::optional<float2> screen_pos = camera.PixelFromWorldPoint(corner);
    if (!screen_pos) {
      if (mode == BoxSelectionWidget::SelectionMode::kContains) return false;

      // If PixelFromWorldPoint returns null opt, it's behind the camera.
      // We can "fake" a projection by using a small positive w. This will
      // push the point far away in the correct screen-space direction.
      const double4 clip_p = camera.ClipFromWorld() * double4(corner, 1.0);
      const double w = std::max(clip_p.w, 1e-6);
      const float3 clip_pos = float3(clip_p.xyz / w);
      const float2 pixel_pos = camera.PixelFromClipPoint(clip_pos);

      // Clamp to viewport to get the "closest valid pixel".
      screen_pos = float2(imp::clamp(pixel_pos.x, 0.0f, viewport_size.x),
                          imp::clamp(pixel_pos.y, 0.0f, viewport_size.y));
    }

    min_corner = min(min_corner, *screen_pos);
    max_corner = max(max_corner, *screen_pos);
  }

  const Rect node_rect = Rect::FromPoints(min_corner, max_corner);

  if (mode == BoxSelectionWidget::SelectionMode::kIntersect) {
    return selection_rect.Intersects(node_rect);
  }

  // kContains mode: The bounds must be fully inside the selection box.
  return selection_rect.Contains(node_rect);
}

}  // namespace

BoxSelectionWidget::BoxSelectionWidget(BaseView& view) : view_(view) {
  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  Dispatcher& dispatcher = editor.GetDispatcher();

  dispatcher.Connect(
      [this](const DragGesture::StartEvent& event) {
        if (event.pointer != kMousePointerIdLeft) return;
        OnDragStart(event.position, event.target);
      },
      this);

  dispatcher.Connect(
      [this](const DragGesture::UpdateEvent& event) {
        OnDragUpdate(event.position);
      },
      this);

  dispatcher.Connect(
      [this](const DragGesture::FinishEvent& event) {
        OnDragFinish(event.position, event.cancelled);
      },
      this);

  dispatcher.Connect(
      [this](const imp::KeyboardEvent& event) {
        if (absl::c_linear_search(kAddSelectKeyCodes, event.key.code)) {
          switch (event.type) {
            case KeyboardEventType::kOnDown:
              held_add_select_keys_.insert(event.key.code);
              break;
            case KeyboardEventType::kOnUp:
              held_add_select_keys_.erase(event.key.code);
              break;
            default:
              break;
          }
        } else if (absl::c_linear_search(kRemoveSelectKeyCodes,
                                         event.key.code)) {
          switch (event.type) {
            case KeyboardEventType::kOnDown:
              held_remove_select_keys_.insert(event.key.code);
              break;
            case KeyboardEventType::kOnUp:
              held_remove_select_keys_.erase(event.key.code);
              break;
            default:
              break;
          }
        }
      },
      this);
}

void BoxSelectionWidget::DrawImGui() { DrawBox(); }

void BoxSelectionWidget::OnDragStart(float2 pos, NodeHandle target) {
  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  if (target &&
      view_.GetPathManager().IsAncestorOf(editor.GetEditorRoot(), target)) {
    return;  // Started dragging on something like the Transform gizmo.
  }

  dragging_ = true;
  start_pos_ = pos;
  current_pos_ = pos;
}

void BoxSelectionWidget::OnDragUpdate(float2 pos) {
  if (!dragging_) return;
  current_pos_ = pos;
}

void BoxSelectionWidget::OnDragFinish(float2 pos, bool cancelled) {
  if (!dragging_) return;

  current_pos_ = pos;

  dragging_ = false;

  if (cancelled) return;

  PerformSelection();
}

void BoxSelectionWidget::DrawBox() {
  if (!dragging_) return;

  Rect selection_rect = Rect::FromPoints(start_pos_, current_pos_);

  const Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  const std::optional<Rect> viewport_rect = editor.GetViewportRect();
  if (viewport_rect) {
    selection_rect.center += viewport_rect->GetMin();
  }

  const float2 min_pt = selection_rect.GetMin();
  const float2 max_pt = selection_rect.GetMax();

  ImDrawList* draw_list = ImGui::GetForegroundDrawList();

  draw_list->AddRect(ImVec2(min_pt.x, min_pt.y), ImVec2(max_pt.x, max_pt.y),
                     kBorderColor, kBoxRounding, ImDrawFlags_None,
                     kBorderThickness);
  draw_list->AddRectFilled(ImVec2(min_pt.x, min_pt.y),
                           ImVec2(max_pt.x, max_pt.y), kBoxFillColor);
}

void BoxSelectionWidget::PerformSelection() {
  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  const ComponentHandle<CameraComponent> camera = editor.GetActiveCamera();
  if (!camera) return;

  const Rect selection_rect = Rect::FromPoints(start_pos_, current_pos_);

  absl::flat_hash_set<NodeHandle> nodes_to_select;

  // Check all nodes in the scene for intersections with the selection box.
  view_.ForEachNode([this, &editor, &camera, &selection_rect,
                     &nodes_to_select](const NodeHandle node) {
    if (!node->IsActive()) return;

    const bool is_editor_node =
        view_.GetPathManager().IsAncestorOf(editor.GetEditorRoot(), node);

    if (is_editor_node) return;  // Skip editor nodes (Transform gizmo, etc).

    const ComponentHandle<GltfRenderer> gltf_renderer =
        node->GetComponent<GltfRenderer>();

    if (gltf_renderer) {
      // For GLTFs we check if their bounding box matches the selection mode.
      if (IsBoundsInSelectionBox(gltf_renderer->GetWorldFullBounds(), *camera,
                                 selection_rect, selection_mode_)) {
        nodes_to_select.insert(node);
      }
      return;
    }

    const ComponentHandle<MeshRenderer> mesh_renderer =
        node->GetComponent<MeshRenderer>();

    if (mesh_renderer) {
      if (IsBoundsInSelectionBox(mesh_renderer->GetWorldFullBounds(), *camera,
                                 selection_rect, selection_mode_)) {
        nodes_to_select.insert(node);
      }
      return;
    }

    // All other types of nodes are selected if their origin is selected.
    const std::optional<float2> screen_pos =
        camera->PixelFromWorldPoint(node->GetWorldPosition());

    if (!screen_pos) return;  // Node is not in view.

    // Check if the node's origin is inside the selection box.
    if (!selection_rect.Contains(*screen_pos)) return;

    nodes_to_select.insert(node);
  });

  SelectionController& selection_controller =
      view_.GetRegistry().Get<SelectionController>()->get();
  const bool add_select = !held_add_select_keys_.empty();
  const bool remove_select = !held_remove_select_keys_.empty();

  if (remove_select) {
    absl::flat_hash_set<NodeHandle> current_selection =
        selection_controller.GetSelectedNodes();
    for (const NodeHandle& node : nodes_to_select) {
      current_selection.erase(node);
    }
    nodes_to_select = std::move(current_selection);
  } else if (add_select) {
    const absl::flat_hash_set<NodeHandle>& current_selection =
        selection_controller.GetSelectedNodes();
    nodes_to_select.insert(current_selection.begin(), current_selection.end());
  }

  selection_controller.SetSelectedNodes(nodes_to_select);
}

}  // namespace imp::editor
