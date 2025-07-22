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

#include "core/editor/widgets/visualize_bounds.h"

#include "core/common/log.h"
#include "core/common/debug_draw.h"
#include "core/common/platform_helpers.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/events.h"
#include "core/geometry/shapes/box.h"
#include "core/math/almost_equal.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_flag.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/gltf_scene.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/collision/capsule_collider.h"
#include "core/view/framework/collision/cone_collider.h"
#include "core/view/framework/collision/cylinder_collider.h"
#include "core/view/framework/collision/sphere_collider.h"

namespace imp::editor {

// The size of the bounds expressed as a ratio from the distance of the camera
// in meters.
constexpr float kEmptyNodeBoundsDistanceRatio = 0.0025f;
constexpr debug_draw::DebugColor kEmptyNodeColor =
    debug_draw::DebugColor::kLightBlue;
constexpr debug_draw::Color kSelectedNodeColor = {0x81, 0xd4, 0xfa,
                                                  0xff};  // Light Blue 200
constexpr debug_draw::DebugColor kMeshNodeColor =
    debug_draw::DebugColor::kLightBlue;

VisualizeBounds::VisualizeBounds(BaseView& view) : view_(view) {
  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  Dispatcher& editor_dispatcher = editor.GetDispatcher();
  editor_dispatcher.Connect(
      [this](const ModelLoadedEvent& event) mutable {
        auto gltf_renderer = event.model->GetComponent<GltfRenderer>();
        if (gltf_renderer) {
          Box bounds = gltf_renderer->GetLocalFullBounds();
          // Log the bounds of any loaded glTF to make it really easy to
          // determine the size of a model for debugging purposes.
          // TODO: Make a Widget to show this in the UI.
          IMP_LOG(imp::INFO) << "Loaded Model " << gltf_renderer->GetAssetUrl()
                    << " Bounds: center=" << ToString(bounds.center)
                    << ", extent=" << ToString(bounds.halfExtent);
        }
      },
      this);
  editor_dispatcher.Connect(
      [this](const NodeSelectionChangedEvent& event) mutable {
        // Track which node is assigned in the hierarchy widget.
        selected_node_ = event.selected;
      },
      this);
  editor_dispatcher.Connect(
      [this](const EditorSettingChangedEvent& event) mutable {
        // Switch modes based on the "show all bounds" setting.
        if (event.show_all_bounds_enabled.has_value()) {
          if (*event.show_all_bounds_enabled) {
            mode_ = Mode::kShowAllBounds;
          } else {
            mode_ = Mode::kShowSelectedBounds;
          }
        }
      },
      this);
}

bool VisualizeBounds::HasCollider(NodeHandle node) const {
  return node->GetComponent<BoxCollider>() ||
         node->GetComponent<SphereCollider>() ||
         node->GetComponent<CapsuleCollider>() ||
         node->GetComponent<CylinderCollider>() ||
         node->GetComponent<ConeCollider>();
}

void VisualizeBounds::DrawImGui() { DrawBounds(); }

void VisualizeBounds::DrawBounds() {
  switch (mode_) {
    case Mode::kShowSelectedBounds: {
      // Only draw the selected node, if there is one.
      if (selected_node_) {
        // If top-level GltfRenderer node, draw all bounds
        auto gltf_renderer = selected_node_->GetComponent<GltfRenderer>();
        if (gltf_renderer) {
          DrawBoundsForNodeRecursive(selected_node_);
          return;
        }
        DrawBoundsForNode(selected_node_);
      }
      break;
    }
    case Mode::kShowAllBounds: {
      // Draw all bounds for all nodes.
      DrawBoundsForAllNodes();
      break;
    }
  }
}

void VisualizeBounds::DrawBoundsForNode(NodeHandle node) {
  if (!node) {
    return;
  }
  bool is_selected = node == selected_node_;
  auto mesh = node->GetComponent<GltfMesh>();
  if (mesh) {
    // Draw a node that contains a mesh using the mesh's real bounds.
    debug_draw::Color color =
        is_selected ? kSelectedNodeColor : debug_draw::GetColor(kMeshNodeColor);
    debug_draw::Local(node->GetEntity())
        .BoxLines(mesh->GetLocalBounds(), color);
  } else if (!is_selected && !HasCollider(node)) {
    // Draw a node that doesn't contain a mesh using tiny bounds.
    // Don't do this for the selected node, because in that case the transform
    // widget indicates selection.

    // The size of the box is determined based on the distance from the node to
    // the camera so that it always appears at a fixed size. This makes it easy
    // to see the nodes. Calling the precise variant is fine because either
    // it will be too far to render or we can truncate it to a float.
    Editor& editor = *view_.GetRegistry().Get<Editor>();
    double3 dist_vec =
        editor.GetActiveCamera()->GetNode()->GetWorldPositionPrecise() -
        node->GetWorldPositionPrecise();
    if (AlmostEqual(dist_vec, double3(kZero3))) {
      return;
    }
    double dist = norm(dist_vec);
    float3 extent(dist * kEmptyNodeBoundsDistanceRatio / node->GetWorldScale());

    Box box{.center = kZero3, .halfExtent = extent};
    debug_draw::Local(node->GetEntity())
        .BoxFaces(box, debug_draw::GetColor(kEmptyNodeColor));
  }
}

void VisualizeBounds::DrawBoundsForNodeRecursive(NodeHandle node) {
  // If this node is part of the editor, return early.
  // Also, make sure that its children are also treated as part of the editor
  // and not shown.
  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  if (node == editor.GetEditorRoot()) {
    return;
  }

  DrawBoundsForNode(node);

  for (NodeHandle child : node->GetChildren()) {
    DrawBoundsForNodeRecursive(child);
  }
}

void VisualizeBounds::DrawBoundsForAllNodes() {
  view_.ForEachNode(
      [this](NodeHandle node) {
        if (node == view_.GetCameraManager().GetCamera()->GetNode()) {
          return;
        }

        DrawBoundsForNodeRecursive(node);
      },
      NodeFlags::kIsRoot);
}

}  // namespace imp::editor
