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

#include "absl/container/flat_hash_set.h"
#include "core/common/log.h"
#include "core/camera/camera_component.h"
#include "core/common/debug_draw.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/events.h"
#include "core/geometry/shapes/box.h"
#include "core/math/almost_equal.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_flag.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/gltf_scene.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/collision/capsule_collider.h"
#include "core/view/framework/collision/cone_collider.h"
#include "core/view/framework/collision/cylinder_collider.h"
#include "core/view/framework/collision/sphere_collider.h"

namespace imp::editor {

// The size of the bounds expressed as a ratio from the distance of the camera
// in meters.
constexpr float kEmptyNodeBoundsDistanceRatio = 0.0025f;
constexpr debug_draw::Color kEmptyNodeColor = debug_draw::kLightBlue;
constexpr debug_draw::Color kSelectedNodeColor = {0x81, 0xd4, 0xfa,
                                                  0xff};  // Light Blue 200
constexpr debug_draw::Color kMeshNodeColor = debug_draw::kLightBlue;

VisualizeBounds::VisualizeBounds(BaseView& view) : view_(view) {
  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  Dispatcher& editor_dispatcher = editor.GetDispatcher();
  editor_dispatcher.Connect(
      [](const ModelLoadedEvent& event) {
        auto gltf_renderer = event.model->GetComponent<GltfRenderer>();
        if (!gltf_renderer) return;
        Box bounds = gltf_renderer->GetLocalFullBounds();
        // Log the bounds of any loaded glTF to make it really easy to
        // determine the size of a model for debugging purposes.
        // TODO: Make a Widget to show this in the UI.
        IMP_LOG(imp::INFO) << "Loaded Model " << gltf_renderer->GetAssetUrl()
                  << " Bounds: center=" << ToString(bounds.center)
                  << ", extent=" << ToString(bounds.halfExtent);
      },
      this);
  editor_dispatcher.Connect(
      [this](const NodeSelectionChangedEvent& event) {
        selected_nodes_ =
            view_.GetRegistry().Get<Editor>()->get().GetSelectedNodes();
      },
      this);
  editor_dispatcher.Connect(
      [this](const EditorSettingChangedEvent& event) {
        // Switch modes based on the "show all bounds" setting.
        if (!event.show_all_bounds_enabled.has_value()) return;

        if (*event.show_all_bounds_enabled) {
          mode_ = Mode::kShowAllBounds;
        } else {
          mode_ = Mode::kShowSelectedBounds;
        }
      },
      this);
}

bool VisualizeBounds::HasCollider(NodeHandle node) const {
  if (!node) return false;

  return node->GetComponent<BoxCollider>() ||
         node->GetComponent<SphereCollider>() ||
         node->GetComponent<CapsuleCollider>() ||
         node->GetComponent<CylinderCollider>() ||
         node->GetComponent<ConeCollider>();
}

void VisualizeBounds::DrawImGui() { DrawBounds(); }

void VisualizeBounds::DrawBounds() {
  visited_nodes_.clear();

  Editor& editor = *view_.GetRegistry().Get<Editor>();
  const ComponentHandle<CameraComponent> active_camera =
      editor.GetActiveCamera();

  if (!active_camera) return;

  const NodeHandle camera_node = active_camera->GetNode();
  const double3 camera_pos = camera_node->GetWorldPositionPrecise();

  switch (mode_) {
    case Mode::kShowSelectedBounds: {
      // Process GltfRenderer nodes first to ensure their subtrees are fully
      // drawn.
      for (const NodeHandle& node : selected_nodes_) {
        if (!node || !node->GetComponent<GltfRenderer>()) continue;

        DrawBoundsForNodeRecursive(node, camera_pos);
      }
      for (const NodeHandle& node : selected_nodes_) {
        DrawBoundsForNode(node, camera_pos);
      }
      break;
    }
    case Mode::kShowAllBounds: {
      // Draw all bounds for all nodes.
      DrawBoundsForAllNodes(camera_pos);
      break;
    }
  }
}

void VisualizeBounds::DrawBoundsForNode(const NodeHandle node,
                                        const double3& camera_pos) {
  if (!node) return;

  if (!visited_nodes_.insert(node).second) return;

  const bool is_selected = selected_nodes_.contains(node);
  const auto mesh = node->GetComponent<GltfMesh>();

  if (mesh) {
    // Draw a node that contains a mesh using the mesh's real bounds.
    debug_draw::Color color = is_selected ? kSelectedNodeColor : kMeshNodeColor;
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
    const double3 dist_vec = camera_pos - node->GetWorldPositionPrecise();

    if (AlmostEqual(dist_vec, double3(kZero3))) return;

    const double dist = norm(dist_vec);
    const float3 extent(dist * kEmptyNodeBoundsDistanceRatio /
                        node->GetWorldScale());

    const Box box{.center = kZero3, .halfExtent = extent};
    debug_draw::Local(node->GetEntity()).BoxFaces(box, kEmptyNodeColor);
  }
}

void VisualizeBounds::DrawBoundsForNodeRecursive(const NodeHandle node,
                                                 const double3& camera_pos) {
  if (!node || visited_nodes_.contains(node)) return;

  // If this node is part of the editor, return early.
  // Also, make sure that its children are also treated as part of the editor
  // and not shown.
  Editor& editor = view_.GetRegistry().Get<Editor>()->get();

  if (node == editor.GetEditorRoot()) return;

  DrawBoundsForNode(node, camera_pos);

  for (const NodeHandle& child : node->GetChildren()) {
    DrawBoundsForNodeRecursive(child, camera_pos);
  }
}

void VisualizeBounds::DrawBoundsForAllNodes(const double3& camera_pos) {
  view_.ForEachNode(
      [this, &camera_pos](const NodeHandle& node) {
        if (!node) return;

        DrawBoundsForNodeRecursive(node, camera_pos);
      },
      NodeFlags::kIsRoot);
}

}  // namespace imp::editor
