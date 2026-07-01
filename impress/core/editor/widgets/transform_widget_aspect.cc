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

#include "core/editor/widgets/transform_widget_aspect.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "filament/libs/math/include/math/TVecHelpers.h"
#include "core/async/future.h"
#include "core/camera/camera_component.h"
#include "core/camera/camera_manager.h"
#include "core/collision/collision_helpers.h"
#include "core/collision/ray.h"
#include "core/common/enum_flags.h"
#include "core/common/registry.h"
#include "core/editor/command.h"
#include "core/editor/command_manager.h"
#include "core/editor/composite_command.h"
#include "core/editor/editor.h"
#include "core/editor/events.h"
#include "core/editor/visualizers/transform_gizmo_assets.h"
#include "core/geometry/shapes/box.h"
#include "core/materials/material.h"
#include "core/math/almost_equal.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/assets/gltf_collider.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/gestures/drag_gesture.h"
#include "core/view/framework/gestures/hover_gesture.h"
#include "core/view/utils/asset.h"
#include "core/view/utils/frame_time.h"

namespace imp::editor {

namespace {

// Distance from the origin to start the colliders for translate/scale aspects.
constexpr float kGizmoDistanceFromOrigin = 2.0f;

// The thickness of the collider for translate/scale aspects.
constexpr float kGizmoBoxColliderHalfExtent = 0.5f;

// Gizmo material parameter names.
constexpr absl::string_view kBaseColorParam = "baseColor";
constexpr absl::string_view kScaleFactorParam = "scaleFactor";

// The scale factors for the transform widget vertices.
// Change these to adjust the thickness of each aspect.
constexpr float kDefaultScaleFactor = 0.0002f;
constexpr float kHoverScaleFactor = 0.0015f;

// The colors for each aspect of the transform widget.
constexpr float3 kXColor = {0.961f, 0.369f, 0.341f};           // GM3 Red 60
constexpr float3 kYColor = {0.502f, 0.855f, 0.533f};           // GM3 Green 80
constexpr float3 kZColor = {0.196f, 0.443f, 0.918f};           // GM3 Blue 50
constexpr float3 kScaleCenterColor = {0.675f, 0.929f, 1.0f};   // Cyan 90
constexpr float3 kCameraFacingColor = {0.675f, 0.929f, 1.0f};  // Cyan 90

constexpr float3 kActiveColor = {1.0f, 0.980f, 0.871f};  // GM3 Yellow 80
constexpr float3 kHoverColor = {0.989f, 0.741f, 0.0f};   // GM3 Yellow 98

// Returns true if node is a descendant of any of the nodes in
// potential_ancestors.
bool IsDescendantOfAny(
    const NodeHandle& node,
    const absl::flat_hash_set<NodeHandle>& potential_ancestors) {
  NodeHandle parent = node->GetParent();
  while (parent) {
    if (potential_ancestors.contains(parent)) {
      return true;
    }
    parent = parent->GetParent();
  }
  return false;
}

}  // namespace

imp::Future<absl::Status> TransformWidgetAspect::Setup() {
  editor_ = &GetView().GetRegistry().Get<Editor>()->get();
  command_manager_ = &GetView().GetRegistry().GetOrCreate<CommandManager>();
  hovered_ = false;

  const float3 axis = GetAxis();
  if (axis.x) active_axes_.Set(Axis::kX);
  if (axis.y) active_axes_.Set(Axis::kY);
  if (axis.z) active_axes_.Set(Axis::kZ);

  Dispatcher& editor_dispatcher = editor_->GetDispatcher();
  editor_dispatcher.Connect(
      [this](const editor::NodeSelectionChangedEvent& event) mutable {
        const absl::flat_hash_set<NodeHandle>& selected =
            editor_->GetSelectedNodes();
        selected_nodes_.assign(selected.begin(), selected.end());
      },
      this);

  // Update material parameters when hovered.
  editor_dispatcher.Connect(
      GetNode(),
      [this](const imp::HoverGesture::HoverEvent& event) mutable {
        // At the time this was implemented, HoverGesture and DragGesture were
        // coupled such that dragging would prevent HoverGesture::EXIT from
        // firing until the drag was both complete AND the pointer moved.
        // As a result if you finish dragging, the handle will remain scaled up
        // until the pointer moves again. If the Gestures are decoupled, this
        // should be revisited.
        if (event.state == HoverGesture::ENTER && !hovered_) {
          hovered_ = true;
          UpdateMaterial();
        } else if (event.state == HoverGesture::EXIT && hovered_) {
          hovered_ = false;
          UpdateMaterial();
        }
        return imp::Dispatcher::kAccept;
      },
      this);

  editor_dispatcher.Connect(
      GetNode(),
      [this](const DragGesture::StartEvent& event) {
        return OnDragStart(event);
      },
      this);

  editor_dispatcher.Connect(
      GetNode(),
      [this](const DragGesture::UpdateEvent& event) {
        return OnDragUpdate(event);
      },
      this);

  editor_dispatcher.Connect(
      GetNode(),
      [this](const DragGesture::FinishEvent& event) {
        return OnDragFinish(event);
      },
      this);

  GetView().GetDispatcher().Connect(
      [this](const TransformWidgetScaleChangedEvent& event) {
        widget_scale_ = event.scale;
        UpdateMaterial();
      },
      this);

  return GetView()
      .GetMaterialFactory()
      .LoadMaterial(GetMaterialAsset())
      .Then([this](absl::StatusOr<OwnedMaterialPtr> material) -> absl::Status {
        if (!material.ok()) {
          IMP_LOG(imp::ERROR) << "Failed to load transform gizmo material: "
                     << material.status();
          return material.status();
        }

        material_ = std::move(*material);
        SetupMaterialParameters();
        UpdateMaterial();

        ComponentHandle<GltfMesh> gltf_mesh =
            GetNode()->GetComponent<GltfMesh>();

        if (!gltf_mesh) {
          return absl::FailedPreconditionError(
              "GltfMesh component not found on node.");
        }

        for (size_t i = 0; i < gltf_mesh->GetPrimitiveCount(); ++i) {
          gltf_mesh->SetMaterialOverride(material_.Borrow(), i);
        }

        const Box bounds = gltf_mesh->GetLocalBounds();
        CreateCollider(bounds);

        GetNode()->RemoveComponent<GltfCollider>();
        return absl::OkStatus();
      });
}

void TransformWidgetAspect::CreateCollider(Box bounds) {
  if (GetNode()->GetComponent<BoxCollider>())
    return;  // Already has a collider.

  const float3 mesh_max_p = bounds.getMax();
  float3 min_p = bounds.getMin();
  float3 max_p = mesh_max_p;
  const float3 center = bounds.center;
  const float3 axis = GetAxis();

  int active_axes = 0;
  for (int i = 0; i < kNumAxes; ++i) {
    if (axis[i] > 0.0f) {
      active_axes++;
    }
  }

  for (int i = 0; i < kNumAxes; ++i) {
    if (axis[i] <= 0.0f) {
      // Not the main axis, set up the thickness of the collider.
      min_p[i] = center[i] - kGizmoBoxColliderHalfExtent;
      max_p[i] = center[i] + kGizmoBoxColliderHalfExtent;
    } else {
      if (active_axes == 1) {
        // We found our axis, cut off the collider near the origin.
        // Prevents overlap at the center preventing accidental selection.
        min_p[i] = kGizmoDistanceFromOrigin;
        // Cap the end of the collider to the end of the mesh.
        max_p[i] = mesh_max_p[i];
      } else {
        // For planar and uniform, use the mesh bounds but ensure minimum
        // thickness so that they are not too thin to select.
        min_p[i] = std::min(bounds.getMin()[i],
                            center[i] - kGizmoBoxColliderHalfExtent);
        max_p[i] =
            std::max(mesh_max_p[i], center[i] + kGizmoBoxColliderHalfExtent);
      }
    }
  }

  bounds.set(min_p, max_p);
  GetNode()->AddComponent<BoxCollider>(bounds);
}

ComponentHandle<CameraComponent> TransformWidgetAspect::GetCamera() const {
  ComponentHandle<CameraComponent> editor_camera = editor_->GetCamera();
  return editor_camera->IsActive() ? editor_camera
                                   : GetView().GetCameraManager().GetCamera();
}

std::optional<float3> TransformWidgetAspect::ComputeClosestPoint() const {
  const float3 position = centroid_start_;
  const float3 axis = GetAxis();
  const Ray pointer = GetCamera()->WorldRayFromPixelPoint(
      pointer_current_ - pointer_start_delta_from_model_center_);

  // Count the number of active axes.
  // Camera-facing aspects should have their axis set to (1,1,1).
  int active_axes = 0;
  if (axis.x) active_axes++;
  if (axis.y) active_axes++;
  if (axis.z) active_axes++;

  // 1D axes use line intersection.
  if (active_axes == 1) {
    const float3 direction = normalize(GetAxis());
    const std::optional<float> distance_to_move =
        collision::ClosestPointOnRayToLine(Ray(position, direction), pointer);

    if (!distance_to_move.has_value()) return position;

    return position + *distance_to_move * direction;
  }

  // 2D/3D axes and camera-facing aspects use plane intersection.
  float3 plane_normal;
  if (active_axes == 2) {
    plane_normal = kOne3 - axis;

    // Align the plane normal to the node's world rotation for scale aspects.
    // We only do this for single-node selections to avoid skewing.
    if (GetAspectType() == AspectType::kScale && selected_nodes_.size() == 1 &&
        selected_nodes_[0].IsValid()) {
      // Use the initial rotation if we're currently dragging.
      quatf rotation = (IsDragging() && !initial_node_transform_data_.empty())
                           ? initial_node_transform_data_[0].rotation_start
                           : selected_nodes_[0]->GetWorldRotation();
      plane_normal = rotation * plane_normal;
    }
  } else {
    // All three axes are active, use the camera's position to define the plane.
    plane_normal =
        normalize(GetCamera()->GetNode()->GetWorldPosition() - position);
  }

  const float denom = dot(pointer.direction, plane_normal);

  if (RoughlyEqual(std::abs(denom), 0.0f)) return std::nullopt;

  // Compute the intersection point of the plane and the ray.
  const float t = dot(position - pointer.origin, plane_normal) / denom;
  return pointer.origin + t * pointer.direction;
}

imp::Dispatcher::PropagationResult TransformWidgetAspect::OnDragStart(
    const DragGesture::StartEvent& event) {
  if (!IsActive() || selected_nodes_.empty()) {
    return imp::Dispatcher::kAccept;
  }

  // Compute the centroid of all the selected nodes including children.
  float3 sum_pos = {0.0f, 0.0f, 0.0f};
  int valid_node_count = 0;
  for (const NodeHandle& node : selected_nodes_) {
    if (!node.IsValid()) continue;
    sum_pos += node->GetWorldPosition();
    valid_node_count++;
  }

  // If there are no valid nodes, return.
  if (valid_node_count == 0) return imp::Dispatcher::kAccept;

  // Centroid is the sum of all positions divided by the # of nodes.
  centroid_start_ = sum_pos / static_cast<float>(valid_node_count);

  // We only want to apply transformations to nodes that are not
  // descendants of other selected nodes. e.g. if we scale a parent and
  // its child by 2x the child will have increased in size by 4x.
  std::vector<NodeHandle> nodes_to_process;
  const absl::flat_hash_set<NodeHandle> selected_nodes_set(
      selected_nodes_.begin(), selected_nodes_.end());
  for (const NodeHandle& node : selected_nodes_) {
    if (IsDescendantOfAny(node, selected_nodes_set)) continue;
    nodes_to_process.push_back(node);
  }

  initial_node_transform_data_.clear();
  initial_node_transform_data_.reserve(nodes_to_process.size());
  for (const NodeHandle& node : nodes_to_process) {
    const float3 pos = node->GetWorldPosition();
    initial_node_transform_data_.push_back(
        {node, pos, node->GetWorldRotation(), node->GetLocalScale()});
  }

  pointer_start_ = event.position;
  pointer_current_ = pointer_start_;

  const std::optional<float2> model_pixel_position =
      GetCamera()->PixelFromWorldPoint(centroid_start_);
  if (model_pixel_position.has_value()) {
    pointer_start_delta_from_model_center_ =
        pointer_start_ - *model_pixel_position;
  } else {
    pointer_start_delta_from_model_center_ = {0.0001f, 0.0001f};
  }

  std::optional<float3> closest_point_to_axis = ComputeClosestPoint();
  if (!closest_point_to_axis.has_value()) {
    closest_point_to_axis = centroid_start_ + GetAxis();
  }
  pointer_start_closest_point_to_axis_ = *closest_point_to_axis;

  dragging_ = true;
  UpdateMaterial();
  return imp::Dispatcher::kAccept;
}

imp::Dispatcher::PropagationResult TransformWidgetAspect::OnDragUpdate(
    const DragGesture::UpdateEvent& event) {
  pointer_current_ = event.position;
  return imp::Dispatcher::kAccept;
}

imp::Dispatcher::PropagationResult TransformWidgetAspect::OnDragFinish(
    const DragGesture::FinishEvent& event) {
  dragging_ = false;
  UpdateMaterial();
  ProcessAspectUpdate(/*commit = */ true);
  return imp::Dispatcher::kAccept;
}

void TransformWidgetAspect::Update(const FrameTime& frame_time) {
  if (selected_nodes_.empty()) return;

  UpdateWidgetTransform();

  if (!dragging_) {
    return;
  }

  ProcessAspectUpdate();
}

float3 TransformWidgetAspect::GetBaseColor() const {
  if (IsCameraFacing()) return kCameraFacingColor;

  if (active_axes_ == ToFlags(Axis::kX) ||
      active_axes_ == ToFlags(Axis::kY, Axis::kZ)) {
    return kXColor;
  }
  if (active_axes_ == ToFlags(Axis::kY) ||
      active_axes_ == ToFlags(Axis::kX, Axis::kZ)) {
    return kYColor;
  }
  if (active_axes_ == ToFlags(Axis::kZ) ||
      active_axes_ == ToFlags(Axis::kX, Axis::kY)) {
    return kZColor;
  }
  if (active_axes_ == ToFlags(Axis::kX, Axis::kY, Axis::kZ)) {
    return kScaleCenterColor;
  }

  return kOne3;  // Should not happen.
}
void TransformWidgetAspect::UpdateMaterial() {
  if (!material_) return;

  float3 color;
  float scale_factor = kDefaultScaleFactor;

  if (dragging_) {
    color = kActiveColor;
    scale_factor = kHoverScaleFactor;
  } else if (hovered_) {
    color = kHoverColor;
    scale_factor = kHoverScaleFactor;
  } else {
    color = GetBaseColor();
  }

  material_->SetParameter(kBaseColorParam, float4(color, 1.0f));
  material_->SetParameter(kScaleFactorParam, scale_factor / widget_scale_);
}

void TransformWidgetAspect::ProcessAspectUpdate(bool commit) {
  if (selected_nodes_.empty()) return;

  std::vector<std::unique_ptr<Command>> commands;
  UpdateAspect(commit, commands);

  if (!commit || commands.empty()) return;

  command_manager_->PerformCommand<CompositeCommand>(std::move(commands));
}

const AssetDefinition& TransformWidgetAspect::GetMaterialAsset() const {
  return transform_gizmo_assets::kTransformGizmoMaterialCmat;
}

}  // namespace imp::editor
