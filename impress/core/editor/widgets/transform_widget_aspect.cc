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
#include "core/common/registry.h"
#include "core/editor/command.h"
#include "core/editor/command_manager.h"
#include "core/editor/composite_command.h"
#include "core/editor/editor.h"
#include "core/editor/events.h"
#include "core/editor/events.proto.imp.h"
#include "core/editor/node_value_command.h"
#include "core/editor/visualizers/transform_gizmo_assets.h"
#include "core/editor/widgets/transform_widget_aspect_state.proto.imp.h"
#include "core/geometry/shapes/box.h"
#include "core/materials/material.h"
#include "core/math/almost_equal.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/scene_handles/scene_handle_status_utils.h"
#include "core/view/framework/assets/gltf_collider.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/collision/cylinder_collider.h"
#include "core/view/framework/gestures/drag_gesture.h"
#include "core/view/framework/gestures/hover_gesture.h"
#include "core/view/framework/gestures/tap_gesture.h"
#include "core/view/utils/asset.h"
#include "core/view/utils/frame_time.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::editor {

namespace {

// The number of axes to control for the transform widget.
// Update this when we add support for 4D hypercubes :)
constexpr int kNumAxes = 3;

// Distance from the origin to start the colliders for translate/scale aspects.
constexpr float kGizmoDistanceFromOrigin = 0.5f;

// The thickness of the collider for translate/scale aspects.
constexpr float kGizmoBoxColliderHalfExtent = 0.5f;

// Gizmo material parameter names.
constexpr absl::string_view kBaseColorParam = "baseColor";
constexpr absl::string_view kScaleFactorParam = "scaleFactor";
constexpr absl::string_view kDiscardThresholdParam = "discardThreshold";

// The scale factors for the transform widget vertices.
constexpr float kDefaultScaleFactor = 0.0f;
constexpr float kHoverScaleFactor = 0.0015f;

// The rotate gizmo discards pixels in its fragment shader based on this value.
// The discard works by grabbing a direction vector from the local-space vertex
// position and dotting it with the local-space view direction.
// Values are from -1 to 1, and any value below the threshold is discarded.
constexpr float kDiscardThreshold = -0.2f;

// The colors for each aspect of the transform widget.
constexpr float3 kXColor = {0.961f, 0.369f, 0.341f};  // GM3 Red 60
constexpr float3 kYColor = {0.502f, 0.855f, 0.533f};  // GM3 Green 80
constexpr float3 kZColor = {0.196f, 0.443f, 0.918f};  // GM3 Blue 50

constexpr float3 kActiveColor = {1.0f, 0.980f, 0.871f};  // GM3 Yellow 80
constexpr float3 kHoverColor = {0.989f, 0.741f, 0.0f};   // GM3 Yellow 98

// Rotations to align a cylinder collider with the X, Y, and Z axes.
// The cylinder is aligned with the Y-axis by default, so index 1 is identity.
const quatf kRotateAspectColliderRotations[3] = {
    quatf::fromAxisAngle(float3(0.0f, 0.0f, 1.0f), -M_PI / 2.0f),
    kIdentityQuatf,
    quatf::fromAxisAngle(float3(1.0f, 0.0f, 0.0f), M_PI / 2.0f)};

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
  MP_RETURN_IF_ERROR(
      IsHandleValid<&TransformWidgetAspectState::transform_widget_mode_control>(
          state_));

  Dispatcher& editor_dispatcher = editor_->GetDispatcher();
  editor_dispatcher.Connect(
      [this](const editor::NodeSelectionChangedEvent& event) mutable {
        const absl::flat_hash_set<NodeHandle>& selected =
            editor_->GetSelectedNodes();
        selected_nodes_.assign(selected.begin(), selected.end());
      },
      this);

  // Catch the TapEvent to prevent the editor from doing anything else.
  editor_dispatcher.Connect(
      GetNode(),
      [this](const imp::TapGesture::TapEvent& event) mutable {
        state_.transform_widget_mode_control->CycleMode();
        return imp::Dispatcher::kAccept;
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

  const AssetDefinition& material_asset =
      state_.aspect == TRANSFORM_ASPECT_ROTATE
          ? transform_gizmo_assets::kTransformGizmoRotateMaterialCmat
          : transform_gizmo_assets::kTransformGizmoMaterialCmat;

  return GetView()
      .GetMaterialFactory()
      .LoadMaterial(material_asset)
      .Then([this](absl::StatusOr<OwnedMaterialPtr> material) -> absl::Status {
        if (!material.ok()) {
          IMP_LOG(imp::ERROR) << "Failed to load transform gizmo material: "
                     << material.status();
          return material.status();
        }

        material_ = std::move(*material);
        UpdateMaterial();

        if (state_.aspect == TRANSFORM_ASPECT_ROTATE) {
          material_->SetParameter(kDiscardThresholdParam, kDiscardThreshold);
        }

        ComponentHandle<GltfMesh> gltf_mesh =
            GetNode()->GetComponent<GltfMesh>();

        if (!gltf_mesh) {
          return absl::FailedPreconditionError(
              "GltfMesh component not found on node.");
        }

        gltf_mesh->SetMaterialOverride(material_.Borrow());
        Box bounds = gltf_mesh->GetLocalBounds();

        if (state_.aspect == TRANSFORM_ASPECT_ROTATE) {
          CreateCylinderCollider(bounds);
        } else {
          CreateBoxCollider(bounds);
        }

        GetNode()->RemoveComponent<GltfCollider>();
        return absl::OkStatus();
      });
}

void TransformWidgetAspect::CreateBoxCollider(Box bounds) {
  const float3 mesh_max_p = bounds.getMax();
  float3 min_p = bounds.getMin();
  float3 max_p = mesh_max_p;
  const float3 center = bounds.center;

  for (int i = 0; i < kNumAxes; ++i) {
    if (state_.axis[i] <= 0.0f) {
      // Not the main axis, set up the thickness of the collider.
      min_p[i] = center[i] - kGizmoBoxColliderHalfExtent;
      max_p[i] = center[i] + kGizmoBoxColliderHalfExtent;
    } else {
      // We found our axis, cut off the collider near the origin.
      // Prevents overlap at the center preventing accidental selection.
      min_p[i] = kGizmoDistanceFromOrigin;

      // Cap the end of the collider to the end of the mesh.
      max_p[i] = mesh_max_p[i];
    }
  }

  bounds.set(min_p, max_p);
  GetNode()->AddComponent<BoxCollider>(bounds);
}

void TransformWidgetAspect::CreateCylinderCollider(const Box& bounds) {
  const float3 extents = bounds.getMax() - bounds.getMin();
  float height = 0.0f;
  float radius = 0.0f;

  // Will lead to a crash if all axes are <= 0.0f in the proto.
  NodeHandle collider_node;

  for (int i = 0; i < kNumAxes; ++i) {
    if (state_.axis[i] <= 0.0f) continue;

    // If this is the x axis, use the y and z extents to determine the radius.
    // If y, use the x and z. If z, use the x and y.
    const int ii = (i + 1) % kNumAxes;
    const int jj = (i + 2) % kNumAxes;
    // Get the radius of the circle formed by the extents.
    radius = std::max(extents[ii], extents[jj]) / 2.0f;
    // Use the size along the axis of rotation as the height of the cylinder.
    height = extents[i];

    // Create a new node to attach the cylinder collider to.
    // We need to do this since we can't rotate the cylinder collider in its
    // component and instead need to rotate the node that holds the collider.
    collider_node = GetNode()->CreateChildNode();
    collider_node->SetLocalRotation(kRotateAspectColliderRotations[i]);
  }

  auto collider = collider_node->AddComponent<CylinderCollider>();
  const float3 center_in_collider =
      inverse(collider_node->GetLocalRotation()) * bounds.center;
  const float3 base = center_in_collider - float3(0.0f, height / 2.0f, 0.0f);
  collider->SetCylinder(base, radius, height);

  // Have this new collider node report back up to the transform widget aspect.
  collider->SetHitNode(GetNode());
}

ComponentHandle<CameraComponent> TransformWidgetAspect::GetCamera() const {
  ComponentHandle<CameraComponent> editor_camera = editor_->GetCamera();
  return editor_camera->IsActive() ? editor_camera
                                   : GetView().GetCameraManager().GetCamera();
}

float3 TransformWidgetAspect::GetAxis() const {
  if (state_.aspect != TRANSFORM_ASPECT_SCALE) {
    return state_.axis;
  }

  if (selected_nodes_.size() > 1) {
    return state_.axis;
  }

  if (!selected_nodes_.empty() && selected_nodes_[0].IsValid()) {
    return normalize(selected_nodes_[0]->WorldFromLocalVector(state_.axis));
  }

  return state_.axis;
}

std::optional<float3> TransformWidgetAspect::ComputeClosestPoint() const {
  const Ray pointer = GetCamera()->WorldRayFromPixelPoint(
      pointer_current_ - pointer_start_delta_from_model_center_);

  const float3 model_position = centroid_start_;

  const float3 direction = normalize(GetAxis());
  const std::optional<float> distance_to_move =
      collision::ClosestPointOnRayToLine(Ray(model_position, direction),
                                         pointer);
  if (!distance_to_move.has_value()) {
    return model_position;
  }
  return model_position + *distance_to_move * direction;
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
  UpdateAspect(/*commit = */ true);
  return imp::Dispatcher::kAccept;
}

void TransformWidgetAspect::Update(const FrameTime& frame_time) {
  if (selected_nodes_.empty()) return;

  if (state_.aspect == TRANSFORM_ASPECT_SCALE) {
    // If we're in scale mode with only a single node selected, align the
    // transform widget to the node's world rotation.
    if (selected_nodes_.size() == 1 && selected_nodes_[0].IsValid()) {
      GetNode()->GetParent()->SetWorldRotation(
          selected_nodes_[0]->GetWorldRotation());
    } else {
      // If multiple nodes are selected, use an identity rotation.
      GetNode()->GetParent()->SetWorldRotation(kIdentityQuatf);
    }
  }

  if (!dragging_) {
    return;
  }

  UpdateAspect();
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
    color = kXColor;
    if (state_.axis.y > 0.5f) {
      color = kYColor;
    } else if (state_.axis.z > 0.5f) {
      color = kZColor;
    }
  }

  material_->SetParameter(kBaseColorParam, float4(color, 1.0f));
  material_->SetParameter(kScaleFactorParam, scale_factor);
}

void TransformWidgetAspect::UpdateAspect(bool commit) {
  if (selected_nodes_.empty()) return;

  std::vector<std::unique_ptr<Command>> commands;

  switch (state_.aspect) {
    case TRANSFORM_ASPECT_TRANSLATE:
      UpdateTranslate(commit, commands);
      break;
    case TRANSFORM_ASPECT_ROTATE:
      UpdateRotate(commit, commands);
      break;
    case TRANSFORM_ASPECT_SCALE:
      UpdateScale(commit, commands);
      break;
  }

  if (!commit || commands.empty()) return;

  command_manager_->PerformCommand<CompositeCommand>(std::move(commands));
}

void TransformWidgetAspect::UpdateTranslate(
    const bool commit, std::vector<std::unique_ptr<Command>>& commands) {
  // Calculate the closest point on the widget's axis to the user's pointer.
  // This is in world-space.
  const std::optional<float3> projection = ComputeClosestPoint();

  if (!projection.has_value()) return;  // Pointer and axis are parallel.

  // The projection on the axis is the new position of the centroid.
  const float3 new_centroid = *projection;
  // Calculate how much the centroid has moved since the drag started.
  const float3 delta = new_centroid - centroid_start_;

  // Apply the same translation delta to all selected nodes.
  for (const NodeTransformData& data : initial_node_transform_data_) {
    if (!data.node.IsValid()) continue;

    const float3 new_pos = data.position_start + delta;
    data.node->SetWorldPosition(new_pos);

    // If commit is false, we only update the node visually but don't create
    // undo/redo commands. This happens continuously during the drag.
    // Commands are only added to the stack when the drag finishes and the
    // position has changed enough to matter.
    if (!commit || AlmostEqual(data.position_start, new_pos)) continue;

    commands.push_back(std::make_unique<NodeValueCommand<float3>>(
        data.node, data.position_start, new_pos,
        [this](NodeHandle target, const float3 value) {
          target->SetWorldPosition(value);
          NodeUpdatedEvent event;
          event.target = target;
          event.translation = value;
          editor_->GetDispatcher().Send(event);
        }));
  }
}

void TransformWidgetAspect::UpdateRotate(
    const bool commit, std::vector<std::unique_ptr<Command>>& commands) {
  // Get the screen-space position of the centroid of the selected nodes.
  const std::optional<float2> model_clip_position =
      GetCamera()->PixelFromWorldPoint(centroid_start_);

  if (!model_clip_position.has_value()) return;  // Centroid is off-screen.

  // Vec from the widget center to the pointer position when the drag started.
  const float2 start = normalize(pointer_start_ - *model_clip_position);
  // Convert the start vector to an angle about the model center.
  const float angle_start = atan2(start.y, start.x);
  // Vec from the widget center to the current pointer position.
  const float2 current = normalize(pointer_current_ - *model_clip_position);
  // Convert the current vector to an angle about the model center.
  const float angle_current = atan2(current.y, current.x);
  // The difference between these two angles is the angle to rotate to.
  const float angle = angle_start - angle_current;

  // TODO: support local mode.
  // Quaternion representing the rotation around the selected axis.
  const quatf delta_rot = quatf::fromAxisAngle(state_.axis, angle);

  for (const NodeTransformData& data : initial_node_transform_data_) {
    if (!data.node.IsValid()) continue;

    // If rotating multiple nodes, they rotate around the group's centroid.
    // Calculate the node's position relative to the centroid before rotation.
    const float3 node_offset = data.position_start - centroid_start_;
    // Rotate the offset vector and add it back to the centroid to get the
    // node's new position.
    const float3 new_pos = centroid_start_ + delta_rot * node_offset;
    // Apply the rotation delta to the node's starting rotation.
    const quatf new_rot = delta_rot * data.rotation_start;

    data.node->SetWorldPosition(new_pos);
    data.node->SetWorldRotation(new_rot);

    // If commit is false, we only update the node visually but don't create
    // undo/redo commands. This happens continuously during the drag.
    // Commands are only added to the stack when the drag finishes.
    if (!commit) continue;

    // Check if the new values are different enough from the starting
    // values to warrant creating an undo entry.
    if (!AlmostEqual(data.position_start, new_pos)) {
      commands.push_back(std::make_unique<NodeValueCommand<float3>>(
          data.node, data.position_start, new_pos,
          [this](NodeHandle target, const float3 value) {
            target->SetWorldPosition(value);
            NodeUpdatedEvent event;
            event.target = target;
            event.translation = value;
            editor_->GetDispatcher().Send(event);
          }));
    }

    if (!AlmostEqual(data.rotation_start, new_rot)) {
      commands.push_back(std::make_unique<NodeValueCommand<quatf>>(
          data.node, data.rotation_start, new_rot,
          [this](NodeHandle target, const quatf value) {
            target->SetWorldRotation(value);
            NodeUpdatedEvent event;
            event.target = target;
            event.rotation = value;
            editor_->GetDispatcher().Send(event);
          }));
    }
  }
}

void TransformWidgetAspect::UpdateScale(
    const bool commit, std::vector<std::unique_ptr<Command>>& commands) {
  // Calculate the closest point on the widget's axis to the user's pointer.
  // This is in world-space.
  const std::optional<float3> projection = ComputeClosestPoint();

  if (!projection) return;  // Pointer and axis are parallel.

  // Difference between the current projected point on the axis and the initial
  // point when the drag started.
  const float3 delta_vec = *projection - pointer_start_closest_point_to_axis_;

  // Project the delta vector onto the axis with dot to get a 1D scale factor.
  // This represents how much the user has dragged along the axis.
  const float scale_1D = dot(delta_vec, GetAxis());

  // Non-uniform scale along this axis as the default.
  float3 s_vec = float3(1.0f, 1.0f, 1.0f) + state_.axis * scale_1D;

  // Scale uniformly if multiple nodes are selected to prevent skew/shear.
  if (selected_nodes_.size() > 1) {
    s_vec = float3(1.0f + scale_1D);
  }

  for (const NodeTransformData& data : initial_node_transform_data_) {
    if (!data.node.IsValid()) continue;

    // When scaling multiple objects we also need to translate them towards/away
    // from the centroid to get a true group scale.
    float3 offset = data.position_start - centroid_start_;
    float3 new_pos = centroid_start_ + s_vec * offset;

    // Apply the scale factor to the node's starting scale.
    float3 new_scale = data.scale_start * s_vec;

    data.node->SetWorldPosition(new_pos);
    data.node->SetLocalScale(new_scale);

    // If commit is false, we only update the node visually but don't create
    // undo/redo commands. This happens continuously during the drag.
    // Commands are only added to the stack when the drag finishes.
    if (!commit) continue;

    // Check if the new values are different enough from the starting
    // values to warrant creating an undo entry.
    if (!AlmostEqual(data.position_start, new_pos)) {
      commands.push_back(std::make_unique<NodeValueCommand<float3>>(
          data.node, data.position_start, new_pos,
          [this](NodeHandle target, const float3 value) {
            target->SetWorldPosition(value);
            NodeUpdatedEvent event;
            event.target = target;
            event.translation = value;
            editor_->GetDispatcher().Send(event);
          }));
    }

    if (!AlmostEqual(data.scale_start, new_scale)) {
      commands.push_back(std::make_unique<NodeValueCommand<float3>>(
          data.node, data.scale_start, new_scale,
          [this](NodeHandle target, const float3 value) {
            target->SetLocalScale(value);
            NodeUpdatedEvent event;
            event.target = target;
            event.scale = value;
            editor_->GetDispatcher().Send(event);
          }));
    }
  }
}

}  // namespace imp::editor
