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

#include "core/editor/widgets/transform.h"

#include <functional>
#include <memory>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/common/registry.h"
#include "core/editor/command.h"
#include "core/editor/command_manager.h"
#include "core/editor/composite_command.h"
#include "core/editor/editor.h"
#include "core/editor/editor_field_control.h"
#include "core/editor/editor_style.h"
#include "core/editor/events.h"
#include "core/editor/events.proto.imp.h"
#include "core/editor/layout/helpers.h"
#include "core/editor/node_value_command.h"
#include "core/editor/widgets/transform_assets.h"
#include "core/editor/widgets/transform_widget.h"
#include "core/editor/widgets/transform_widget_aspect_rotate.h"
#include "core/editor/widgets/transform_widget_aspect_scale.h"
#include "core/editor/widgets/transform_widget_aspect_translate.h"
#include "core/editor/widgets/transform_widget_mode_control.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/scene_metadata.h"
#include "core/view/base_view.h"
#include "core/view/framework/scene/scene_system.h"

namespace imp::editor {
namespace {

// Applies a transform change to all selected nodes via a CompositeCommand.
template <typename ValueType, typename GetValueFn, typename ApplyValueFn>
void ApplyTransformChanges(const absl::flat_hash_set<NodeHandle>& active_nodes,
                           CommandManager& command_manager,
                           const ValueType& new_value,
                           const GetValueFn get_value_fn,
                           const ApplyValueFn apply_value_fn) {
  if (active_nodes.empty()) return;

  std::vector<std::unique_ptr<Command>> commands;
  commands.reserve(active_nodes.size());

  const ValueType base_old_value = get_value_fn(*active_nodes.begin());

  for (const NodeHandle& node : active_nodes) {
    const ValueType node_old_value = get_value_fn(node);
    ValueType node_new_value = node_old_value;

    if constexpr (std::is_same_v<ValueType, float3> ||
                  std::is_same_v<ValueType, double3>) {
      // Apply only the components that actually changed.
      if (!AlmostEqual(base_old_value.x, new_value.x))
        node_new_value.x = new_value.x;
      if (!AlmostEqual(base_old_value.y, new_value.y))
        node_new_value.y = new_value.y;
      if (!AlmostEqual(base_old_value.z, new_value.z))
        node_new_value.z = new_value.z;
    } else {
      node_new_value = new_value;
    }

    commands.push_back(std::make_unique<NodeValueCommand<ValueType>>(
        node, node_old_value, node_new_value, apply_value_fn));
  }
  command_manager.PerformCommand<CompositeCommand>(std::move(commands));
}

// Determines which axes of a property are identical/mixed across each node.
template <typename ValueType, typename GetValueFn>
void ComputeMixedAxes(const absl::flat_hash_set<NodeHandle>& active_nodes,
                      const GetValueFn get_value_fn, bool all_same[3]) {
  all_same[0] = true;
  all_same[1] = true;
  all_same[2] = true;

  if (active_nodes.empty()) return;

  const ValueType base_val = get_value_fn(*active_nodes.begin());

  for (const NodeHandle& node : active_nodes) {
    const ValueType node_val = get_value_fn(node);
    if (!AlmostEqual(node_val.x, base_val.x)) all_same[0] = false;
    if (!AlmostEqual(node_val.y, base_val.y)) all_same[1] = false;
    if (!AlmostEqual(node_val.z, base_val.z)) all_same[2] = false;
  }
}

}  // namespace

// Helper function for editing a float3 element of the nodes transform while
// correctly handlings the base value.
//
// This intentionally doesn't use the overload of
// EditorControlField::ShowDefaultControl that takes a float3 as a parameter.
//
// The reason is because ShowDefaultControl follows normal proto merging rules
// for handling the base value per-float instead of handling the entire float3
// together. However, in the case of the transform, SceneSystem handles the
// position and scale specially as a unit so that it is possible to override a
// transform to the default value (zero).
template <typename VecT, typename Fn>
bool EditTransformVectorWithBase(VecT* value, std::optional<VecT> base_value,
                                 absl::string_view revert_to_base_label,
                                 Fn editor_control_fn) {
  VecT* base_value_ptr = base_value.has_value() ? &base_value.value() : nullptr;

  EditorFieldControl::EditingElementMode mode =
      EditorFieldControl::BeginEditingElement(value, base_value_ptr);

  bool updated = editor_control_fn(value);

  // False is passed so that value is not used in the revert to base popup
  // label, since it isn't pointer stable.
  updated |= EditorFieldControl::EndEditingElement(mode, value, base_value_ptr,
                                                   revert_to_base_label, false);
  return updated;
}

Transform::Transform(BaseView& view)
    : view_(view),
      command_manager_(view.GetRegistry().GetOrCreate<CommandManager>()),
      editor_(view.GetRegistry().Get<Editor>()->get()) {
  view.GetSceneSystem().RegisterComponentsIsfInfo<TransformWidget>();
  view.GetSceneSystem()
      .RegisterComponentsIsfInfo<TransformWidgetAspectRotate>();
  view.GetSceneSystem().RegisterComponentsIsfInfo<TransformWidgetAspectScale>();
  view.GetSceneSystem()
      .RegisterComponentsIsfInfo<TransformWidgetAspectTranslate>();
  view.GetSceneSystem().RegisterComponentsIsfInfo<TransformWidgetModeControl>();
  view.GetSceneSystem()
      .LoadScene(transform_assets::kTransformWidgetIsf)
      .Then([this](NodeHandle node) {
        transform_widget_ = node;
        transform_widget_->SetParent(editor_.GetEditorRoot());
      })
      .KeptBy(this);
  editor_.GetDispatcher().Connect(
      [this](const editor::NodeSelectionChangedEvent& event) mutable {
        active_nodes_ = editor_.GetSelectedNodes();
      },
      this);
}

bool Transform::HasContent() const {
  if (!transform_widget_.IsValid()) return false;

  for (const NodeHandle& node : active_nodes_) {
    if (node.IsValid()) return true;
  }
  return false;
}

void Transform::DrawImGui() {
  PruneInvalidHandles(active_nodes_);
  if (active_nodes_.empty()) return;

  bool enabled = transform_widget_->IsEnabled();
  if (ImGui::Checkbox("3D Widget", &enabled)) {
    transform_widget_->SetEnabled(enabled);
  }

  const NodeHandle first_node = *active_nodes_.begin();
  const auto scene_metadata = first_node->GetComponent<SceneMetadata>();

  if (view_.IsPreciseTranslationEnabled()) {
    DrawPositionFieldsPrecise(first_node, scene_metadata);
  } else {
    DrawPositionFields(first_node, scene_metadata);
  }

  DrawRotationFields(first_node, scene_metadata);
  DrawScaleFields(first_node, scene_metadata);
}

template <bool IsPrecise>
void Transform::DrawPositionFieldsImpl(
    const NodeHandle first_node,
    const ComponentHandle<SceneMetadata> scene_metadata) {
  using VecT = std::conditional_t<IsPrecise, double3, float3>;
  VecT position = IsPrecise ? VecT(first_node->GetLocalPositionPrecise())
                            : VecT(first_node->GetLocalPosition());
  const VecT old_position = position;

  bool all_same_position[3];
  const auto get_pos = [](const NodeHandle& node) -> VecT {
    if constexpr (IsPrecise) {
      return node->GetLocalPositionPrecise();
    } else {
      return node->GetLocalPosition();
    }
  };
  ComputeMixedAxes<VecT>(active_nodes_, get_pos, all_same_position);

  const bool use_metadata = active_nodes_.size() == 1 && scene_metadata;
  std::optional<VecT> base_position;
  if (use_metadata) {
    if (auto base_pos_f = scene_metadata->GetBaseLocalPosition()) {
      base_position = VecT(base_pos_f->x, base_pos_f->y, base_pos_f->z);
    }
  }

  const char* format = IsPrecise ? "%lf" : kFloatFormat;
  const char* label = IsPrecise ? "position_precise" : "position";
  const auto draw_position_fields = [label, format, old_position,
                                     &all_same_position](VecT* value) {
    return EditorFieldControl::DrawMultiInput3(
               label, value->v, all_same_position, format,
               ImGuiInputTextFlags_CharsScientific) &&
           !AlmostEqual(*value, old_position);
  };

  const bool position_updated = EditTransformVectorWithBase(
      &position, base_position, "TransformWidgetPosition",
      draw_position_fields);

  if (!position_updated) return;

  const auto apply_position_change = [&editor = editor_](NodeHandle target,
                                                         VecT value) {
    if constexpr (IsPrecise) {
      target->SetLocalPositionPrecise(value);
    } else {
      target->SetLocalPosition(value);
    }
    NodeUpdatedEvent event;
    event.target = target;
    event.translation = value;
    editor.GetDispatcher().Send(event);
  };

  ApplyTransformChanges(active_nodes_, command_manager_, position, get_pos,
                        apply_position_change);
}

void Transform::DrawPositionFieldsPrecise(
    NodeHandle first_node, ComponentHandle<SceneMetadata> scene_metadata) {
  DrawPositionFieldsImpl</*IsPrecise=*/true>(first_node, scene_metadata);
}

void Transform::DrawPositionFields(
    const NodeHandle first_node,
    const ComponentHandle<SceneMetadata> scene_metadata) {
  DrawPositionFieldsImpl</*IsPrecise=*/false>(first_node, scene_metadata);
}

void Transform::DrawRotationFields(
    const NodeHandle first_node,
    const ComponentHandle<SceneMetadata> scene_metadata) {
  // As described in the comment for EditTransformFloat3WithBase, we handle the
  // rotation field of the transform in a special way for similar reasons.
  //
  // Additionally, rotation has special handling to deal with stable conversion
  // between the quatf stored in the nodes actual transform and the euler angles
  // used in the editor & scene files.
  quatf current_rotation = first_node->GetLocalRotation();

  // Determine which axes of rotation are identical or mixed.
  // This uses Euler angles because rotation is displayed and manipulated in
  // Euler angles in the UI.
  bool all_same_rotation[3];
  const auto get_eulers = [](const NodeHandle& node) -> float3 {
    return EulerFromQuatClamped(node->GetLocalRotation());
  };
  ComputeMixedAxes<float3>(active_nodes_, get_eulers, all_same_rotation);

  const bool use_metadata = active_nodes_.size() == 1 && scene_metadata;
  std::optional<quatf> base_rotation =
      use_metadata ? scene_metadata->GetBaseLocalRotation() : std::nullopt;
  quatf* base_rotation_ptr =
      base_rotation.has_value() ? &base_rotation.value() : nullptr;
  const EditorFieldControl::EditingElementMode mode =
      EditorFieldControl::BeginEditingElement(&current_rotation,
                                              base_rotation_ptr);

  bool rotation_updated = rotation_field_.DrawFields(
      "rotation", current_rotation, all_same_rotation);

  // False is passed so that value is not used in the revert to base popup
  // label, since it isn't pointer stable.
  if (EditorFieldControl::EndEditingElement(mode, &current_rotation,
                                            base_rotation_ptr,
                                            "TransformWidgetRotation", false)) {
    // If EndEditingElement returns true, then the rotation was edited via the
    // popup menu to revert to the base value. We need to update the current
    // rotation to the new value.
    rotation_field_.UpdateCurrentRotation(current_rotation);
    rotation_updated = true;
  }

  if (!rotation_updated) return;

  const auto apply_rotation_change =
      [&editor = editor_](const NodeHandle target, const float3 value) {
        const float3 eulers = FloatModulo(value, 360.0f, Clamp::kNonNegative);
        const quatf rotation = QuatFromEuler(eulers);
        target->SetLocalRotation(rotation);
        NodeUpdatedEvent event;
        event.target = target;
        event.rotation = rotation;
        editor.GetDispatcher().Send(event);
      };

  ApplyTransformChanges(active_nodes_, command_manager_,
                        rotation_field_.GetCurrentEulerAngles(), get_eulers,
                        apply_rotation_change);
}

void Transform::DrawScaleFields(NodeHandle first_node,
                                ComponentHandle<SceneMetadata> scene_metadata) {
  float3 scale = first_node->GetLocalScale();
  const float3 old_scale = scale;

  bool all_same_scale[3];
  const auto get_scale = [](const NodeHandle& node) -> float3 {
    return node->GetLocalScale();
  };
  ComputeMixedAxes<float3>(active_nodes_, get_scale, all_same_scale);

  const bool use_metadata = active_nodes_.size() == 1 && scene_metadata;
  const std::optional<float3> base_scale =
      use_metadata ? scene_metadata->GetBaseLocalScale() : std::nullopt;

  const auto draw_scale_fields = [old_scale, &all_same_scale](float3* value) {
    return EditorFieldControl::DrawMultiInput3(
               "scale", value->v, all_same_scale, kFloatFormat,
               ImGuiInputTextFlags_CharsScientific) &&
           !AlmostEqual(*value, old_scale);
  };

  const bool scale_updated = EditTransformVectorWithBase(
      &scale, base_scale, "TransformWidgetScale", draw_scale_fields);

  if (!scale_updated) return;

  const auto apply_scale_change = [&editor = editor_](const NodeHandle target,
                                                      const float3 value) {
    target->SetLocalScale(value);
    NodeUpdatedEvent event;
    event.target = target;
    event.scale = value;
    editor.GetDispatcher().Send(event);
  };

  ApplyTransformChanges(active_nodes_, command_manager_, scale, get_scale,
                        apply_scale_change);
}

}  // namespace imp::editor
