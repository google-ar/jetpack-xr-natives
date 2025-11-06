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

#include <optional>

#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/common/registry.h"
#include "core/editor/command_manager.h"
#include "core/editor/editor.h"
#include "core/editor/editor_field_control.h"
#include "core/editor/editor_style.h"
#include "core/editor/events.h"
#include "core/editor/events.proto.imp.h"
#include "core/editor/node_value_command.h"
#include "core/editor/widgets/transform_assets.h"
#include "core/editor/widgets/transform_widget.h"
#include "core/editor/widgets/transform_widget_aspect.h"
#include "core/editor/widgets/transform_widget_mode_control.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/scene_metadata.h"
#include "core/view/base_view.h"
#include "core/view/framework/scene/scene_system.h"

namespace imp::editor {

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
template <typename Fn>
bool EditTransformFloat3WithBase(float3* value,
                                 std::optional<float3> base_value,
                                 absl::string_view revert_to_base_label,
                                 Fn editor_control_fn) {
  float3* base_value_ptr =
      base_value.has_value() ? &base_value.value() : nullptr;

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
  view.GetSceneSystem().RegisterComponentsIsfInfo<TransformWidgetAspect>();
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
        // We only support single selection for the transform widget.
        active_node_ = editor_.GetSingleSelectedNode();
      },
      this);
}

bool Transform::HasContent() const {
  return transform_widget_.IsValid() && active_node_.IsValid();
}

void Transform::DrawImGui() {
  bool enabled = transform_widget_->IsEnabled();
  if (ImGui::Checkbox("3D widget", &enabled)) {
    transform_widget_->SetEnabled(enabled);
  }

  auto scene_metadata = active_node_->GetComponent<SceneMetadata>();

  if (view_.IsPreciseTranslationEnabled()) {
    // TODO: Support precise translation in an ISF file, including
    // base position.
    double3 position = active_node_->GetLocalPositionPrecise();
    double3 old_position = position;
    // To avoid using ImGUI's equal check, which is not reliable sometimes.
    if (ImGui::InputScalarN("position_precise", ImGuiDataType_Double,
                            position.v, 3, nullptr, nullptr, "%lf") &&
        !AlmostEqual(position, old_position)) {
      command_manager_.PerformCommand<NodeValueCommand<double3>>(
          active_node_, old_position, position,
          [this](NodeHandle target, double3 value) {
            target->SetLocalPositionPrecise(value);

            // Send an event to notify that the transform has been updated.
            NodeUpdatedEvent event;
            event.target = target;
            event.translation = value;
            editor_.GetDispatcher().Send(event);
          });
    }
  } else {
    float3 position = active_node_->GetLocalPosition();
    float3 old_position = position;

    std::optional<float3> base_position =
        scene_metadata ? scene_metadata->GetBaseLocalPosition() : std::nullopt;
    bool position_updated = EditTransformFloat3WithBase(
        &position, base_position, "TransformWidgetPosition",
        [old_position](float3* value) {
          // Use AlmostEqual to avoid precision issues with ImGui's float
          // comparison.
          return ImGui::InputFloat3("position", value->v, kFloatFormat,
                                    ImGuiInputTextFlags_CharsScientific) &&
                 !AlmostEqual(*value, old_position);
        });

    if (position_updated) {
      command_manager_.PerformCommand<NodeValueCommand<float3>>(
          active_node_, old_position, position,
          [this](NodeHandle target, float3 value) {
            target->SetLocalPosition(value);

            // Send an event to notify that the transform has been updated.
            NodeUpdatedEvent event;
            event.target = target;
            event.translation = value;
            editor_.GetDispatcher().Send(event);
          });
    }
  }

  // As described in the comment for EditTransformFloat3WithBase, we handle the
  // rotation field of the transform in a special way for similar reasons.
  //
  // Additionally, rotation has special handling to deal with stable conversion
  // between the quatf stored in the nodes actual transform and the euler angles
  // used in the editor & scene files.
  quatf current_rotation = active_node_->GetLocalRotation();
  float3 old_eulers = rotation_field_.GetCurrentEulerAngles();

  std::optional<quatf> base_rotation =
      scene_metadata ? scene_metadata->GetBaseLocalRotation() : std::nullopt;
  quatf* base_rotation_ptr =
      base_rotation.has_value() ? &base_rotation.value() : nullptr;
  EditorFieldControl::EditingElementMode mode =
      EditorFieldControl::BeginEditingElement(&current_rotation,
                                              base_rotation_ptr);

  bool rotation_updated =
      rotation_field_.DrawFields("rotation", current_rotation);

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

  if (rotation_updated) {
    command_manager_.PerformCommand<NodeValueCommand<float3>>(
        active_node_, old_eulers, rotation_field_.GetCurrentEulerAngles(),
        [this](NodeHandle target, float3 value) {
          float3 eulers = FloatModulo(value, 360.0f, Clamp::kNonNegative);
          quatf rotation = QuatFromEuler(eulers);
          target->SetLocalRotation(rotation);

          // Send an event to notify that the transform has been updated.
          NodeUpdatedEvent event;
          event.target = target;
          event.rotation = rotation;
          editor_.GetDispatcher().Send(event);
        });
  }

  float3 scale = active_node_->GetLocalScale();
  float3 old_scale = scale;

  std::optional<float3> base_scale =
      scene_metadata ? scene_metadata->GetBaseLocalScale() : std::nullopt;
  bool scale_updated = EditTransformFloat3WithBase(
      &scale, base_scale, "TransformWidgetScale", [old_scale](float3* value) {
        // Use AlmostEqual to avoid precision issues with ImGui's float
        // comparison.
        return ImGui::InputFloat3("scale", value->v, kFloatFormat,
                                  ImGuiInputTextFlags_CharsScientific) &&
               !AlmostEqual(*value, old_scale);
      });

  if (scale_updated) {
    command_manager_.PerformCommand<NodeValueCommand<float3>>(
        active_node_, old_scale, scale,
        [this](NodeHandle target, float3 value) {
          target->SetLocalScale(value);

          // Send an event to notify that the transform has been updated.
          NodeUpdatedEvent event;
          event.target = target;
          event.scale = value;
          editor_.GetDispatcher().Send(event);
        });
  }
}

}  // namespace imp::editor
