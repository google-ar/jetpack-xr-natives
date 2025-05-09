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

#include "dear_imgui/imgui.h"
#include "core/common/registry.h"
#include "core/editor/command_manager.h"
#include "core/editor/editor.h"
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
#include "core/view/base_view.h"
#include "core/view/framework/scene/scene_system.h"

namespace imp::editor {

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
        active_node_ = event.selected;
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

  if (view_.IsPreciseTranslationEnabled()) {
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
    // To avoid using ImGUI's equal check, which is not reliable for float3
    if (ImGui::InputFloat3("position", position.v, kFloatFormat,
                           ImGuiInputTextFlags_CharsScientific) &&
        !AlmostEqual(position, old_position)) {
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

  quatf current_rotation = active_node_->GetLocalRotation();
  float3 old_eulers = rotation_field_.GetCurrentEulerAngles();

  if (rotation_field_.DrawFields("rotation", current_rotation)) {
    command_manager_.PerformCommand<NodeValueCommand<float3>>(
        active_node_, old_eulers, rotation_field_.GetCurrentEulerAngles(),
        [this](NodeHandle target, float3 value) {
          float3 eulers = FloatModulo(value, 360.0f, Clamp::kNonNegative);
          quatf rotation = QuatFromEuler(eulers);
          target->SetLocalRotation(QuatFromEuler(eulers));

          // Send an event to notify that the transform has been updated.
          NodeUpdatedEvent event;
          event.target = target;
          event.rotation = rotation;
          editor_.GetDispatcher().Send(event);
        });
  }

  float3 scale = active_node_->GetLocalScale();
  float3 old_scale = scale;
  // To avoid using ImGUI's equal check, which is not reliable for float3
  if (ImGui::InputFloat3("scale", scale.v, kFloatFormat,
                         ImGuiInputTextFlags_CharsScientific) &&
      !AlmostEqual(scale, old_scale)) {
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
