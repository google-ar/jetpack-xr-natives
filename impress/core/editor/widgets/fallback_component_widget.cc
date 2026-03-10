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

#include "core/editor/widgets/fallback_component_widget.h"

#include <cstdint>
#include <optional>
#include <string>
#if IMP_RUNTIME(DEV)
#include <vector>
#endif

#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/common/registry.h"
#include "core/config.h"
#include "core/editor/command_manager.h"
#include "core/editor/editor_field_control.h"
#include "core/editor/editor_style.h"
#include "core/editor/layout/editor_control_flags.h"
#include "core/editor/layout/helpers.h"
#include "core/editor/node_value_command.h"
#include "core/ncsb/base_component_pool.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/scene_metadata.h"
#if IMP_RUNTIME(DEV)
#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#endif

namespace imp::editor {

static constexpr int32_t kEnabledLineWidth = 50;

FallbackComponentWidget::FallbackComponentWidget(NodeHandle node,
                                                 BaseComponentPool* pool)
    : node_(node),
      pool_(pool),
      component_(node.GetEntity(), pool_,
                 pool->TryGetRawComponentFromEntity(node.GetEntity())),
      command_manager_(node->GetView()
                           .GetRegistry()
                           .template GetOrCreate<CommandManager>()) {
#if IMP_RUNTIME(DEV)
  std::vector<std::string> segments =
      absl::StrSplit(std::string(pool->GetTypeName()), "::");
  name_ = absl::StrCat(segments.back(),
                       absl::StrFormat("##%i", node_->GetEntity().getId()));
#else
  name_ = absl::StrFormat("Component Id: %i", component_->GetComponentId());
#endif

  metadata_ = node_->GetComponent<SceneMetadata>();
}

absl::string_view FallbackComponentWidget::GetName() const { return name_; }

bool FallbackComponentWidget::HasContent() const {
  return component_.IsValid();
}

void FallbackComponentWidget::DrawImGui() {
  // First render enabled and active details
  ImGui::PushItemWidth(kEnabledLineWidth);

  bool enabled = component_->IsEnabled();

  std::string label =
      GenerateUniqueImGuiLabel("enabled", this, EditorControlFlags::kDefault);

  // Draw checkbox to toggle enabled state.
  std::optional<bool> base_enabled;
  const SceneMetadata::ComponentSource* base_component_source =
      GetBaseComponentSource();
  if (base_component_source) {
    base_enabled = !base_component_source->disabled;
  }
  bool* base_enabled_ptr = base_enabled ? &base_enabled.value() : nullptr;

  EditorFieldControl::EditingElementMode mode =
      EditorFieldControl::BeginEditingElement(
          &enabled, base_enabled_ptr, editor::EditorControlFlags::kNone);

  bool is_enabled_changed = ImGui::Checkbox(label.c_str(), &enabled);
  is_enabled_changed |=
      EditorFieldControl::EndEditingElement(mode, &enabled, base_enabled_ptr);

  if (is_enabled_changed) {
    command_manager_.PerformCommand<NodeValueCommand<bool>>(
        component_->GetNode(), !enabled, enabled,
        [this](NodeHandle target, bool value) mutable {
          if (component_) {
            SetComponentEnabled(value);
            return absl::OkStatus();
          }
          return absl::NotFoundError("component not found.");
        });
  }

  ImGui::SameLine();

  bool active = component_->IsActive();

  ImGui::PushStyleColor(ImGuiCol_Text, active ? kDarkGreen : kDarkRed);
  ImGui::LabelText(active ? "active" : "inactive", "");
  ImGui::PopStyleColor();

  ImGui::PopItemWidth();

// Draw custom editor UI for this component.
#if IMP_RUNTIME(DEV)
  pool_->DrawEditorUi(node_.GetEntity());
#endif
}

void FallbackComponentWidget::SetComponentEnabled(bool enabled) {
  component_->SetEnabled(enabled);
}

CommandManager& FallbackComponentWidget::GetCommandManager() const {
  return command_manager_;
}

ComponentHandle<SceneMetadata> FallbackComponentWidget::GetMetadata() const {
  return metadata_;
}

const SceneMetadata::ComponentSource*
FallbackComponentWidget::GetBaseComponentSource() const {
  return nullptr;
}

}  // namespace imp::editor
