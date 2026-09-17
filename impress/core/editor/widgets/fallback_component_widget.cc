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
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/imgui_internal.h"
#include "core/common/registry.h"
#include "core/config.h"
#include "core/editor/command.h"
#include "core/editor/command_manager.h"
#include "core/editor/composite_command.h"
#include "core/editor/editor_field_control.h"
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

namespace {

static constexpr int32_t kEnabledLineWidth = 50;

}  // namespace

FallbackComponentWidget::FallbackComponentWidget(
    absl::flat_hash_set<NodeHandle> nodes, BaseComponentPool* pool)
    : nodes_(std::move(nodes)),
      reference_node_(*nodes_.begin()),
      pool_(pool),
      command_manager_(reference_node_->GetView()
                           .GetRegistry()
                           .template GetOrCreate<CommandManager>()) {
  for (const NodeHandle& node : nodes_) {
    Component* comp = pool->TryGetRawComponentFromEntity(node.GetEntity());
    if (comp) {
      components_.insert(ComponentHandle<Component>(*comp));
    }
  }

#if IMP_RUNTIME(DEV)
  std::vector<std::string> segments =
      absl::StrSplit(std::string(pool->GetTypeName()), "::");
  name_ = segments.back();
#else
  if (!components_.empty()) {
    name_ = absl::StrFormat("Component Id: %i",
                            (*components_.begin())->GetComponentId());
  }
#endif

  metadata_ = reference_node_->GetComponent<SceneMetadata>();
}

absl::string_view FallbackComponentWidget::GetName() const { return name_; }

bool FallbackComponentWidget::HasContent() const {
  return !components_.empty();
}

void FallbackComponentWidget::DrawImGui() {
  if (components_.empty()) return;

  auto first_component = *components_.begin();
  if (!first_component.IsValid()) return;

  // First render enabled and active details
  ImGui::PushItemWidth(kEnabledLineWidth);

  bool enabled = first_component->IsEnabled();

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

  bool all_same_enabled = true;
  for (const ComponentHandle<Component>& comp : components_) {
    if (!comp.IsValid()) continue;

    if (comp->IsEnabled() != enabled) {
      all_same_enabled = false;
      break;
    }
  }

  if (!all_same_enabled) {
    ImGui::PushItemFlag(ImGuiItemFlags_MixedValue, true);
  }

  const EditorFieldControl::EditingElementMode mode =
      EditorFieldControl::BeginEditingElement(
          &enabled, base_enabled_ptr, editor::EditorControlFlags::kNone);

  bool is_enabled_changed = ImGui::Checkbox(label.c_str(), &enabled);
  is_enabled_changed |=
      EditorFieldControl::EndEditingElement(mode, &enabled, base_enabled_ptr);

  if (!all_same_enabled) {
    ImGui::PopItemFlag();  // ImGuiItemFlags_MixedValue
  }

  if (is_enabled_changed) {
    CreateUndoCommand(components_, enabled);
  }

  ImGui::SameLine();

  DrawActiveLabel(components_);

  ImGui::PopItemWidth();

// Draw custom editor UI for this component.
#if IMP_RUNTIME(DEV)
  pool_->DrawEditorUi(reference_node_.GetEntity());
#endif
}

void FallbackComponentWidget::DrawActiveLabel(
    const absl::flat_hash_set<ComponentHandle<Component>>& active_components) {
  bool all_active = true;
  bool all_inactive = true;

  for (const ComponentHandle<Component>& comp : active_components) {
    if (!comp.IsValid()) continue;

    if (comp->IsActive()) {
      all_inactive = false;
    } else {
      all_active = false;
    }
  }

  DrawActiveLabelUi(all_active, all_inactive);
}

void FallbackComponentWidget::SetComponentEnabled(bool enabled) {
  for (auto& comp : components_) {
    if (comp.IsValid()) {
      comp.Get()->SetEnabled(enabled);
    }
  }
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

void FallbackComponentWidget::CreateUndoCommand(
    const absl::flat_hash_set<ComponentHandle<Component>>& components,
    const bool enabled) {
  std::vector<std::unique_ptr<Command>> commands;
  commands.reserve(components.size());

  auto command = [pool = pool_](NodeHandle target, bool value) mutable {
    Component* active_comp =
        pool->TryGetRawComponentFromEntity(target.GetEntity());

    if (!active_comp) return absl::NotFoundError("component not found.");

    active_comp->SetEnabled(value);

    return absl::OkStatus();
  };

  for (const ComponentHandle<Component>& comp : components) {
    if (!comp.IsValid()) continue;

    const bool node_prev_enabled = comp->IsEnabled();

    commands.push_back(std::make_unique<NodeValueCommand<bool>>(
        comp->GetNode(), node_prev_enabled, enabled, command));
  }

  if (commands.empty()) return;

  command_manager_.PerformCommand<CompositeCommand>(std::move(commands));
}

}  // namespace imp::editor
