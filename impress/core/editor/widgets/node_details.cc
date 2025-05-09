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

#include "core/editor/widgets/node_details.h"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/misc/cpp/imgui_stdlib.h"
#include "core/common/file_helpers.h"
#include "core/common/platform_helpers.h"
#include "core/common/registry.h"
#include "core/config.h"
#include "core/editor/command_manager.h"
#include "core/editor/editor.h"
#include "core/editor/editor_style.h"
#include "core/editor/events.h"
#include "core/editor/events.proto.imp.h"
#include "core/editor/node_value_command.h"
#include "core/editor/widgets/asset_library.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_data.proto.imp.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/scene_metadata.h"
#include "core/proto/proto_writer.h"
#include "core/proto/textproto_writer.h"
#include "core/view/framework/scene/scene_reference.h"
#include "core/view/framework/scene/scene_system.h"
#if IMP_PLATFORM(DESKTOP)
#include "tinyfiledialogs/tinyfiledialogs.h"
#endif
#include "mediapipe/framework/port/status_macros.h"

namespace imp::editor {

static constexpr int32_t kDetailsUiWidth = 50;
static constexpr absl::string_view kTextprotoExtension = ".textproto";
static constexpr int32_t kFilterPatternsNum = 1;
static constexpr char const* kFilterPatterns[kFilterPatternsNum] = {
    kTextprotoExtension.data()};

NodeDetails::NodeDetails(BaseView& view)
    : view_(view),
      command_manager_(view_.GetRegistry().GetOrCreate<CommandManager>()) {
  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  editor.GetDispatcher().Connect(
      [this](const editor::NodeSelectionChangedEvent& event) mutable {
        active_node_ = event.selected;
      },
      this);
}

bool NodeDetails::HasContent() const { return active_node_.IsValid(); }

void NodeDetails::DrawImGui() {
  std::string name = std::string(active_node_->GetName());
  std::string old_name = name;
  if (ImGui::InputText("name", &name)) {
    command_manager_.PerformCommand<NodeValueCommand<std::string>>(
        active_node_, old_name, name,
        [](NodeHandle target, absl::string_view value) {
          target->SetName(value);
        });
  }

  auto scene_metadata = active_node_->GetComponent<SceneMetadata>();
  if (scene_metadata && !scene_metadata->GetBaseUrl().empty()) {
    Editor& editor = view_.GetRegistry().Get<Editor>()->get();
    std::string base = editor.GetAssetLibrary()->RemoveHomeDirectoryFromPath(
        scene_metadata->GetBaseUrl());
    ImGui::LabelText("base", base.c_str());
  }

  ImGui::PushItemWidth(kDetailsUiWidth);

  bool enabled = active_node_->IsEnabled();
  if (ImGui::Checkbox("enabled", &enabled)) {
    command_manager_.PerformCommand<NodeValueCommand<bool>>(
        active_node_, !enabled, enabled, [this](NodeHandle target, bool value) {
          target->SetEnabled(value);

          Editor& editor = view_.GetRegistry().Get<Editor>()->get();
          NodeUpdatedEvent event;
          event.target = target;
          event.enabled = value;
          editor.GetDispatcher().Send(event);
        });
  }
  ImGui::SameLine();
  ImGui::PushStyleColor(ImGuiCol_Text,
                        active_node_->IsActive() ? kGreen300 : kRed500);
  ImGui::LabelText(active_node_->IsActive() ? "active" : "inactive", "");
  ImGui::PopStyleColor();
  ImGui::PopItemWidth();

#if IMP_PLATFORM(DESKTOP)
  auto scene_reference = active_node_->GetComponent<SceneReference>();
  if (scene_reference && !scene_reference->GetAssetUrl().empty() &&
      scene_reference->GetAssetUrl() != SceneSystem::kRuntimeNodeDataPath &&
      scene_metadata) {
    if (ImGui::Button("Save##save-button")) {
      absl::Status save_status = Save();
      if (!save_status.ok()) {
        IMP_LOG(imp::ERROR) << "Failed to save scene: " << save_status;
      }
    }
  }
#endif
}

absl::Status NodeDetails::Save() {
#if IMP_PLATFORM(DESKTOP)
  auto scene_reference = active_node_->GetComponent<SceneReference>();
  absl::string_view asset_url_path = scene_reference->GetAssetUrl();
  absl::string_view path_without_extension =
      RemoveExtensionFromFilename(asset_url_path);

  std::string save_full_path =
      absl::StrCat(path_without_extension, kTextprotoExtension);

  MP_ASSIGN_OR_RETURN(NodeData data,
                   view_.GetSceneSystem().SaveToData(
                       active_node_, SceneSystem::SaveMode::kAuthoredContent));

  std::string isf_data;
  proto::SerializeTo(&data, &isf_data);

  std::string textproto_data;
  proto::ToTextproto(&data, &textproto_data);

  // Save the textproto to disk.
  MP_RETURN_IF_ERROR(SaveFile(
      save_full_path, reinterpret_cast<const uint8_t*>(textproto_data.c_str()),
      textproto_data.size()));

  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  editor.GetAssetLibrary()->AddResourceAtPath(asset_url_path, isf_data);

  return absl::OkStatus();

#else
  return absl::UnimplementedError("Save not supported on this platform");
#endif
}

}  // namespace imp::editor
