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

#include "core/editor/widgets/asset_library.h"

#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>

#include "absl/container/btree_set.h"
#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/imgui_internal.h"
#include "dear_imgui/misc/cpp/imgui_stdlib.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/async/future.h"
#include "core/common/file_helpers.h"
#include "core/config.h"
#include "core/editor/layout/editor_control_flags.h"
#include "core/editor/layout/helpers.h"
#include "core/editor/ui/directory_ui.h"
#include "core/editor/ui/drag_and_drop.h"
#include "core/editor/ui/drag_and_drop_node.h"
#include "core/editor/ui/ui_helpers.h"
#include "core/editor/widgets/asset_thumbnail_provider.h"
#include "core/editor/widgets/asset_type_helpers.h"
#include "core/ncsb/node_data.proto.imp.h"
#include "core/proto/proto_writer.h"
#include "core/proto/textproto_writer.h"
#include "core/render/texture.h"
#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"
#include "core/view/framework/scene/scene_reference.h"
#include "core/view/framework/scene/scene_system.h"
#include "core/view/utils/string_map.h"

#if IMP_PLATFORM(DESKTOP)
#include "core/editor/ui/directory_ui_desktop.h"
#else
#include "core/editor/ui/directory_ui_noop.h"
#endif

#if IMP_RUNTIME(DEV)
#include "core/view/framework/assets/asset_manager.h"
#endif

namespace imp::editor {

constexpr absl::string_view kImpressCoreAssetPath = "core";
constexpr int32_t kAssetLibraryPanelHeight = 250;
constexpr int32_t kFilterWidth = 200;
constexpr int32_t kTypesWidth = 200;
constexpr float kColumnWidth = kTableEntryImageSize.x * 4.0f;
constexpr float kTableBorderWidth = 1.0f;

// This alias helps avoid guarding huge swaths of code under IMP_DEV_RUNTIME.
const absl::btree_set<std::string>& GetRegisteredResources(BaseView& view) {
#if IMP_RUNTIME(DEV)
  return view.GetAssetManager().GetRegisteredResources();
#else
  // Empty set when this file is compiled in non-IMP_DEV_RUNTIME.
  static absl::btree_set<std::string>* registered_resources =
      new absl::btree_set<std::string>();
  return *registered_resources;
#endif
}

AssetLibrary::AssetLibrary(BaseView& view)
    : view_(view),
      resource_types_({{kIsfExt, DragAndDropType::kNodeAsset},
                       {kGlbExt, DragAndDropType::kNodeAsset},
                       {kGltfExt, DragAndDropType::kNodeAsset},
                       {kCmatExt, DragAndDropType::kMaterial},
                       {kPngExt, DragAndDropType::kTexture}}) {
  // Load the thumbnail provider.
  Future<std::unique_ptr<AssetThumbnailProvider>> thumbnail_provider_future =
      AssetThumbnailProvider::Create(view_);

  // Load the directory ui.
#if IMP_PLATFORM(DESKTOP)
  saving_to_disk_enabled_ = true;
  Future<std::unique_ptr<DirectoryUi>> directory_ui_future =
      DirectoryUiDesktop::Create(view_, filter_);
#else
  // TODO: Add a virtual directory ui for the web version.
  Future<std::unique_ptr<DirectoryUi>> directory_ui_future(
      std::make_unique<DirectoryUiNoop>());
#endif

  thumbnail_provider_future.Merge(directory_ui_future)
      .Then([this](std::tuple<std::unique_ptr<AssetThumbnailProvider>,
                              std::unique_ptr<DirectoryUi>>
                       tuple) mutable {
        auto [thumbnail_provider, directory_ui] = std::move(tuple);

        thumbnail_provider_ = std::move(thumbnail_provider);
        // TODO: The directory UI can be accessed before this
        // future is complete.
        directory_ui_ = std::move(directory_ui);
      })
      .KeptBy(&rememberer_);
}

bool IsCoreResource(std::string_view resource) {
  return resource.rfind(kImpressCoreAssetPath) != std::string::npos;
}

void AssetLibrary::DrawImGui() {
  if (!thumbnail_provider_ || !directory_ui_) {
    return;
  }

  bool is_docking_layout =
      ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_DockingEnable;

  if (ImGui::BeginChild(
          "##Asset Library (Child)",
          ImVec2(ImGui::GetContentRegionAvail().x,
                 is_docking_layout ? /*unlimited height*/ -1
                                   : kAssetLibraryPanelHeight),
          false,
          ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollWithMouse)) {
    directory_ui_->DrawDirectoriesHeader();

    ImGui::Separator();

    filter_.Draw(
        GenerateUniqueImGuiLabel("Filter", this, EditorControlFlags::kDefault)
            .c_str(),
        kFilterWidth);

    ImGui::SameLine();

    DrawTypeFilter();

    int32_t num_entries_per_row =
        std::ceil(ImGui::GetWindowWidth() / kColumnWidth);

    if (!ImGui::BeginTable(
            "Directory", num_entries_per_row,
            ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY |
                ImGuiTableFlags_Sortable,
            ImVec2(ImGui::GetContentRegionAvail().x - 1 -
                       (ImGui::GetStyle().WindowPadding.x * 2.0f),
                   ImGui::GetContentRegionAvail().y - kTableBorderWidth))) {
      IMP_LOG(imp::ERROR) << "BeginTable() failed";
      ImGui::EndChild();

      return;
    }

    directory_ui_->DrawDirectoriesInCurrentDirectory();

    const absl::btree_set<std::string>& registered_resources =
        GetRegisteredResources(view_);
    for (const std::string& resource : registered_resources) {
      if (absl::StartsWith(resource, kImpressCoreAssetPath)) {
        continue;
      }

      if (!filter_.PassFilter(resource.c_str())) {
        continue;
      }

      if (!directory_ui_->IsResourceInDirectory(resource)) {
        continue;
      }

      absl::string_view extension = GetAllExtensionsFromFilename(resource);
      if (filtered_extensions_.contains(extension)) {
        continue;
      }

      ImGui::TableNextColumn();

      ImGuiCenterNextHorizontally(kTableEntryImageSize.x,
                                  IncludePadding::kCell);

      filament::Texture* thumbnail =
          thumbnail_provider_->GetThumbnailForResource(resource);
      ImGui::Image(thumbnail, kTableEntryImageSize);

      auto itr = resource_types_.find(extension);
      if (itr != resource_types_.end()) {
        BeginDragAndDropSource(itr.value(),
                               absl::StrFormat("Load %s", extension), resource);
      } else {
        std::string payload_type = resource.substr(resource.find('.') + 1);

        // Insert the message type if we haven't seen it yet. This allows the
        // type to show up in the extensions filter.
        message_types_.insert(payload_type);

        if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
          ImGui::SetDragDropPayload(payload_type.c_str(), resource.data(),
                                    resource.size());
          ImGui::Text("apply");
          ImGui::EndDragDropSource();
        }
      }

      if (extension == kIsfExt) {
        DrawIsfPopupMenu(resource);
      }

      std::string resource_name =
          std::string(GetLocalFilenameFromFilename(resource));
      ImVec2 text_size =
          ImGui::CalcTextSize(resource_name.c_str(), nullptr, false,
                              ImGui::GetContentRegionAvail().x);
      ImGuiCenterNextHorizontally(text_size.x, IncludePadding::kNone);
      ImGui::TextWrapped(resource_name.c_str(), "");
    }

    ImGui::EndTable();
  }

  ImGui::EndChild();

  // Allow dragging registered protobuf messages back onto the library.
  // This needs to be outside of the child window so the entire asset library
  // is a drag-and-drop target regardless of the current vertical scroll.
  HandleDragAndDropNewResource();
}

void AssetLibrary::DrawIsfPopupMenu(absl::string_view resource_path) {
  if (ImGui::BeginPopupContextItem(
          absl::StrCat("Popup", resource_path).c_str())) {
    if (ImGui::MenuItem("Spawn", nullptr, false)) {
      view_.GetSceneSystem()
          .LoadScene(resource_path,
                     {.metadata_mode = SceneSystem::MetadataMode::kInclude})
          .KeptBy(&rememberer_);
    }
    if (ImGui::MenuItem("Spawn Derived", nullptr, false)) {
      NodeData node_data;
      node_data.base = resource_path;
      view_.GetSceneSystem()
          .LoadScene(node_data, "",
                     {.metadata_mode = SceneSystem::MetadataMode::kInclude})
          .KeptBy(&rememberer_);
    }
    if (ImGui::MenuItem("Create Derived", nullptr, false)) {
      CreateDerivedIsf(resource_path);
    }

    ImGui::EndPopup();
  }
}

void AssetLibrary::HandleDragAndDropNewResource() {
  ImGuiWindow* window = ImGui::GetCurrentWindow();
  if (ImGui::BeginDragDropTargetCustom(window->ContentRegionRect, window->ID)) {
    // Allow nodes from the hierarchy to be dragged to create ISFs.
    NodeHandle isf = AcceptDragAndDropPayloadNode();
    if (isf) {
      absl::StatusOr<NodeData> node_data = view_.GetSceneSystem().SaveToData(
          isf, SceneSystem::SaveMode::kAuthoredContent);
      if (!node_data.ok()) {
        IMP_LOG(imp::ERROR) << "Failed to save node data: " << node_data.status();
        return;
      }
      std::string isf_data;
      proto::SerializeTo(&(*node_data), &isf_data);

      std::string saved_data;
      proto::ToTextproto(&(*node_data), &saved_data);

      auto on_save_callback = [isf](absl::string_view saved_file_path) {
        isf->AddComponent<SceneReference>(saved_file_path);
      };

      pending_new_resource_.emplace(PendingResource{
          .type = std::string(kIsfExt).substr(1, kIsfExt.length()),
          .data = std::move(isf_data),
          .name = std::string(isf->GetName()),
          .saved_data = std::move(saved_data),
          .saved_extension = ".textproto",
          .on_save_callback = std::move(on_save_callback)});
    }

    // Iterate over all registered message types and accept a drop.
    absl::string_view data_type = ImGui::GetDragDropPayload()->DataType;
    if (absl::StartsWith(data_type, kProtoDragAndDropScheme)) {
      absl::StartsWith(data_type, kProtoDragAndDropScheme);

      std::optional<std::string> payload = AcceptDragAndDropPayload(data_type);

      // If there is a drop, set the "pending" resource, which will trigger
      // a modal dialog for the user to name the resource.
      // pending_new_resource_.emplace(message_type, payload.value());
      if (payload.has_value()) {
        std::string message_type =
            std::string(data_type.substr(kProtoDragAndDropScheme.length()));
        pending_new_resource_.emplace(PendingResource{
            .type = message_type,
            .data = payload.value(),
        });
      }
    }

    ImGui::EndDragDropTarget();
  }

  std::string popup_label =
      editor::GenerateUniqueImGuiLabel("Enter resource name:", this);
  if (pending_new_resource_.has_value()) {
    ImGui::OpenPopup(popup_label.c_str());
  }
  bool open = true;
  if (ImGui::BeginPopupModal(popup_label.c_str(), &open,
                             ImGuiWindowFlags_AlwaysAutoResize |
                                 ImGuiWindowFlags_Popup |
                                 ImGuiWindowFlags_Modal)) {
    if (ImGui::InputText(editor::GenerateUniqueImGuiLabel(
                             "resource_name", this, EditorControlFlags::kNone)
                             .c_str(),
                         &pending_new_resource_->name,
                         ImGuiInputTextFlags_EnterReturnsTrue) ||
        ImGui::Button(editor::GenerateUniqueImGuiLabel("Save", this).c_str())) {
      if (absl::StrContains(pending_new_resource_->name, ' ')) {
        ImGui::OpenPopup("Invalid Filename");
      } else {
        AddResource(*pending_new_resource_);
        pending_new_resource_.reset();
        ImGui::CloseCurrentPopup();
      }
    }

    if (ImGui::BeginPopupModal("Invalid Filename", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::Text("Filename cannot contain whitespaces.");
      if (ImGui::Button("OK")) {
        ImGui::CloseCurrentPopup();
      }
      ImGui::EndPopup();
    }
    ImGui::EndPopup();
  } else {
    pending_new_resource_.reset();
  }
}

StringMap<std::unique_ptr<AssetLibrary::DynamicResource>>&
AssetLibrary::GetDynamicResources() {
  return dynamic_resources_;
}

void AssetLibrary::SetSavingToDiskEnabled(bool enabled) {
  saving_to_disk_enabled_ = enabled;
}

bool AssetLibrary::IsSavingToDiskEnabled() const {
  return saving_to_disk_enabled_;
}

std::string AssetLibrary::RemoveHomeDirectoryFromPath(
    absl::string_view path) const {
  return directory_ui_->RemoveHomeDirectoryFromPath(path);
}

void AssetLibrary::DrawTypeFilter() {
  ImGui::SetNextItemWidth(kTypesWidth);

  // Helper lambda to toggle if an extension is filtered or not using an imgui
  // selectable.
  auto toggle_type_selectable_fn = [this](const std::string& extension) {
    bool was_selected = !filtered_extensions_.contains(extension);
    bool selected = was_selected;
    ImGui::Selectable(extension.c_str(), &selected);
    if (selected != was_selected) {
      if (selected) {
        filtered_extensions_.erase(extension);
      } else {
        filtered_extensions_.insert(extension);
      }
    }
  };

  if (ImGui::BeginCombo("Types", "extensions filter...")) {
    bool selected = false;
    ImGui::Selectable(filtered_extensions_.empty() ? "Hide All" : "Show All",
                      &selected);
    if (selected) {
      if (!filtered_extensions_.empty()) {
        filtered_extensions_.clear();
      } else {
        for (const auto& resource_type : resource_types_) {
          std::string extension = std::string(resource_type.first);
          filtered_extensions_.insert(extension);
        }

        for (absl::string_view message_type : message_types_) {
          std::string extension = absl::StrCat(".", message_type);
          filtered_extensions_.insert(extension);
        }
      }
    }

    ImGui::Separator();

    for (const auto& resource_type : resource_types_) {
      std::string extension = std::string(resource_type.first);
      toggle_type_selectable_fn(extension);
    }

    for (absl::string_view message_type : message_types_) {
      std::string extension = absl::StrCat(".", message_type);
      toggle_type_selectable_fn(extension);
    }

    ImGui::EndCombo();
  }
}

std::string AssetLibrary::AddResourceInCurrentDirectory(
    absl::string_view name, absl::string_view type, absl::string_view data,
    absl::string_view description) {
  return AddResource(PendingResource{.type = std::string(type),
                                     .data = std::string(data),
                                     .name = std::string(name),
                                     .description = std::string(description)});
}
void AssetLibrary::AddResourceAtPath(absl::string_view resource_path,
                                     absl::string_view data,
                                     absl::string_view description) {
  auto [itr, success] = dynamic_resources_.insert_or_assign(
      std::string(resource_path),
      std::make_unique<AssetLibrary::DynamicResource>(
          std::string(RemoveDirectoryAndExtensionFromFilename(resource_path)),
          std::string(GetExtensionFromFilename(resource_path).substr(1)),
          std::string(data)));

  absl::string_view stored_data = itr->second->data;
  resources::ResourceManager::RegisterResource(resource_path, stored_data,
                                               /* allow_overwrite = */ true);
}

void AssetLibrary::SetThumbnailOverride(absl::string_view resource,
                                        TexturePtr texture) {
  thumbnail_provider_->SetThumbnailOverride(resource, std::move(texture));
}

std::string AssetLibrary::AddResource(
    const AssetLibrary::PendingResource& pending_resource) {
  // TODO: Figure out why this target gets built in non-dev.
#if IMP_RUNTIME(DEV)

  // Need to permanently store the resource here.
  const auto resource_name =
      absl::StrFormat("%s.%s", pending_resource.name, pending_resource.type);

  std::string resource_path = directory_ui_->GetPathInDirectory(resource_name);

  IMP_LOG(imp::INFO) << "Saving resource " << resource_path;

  std::pair<StringMap<std::unique_ptr<AssetLibrary::DynamicResource>>::iterator,
            bool>
      result = dynamic_resources_.insert_or_assign(
          resource_path, std::make_unique<AssetLibrary::DynamicResource>(
                             pending_resource.name, pending_resource.type,
                             pending_resource.data));

  absl::string_view stored_data = result.first->second->data;
  resources::ResourceManager::RegisterResource(resource_path, stored_data,
                                               /* allow_overwrite = */ true);

  std::string saved_name;
  if (!pending_resource.saved_extension.empty()) {
    saved_name =
        absl::StrCat(pending_resource.name, pending_resource.saved_extension);
  } else {
    saved_name = resource_name;
  }

  absl::string_view data_to_save;
  if (!pending_resource.saved_data.empty()) {
    data_to_save = pending_resource.saved_data;
  } else {
    data_to_save = stored_data;
  }

  if (saving_to_disk_enabled_) {
    directory_ui_->SaveInDirectory(saved_name, data_to_save);
  }

  if (pending_resource.on_save_callback) {
    pending_resource.on_save_callback(resource_path);
  }

  return resource_path;
#else
  return "";
#endif
}

void AssetLibrary::CreateDerivedIsf(absl::string_view base_isf_path) {
  NodeData node_data;
  node_data.base = base_isf_path;

  std::string isf_data;
  proto::SerializeTo(&node_data, &isf_data);

  std::string textproto_data;
  proto::ToTextproto(&node_data, &textproto_data);

  std::string new_name = absl::StrCat(
      "derived_", RemoveDirectoryAndExtensionFromFilename(base_isf_path));

  pending_new_resource_.emplace(
      PendingResource{.type = std::string(kIsfExt).substr(1),
                      .data = isf_data,
                      .name = new_name,
                      .saved_data = textproto_data,
                      .saved_extension = ".textproto"});
}

}  // namespace imp::editor
