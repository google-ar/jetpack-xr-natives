/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_ASSET_LIBRARY_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_ASSET_LIBRARY_H_

#include <memory>
#include <optional>
#include <string>

#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/common/invocable.h"
#include "core/common/rememberer.h"
#include "core/editor/ui/directory_ui.h"
#include "core/editor/ui/drag_and_drop.h"
#include "core/editor/widget.h"
#include "core/editor/widgets/asset_thumbnail_provider.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"
#include "core/view/utils/string_map.h"
#include "core/view/utils/string_set.h"

namespace imp::editor {

// Shows UI to allow drag-and-drop of all registered assets.
class AssetLibrary : public editor::Widget, public imp::Rememberer {
 public:
  explicit AssetLibrary(BaseView& view);
  void DrawImGui() override;
  absl::string_view GetName() const override { return "Asset Library"; }

  // Adds a resource to the asset library, storing the given data permanently.
  std::string AddResourceInCurrentDirectory(absl::string_view name,
                                            absl::string_view type,
                                            absl::string_view data,
                                            absl::string_view description = "");

  void AddResourceAtPath(absl::string_view resource_path,
                         absl::string_view data,
                         absl::string_view description = "");

  void SetThumbnailOverride(absl::string_view resource, TexturePtr texture);

  // Resource that is loaded in at runtime.
  struct DynamicResource {
    std::string name;
    std::string type;
    std::string data;
    std::string description;

    DynamicResource(absl::string_view name, absl::string_view type,
                    absl::string_view data, absl::string_view description = "")
        : name(name), type(type), data(data) {}
  };

  // TODO: for long term, move the concept of "dynamic resources"
  // from AssetLibrary to ResourceManager, then we can have one combined data
  // source for everything that isn't so tied to the editor.
  StringMap<DynamicResource>& GetDynamicResources();

  // Toggles autosave of drag-and-dropped files to the app source location.
  void SetSavingToDiskEnabled(bool enabled);

  bool IsSavingToDiskEnabled() const;

  std::string RemoveHomeDirectoryFromPath(absl::string_view path) const;

 private:
  // A pending drag-and-dropped message resource to store in the library.
  struct PendingResource {
    std::string type;
    std::string data;
    std::string name;
    std::string description;

    std::string saved_data;
    std::string saved_extension;

    Invocable<void(absl::string_view)> on_save_callback;
  };

  void DrawTypeFilter();
  void DrawIsfPopupMenu(absl::string_view resource_path);

  std::string AddResource(const PendingResource& pending_resource);

  // Handles drag-and-drop from other parts of the editor into the library.
  void HandleDragAndDropNewResource();

  void CreateDerivedIsf(absl::string_view base_isf_path);

  BaseView& view_;

  // Fields used to manage the types of assets the library can display.
  StringViewMap<DragAndDropType> resource_types_;
  StringSet message_types_;

  // Fields used to manage assets created at runtime.
  StringMap<DynamicResource> dynamic_resources_;
  std::optional<PendingResource> pending_new_resource_;

  // Used to filter the assets shown in the library by a string.
  ImGuiTextFilter filter_;

  // Used to filter the assets shown in the library by extension.
  StringSet filtered_extensions_;

  std::unique_ptr<AssetThumbnailProvider> thumbnail_provider_;
  std::unique_ptr<DirectoryUi> directory_ui_;
  bool saving_to_disk_enabled_ = false;

  Rememberer rememberer_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_ASSET_LIBRARY_H_
