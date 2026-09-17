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
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/async/future.h"
#include "core/common/invocable.h"
#include "core/common/rememberer.h"
#include "core/editor/ui/directory_ui.h"
#include "core/editor/ui/drag_and_drop.h"
#include "core/editor/widget.h"
#include "core/render/material_registry.h"
#include "core/render/texture.h"
#include "core/scene_handles/material_handle.h"
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

  std::string AddResourceInCurrentDirectory(absl::string_view name,
                                            absl::string_view type,
                                            absl::Cord data,
                                            absl::string_view description = "");

  void AddResourceAtPath(absl::string_view resource_path,
                         absl::string_view data,
                         absl::string_view description = "");

  void SetThumbnailOverride(absl::string_view resource,
                            OwnedTexturePtr texture);

  // Resource that is loaded in at runtime.
  struct DynamicResource {
    std::string name;
    std::string type;
    absl::Cord data;
    std::string description;

    DynamicResource(std::string name, std::string type, absl::Cord data,
                    std::string description = "")
        : name(std::move(name)),
          type(std::move(type)),
          data(std::move(data)),
          description(std::move(description)) {
      this->data.Flatten();
    }
  };

  // TODO: for long term, move the concept of "dynamic resources"
  // from AssetLibrary to ResourceManager, then we can have one combined data
  // source for everything that isn't so tied to the editor.
  StringMap<std::unique_ptr<DynamicResource>>& GetDynamicResources();

  // Toggles autosave of drag-and-dropped files to the app source location.
  void SetSavingToDiskEnabled(bool enabled);

  bool IsSavingToDiskEnabled() const;

  std::string RemoveHomeDirectoryFromPath(absl::string_view path) const;

  std::optional<std::string> GetSelectedAsset() const;
  void SetSelectedAsset(std::optional<std::string> asset);

 private:
  // A pending drag-and-dropped message resource to store in the library.
  struct PendingResource {
    std::string name;
    std::string type;
    absl::Cord data;
    std::string description;

    std::string saved_data;
    std::string saved_extension;

    Invocable<void(absl::string_view)> on_save_callback;
  };

  void DrawTypeFilter();
  void DrawIsfPopupMenu(absl::string_view resource_path);

  std::string AddResource(PendingResource pending_resource);

  void PrepareMaterialHandleDragAndDrop(const std::string& resource);

  void LoadDraggedMaterial();

  void SetAsDefaultDragAndDropSource(std::string resource_path);

  // Handles drag-and-drop from other parts of the editor into the library.
  void HandleDragAndDropNewResource();

  void CreateDerivedIsf(absl::string_view base_isf_path);

  BaseView& view_;

  // Fields used to manage the types of assets the library can display.
  StringViewMap<DragAndDropType> resource_types_;
  StringSet message_types_;

  // Fields used to manage assets created at runtime.
  //
  // The DynamicResource is stored as a unique_ptr because it's important that
  // the resources memory is pointer stable. It must be pointer stable because
  // when the resource is registered with the ResourceManager it references the
  // data stored here. If the pointer changes, the ResourceManager may be
  // pointing to invalid memory.
  //
  // TODO: This solution works, but is brittle. It would be less
  // likely to break if we moved the concept of "dynamic resources" from
  // AssetLibrary to ResourceManager so that the memory is owned in the same
  // place that the resource is registered.
  StringMap<std::unique_ptr<DynamicResource>> dynamic_resources_;
  std::optional<PendingResource> pending_new_resource_;

  // Used to filter the assets shown in the library by a string.
  ImGuiTextFilter filter_;

  // Used to filter the assets shown in the library by extension.
  StringSet filtered_extensions_;

  std::unique_ptr<DirectoryUi> directory_ui_;
  bool saving_to_disk_enabled_ = false;

  // Used to store the material handle that is being dragged out to be dropped
  // onto a material slot. So that the material can be preloaded and not
  // released until the drop is complete.
  std::string dragged_material_url_;
  Future<absl::Status> preload_dragged_material_future_;

  Rememberer rememberer_;
  MaterialRegistry& material_registry_;

  std::optional<std::string> selected_asset_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_ASSET_LIBRARY_H_
