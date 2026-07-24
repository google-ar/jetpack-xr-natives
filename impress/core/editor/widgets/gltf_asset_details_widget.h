/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_GLTF_ASSET_DETAILS_WIDGET_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_GLTF_ASSET_DETAILS_WIDGET_H_

#include <string>

#include "absl/strings/string_view.h"
#include "core/assets/gltf/gltf_asset.h"
#include "core/async/future.h"
#include "core/common/rememberer.h"
#include "core/editor/widget.h"
#include "core/editor/widgets/asset_library.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"

namespace imp {
namespace editor {

// A widget that displays detailed information about a selected GLTF/GLB asset.
//
// It determines the asset to show by first checking the current scene selection
// for nodes with a GltfRenderer or GltfMesh component. If no such node is
// selected, it falls back to the asset currently selected in the AssetLibrary.
// Currently, it supports displaying detailed statistics (vertices, triangles,
// materials, animations) for GLTF/GLB assets.
class GltfAssetDetailsWidget : public Widget, public imp::Rememberer {
 public:
  GltfAssetDetailsWidget(BaseView& view, AssetLibrary* asset_library);
  ~GltfAssetDetailsWidget() override = default;

  absl::string_view GetName() const override;
  void DrawImGui() override;
  std::optional<std::string> GetAssetToShow();

 private:
  enum class LastSelectionType {
    kNone,
    kNode,
    kAssetLibrary,
  };

  std::optional<std::string> GetValidLibraryAsset() const;
  void DrawGltfDetails(absl::string_view asset_path);

  BaseView& view_;
  AssetLibrary* asset_library_;
  std::string active_asset_path_;
  Future<AssetPtr<GltfAsset>> gltf_asset_future_;

  LastSelectionType last_selection_type_ = LastSelectionType::kNone;
  std::optional<std::string> last_selected_asset_library_asset_;
  std::string current_source_label_;
  NodeHandle last_selected_gltf_scene_node_;
};

}  // namespace editor
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_GLTF_ASSET_DETAILS_WIDGET_H_
