// Copyright 2026 Google LLC
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

#include "core/editor/widgets/gltf_asset_details_widget.h"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/strings/match.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/gltf/gltf_asset.h"
#include "core/async/future.h"
#include "core/common/typed_set_vector.h"
#include "core/editor/editor.h"
#include "core/editor/events.h"
#include "core/model/entity_data.h"
#include "core/model/model_data.h"
#include "core/ncsb/component_handle.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/assets/gltf_renderer.h"

namespace imp {
namespace editor {

GltfAssetDetailsWidget::GltfAssetDetailsWidget(BaseView& view,
                                               AssetLibrary* asset_library)
    : view_(view), asset_library_(asset_library) {
  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  editor.GetDispatcher().Connect(
      [this, &editor](const editor::NodeSelectionChangedEvent& event) mutable {
        // When the scene selection changes, we must determine if a glTF node is
        // part of the active selection. If so, we prioritize displaying its
        // details here. We track `last_selected_gltf_scene_node_` to maintain
        // focus on the same glTF node during a multi-select operation (e.g.
        // ctrl-clicking a new node), preventing the widget from abruptly
        // switching or clearing its contents.
        const absl::flat_hash_set<NodeHandle>& selected_nodes =
            editor.GetSelectedNodes();
        if (!selected_nodes.empty()) {
          last_selection_type_ = LastSelectionType::kNode;

          // If the previously tracked glTF node is no longer in the selection,
          // scan the new selection set for any node that contains glTF
          // components.
          if (!last_selected_gltf_scene_node_.IsValid() ||
              !selected_nodes.contains(last_selected_gltf_scene_node_)) {
            last_selected_gltf_scene_node_ = NodeHandle();
            for (NodeHandle node : selected_nodes) {
              if (node->GetComponent<GltfRenderer>() ||
                  node->GetComponent<GltfMesh>()) {
                last_selected_gltf_scene_node_ = node;
                break;
              }
            }
          }

          // If a glTF node is selected in the scene, ensure we deselect any
          // active asset in the AssetLibrary to avoid conflicting UI states.
          if (last_selected_gltf_scene_node_.IsValid()) {
            if (asset_library_) {
              asset_library_->SetSelectedAsset(std::nullopt);
              last_selected_asset_library_asset_ = std::nullopt;
            }
          }
        } else {
          last_selected_gltf_scene_node_ = NodeHandle();
        }
      },
      this);
}

absl::string_view GltfAssetDetailsWidget::GetName() const {
  return "GLTF Asset Details";
}

std::optional<std::string> GltfAssetDetailsWidget::GetValidLibraryAsset()
    const {
  if (asset_library_ && asset_library_->GetSelectedAsset().has_value()) {
    std::string asset = *asset_library_->GetSelectedAsset();
    if (absl::EndsWith(asset, ".gltf") || absl::EndsWith(asset, ".glb")) {
      return asset;
    }
  }
  return std::nullopt;
}

std::optional<std::string> GltfAssetDetailsWidget::GetAssetToShow() {
  Editor& editor = view_.GetRegistry().Get<Editor>()->get();

  if (asset_library_) {
    std::optional<std::string> current_selected_asset =
        asset_library_->GetSelectedAsset();
    if (current_selected_asset != last_selected_asset_library_asset_) {
      last_selected_asset_library_asset_ = current_selected_asset;
      if (current_selected_asset.has_value()) {
        std::string asset = *current_selected_asset;
        if (absl::EndsWith(asset, ".gltf") || absl::EndsWith(asset, ".glb")) {
          last_selection_type_ = LastSelectionType::kAssetLibrary;
          if (last_selected_gltf_scene_node_.IsValid()) {
            editor.SelectNode(NodeHandle());
          }
        }
      }
    }
  }

  const absl::flat_hash_set<NodeHandle>& selected_nodes =
      editor.GetSelectedNodes();

  if (last_selection_type_ == LastSelectionType::kAssetLibrary) {
    if (std::optional<std::string> asset = GetValidLibraryAsset()) {
      current_source_label_ = "Asset Library";
      return asset;
    }
  }

  if (!selected_nodes.empty()) {
    NodeHandle selected_node = last_selected_gltf_scene_node_;
    if (!selected_node.IsValid() || !selected_nodes.contains(selected_node)) {
      selected_node = *selected_nodes.begin();
    }

    // Try to find GltfRenderer
    ComponentHandle<GltfRenderer> renderer =
        selected_node->GetComponent<GltfRenderer>();
    if (!renderer) {
      // Try GltfMesh
      ComponentHandle<GltfMesh> mesh = selected_node->GetComponent<GltfMesh>();
      if (mesh) {
        renderer = mesh->GetGltfRenderer();
      }
    }

    if (renderer) {
      if (renderer->GetGltfAsset()) {
        active_asset_path_ = std::string(renderer->GetAssetUrl());
        gltf_asset_future_ =
            Future<AssetPtr<GltfAsset>>(renderer->GetGltfAsset());
      }
      current_source_label_ = "Scene Node";
      return std::string(renderer->GetAssetUrl());
    }
  }

  // Fallback to AssetLibrary if no scene selection, last_selection_type_
  // was not kAssetLibrary, or scene selection was not a GLTF asset.
  if (std::optional<std::string> asset = GetValidLibraryAsset()) {
    current_source_label_ = "Asset Library";
    return asset;
  }
  current_source_label_ = "";
  return std::nullopt;
}

void GltfAssetDetailsWidget::DrawImGui() {
  const std::optional<std::string> asset_to_show = GetAssetToShow();

  if (!asset_to_show) {
    ImGui::Text("No GLTF Asset/Node Selected");
    return;
  }

  ImGui::Text("Source: %s", current_source_label_.c_str());
  ImGui::Text("Asset: %s", asset_to_show->c_str());

  // Currently we only support GLTF detailed stats.
  if (absl::EndsWith(*asset_to_show, ".gltf") ||
      absl::EndsWith(*asset_to_show, ".glb")) {
    DrawGltfDetails(*asset_to_show);
  } else {
    ImGui::Text("Detailed info not available for this asset type.");
  }
}

void GltfAssetDetailsWidget::DrawGltfDetails(absl::string_view asset_path) {
  if (active_asset_path_ != asset_path) {
    active_asset_path_ = std::string(asset_path);
    gltf_asset_future_ = view_.GetAssetManager().LoadGltfAsset(asset_path);
  }

  if (!gltf_asset_future_.Ready()) {
    ImGui::Text("Loading GLTF details...");
    return;
  }

  const absl::StatusOr<AssetPtr<GltfAsset>>& result = gltf_asset_future_.Get();
  if (!result.ok()) {
    ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f),
                       "Failed to load GLTF asset.");
    return;
  }
  AssetPtr<GltfAsset> asset = *result;

  if (!asset) {
    ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f),
                       "Failed to load GLTF asset.");
    return;
  }

  ImGui::Separator();
  ImGui::Text("GLTF Model Details");
  ImGui::Separator();

  // Basic stats
  const model::ModelData& model_data = asset->GetModelData();

  // Calculate vertices and triangles
  size_t vertex_count = 0;
  for (const filament::VertexBuffer* vb : model_data.VertexBuffers()) {
    vertex_count += vb->getVertexCount();
  }

  size_t triangle_count = 0;
  const TypedSetVector<model::EntityData>& entities = model_data.Entities();
  for (model::ModelData::EntityId entity_id :
       entities.Ids<model::ModelData::EntityId>()) {
    const model::EntityData::Proxy entity_data = entities[entity_id];
    const std::vector<model::ModelData::PartData>& parts = entity_data.parts;
    for (const model::ModelData::PartData& part : parts) {
      if (part.primitive_type ==
          filament::RenderableManager::PrimitiveType::TRIANGLES) {
        triangle_count += part.index_count / 3;
      }
    }
  }

  ImGui::Text("Vertices: %zu", vertex_count);
  ImGui::Text("Triangles: %zu", triangle_count);
  ImGui::Text("Materials: %zu", model_data.Materials().size());
  ImGui::Text("Meshes: %zu", model_data.Entities().size());

  // Animations
  size_t anim_count = asset->AnimationCount();
  ImGui::Text("Animations: %zu", anim_count);

  if (anim_count > 0 && ImGui::CollapsingHeader("Animation List")) {
    for (size_t i = 0; i < anim_count; ++i) {
      ImGui::BulletText("%s", std::string(asset->GetAnimNames()[i]).c_str());
    }
  }
}

}  // namespace editor
}  // namespace imp
