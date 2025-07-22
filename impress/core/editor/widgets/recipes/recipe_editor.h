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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_RECIPES_RECIPE_EDITOR_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_RECIPES_RECIPE_EDITOR_H_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "third_party/imgui_node_editor/imgui_node_editor.h"
#include "core/async/future.h"
#include "core/common/rememberer.h"
#include "core/editor/widget.h"
#include "core/editor/widgets/recipes/recipe_editor_graph.h"
#include "core/ncsb/component_handle.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_runtime_graph.h"
#include "core/recipes/recipe_runner.h"
#include "core/view/base_view.h"

namespace imp::editor {

// Window at the bottom of the screen showing Impress info, warning, and error
// logs. Useful for identifying problems with the model parsing and loading.
class RecipeEditor : public Widget, public imp::Rememberer {
 public:
  explicit RecipeEditor(BaseView& view);
  ~RecipeEditor() override;

  absl::string_view GetName() const override { return "Node Editor"; }
  void DrawImGui() override;
  bool HasContent() const override;

 private:
  absl::Status LoadRecipeGraph(RecipeRuntimeGraph& graph);
  void ClearCurrentGraph();
  void DrawWindow();

  // Checks if the user has attempted to modify the graph inside the node editor
  // tab, and if so, updates only the element that was created/deleted
  void CheckForGraphUpdates();
  void DrawGraph();
  void DrawRecipeVariables();
  void DrawSearchBar();
  void DrawCreateNodePopup();

  void DrawNode(const RecipeEditorGraph::Node& node);
  void DrawLink(const RecipeEditorGraph::Link& link);
  void DrawSocket(const RecipeEditorGraph::Socket& socket, bool connected);

  RecipeEditorGraph::Node GetEditorNode(const RecipeNode& node);

  RecipeEditorGraph::NodeId ConvertId(ax::NodeEditor::NodeId id);
  RecipeEditorGraph::PinId ConvertId(ax::NodeEditor::PinId id);
  RecipeEditorGraph::LinkId ConvertId(ax::NodeEditor::LinkId id);

  BaseView& view_;
  bool first_frame_ = true;
  std::unique_ptr<RecipeEditorGraph> graph_;
  ax::NodeEditor::EditorContext* context_ = nullptr;
  Future<absl::Status> graph_loading_future_;
  ComponentHandle<RecipeRunner> recipe_runner_;

  std::string search_buffer_;
  bool search_selection_changed_ = false;
  // According to ImGui documentation, OpenPopup should only be called once, not
  // on every frame. So we achieve this through create_node_popup_open_.
  bool create_node_popup_open_ = false;
  size_t search_result_index_ = 0;
  std::vector<RecipeEditorGraph::NodeId> search_results_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_RECIPES_RECIPE_EDITOR_H_
