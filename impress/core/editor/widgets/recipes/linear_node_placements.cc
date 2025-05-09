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

#include "core/editor/widgets/recipes/linear_node_placements.h"

#include "dear_imgui/imgui.h"
#include "third_party/imgui_node_editor/imgui_node_editor.h"
#include "core/common/robin_map.h"
#include "core/editor/widgets/recipes/recipe_editor_graph.h"
#include "core/math/vec.h"

namespace imp::editor::recipe_internal {

namespace {

constexpr float2 kSpacing = {10, 10};

}

RobinMap<int, float2> LinearNodePlacements::GeneratePlacements(
    const RecipeEditorGraph& recipe_editor_graph) {
  RobinMap<int, float2> placements;

  float2 current_position = {0, 0};
  for (auto& [id, _] : recipe_editor_graph.nodes) {
    placements[id] = current_position;
    ImVec2 node_size = ax::NodeEditor::GetNodeSize(ax::NodeEditor::NodeId(id));
    current_position.x += node_size.x + kSpacing.x;
    current_position.y += node_size.y + kSpacing.y;
  }

  return placements;
}

}  // namespace imp::editor::recipe_internal
