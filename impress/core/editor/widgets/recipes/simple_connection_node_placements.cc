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

#include "core/editor/widgets/recipes/simple_connection_node_placements.h"

#include <algorithm>

#include "core/common/log.h"
#include "dear_imgui/imgui.h"
#include "third_party/imgui_node_editor/imgui_node_editor.h"
#include "core/common/robin_map.h"
#include "core/editor/widgets/recipes/recipe_editor_graph.h"
#include "core/math/vec.h"

namespace imp::editor::recipe_internal {

namespace {

constexpr float2 kSpacing = {10, 10};

float2 PlaceNodes(const RecipeEditorGraph& recipe_editor_graph,
                  const RecipeEditorGraph::NodeId& node_id,
                  RobinMap<int, float2>& placements, float2 current_position) {
  auto it = recipe_editor_graph.nodes.find(node_id);
  if (it == recipe_editor_graph.nodes.end()) {
    IMP_LOG(imp::ERROR) << "Could not find node " << node_id << " in RecipeEditorGraph";
    return {0, 0};
  }

  const RecipeEditorGraph::Node& node = it->second;

  if (placements.contains(node_id)) {
    return {0, 0};
  }
  placements[node_id] = current_position;

  float original_y = current_position.y;
  float2 graph_size = {0, 0};

  ImVec2 node_size =
      ax::NodeEditor::GetNodeSize(ax::NodeEditor::NodeId(node_id));

  if (!node.in_flows.empty()) {
    // Place in flow graph.
    float2 in_flow_graph_size = {0, 0};

    // Shift the in flow graph down and expand the in flow graph size.
    current_position.y += node_size.y;
    in_flow_graph_size += node_size.y;

    for (const RecipeEditorGraph::NodeId& in_flow : node.in_flows) {
      float2 sub_graph_size = PlaceNodes(recipe_editor_graph, in_flow,
                                         placements, current_position);

      // In flow graph width should equal the width of the widest sub graph.
      in_flow_graph_size.x = std::max(in_flow_graph_size.y, sub_graph_size.x);

      current_position.y += sub_graph_size.y + kSpacing.y;
      in_flow_graph_size.y += sub_graph_size.y + kSpacing.y;
    }

    current_position.x += in_flow_graph_size.x + kSpacing.x;
    in_flow_graph_size.x += kSpacing.x;

    graph_size.x += in_flow_graph_size.x;
    graph_size.y += in_flow_graph_size.y;
  }

  // Place the node.
  // Shift the cursor back up to the original y position.
  current_position.y = original_y;
  placements[node.id] = current_position;
  current_position.x += node_size.x;

  graph_size.x += node_size.x;
  graph_size.y = std::max(graph_size.y, node_size.y);

  if (!node.out_flows.empty()) {
    current_position.x += kSpacing.x;
    graph_size.x += kSpacing.x;

    // Place out_flows.
    float2 out_flow_graph_size = {0, 0};

    for (const RecipeEditorGraph::NodeId& out_flow : node.out_flows) {
      float2 sub_graph_size = PlaceNodes(recipe_editor_graph, out_flow,
                                         placements, current_position);

      // Out flow graph width should equal the width of the widest sub graph.
      out_flow_graph_size.x = std::max(out_flow_graph_size.x, sub_graph_size.x);

      current_position.y += sub_graph_size.y + kSpacing.y;
      out_flow_graph_size.y += sub_graph_size.y + kSpacing.y;
    }

    graph_size.x += out_flow_graph_size.x;
    graph_size.y = std::max(graph_size.y, out_flow_graph_size.y);
  }

  return graph_size;
}

}  // namespace

RobinMap<int, float2> SimpleConnectionNodePlacements::GeneratePlacements(
    const RecipeEditorGraph& recipe_editor_graph) {
  RobinMap<int, float2> placements;

  float2 current_position = {0, 0};
  for (const RecipeEditorGraph::NodeId& id :
       recipe_editor_graph.entry_point_node_ids) {
    float2 size =
        PlaceNodes(recipe_editor_graph, id, placements, current_position);
    current_position.y += size.y + kSpacing.y;
  }

  for (auto& [id, node] : recipe_editor_graph.nodes) {
    auto it = placements.find(id);
    if (it == placements.end()) {
      IMP_LOG(imp::WARNING) << "Found node that is not connected to any entry point: "
                   << id;

      float2 size =
          PlaceNodes(recipe_editor_graph, id, placements, current_position);
      current_position.y += size.y + kSpacing.y;
    }
  }

  return placements;
}

}  // namespace imp::editor::recipe_internal
