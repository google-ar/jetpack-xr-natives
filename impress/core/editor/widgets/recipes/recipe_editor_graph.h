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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_RECIPES_RECIPE_EDITOR_GRAPH_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_RECIPES_RECIPE_EDITOR_GRAPH_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "core/common/robin_map.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_runtime_graph.h"
#include "core/recipes/language/recipe_utils.h"
#include "core/view/utils/string_map.h"

namespace imp::editor {

// A representation of RecipeGraph more optimally formatted for visualization
// purposes.
struct RecipeEditorGraph {
  using ReicpeNodeId = imp::NodeId;
  using NodeId = int64_t;
  using PinId = int64_t;
  using LinkId = int64_t;

  enum class ConnectionType { Flow, Value };
  enum class SocketKind { Output, Input };

  // TODO : Store and visualize socket values in place.
  struct Socket {
    PinId id;
    std::string name;
    ConnectionType type;
    SocketKind kind;
  };

  struct Node {
    const RecipeNode* recipe_node;
    NodeId id;
    std::string type_name;
    std::string name;
    std::string content;
    StringMap<Socket> sockets;

    std::vector<NodeId> in_flows;
    std::vector<NodeId> out_flows;

    recipe::Variables return_values;
  };

  struct Link {
    LinkId id;
    PinId start_socket_id;
    PinId end_socket_id;
    ConnectionType type;
  };

  static absl::StatusOr<std::unique_ptr<RecipeEditorGraph>> Create(
      RecipeRuntimeGraph& recipe_runtime_graph);

  RobinMap<NodeId, Node> nodes;
  std::vector<Link> links;
  std::vector<NodeId> entry_point_node_ids;
  RobinMap<ReicpeNodeId, NodeId, recipe::NodeIdHash, recipe::NodeIdEqual>
      node_lookup;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_RECIPES_RECIPE_EDITOR_GRAPH_H_
