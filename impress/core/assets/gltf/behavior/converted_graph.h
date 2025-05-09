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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_BEHAVIOR_CONVERTED_GRAPH_H_
#define THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_BEHAVIOR_CONVERTED_GRAPH_H_

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/model/model_data.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "robin_map/include/tsl/robin_map.h"

namespace imp {

namespace gltf::behavior {

// ConvertedGraph keeps track of all the RecipeNodes converted from a behavior
// graph and is also responsible for generating RecipeNodes with NodeIds that
// are unique within the converted graphs.
//
// TODO: Explore the option of changing ConvertedGraph to
// GraphConverter.
class ConvertedGraph {
 public:
  // GlobalNodes is a collection of RecipeNodes that are not converted from
  // behavior graph and can be referenced by the other nodes when converting the
  // graph.
  //
  // TODO: Modify GlobalNodes so that nodes are lazily created.
  struct GlobalNodes {
    GlobalNodes(ConvertedGraph& converted_graph);
    // Identifier to NodeSelf variable.
    RecipeNode& self_node;
    // Recipe node for OnTapEvent.
    RecipeNode& on_tap_event_node;
    // Recipe node for the sequence node connected to `on_tap_event_node`.
    RecipeNode& on_tap_sequence_node;
  };

  // Creates a ConvertedGraph from the provided BehaviorData.
  // Each behavior node in the BehaviorData will be converted into a RecipeNode
  // with unique NodeIds.
  ConvertedGraph(const model::ModelData::BehaviorData& behavior_data);

  // Creates a RecipeNode with NodeId populated.
  RecipeNode& CreateNode();

  // Returns a corresponding RecipeNode by behavior node index. The node has to
  // be pre-registered within the constructor of ConvertedGraph to be valid.
  // Otherwise an error will be returned.
  absl::StatusOr<std::reference_wrapper<RecipeNode>> GetNode(
      int behavior_node_index) const;

  // Returns the AuxiliaryNodes associated with this ConvertedGraph.
  const GlobalNodes& GetGlobalNodes() const { return global_nodes_; }

  // Returns a copy of all the RecipeNodes within the ConvertedGraph. This
  // includes the auxiliary nodes, all the nodes generated through CreateNode()
  // and the converted behavior nodes.
  std::vector<RecipeNode> GetRecipeNodes() const;

  // Returns the event info referenced by the behavior custom event index.
  absl::StatusOr<model::ModelData::BehaviorData::CustomEventData> GetEvent(
      int behavior_event_index) const;

  void RegisterTapNode(int gltf_node_index);

  std::vector<int> GetTapNodeIndices() const { return tap_node_indices_; }

 private:
  absl::Status RegisterBehaviorNode(int behavior_node_index);

  // Maps behavior node index to its corresponding main RecipeNode.
  tsl::robin_map<int, std::reference_wrapper<RecipeNode>> map_;
  // Owns the memory of all RecipeNodes in the converted graph.
  std::vector<std::unique_ptr<RecipeNode>> recipe_nodes_;
  std::vector<model::ModelData::BehaviorData::CustomEventData> custom_events_;

  std::vector<int> tap_node_indices_;

  GlobalNodes global_nodes_;
};

}  // namespace gltf::behavior

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_BEHAVIOR_CONVERTED_GRAPH_H_
