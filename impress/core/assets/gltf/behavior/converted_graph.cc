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

#include "core/assets/gltf/behavior/converted_graph.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "core/common/platform_helpers.h"
#include "core/model/model_data.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp {

namespace gltf::behavior {

constexpr absl::string_view kOnTapSequenceNodeName = "On tap sequence node";

ConvertedGraph::GlobalNodes::GlobalNodes(ConvertedGraph& converted_graph)
    : self_node(converted_graph.CreateNode()),
      on_tap_event_node(converted_graph.CreateNode()),
      on_tap_sequence_node(converted_graph.CreateNode()) {
  self_node.node = ValueNode{
      .value = Identifier{.name = std::string(recipe::kNodeSelfVariableName)}};
  self_node.name = std::string(recipe::kNodeSelfVariableName);

  on_tap_event_node.node = EventNode{
      .event_name = std::string(recipe::kOnTapEventName),
      .next_node =
          ExecutableNodeConnection{.node_id = on_tap_sequence_node.id}};
  on_tap_event_node.name = std::string(recipe::kOnTapEventName);

  on_tap_sequence_node.node = ExecutableNode{.statement = SequenceStatement{}};
  on_tap_sequence_node.name = std::string(kOnTapSequenceNodeName);
}

ConvertedGraph::ConvertedGraph(
    const model::ModelData::BehaviorData& behavior_data)
    : global_nodes_(*this) {
  // Create empty recipe nodes that corresponds to behavior nodes.
  for (const auto& node_data : behavior_data.nodes) {
    absl::Status register_status = RegisterBehaviorNode(node_data.index);
    if (!register_status.ok()) {
      IMP_LOG(imp::FATAL) << "Failed to create ConvertedGraph: " << register_status;
    }
  }

  custom_events_.resize(behavior_data.custom_events.size());
  for (int i = 0; i < behavior_data.custom_events.size(); i++) {
    custom_events_[i] = behavior_data.custom_events[i];
  }
}

absl::StatusOr<std::reference_wrapper<RecipeNode>> ConvertedGraph::GetNode(
    int behavior_node_index) const {
  auto it = map_.find(behavior_node_index);
  if (it == map_.end()) {
    return absl::NotFoundError(absl::StrFormat(
        "NodeID associated with index %d not found", behavior_node_index));
  }
  return it.value();
}

absl::Status ConvertedGraph::RegisterBehaviorNode(int behavior_node_index) {
  auto [_, inserted] = map_.insert({behavior_node_index, CreateNode()});
  if (!inserted) {
    return absl::AlreadyExistsError(absl::StrFormat(
        "NodeID associated with index %d already exists", behavior_node_index));
  }

  return absl::OkStatus();
}

std::vector<RecipeNode> ConvertedGraph::GetRecipeNodes() const {
  std::vector<RecipeNode> recipe_nodes;

  recipe_nodes.reserve(recipe_nodes_.size());
  for (const auto& recipe_node : recipe_nodes_) {
    recipe_nodes.push_back(*recipe_node);
  }

  return recipe_nodes;
}

RecipeNode& ConvertedGraph::CreateNode() {
  auto node = std::make_unique<RecipeNode>();
  node->id = NodeId{.index = static_cast<uint32_t>(recipe_nodes_.size())};
  recipe_nodes_.push_back(std::move(node));
  return *recipe_nodes_.back();
}

absl::StatusOr<model::ModelData::BehaviorData::CustomEventData>
ConvertedGraph::GetEvent(int behavior_event_index) const {
  if (behavior_event_index >= custom_events_.size()) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Index %d provided in GetEvent is invalid.", behavior_event_index));
  }
  return custom_events_[behavior_event_index];
}

void ConvertedGraph::RegisterTapNode(int gltf_node_index) {
  tap_node_indices_.push_back(gltf_node_index);
}

}  // namespace gltf::behavior

}  // namespace imp
