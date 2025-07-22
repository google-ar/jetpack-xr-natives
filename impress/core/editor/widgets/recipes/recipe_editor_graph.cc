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

#include "core/editor/widgets/recipes/recipe_editor_graph.h"

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/variant.h"
#include "core/common/robin_map.h"
#include "core/editor/widgets/recipes/editor_constants.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_runtime_graph.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp::editor {

namespace {

using NodeMap = RobinMap<RecipeEditorGraph::NodeId, RecipeEditorGraph::Node>;

class RecipeEditorGraphBuilder {
 public:
  RecipeEditorGraphBuilder() { graph_ = std::make_unique<RecipeEditorGraph>(); }

  void AddNode(const RecipeNode& node);

  void PopulateNode(const RecipeNode& node);

  std::unique_ptr<RecipeEditorGraph> Build(
      RecipeRuntimeGraph& recipe_runtime_graph);

 private:
  struct ValueConnectionVisitor {
    RecipeEditorGraphBuilder& builder_;
    RecipeEditorGraph::Node& node_;
    std::optional<absl::string_view> socket_name_;

    template <typename T>
    void operator()(const T& value) {
      builder_.PopulateValueConnection(value, node_, socket_name_);
    }

    void operator()(const absl::monostate& monostate) {}
  };

  struct RecipeNodeVisitor {
    RecipeEditorGraphBuilder& builder_;
    RecipeEditorGraph::Node& node_;

    template <typename T>
    void operator()(const T& node) {
      builder_.PopulateNode(node, node_);
    }

    template <>
    void operator()(const absl::monostate& node) {}
  };

  template <typename T>
  void PopulateValueConnection(
      const T& connection, RecipeEditorGraph::Node& graph_node,
      std::optional<absl::string_view> socket_name = std::nullopt);

  void PopulateExecutableConnection(const ExecutableNodeConnection& connection,
                                    absl::string_view out_socket_name,
                                    absl::string_view in_socket_name,
                                    RecipeEditorGraph::Node& graph_node);

  template <typename T>
  void PopulateNode(const T& node, RecipeEditorGraph::Node& graph_node);

  RecipeEditorGraph::PinId GetOrCreateSocket(
      RecipeEditorGraph::Node& node, absl::string_view socket_name,
      RecipeEditorGraph::ConnectionType type,
      RecipeEditorGraph::SocketKind kind);

  void CreateLink(RecipeEditorGraph::PinId start_pin_id,
                  RecipeEditorGraph::PinId end_pin_id,
                  RecipeEditorGraph::ConnectionType type);

  RecipeEditorGraph::NodeId GetNodeId(const RecipeNode& node);
  RecipeEditorGraph::PinId CreatePinId(const RecipeEditorGraph::Node& node);
  RecipeEditorGraph::LinkId CreateLinkId();

  NodeMap nodes_;
  std::unique_ptr<RecipeEditorGraph> graph_;
  std::vector<RecipeEditorGraph::NodeId> entry_point_node_ids_;
};

void RecipeEditorGraphBuilder::AddNode(const RecipeNode& node) {
  RecipeEditorGraph::NodeId node_id = GetNodeId(node);
  nodes_.emplace(node_id,
                 RecipeEditorGraph::Node{
                     .recipe_node = &node, .id = node_id, .name = node.name});
}

void RecipeEditorGraphBuilder::PopulateNode(const RecipeNode& node) {
  std::visit(RecipeNodeVisitor{*this, nodes_[GetNodeId(node)]}, node.node);
}

template <typename T>
void RecipeEditorGraphBuilder::PopulateValueConnection(
    const T& connection, RecipeEditorGraph::Node& graph_node,
    std::optional<absl::string_view> socket_name) {
  IMP_LOG(imp::ERROR) << "PopulateValueConnection does not support connection "
             << T::kTypeUrl;
}

template <>
void RecipeEditorGraphBuilder::PopulateValueConnection(
    const ValueConnection& connection, RecipeEditorGraph::Node& graph_node,
    std::optional<absl::string_view> socket_name) {
  std::visit(ValueConnectionVisitor{*this, graph_node, socket_name},
             connection.connection);
}

template <>
void RecipeEditorGraphBuilder::PopulateValueConnection(
    const Literal& connection, RecipeEditorGraph::Node& graph_node,
    std::optional<absl::string_view> socket_name) {
  std::string value = recipe::ToString(connection);
  if (socket_name.has_value()) {
    value = absl::StrFormat("%s : %s", *socket_name, value);
  }

  GetOrCreateSocket(graph_node, value, RecipeEditorGraph::ConnectionType::Value,
                    RecipeEditorGraph::SocketKind::Input);
}

template <>
void RecipeEditorGraphBuilder::PopulateValueConnection(
    const SocketConnection& connection, RecipeEditorGraph::Node& graph_node,
    std::optional<absl::string_view> socket_name) {
  graph_node.in_flows.push_back(nodes_[connection.node_id.index].id);

  RecipeEditorGraph::Node& input_node = nodes_[connection.node_id.index];
  RecipeEditorGraph::PinId input_pin_id =
      GetOrCreateSocket(graph_node, socket_name.value_or("In"),
                        RecipeEditorGraph::ConnectionType::Value,
                        RecipeEditorGraph::SocketKind::Input);
  RecipeEditorGraph::PinId output_pin_id =
      GetOrCreateSocket(input_node, connection.socket_name,
                        RecipeEditorGraph::ConnectionType::Value,
                        RecipeEditorGraph::SocketKind::Output);
  CreateLink(input_pin_id, output_pin_id,
             RecipeEditorGraph::ConnectionType::Value);
}

void RecipeEditorGraphBuilder::PopulateExecutableConnection(
    const ExecutableNodeConnection& connection,
    absl::string_view out_socket_name, absl::string_view in_socket_name,
    RecipeEditorGraph::Node& graph_node) {
  RecipeEditorGraph::PinId output_pin_id = GetOrCreateSocket(
      graph_node, out_socket_name, RecipeEditorGraph::ConnectionType::Flow,
      RecipeEditorGraph::SocketKind::Output);
  if (connection.node_id.has_value()) {
    graph_node.out_flows.push_back(nodes_[connection.node_id->index].id);

    RecipeEditorGraph::Node& next_node = nodes_[connection.node_id->index];
    RecipeEditorGraph::PinId input_pin_id = GetOrCreateSocket(
        next_node, in_socket_name, RecipeEditorGraph::ConnectionType::Flow,
        RecipeEditorGraph::SocketKind::Input);
    CreateLink(output_pin_id, input_pin_id,
               RecipeEditorGraph::ConnectionType::Flow);
  }
}

template <typename T>
void RecipeEditorGraphBuilder::PopulateNode(
    const T& node, RecipeEditorGraph::Node& graph_node) {
  IMP_LOG(imp::WARNING) << "RecipeEditorGraph does not support populating node "
               << T::kTypeUrl;
}

template <>
void RecipeEditorGraphBuilder::PopulateNode(
    const Identifier& node, RecipeEditorGraph::Node& graph_node) {
  graph_node.type_name = "Identifier";
  graph_node.content = node.name;
}

template <>
void RecipeEditorGraphBuilder::PopulateNode(
    const BinaryExpression& node, RecipeEditorGraph::Node& graph_node) {
  graph_node.type_name = "Binary Expression";

  graph_node.content =
      proto::EnumMetaData<BinaryExpression::BinaryOps>::GetName(node.op);

  PopulateValueConnection(node.left, graph_node, "Left");
  PopulateValueConnection(node.right, graph_node, "Right");
}

template <>
void RecipeEditorGraphBuilder::PopulateNode(
    const UnaryExpression& node, RecipeEditorGraph::Node& graph_node) {
  graph_node.type_name = "Unary Expression";

  graph_node.content =
      proto::EnumMetaData<UnaryExpression::UnaryOps>::GetName(node.op);

  PopulateValueConnection(node.input, graph_node, "Value");
}

template <>
void RecipeEditorGraphBuilder::PopulateNode(
    const CallExpression& node, RecipeEditorGraph::Node& graph_node) {
  graph_node.type_name = "Call Expression";
  graph_node.content = node.name;

  int i = 0;
  for (const ValueConnection& connection : node.args) {
    PopulateValueConnection(connection, graph_node,
                            absl::StrFormat("Arg%d", i++));
  }
}

template <>
void RecipeEditorGraphBuilder::PopulateNode(
    const CallStatement& node, RecipeEditorGraph::Node& graph_node) {
  graph_node.type_name = "Call Statement";
  graph_node.content = node.expression.name;

  for (const ValueConnection& connection : node.expression.args) {
    PopulateValueConnection(connection, graph_node);
  }

  PopulateExecutableConnection(node.next_node, "Next", "In", graph_node);
}

template <>
void RecipeEditorGraphBuilder::PopulateNode(
    const AsyncCallStatement& node, RecipeEditorGraph::Node& graph_node) {
  graph_node.type_name = "Async Call Statement";
  graph_node.content = node.expression.name;

  for (const ValueConnection& connection : node.expression.args) {
    PopulateValueConnection(connection, graph_node);
  }

  PopulateExecutableConnection(node.next_node, "Next", "In", graph_node);
  PopulateExecutableConnection(node.on_done_node, "Done", "In", graph_node);
}

template <>
void RecipeEditorGraphBuilder::PopulateNode(
    const AssignmentStatement& node, RecipeEditorGraph::Node& graph_node) {
  graph_node.type_name = "Assignment Statement";

  if (node.targets.empty()) {
    graph_node.content = "unknown";
  } else {
    // TODO: Support multiple targets.
    graph_node.content = absl::StrFormat(
        "%s : %s",
        proto::EnumMetaData<AssignmentStatement::AssignmentOps>::GetName(
            node.op),
        node.targets[0].name);
  }

  PopulateValueConnection(node.value, graph_node, "Value");
  PopulateExecutableConnection(node.next_node, "Next", "In", graph_node);
}

template <>
void RecipeEditorGraphBuilder::PopulateNode(
    const BranchStatement& node, RecipeEditorGraph::Node& graph_node) {
  graph_node.type_name = "Branch Statement";

  PopulateValueConnection(node.test, graph_node, "Test");
  PopulateExecutableConnection(node.true_next_node, "True", "In", graph_node);
  PopulateExecutableConnection(node.false_next_node, "False", "In", graph_node);
}

template <>
void RecipeEditorGraphBuilder::PopulateNode(
    const VariableDeclarationStatement& node,
    RecipeEditorGraph::Node& graph_node) {
  graph_node.type_name = "Variable Declaration";
  graph_node.content =
      absl::StrFormat("%s\n%s", node.declaration.name,
                      proto::EnumMetaData<VariableDeclaration::Type>::GetName(
                          node.declaration.type));

  if (node.declaration.init_value) {
    graph_node.content +=
        "\nInitial Value: " + recipe::ToString(*node.declaration.init_value);
  }

  PopulateExecutableConnection(node.next_node, "Next", "In", graph_node);
}

template <>
void RecipeEditorGraphBuilder::PopulateNode(
    const LoopStatement& node, RecipeEditorGraph::Node& graph_node) {
  graph_node.type_name = "Loop Statement";

  PopulateValueConnection(node.start_index, graph_node, "StartIndex");
  PopulateValueConnection(node.end_index, graph_node, "EndIndex");
  PopulateValueConnection(node.increment, graph_node, "Increment");

  PopulateExecutableConnection(node.looped_node, "Loop", "In", graph_node);
  PopulateExecutableConnection(node.completed_node, "Completed", "In",
                               graph_node);
}

template <>
void RecipeEditorGraphBuilder::PopulateNode(
    const SequenceStatement& node, RecipeEditorGraph::Node& graph_node) {
  graph_node.type_name = "Sequence Statement";

  for (int i = 0; i < node.next_nodes.size(); i++) {
    PopulateExecutableConnection(
        node.next_nodes[i], absl::StrFormat("Out %d", i), "In", graph_node);
  }
}

template <>
void RecipeEditorGraphBuilder::PopulateNode(
    const EventTrigger& node, RecipeEditorGraph::Node& graph_node) {
  graph_node.type_name = "Event Trigger";
  graph_node.content = node.event_name;

  for (auto& [arg_name, value] : node.args) {
    PopulateValueConnection(value, graph_node, arg_name);
  }

  PopulateExecutableConnection(node.next_node, "Next", "In", graph_node);
}

template <>
void RecipeEditorGraphBuilder::PopulateNode(
    const EventNode& node, RecipeEditorGraph::Node& graph_node) {
  graph_node.type_name = absl::StrFormat("Event: %s", node.event_name);

  PopulateExecutableConnection(node.next_node, "Next", "In", graph_node);

  entry_point_node_ids_.push_back(graph_node.id);
}

template <>
void RecipeEditorGraphBuilder::PopulateNode(
    const ExecutableNode& node, RecipeEditorGraph::Node& graph_node) {
  std::visit(RecipeNodeVisitor{*this, graph_node}, node.statement);
}

template <>
void RecipeEditorGraphBuilder::PopulateNode(
    const ValueNode& node, RecipeEditorGraph::Node& graph_node) {
  std::visit(RecipeNodeVisitor{*this, graph_node}, node.value);
}

RecipeEditorGraph::PinId RecipeEditorGraphBuilder::GetOrCreateSocket(
    RecipeEditorGraph::Node& node, absl::string_view socket_name,
    RecipeEditorGraph::ConnectionType type,
    RecipeEditorGraph::SocketKind kind) {
  auto it = node.sockets.find(socket_name);
  if (it != node.sockets.end()) {
    return it->second.id;
  }

  RecipeEditorGraph::PinId pin_id = CreatePinId(node);
  node.sockets.emplace(
      socket_name, RecipeEditorGraph::Socket{.id = pin_id,
                                             .name = std::string(socket_name),
                                             .type = type,
                                             .kind = kind});
  return pin_id;
}

void RecipeEditorGraphBuilder::CreateLink(
    RecipeEditorGraph::PinId start_pin_id, RecipeEditorGraph::PinId end_pin_id,
    RecipeEditorGraph::ConnectionType type) {
  graph_->links.emplace_back(
      RecipeEditorGraph::Link{.id = CreateLinkId(),
                              .start_socket_id = start_pin_id,
                              .end_socket_id = end_pin_id,
                              .type = type});
}

RecipeEditorGraph::NodeId RecipeEditorGraphBuilder::GetNodeId(
    const RecipeNode& node) {
  return RecipeEditorGraph::NodeId(node.id.index);
}

RecipeEditorGraph::PinId RecipeEditorGraphBuilder::CreatePinId(
    const RecipeEditorGraph::Node& node) {
  return RecipeEditorGraph::PinId(node.id * kMaxSocketsPerNode +
                                  node.sockets.size());
}

RecipeEditorGraph::LinkId RecipeEditorGraphBuilder::CreateLinkId() {
  return RecipeEditorGraph::LinkId(graph_->links.size());
}

std::unique_ptr<RecipeEditorGraph> RecipeEditorGraphBuilder::Build(
    RecipeRuntimeGraph& recipe_runtime_graph) {
  for (const auto& [id, node] : nodes_) {
    graph_->node_lookup[node.recipe_node->id] = id;
  }

  graph_->nodes = nodes_;
  graph_->entry_point_node_ids = entry_point_node_ids_;

  return std::move(graph_);
}

}  // namespace

absl::StatusOr<std::unique_ptr<RecipeEditorGraph>> RecipeEditorGraph::Create(
    RecipeRuntimeGraph& recipe_runtime_graph) {
  RecipeEditorGraphBuilder builder;
  const RecipeGraph& recipe_graph = recipe_runtime_graph.GetRecipeGraph();

  for (const RecipeNode& node : recipe_graph.recipe_nodes) {
    builder.AddNode(node);
  }

  for (const RecipeNode& node : recipe_graph.recipe_nodes) {
    builder.PopulateNode(node);
  }

  std::unique_ptr<RecipeEditorGraph> recipe_editor_graph =
      builder.Build(recipe_runtime_graph);

  recipe_runtime_graph.SetSocketValueListener(
      [editor_graph = recipe_editor_graph.get()](
          const imp::NodeId& recipe_node_id,
          const recipe::Variables& return_values) {
        auto it = editor_graph->node_lookup.find(recipe_node_id);
        if (it == editor_graph->node_lookup.end()) {
          IMP_LOG(imp::WARNING) << "Recipe node not found in graph: "
                       << recipe_node_id.index;
        } else {
          editor_graph->nodes[it->second].return_values = return_values;
        }
      });

  return recipe_editor_graph;
}

}  // namespace imp::editor
