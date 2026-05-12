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

#include "core/assets/gltf/behavior/node_converters.h"

#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/variant.h"
#include "core/assets/gltf/behavior/converted_graph.h"
#include "core/assets/gltf/behavior/node_converter_constants.h"
#include "core/assets/gltf/behavior/world_pointer.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/model/model_data.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::gltf::behavior {

using BehaviorData = model::ModelData::BehaviorData;

namespace {

absl::string_view GetRecipeSocketName(absl::string_view behavior_socket_name) {
  // TODO Remove "val" and "result" when Adobe removes them.
  if (behavior_socket_name == "val" || behavior_socket_name == "result" ||
      behavior_socket_name == kDefaultOutputValueSocket) {
    return recipe::kDefaultOutputSocketName;
  }

  if (behavior_socket_name == kDeltaTimeValueSocket) {
    return recipe::kDeltaSecondsSocketName;
  }

  if (behavior_socket_name == kElapsedTimeValueSocket) {
    return recipe::kElapsedSecondsSocketName;
  }

  return behavior_socket_name;
}

ValueConnection GetNodeSelfConnection(ConvertedGraph& converted_graph) {
  return ValueConnection{
      .connection = SocketConnection{
          .node_id = converted_graph.GetGlobalNodes().self_node.id,
          .socket_name = std::string(recipe::kDefaultOutputSocketName)}};
}

Literal ToLiteral(const BehaviorData::VariableData& value) {
  return Literal{
      .value = absl::ConvertVariantTo<decltype(Literal::value)>(value.value)};
}

absl::StatusOr<ValueConnection> ToValueConnection(
    const BehaviorData::NodeData::ValueData& value_data,
    const ConvertedGraph& converted_graph) {
  ValueConnection value_connection;
  if (std::holds_alternative<BehaviorData::NodeData::FlowData>(value_data)) {
    const BehaviorData::NodeData::FlowData& flow_data =
        std::get<BehaviorData::NodeData::FlowData>(value_data);
    MP_ASSIGN_OR_RETURN(RecipeNode & left_node,
                     converted_graph.GetNode(flow_data.node));

    value_connection.connection = SocketConnection{
        .node_id = left_node.id,
        .socket_name = std::string(GetRecipeSocketName(flow_data.socket))};
  } else if (std::holds_alternative<BehaviorData::VariableData>(value_data)) {
    value_connection.connection =
        ToLiteral(std::get<BehaviorData::VariableData>(value_data));
  } else {
    return absl::InvalidArgumentError("Invalid value flow.");
  }
  return value_connection;
}

std::string GetValueId(const BehaviorData::NodeData::ValueData& value_data) {
  struct Visitor {
    std::string operator()(const BehaviorData::NodeData::FlowData& value) {
      return value.id;
    }
    std::string operator()(const BehaviorData::VariableData& value) {
      return value.id;
    }
  };

  return std::visit(Visitor{}, value_data);
}

std::optional<BehaviorData::NodeData::ConfigurationData> FindConfiguration(
    BehaviorData::NodeData::ConfigurationType configuration_id,
    const std::vector<BehaviorData::NodeData::ConfigurationData>& configuration,
    const ConvertedGraph& converted_graph) {
  for (const auto& configuration : configuration) {
    if (configuration.id == configuration_id) {
      return configuration;
    }
  }
  return std::nullopt;
}

absl::StatusOr<ValueConnection> GetNodeConfigurationByType(
    BehaviorData::NodeData::ConfigurationType configuration_id,
    const BehaviorData::NodeData& node_data,
    const ConvertedGraph& converted_graph) {
  std::optional<BehaviorData::NodeData::ConfigurationData> config =
      FindConfiguration(configuration_id, node_data.configuration,
                        converted_graph);
  if (!config.has_value()) {
    return absl::NotFoundError(
        absl::StrFormat("Configuration %d not found for node index %d",
                        configuration_id, node_data.index));
  }

  return ValueConnection{
      .connection =
          Literal{.value = absl::ConvertVariantTo<decltype(Literal::value)>(
                      config->value)}};
}

absl::StatusOr<ValueConnection> GetNodeValueByName(
    absl::string_view name, const BehaviorData::NodeData& node_data,
    const ConvertedGraph& converted_graph) {
  for (const BehaviorData::NodeData::ValueData& value : node_data.values) {
    if (const BehaviorData::VariableData* val =
            std::get_if<BehaviorData::VariableData>(&value);
        val && val->id == name) {
      return ToValueConnection(value, converted_graph);
    } else if (const BehaviorData::NodeData::FlowData* flow =
                   std::get_if<BehaviorData::NodeData::FlowData>(&value);
               flow && flow->id == name) {
      return ToValueConnection(value, converted_graph);
    }
  }

  return absl::NotFoundError(absl::StrFormat(
      "Value %s not found for node index %d", name, node_data.index));
}

ValueConnection GetNodeValueByNameOr(absl::string_view name,
                                     const BehaviorData::NodeData& node_data,
                                     const ConvertedGraph& converted_graph,
                                     Literal default_value) {
  if (auto value = GetNodeValueByName(name, node_data, converted_graph);
      value.ok()) {
    return value.value();
  }

  return ValueConnection{.connection = default_value};
}

absl::StatusOr<ExecutableNodeConnection> GetNodeFlowByName(
    absl::string_view node_name, const BehaviorData::NodeData& node_data,
    const ConvertedGraph& converted_graph) {
  for (const auto& flow : node_data.flows) {
    if (flow.id == node_name) {
      MP_ASSIGN_OR_RETURN(RecipeNode & next_node,
                       converted_graph.GetNode(flow.node));
      return ExecutableNodeConnection{.node_id = next_node.id};
    }
  }

  return ExecutableNodeConnection{};
}

absl::Status ConvertBinaryExpressionAndAddtoGraph(
    const BehaviorData::NodeData& node_data, ConvertedGraph& converted_graph,
    BinaryExpression::BinaryOps op) {
  if (node_data.values.size() != 2) {
    return absl::InvalidArgumentError(
        "Expected 2 values for binary expression.");
  }
  MP_ASSIGN_OR_RETURN(ValueConnection left_connection,
                   ToValueConnection(node_data.values[0], converted_graph));
  MP_ASSIGN_OR_RETURN(ValueConnection right_connection,
                   ToValueConnection(node_data.values[1], converted_graph));

  MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                   converted_graph.GetNode(node_data.index));

  main_node.node = ValueNode{
      .value = BinaryExpression{
          .op = op, .left = left_connection, .right = right_connection}};
  return absl::OkStatus();
}

absl::Status ConvertUnaryExpressionAndAddtoGraph(
    const BehaviorData::NodeData& node_data, ConvertedGraph& converted_graph,
    UnaryExpression::UnaryOps op) {
  if (node_data.values.size() != 1) {
    return absl::InvalidArgumentError("Expected 1 value for unary expression.");
  }
  MP_ASSIGN_OR_RETURN(ValueConnection value_connection,
                   ToValueConnection(node_data.values[0], converted_graph));

  MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                   converted_graph.GetNode(node_data.index));

  main_node.node =
      ValueNode{.value = UnaryExpression{.op = op, .input = value_connection}};
  return absl::OkStatus();
}

}  // namespace

NodeConverter GetLifeCycleOnStartConverter() {
  return NodeConverter{
      .node_type = "lifecycle/onStart",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & recipe_node,
                         converted_graph.GetNode(node_data.index));
        recipe_node.name = absl::StrFormat(
            "lifecycle/onStart [Behavior node id %d]", node_data.index);
        ExecutableNodeConnection connection;
        if (!node_data.flows.empty()) {
          MP_ASSIGN_OR_RETURN(
              RecipeNode & next_node,
              converted_graph.GetNode(node_data.flows.front().node));
          connection.node_id = next_node.id;
        }
        recipe_node.node = EventNode{
            .event_name = std::string(recipe::kOnStartEventName),
            .next_node = connection,
        };
        return absl::OkStatus();
      }};
}

NodeConverter GetLifeCycleOnTickConverter() {
  return NodeConverter{
      .node_type = "lifecycle/onTick",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & recipe_node,
                         converted_graph.GetNode(node_data.index));
        recipe_node.name = absl::StrFormat(
            "lifecycle/onTick [Behavior node id %d]", node_data.index);
        ExecutableNodeConnection connection;
        if (!node_data.flows.empty()) {
          MP_ASSIGN_OR_RETURN(
              RecipeNode & next_node,
              converted_graph.GetNode(node_data.flows.front().node));
          connection.node_id = next_node.id;
        }
        recipe_node.node = EventNode{
            .event_name = std::string(recipe::kOnUpdateEventName),
            .next_node = connection,
        };
        return absl::OkStatus();
      }};
}

// TODO Clean this up before I/O, this is mainly so we can parse
// Adobe's gltfs but not part of the spec
NodeConverter GetDebugConsoleConverter() {
  return NodeConverter{
      .node_type = "ADBE/output_console_node",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & recipe_node,
                         converted_graph.GetNode(node_data.index));
        recipe_node.name = absl::StrFormat(
            "ADBE/output_console_node [Behavior node id %d]", node_data.index);
        ExecutableNodeConnection connection;
        if (!node_data.flows.empty()) {
          MP_ASSIGN_OR_RETURN(
              RecipeNode & next_node,
              converted_graph.GetNode(node_data.flows.front().node));
          connection.node_id = next_node.id;
        }

        const BehaviorData::NodeData::ValueData& value_data =
            node_data.values.front();
        MP_ASSIGN_OR_RETURN(ValueConnection value_connection,
                         ToValueConnection(value_data, converted_graph));

        recipe_node.node = ExecutableNode{
            .statement = CallStatement{.expression =
                                           CallExpression{
                                               .name = "ConsoleLog",
                                               .args = {value_connection},
                                           },
                                       .next_node = connection}};

        return absl::OkStatus();
      }};
}

NodeConverter GetVariableSetConverter() {
  return NodeConverter{
      .node_type = "variable/set",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name = absl::StrFormat("variable/set [Behavior node id %d]",
                                         node_data.index);

        if (node_data.configuration.empty()) {
          return absl::InvalidArgumentError(
              "No variable specified in variable/set.");
        }

        const BehaviorData::NodeData::ConfigurationData& configuration =
            node_data.configuration[0];
        if (configuration.id !=
                BehaviorData::NodeData::ConfigurationType::VARIABLE ||
            !std::holds_alternative<std::string>(configuration.value)) {
          return absl::InvalidArgumentError(
              "Variable not declared in configuration.");
        }

        if (node_data.values.size() > 1) {
          return absl::InvalidArgumentError(
              "Variable/set has multiple values to read from.");
        }

        MP_ASSIGN_OR_RETURN(
            ValueConnection value_connection,
            ToValueConnection(node_data.values[0], converted_graph));

        ExecutableNodeConnection connection;
        if (!node_data.flows.empty()) {
          MP_ASSIGN_OR_RETURN(
              RecipeNode & next_node,
              converted_graph.GetNode(node_data.flows.front().node));
          connection.node_id = next_node.id;
        }
        main_node.node = ExecutableNode{
            .statement = AssignmentStatement{
                .op = AssignmentStatement::ASSIGN,
                .targets = {Identifier{
                    .name = std::get<std::string>(configuration.value)}},
                .value = value_connection,
                .next_node = connection,
            }};

        return absl::OkStatus();
      }};
}

NodeConverter GetVariableGetConverter() {
  return NodeConverter{
      .node_type = "variable/get",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name = absl::StrFormat("variable/get [Behavior node id %d]",
                                         node_data.index);

        if (node_data.configuration.empty()) {
          return absl::InvalidArgumentError(
              "No variable specified in variable/get.");
        }

        const BehaviorData::NodeData::ConfigurationData& configuration =
            node_data.configuration[0];
        if (configuration.id !=
                BehaviorData::NodeData::ConfigurationType::VARIABLE ||
            !std::holds_alternative<std::string>(configuration.value)) {
          return absl::InvalidArgumentError(
              "Variable not declared in configuration.");
        }

        main_node.node =
            ValueNode{.value = Identifier{
                          .name = std::get<std::string>(configuration.value),
                      }};

        return absl::OkStatus();
      }};
}

NodeConverter GetWorldStartAnimationConverter() {
  return NodeConverter{
      .node_type = "world/startAnimation",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name = absl::StrFormat(
            "world/startAnimation [Behavior node id %d]", node_data.index);

        if (node_data.values.empty()) {
          return absl::InvalidArgumentError(
              "No animation parameters specified.");
        }

        MP_ASSIGN_OR_RETURN(
            ValueConnection animation,
            GetNodeValueByName("animation", node_data, converted_graph));
        ValueConnection speed = GetNodeValueByNameOr(
            "speed", node_data, converted_graph, Literal{1.0f});
        ValueConnection start_time = GetNodeValueByNameOr(
            "startTime", node_data, converted_graph, Literal{0.0f});
        ValueConnection end_time = GetNodeValueByNameOr(
            "endTime", node_data, converted_graph, Literal{-1.0f});

        ExecutableNodeConnection connection;
        if (!node_data.flows.empty()) {
          MP_ASSIGN_OR_RETURN(
              RecipeNode & next_node,
              converted_graph.GetNode(node_data.flows.front().node));
          connection.node_id = next_node.id;
        }

        main_node.node = ExecutableNode{
            .statement = CallStatement{
                .expression =
                    CallExpression{
                        .name = "StartGltfAnimation",
                        .args = {GetNodeSelfConnection(converted_graph),
                                 animation, speed, start_time, end_time},
                    },
                .next_node = connection,
            }};

        return absl::OkStatus();
      }};
}

NodeConverter GetWorldStopAnimationConverter() {
  return NodeConverter{
      .node_type = "world/stopAnimation",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name = absl::StrFormat(
            "world/stopAnimation [Behavior node id %d]", node_data.index);

        if (node_data.values.empty()) {
          return absl::InvalidArgumentError(
              "No animation parameters specified.");
        }

        MP_ASSIGN_OR_RETURN(
            ValueConnection animation,
            GetNodeValueByName("animation", node_data, converted_graph));

        ExecutableNodeConnection connection;
        if (!node_data.flows.empty()) {
          MP_ASSIGN_OR_RETURN(
              RecipeNode & next_node,
              converted_graph.GetNode(node_data.flows.front().node));
          connection.node_id = next_node.id;
        }

        main_node.node = ExecutableNode{
            .statement = CallStatement{
                .expression =
                    CallExpression{
                        .name = "StopGltfAnimation",
                        .args = {GetNodeSelfConnection(converted_graph),
                                 animation},
                    },
                .next_node = connection,
            }};

        return absl::OkStatus();
      }};
}

NodeConverter GetNodeOnSelectConverter() {
  // TODO: Add support in kOnSelectConverter so that it only
  // triggers the graph when a certain node is tapped.
  return NodeConverter{
      .node_type = "node/OnSelect",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name = absl::StrFormat("node/OnSelect [Behavior node id %d]",
                                         node_data.index);

        RecipeNode& event_node =
            converted_graph.GetGlobalNodes().on_tap_event_node;
        RecipeNode& sequence_node =
            converted_graph.GetGlobalNodes().on_tap_sequence_node;

        RecipeNode& get_node_value_node = converted_graph.CreateNode();
        MP_ASSIGN_OR_RETURN(
            ValueConnection node_index,
            GetNodeConfigurationByType(
                BehaviorData::NodeData::ConfigurationType::NODE_INDEX,
                node_data, converted_graph));

        if (!node_index.literal_value() ||
            !node_index.literal_value()->int_value()) {
          return absl::InvalidArgumentError(
              "Invalid node index in node/OnSelect.");
        }
        converted_graph.RegisterTapNode(
            *node_index.literal_value()->int_value());

        get_node_value_node.node = ValueNode{
            .value = CallExpression{
                .name = std::string(kGetNodeByIndexFunctionName),
                .args = {GetNodeSelfConnection(converted_graph), node_index},
            }};

        RecipeNode& compare_value_node = converted_graph.CreateNode();
        compare_value_node.node = ValueNode{
            .value =
                CallExpression{
                    .name = "IsAncestorOf",
                    .args =
                        {ValueConnection{
                             .connection =
                                 SocketConnection{
                                     .node_id = get_node_value_node.id,
                                     .socket_name = std::string(
                                         recipe::kDefaultOutputSocketName)}},
                         ValueConnection{
                             .connection =
                                 SocketConnection{
                                     .node_id = event_node.id,
                                     .socket_name = std::string(
                                         recipe::kTapTargetSocketName)}}},
                },
        };

        ExecutableNodeConnection next_node_connection;
        if (!node_data.flows.empty()) {
          MP_ASSIGN_OR_RETURN(
              RecipeNode & next_node,
              converted_graph.GetNode(node_data.flows.front().node));
          next_node_connection.node_id = next_node.id;
        }

        main_node.node = ExecutableNode{
            .statement = BranchStatement{
                .test =
                    ValueConnection{
                        .connection =
                            SocketConnection{
                                .node_id = compare_value_node.id,
                                .socket_name = std::string(
                                    recipe::kDefaultOutputSocketName)}},
                .true_next_node = next_node_connection}};

        sequence_node.mutable_executable_node()
            ->mutable_sequence()
            ->next_nodes.push_back(
                {ExecutableNodeConnection{.node_id = main_node.id}});

        return absl::OkStatus();
      }};
}

NodeConverter GetCustomEventSendConverter() {
  return NodeConverter{
      .node_type = "customEvent/send",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));

        if (node_data.configuration.empty()) {
          return absl::InvalidArgumentError(absl::StrFormat(
              "No configuration for customEvent/send for behavior node %d.",
              node_data.index));
        }

        const BehaviorData::NodeData::ConfigurationData& configuration =
            node_data.configuration[0];
        if (configuration.id !=
                BehaviorData::NodeData::ConfigurationType::CUSTOM_EVENT ||
            !std::holds_alternative<int>(configuration.value)) {
          return absl::InvalidArgumentError(
              absl::StrFormat("Custom event information in behavior node %d "
                              "not declared correctly in configuration.",
                              node_data.index));
        }

        absl::StatusOr<BehaviorData::CustomEventData> event_data =
            converted_graph.GetEvent(std::get<int>(configuration.value));
        if (!event_data.ok()) {
          return event_data.status();
        }

        ExecutableNodeConnection connection;
        if (!node_data.flows.empty()) {
          MP_ASSIGN_OR_RETURN(
              RecipeNode & next_node,
              converted_graph.GetNode(node_data.flows.front().node));
          connection.node_id = next_node.id;
        }

        EventTrigger event_trigger{.event_name = event_data->id,
                                   .next_node = connection};

        for (const BehaviorData::VariableData& value : event_data->values) {
          ValueConnection value_connection;
          value_connection.connection = ToLiteral(value);
          event_trigger.args[value.id] = value_connection;
        }

        for (const BehaviorData::NodeData::ValueData& value :
             node_data.values) {
          MP_ASSIGN_OR_RETURN(ValueConnection value_connection,
                           ToValueConnection(value, converted_graph));
          event_trigger.args[GetValueId(value)] = value_connection;
        }

        main_node.node = ExecutableNode{.statement = event_trigger};
        return absl::OkStatus();
      }};
}

NodeConverter GetCustomEventReceiveConverter() {
  return NodeConverter{
      .node_type = "customEvent/receive",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & recipe_node,
                         converted_graph.GetNode(node_data.index));
        ExecutableNodeConnection connection;

        if (node_data.configuration.empty()) {
          return absl::InvalidArgumentError(absl::StrFormat(
              "No configuration for customEvent/send for behavior node %d.",
              node_data.index));
        }

        const BehaviorData::NodeData::ConfigurationData& configuration =
            node_data.configuration[0];
        if (configuration.id !=
                BehaviorData::NodeData::ConfigurationType::CUSTOM_EVENT ||
            !std::holds_alternative<int>(configuration.value)) {
          return absl::InvalidArgumentError(
              absl::StrFormat("Custom event information in behavior node %d "
                              "not declared in configuration.",
                              node_data.index));
        }

        if (!node_data.flows.empty()) {
          MP_ASSIGN_OR_RETURN(
              RecipeNode & next_node,
              converted_graph.GetNode(node_data.flows.front().node));
          connection.node_id = next_node.id;
        }

        absl::StatusOr<BehaviorData::CustomEventData> event_data =
            converted_graph.GetEvent(std::get<int>(configuration.value));
        if (!event_data.ok()) {
          return event_data.status();
        }

        recipe_node.node = EventNode{
            .event_name = event_data->id,
            .next_node = connection,
        };
        return absl::OkStatus();
      }};
}

NodeConverter GetWorldSetConverter() {
  return NodeConverter{
      .node_type = "world/set",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("world/set [Behavior node id %d]", node_data.index);

        if (node_data.configuration.empty()) {
          return absl::InvalidArgumentError("Failed to get world pointer.");
        }

        const BehaviorData::NodeData::ConfigurationData& configuration =
            node_data.configuration[0];
        if (configuration.id !=
                BehaviorData::NodeData::ConfigurationType::PATH ||
            !std::holds_alternative<std::string>(configuration.value)) {
          return absl::InvalidArgumentError("Malformed configutation.");
        }
        std::string path =
            std::get<std::string>(node_data.configuration[0].value);
        MP_ASSIGN_OR_RETURN(WorldPointer world_pointer,
                         WorldPointer::FromString(path));

        // FindNodeByName node.
        RecipeNode& find_node_recipe_node = converted_graph.CreateNode();
        find_node_recipe_node.node = ValueNode{
            .value = CallExpression{
                .name = std::string(kGetNodeByIndexFunctionName),
                .args = {
                    GetNodeSelfConnection(converted_graph),
                    ValueConnection{.connection =
                                        Literal{.value = static_cast<int>(
                                                    world_pointer.index)}},
                }}};

        // CallExpression for the setter function call.
        CallExpression call_expression;
        call_expression.args.push_back(ValueConnection{
            .connection = SocketConnection{
                .node_id = find_node_recipe_node.id,
                .socket_name = std::string(recipe::kDefaultOutputSocketName),
            }});
        if (node_data.values.empty()) {
          return absl::InvalidArgumentError("Failed to get variable data.");
        }

        const BehaviorData::NodeData::ValueData& value_data =
            node_data.values.front();
        MP_ASSIGN_OR_RETURN(ValueConnection value_connection,
                         ToValueConnection(value_data, converted_graph));

        if (world_pointer.type == WorldPointer::Type::kNodeTranslation) {
          if (value_connection.literal_value() &&
              !value_connection.literal_value()->float3_value()) {
            return absl::InvalidArgumentError(
                "Calling world/set translation with non float3 values.");
          }

          call_expression.name = "SetLocalPosition";
        } else if (world_pointer.type == WorldPointer::Type::kNodeRotation) {
          if (value_connection.literal_value()) {
            if (!value_connection.literal_value()->float4_value()) {
              return absl::InvalidArgumentError(
                  "Calling world/set rotation with non float4 values.");
            }
          }

          call_expression.name = "SetLocalRotationVec4";
        } else if (world_pointer.type == WorldPointer::Type::kNodeScale) {
          if (value_connection.literal_value() &&
              !value_connection.literal_value()->float3_value()) {
            return absl::InvalidArgumentError(
                "Calling world/set scale with non float3 values.");
          }

          call_expression.name = "SetLocalScale";
        } else if (world_pointer.type ==
                   WorldPointer::Type::kNodeExtensionKhrVisibilityVisible) {
          if (value_connection.literal_value() &&
              !value_connection.literal_value()->bool_value()) {
            return absl::InvalidArgumentError(
                "Calling world/set KHR_visibility/visible with non bool "
                "values.");
          }

          call_expression.name = "SetNodeEnabled";
        } else {
          return absl::InvalidArgumentError(
              absl::StrFormat("Unsupported world pointer type %s.", path));
        }
        call_expression.args.push_back(value_connection);

        ExecutableNodeConnection next_node_connection;
        if (!node_data.flows.empty()) {
          MP_ASSIGN_OR_RETURN(
              RecipeNode & next_node,
              converted_graph.GetNode(node_data.flows.front().node));
          next_node_connection.node_id = next_node.id;
        }
        main_node.node = ExecutableNode{
            .statement = CallStatement{.expression = call_expression,
                                       .next_node = next_node_connection}};

        return absl::OkStatus();
      }};
}

NodeConverter GetWorldGetConverter() {
  return NodeConverter{
      .node_type = "world/get",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("world/get [Behavior node id %d]", node_data.index);

        if (node_data.configuration.empty()) {
          return absl::InvalidArgumentError("Failed to get world pointer.");
        }

        const BehaviorData::NodeData::ConfigurationData& configuration =
            node_data.configuration[0];
        if (configuration.id !=
                BehaviorData::NodeData::ConfigurationType::PATH ||
            !std::holds_alternative<std::string>(configuration.value)) {
          return absl::InvalidArgumentError("Malformed configuration.");
        }
        std::string world_pointer_path =
            std::get<std::string>(configuration.value);
        MP_ASSIGN_OR_RETURN(WorldPointer world_pointer,
                         WorldPointer::FromString(world_pointer_path));

        if (world_pointer.type == WorldPointer::Type::kNodeSceneMatrix ||
            world_pointer.type == WorldPointer::Type::kNodeRotation ||
            world_pointer.type == WorldPointer::Type::kNodeScale ||
            world_pointer.type == WorldPointer::Type::kNodeTranslation ||
            world_pointer.type ==
                WorldPointer::Type::kNodeExtensionKhrVisibilityVisible) {
          // FindNodeByName node.
          RecipeNode& find_child_recipe_node = converted_graph.CreateNode();
          find_child_recipe_node.node = ValueNode{
              .value = CallExpression{
                  .name = std::string(kGetNodeByIndexFunctionName),
                  .args = {
                      GetNodeSelfConnection(converted_graph),
                      ValueConnection{.connection =
                                          Literal{.value = static_cast<int>(
                                                      world_pointer.index)}},
                  }}};

          // CallExpression for the getter function call.
          CallExpression call_expression;
          call_expression.args.push_back(ValueConnection{
              .connection = SocketConnection{
                  .node_id = find_child_recipe_node.id,
                  .socket_name = std::string(recipe::kDefaultOutputSocketName),
              }});

          switch (world_pointer.type) {
            case WorldPointer::Type::kNodeTranslation:
              call_expression.name = "GetLocalPosition";
              break;
            case WorldPointer::Type::kNodeRotation:
              call_expression.name = "GetLocalRotationVec4";
              break;
            case WorldPointer::Type::kNodeScale:
              call_expression.name = "GetLocalScale";
              break;
            case WorldPointer::Type::kNodeSceneMatrix:
              call_expression.name = "GetSceneTransformMatrix";
              call_expression.args.push_back(
                  GetNodeSelfConnection(converted_graph));
              break;
            case WorldPointer::Type::kNodeExtensionKhrVisibilityVisible:
              call_expression.name = "IsNodeEnabled";
              break;
            default:
              return absl::InvalidArgumentError(absl::StrFormat(
                  "Unsupported world pointer type %s.", world_pointer_path));
          }

          main_node.node = ValueNode{.value = call_expression};
          return absl::OkStatus();
        } else if (world_pointer.type ==
                   WorldPointer::Type::kActiveCameraSceneMatrix) {
          RecipeNode& get_camera_node_recipe_node =
              converted_graph.CreateNode();
          get_camera_node_recipe_node.node =
              ValueNode{.value = CallExpression{
                            .name = "GetCameraNode",
                        }};
          main_node.node = ValueNode{
              .value = CallExpression{
                  .name = "GetSceneTransformMatrix",
                  .args = {
                      ValueConnection{
                          .connection =
                              SocketConnection{
                                  .node_id = get_camera_node_recipe_node.id,
                                  .socket_name = std::string(
                                      recipe::kDefaultOutputSocketName),
                              }},
                      GetNodeSelfConnection(converted_graph),
                  }}};

          return absl::OkStatus();
        }

        return absl::InvalidArgumentError(absl::StrFormat(
            "Unsupported world pointer type %s.", world_pointer_path));
      }};
}

NodeConverter GetWorldAnimateToConverter() {
  return NodeConverter{
      .node_type = "world/animateTo",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name = absl::StrFormat(
            "world/animateTo [Behavior node id %d]", node_data.index);

        if (node_data.configuration.empty()) {
          return absl::InvalidArgumentError("Failed to get world pointer.");
        }

        std::optional<BehaviorData::NodeData::ConfigurationData> path_config =
            FindConfiguration(BehaviorData::NodeData::ConfigurationType::PATH,
                              node_data.configuration, converted_graph);
        if (!path_config.has_value()) {
          return absl::InvalidArgumentError("Malformed path config.");
        }
        std::string path = std::get<std::string>(path_config->value);
        MP_ASSIGN_OR_RETURN(WorldPointer world_pointer,
                         WorldPointer::FromString(path));

        // FindNodeByName of the node being animated.
        RecipeNode& find_node_recipe_node = converted_graph.CreateNode();
        find_node_recipe_node.node = ValueNode{
            .value = CallExpression{
                .name = std::string(kGetNodeByIndexFunctionName),
                .args = {
                    GetNodeSelfConnection(converted_graph),
                    ValueConnection{.connection =
                                        Literal{.value = static_cast<int>(
                                                    world_pointer.index)}},
                }}};

        MP_ASSIGN_OR_RETURN(
            ValueConnection easing_duration,
            GetNodeConfigurationByType(
                BehaviorData::NodeData::ConfigurationType::EASING_DURATION,
                node_data, converted_graph));

        // TODO: Add easing type

        // TODO: Ensure that the target is always "a" or change it.
        MP_ASSIGN_OR_RETURN(ValueConnection target,
                         GetNodeValueByName("a", node_data, converted_graph));

        // CallExpression for the animateTo function call.
        CallExpression call_expression;
        call_expression.args.push_back(GetNodeSelfConnection(converted_graph));
        call_expression.args.push_back(ValueConnection{
            .connection = SocketConnection{
                .node_id = find_node_recipe_node.id,
                .socket_name = std::string(recipe::kDefaultOutputSocketName),
            }});
        call_expression.args.push_back(target);
        call_expression.args.push_back(easing_duration);
        if (node_data.values.empty()) {
          return absl::InvalidArgumentError("Failed to get variable data.");
        }

        switch (world_pointer.type) {
          case WorldPointer::Type::kNodeTranslation:
            call_expression.name = kWorldAnimateToTranslationRecipeFunctionName;
            break;
          case WorldPointer::Type::kNodeRotation:
            call_expression.name = kWorldAnimateToRotationRecipeFunctionName;
            break;
          case WorldPointer::Type::kNodeScale:
            call_expression.name = kWorldAnimateToScaleRecipeFunctionName;
            break;
          default:
            return absl::UnimplementedError(
                absl::StrFormat("Unsupported world pointer type %s.", path));
        }

        ExecutableNodeConnection next_node_connection;
        if (!node_data.flows.empty()) {
          MP_ASSIGN_OR_RETURN(
              RecipeNode & next_node,
              converted_graph.GetNode(node_data.flows.front().node));
          next_node_connection.node_id = next_node.id;
        }
        main_node.node = ExecutableNode{
            .statement = CallStatement{.expression = call_expression,
                                       .next_node = next_node_connection}};

        return absl::OkStatus();
      }};
}

NodeConverter GetMathPiConverter() {
  return NodeConverter{
      .node_type = "math/pi",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/pi [Behavior node id %d]", node_data.index);

        main_node.node = ValueNode{.value = Identifier{
                                       .name = std::string(recipe::kMathPi),
                                   }};

        return absl::OkStatus();
      }};
}

NodeConverter GetMathAddConverter() {
  return NodeConverter{
      .node_type = "math/add",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/add [Behavior node id %d]", node_data.index);
        return ConvertBinaryExpressionAndAddtoGraph(
            node_data, converted_graph, BinaryExpression::BinaryOps::ADD);
      }};
}

NodeConverter GetMathSubConverter() {
  return NodeConverter{
      .node_type = "math/sub",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/sub [Behavior node id %d]", node_data.index);
        return ConvertBinaryExpressionAndAddtoGraph(
            node_data, converted_graph, BinaryExpression::BinaryOps::SUBTRACT);
      }};
}

NodeConverter GetMathMulConverter() {
  return NodeConverter{
      .node_type = "math/mul",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/mul [Behavior node id %d]", node_data.index);
        return ConvertBinaryExpressionAndAddtoGraph(
            node_data, converted_graph, BinaryExpression::BinaryOps::MULTIPLY);
      }};
}

NodeConverter GetMathDivConverter() {
  return NodeConverter{
      .node_type = "math/div",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/div [Behavior node id %d]", node_data.index);
        return ConvertBinaryExpressionAndAddtoGraph(
            node_data, converted_graph, BinaryExpression::BinaryOps::DIVIDE);
      }};
}

NodeConverter GetMathRemConverter() {
  return NodeConverter{
      .node_type = "math/rem",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/rem [Behavior node id %d]", node_data.index);
        return ConvertBinaryExpressionAndAddtoGraph(
            node_data, converted_graph, BinaryExpression::BinaryOps::MOD);
      }};
}

NodeConverter GetMathEqConverter() {
  return NodeConverter{
      .node_type = "math/eq",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/eq [Behavior node id %d]", node_data.index);
        return ConvertBinaryExpressionAndAddtoGraph(
            node_data, converted_graph, BinaryExpression::BinaryOps::EQUALS);
      }};
}

NodeConverter GetMathLtConverter() {
  return NodeConverter{
      .node_type = "math/lt",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/lt [Behavior node id %d]", node_data.index);
        return ConvertBinaryExpressionAndAddtoGraph(
            node_data, converted_graph, BinaryExpression::BinaryOps::LESS_THAN);
      }};
}

NodeConverter GetMathLeConverter() {
  return NodeConverter{
      .node_type = "math/le",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/le [Behavior node id %d]", node_data.index);
        return ConvertBinaryExpressionAndAddtoGraph(
            node_data, converted_graph,
            BinaryExpression::BinaryOps::LESS_THAN_OR_EQUAL);
      }};
}

NodeConverter GetMathGtConverter() {
  return NodeConverter{
      .node_type = "math/gt",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/gt [Behavior node id %d]", node_data.index);
        return ConvertBinaryExpressionAndAddtoGraph(
            node_data, converted_graph,
            BinaryExpression::BinaryOps::GREATER_THAN);
      }};
}

NodeConverter GetMathGeConverter() {
  return NodeConverter{
      .node_type = "math/ge",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/ge [Behavior node id %d]", node_data.index);
        return ConvertBinaryExpressionAndAddtoGraph(
            node_data, converted_graph,
            BinaryExpression::BinaryOps::GREATER_THAN_OR_EQUAL);
      }};
}

NodeConverter GetMathDotConverter() {
  return NodeConverter{
      .node_type = "math/dot",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/dot [Behavior node id %d]", node_data.index);
        return ConvertBinaryExpressionAndAddtoGraph(
            node_data, converted_graph, BinaryExpression::BinaryOps::DOT);
      }};
}

NodeConverter GetMathCrossConverter() {
  return NodeConverter{
      .node_type = "math/cross",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name = absl::StrFormat("math/cross [Behavior node id %d]",
                                         node_data.index);
        return ConvertBinaryExpressionAndAddtoGraph(
            node_data, converted_graph, BinaryExpression::BinaryOps::CROSS);
      }};
}

NodeConverter GetMathMinConverter() {
  return NodeConverter{
      .node_type = "math/min",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/min [Behavior node id %d]", node_data.index);
        return ConvertBinaryExpressionAndAddtoGraph(
            node_data, converted_graph, BinaryExpression::BinaryOps::MIN);
      }};
}

NodeConverter GetMathMaxConverter() {
  return NodeConverter{
      .node_type = "math/max",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/max [Behavior node id %d]", node_data.index);
        return ConvertBinaryExpressionAndAddtoGraph(
            node_data, converted_graph, BinaryExpression::BinaryOps::MAX);
      }};
}

NodeConverter GetMathClampConverter() {
  return NodeConverter{
      .node_type = "math/clamp",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        // Input value ids are derived from the Khronos spec doc.
        // 'a' - input
        // 'b' - min
        // 'c' - max
        const absl::string_view ids[] = {"a", "b", "c"};

        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name = absl::StrFormat("math/clamp [Behavior node id %d]",
                                         node_data.index);

        if (node_data.values.size() != 3) {
          return absl::InvalidArgumentError(
              absl::StrFormat("Not enough values for clamp node. "
                              "Provided %d. Must have 3.",
                              node_data.values.size()));
        }

        CallExpression call_expression;
        call_expression.name = "Clamp";
        // Change this to use 'ids'
        for (int i = 0; i < 3; ++i) {
          MP_ASSIGN_OR_RETURN(
              ValueConnection input,
              GetNodeValueByName(ids[i], node_data, converted_graph));
          call_expression.args.push_back(input);
        }
        main_node.node = ValueNode{.value = call_expression};
        return absl::OkStatus();
      }};
}

NodeConverter GetMathAbsConverter() {
  return NodeConverter{
      .node_type = "math/abs",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/abs [Behavior node id %d]", node_data.index);
        return ConvertUnaryExpressionAndAddtoGraph(
            node_data, converted_graph, UnaryExpression::UnaryOps::ABSOLUTE);
      }};
}

NodeConverter GetMathSqrtConverter() {
  return NodeConverter{
      .node_type = "math/sqrt",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/sqrt [Behavior node id %d]", node_data.index);
        return ConvertUnaryExpressionAndAddtoGraph(
            node_data, converted_graph, UnaryExpression::UnaryOps::SQRT);
      }};
}

NodeConverter GetMathLogConverter() {
  return NodeConverter{
      .node_type = "math/log",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/log [Behavior node id %d]", node_data.index);
        return ConvertUnaryExpressionAndAddtoGraph(
            node_data, converted_graph, UnaryExpression::UnaryOps::LOG);
      }};
}

NodeConverter GetMathSinConverter() {
  return NodeConverter{
      .node_type = "math/sin",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/sin [Behavior node id %d]", node_data.index);
        return ConvertUnaryExpressionAndAddtoGraph(
            node_data, converted_graph, UnaryExpression::UnaryOps::SIN);
      }};
}

NodeConverter GetMathCosConverter() {
  return NodeConverter{
      .node_type = "math/cos",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/cos [Behavior node id %d]", node_data.index);
        return ConvertUnaryExpressionAndAddtoGraph(
            node_data, converted_graph, UnaryExpression::UnaryOps::COS);
      }};
}

NodeConverter GetMathTanConverter() {
  return NodeConverter{
      .node_type = "math/tan",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/tan [Behavior node id %d]", node_data.index);
        return ConvertUnaryExpressionAndAddtoGraph(
            node_data, converted_graph, UnaryExpression::UnaryOps::TAN);
      }};
}

NodeConverter GetMathAsinConverter() {
  return NodeConverter{
      .node_type = "math/asin",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/asin [Behavior node id %d]", node_data.index);
        return ConvertUnaryExpressionAndAddtoGraph(
            node_data, converted_graph, UnaryExpression::UnaryOps::ASIN);
      }};
}

NodeConverter GetMathAcosConverter() {
  return NodeConverter{
      .node_type = "math/acos",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/acos [Behavior node id %d]", node_data.index);
        return ConvertUnaryExpressionAndAddtoGraph(
            node_data, converted_graph, UnaryExpression::UnaryOps::ACOS);
      }};
}

NodeConverter GetMathAtanConverter() {
  return NodeConverter{
      .node_type = "math/atan",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/atan [Behavior node id %d]", node_data.index);
        return ConvertUnaryExpressionAndAddtoGraph(
            node_data, converted_graph, UnaryExpression::UnaryOps::ATAN);
      }};
}

NodeConverter GetMathAtanTwoConverter() {
  return NodeConverter{
      .node_type = "math/atan2",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name = absl::StrFormat("math/atan2 [Behavior node id %d]",
                                         node_data.index);

        if (node_data.values.size() != 2) {
          return absl::InvalidArgumentError(
              absl::StrFormat("math/atan2 expected 2 values, but received %d.",
                              node_data.values.size()));
        }

        MP_ASSIGN_OR_RETURN(
            ValueConnection a,
            ToValueConnection(node_data.values[0], converted_graph));

        MP_ASSIGN_OR_RETURN(
            ValueConnection b,
            ToValueConnection(node_data.values[1], converted_graph));

        RecipeNode& divide_node = converted_graph.CreateNode();
        divide_node.node = ValueNode{
            .value = BinaryExpression{
                .op = BinaryExpression::DIVIDE, .left = a, .right = b}};

        MP_ASSIGN_OR_RETURN(
            ValueConnection value_connection,
            ToValueConnection(node_data.values[0], converted_graph));

        main_node.node =
            ValueNode{.value = UnaryExpression{
                          .op = UnaryExpression::ATAN,
                          .input = ValueConnection{
                              .connection = SocketConnection{
                                  .node_id = divide_node.id,
                                  .socket_name = std::string(
                                      recipe::kDefaultOutputSocketName),
                              }}}};

        return absl::OkStatus();
      }};
}

NodeConverter GetMathSignConverter() {
  return NodeConverter{
      .node_type = "math/sign",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name =
            absl::StrFormat("math/sign [Behavior node id %d]", node_data.index);
        return ConvertUnaryExpressionAndAddtoGraph(
            node_data, converted_graph, UnaryExpression::UnaryOps::SIGN);
      }};
}

NodeConverter GetMathNormalizeConverter() {
  return NodeConverter{
      .node_type = "math/normalize",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name = absl::StrFormat("math/normalize [Behavior node id %d]",
                                         node_data.index);
        return ConvertUnaryExpressionAndAddtoGraph(
            node_data, converted_graph, UnaryExpression::UnaryOps::NORMALIZE);
      }};
}

NodeConverterFunction GetTypeCastFunction(absl::string_view node_signature,
                                          absl::string_view function_name) {
  return [node_signature, function_name](
             const BehaviorData::NodeData& node_data,
             ConvertedGraph& converted_graph) -> absl::Status {
    MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                     converted_graph.GetNode(node_data.index));
    main_node.name =
        absl::StrFormat("math/%s [Behavior node id %d]",
                        std::string(node_signature), node_data.index);

    if (node_data.values.size() != 1) {
      return absl::InvalidArgumentError(
          absl::StrFormat("Invalid number of inputs for compose transform"
                          ": %d. Must be one.",
                          node_data.values.size()));
    }

    MP_ASSIGN_OR_RETURN(ValueConnection input,
                     ToValueConnection(node_data.values[0], converted_graph));

    CallExpression call_expression;
    call_expression.name = std::string(function_name);
    call_expression.args.push_back(input);

    main_node.node = ValueNode{.value = call_expression};
    return absl::OkStatus();
  };
}

NodeConverter GetTypeCastBoolToIntConverter() {
  return NodeConverter{
      .node_type = "type/boolToInt",
      .function = GetTypeCastFunction("boolToInt", "CastBoolToInt")};
}
NodeConverter GetTypeCastBoolToFloatConverter() {
  return NodeConverter{
      .node_type = "type/boolToFloat",
      .function = GetTypeCastFunction("boolToFloat", "CastBoolToFloat")};
}
NodeConverter GetTypeCastIntToBoolConverter() {
  return NodeConverter{
      .node_type = "type/intToBool",
      .function = GetTypeCastFunction("intToBool", "CastIntToBool")};
}
NodeConverter GetTypeCastIntToFloatConverter() {
  return NodeConverter{
      .node_type = "type/intToFloat",
      .function = GetTypeCastFunction("intToFloat", "CastIntToFloat")};
}
NodeConverter GetTypeCastFloatToBoolConverter() {
  return NodeConverter{
      .node_type = "type/floatToBool",
      .function = GetTypeCastFunction("floatToBool", "CastFloatToBool")};
}
NodeConverter GetTypeCastFloatToIntConverter() {
  return NodeConverter{
      .node_type = "type/floatToInt",
      .function = GetTypeCastFunction("floatToInt", "CastFloatToInt")};
}

NodeConverterFunction GetMakeVectorFunction() {
  return [](const BehaviorData::NodeData& node_data,
            ConvertedGraph& converted_graph) -> absl::Status {
    const std::vector<absl::string_view> ids = {"x", "y", "z", "w"};

    MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                     converted_graph.GetNode(node_data.index));

    const int num_inputs = node_data.values.size();
    if (num_inputs < 2) {
      return absl::InvalidArgumentError(
          absl::StrFormat("Not enough values for makeVector node. "
                          "Provided %d. Must have at least 2.",
                          num_inputs));
    } else if (num_inputs > 4) {
      return absl::InvalidArgumentError(
          absl::StrFormat("Too many values for makeVector node. "
                          "Provided %d. Must have at most 4.",
                          num_inputs));
    }

    main_node.name = absl::StrFormat("math/makeVector%d [Behavior node id %d]",
                                     num_inputs, node_data.index);

    CallExpression call_expression;
    call_expression.name = absl::StrFormat("MakeVector%d", num_inputs);
    // Change this to use 'ids'
    for (int i = 0; i < num_inputs; ++i) {
      MP_ASSIGN_OR_RETURN(ValueConnection input,
                       GetNodeValueByName(ids[i], node_data, converted_graph));
      call_expression.args.push_back(input);
    }
    main_node.node = ValueNode{.value = call_expression};
    return absl::OkStatus();
  };
}

NodeConverter GetMathMakeVector2Converter() {
  return NodeConverter{.node_type = "math/makeVector2",
                       .function = GetMakeVectorFunction()};
}

NodeConverter GetMathMakeVector3Converter() {
  return NodeConverter{.node_type = "math/makeVector3",
                       .function = GetMakeVectorFunction()};
}

NodeConverter GetMathMakeVector4Converter() {
  return NodeConverter{.node_type = "math/makeVector4",
                       .function = GetMakeVectorFunction()};
}

NodeConverterFunction GetBreakVectorFunction(int vector_size) {
  return [vector_size](const BehaviorData::NodeData& node_data,
                       ConvertedGraph& converted_graph) -> absl::Status {
    MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                     converted_graph.GetNode(node_data.index));
    main_node.name = absl::StrFormat("math/breakVector%d [Behavior node id %d]",
                                     vector_size, node_data.index);

    if (node_data.values.size() != 1) {
      return absl::InvalidArgumentError(
          absl::StrFormat("Invalid number of inputs for breakVector: %d",
                          node_data.values.size()));
    }

    MP_ASSIGN_OR_RETURN(ValueConnection input,
                     ToValueConnection(node_data.values[0], converted_graph));

    CallExpression call_expression;
    call_expression.name = absl::StrFormat("BreakVector%d", vector_size);
    call_expression.args.push_back(input);

    main_node.node = ValueNode{.value = call_expression};
    return absl::OkStatus();
  };
}

NodeConverter GetMathBreakVector2Converter() {
  return NodeConverter{.node_type = "math/breakVector2",
                       .function = GetBreakVectorFunction(2)};
}

NodeConverter GetMathBreakVector3Converter() {
  return NodeConverter{.node_type = "math/breakVector3",
                       .function = GetBreakVectorFunction(3)};
}

NodeConverter GetMathBreakVector4Converter() {
  return NodeConverter{.node_type = "math/breakVector4",
                       .function = GetBreakVectorFunction(4)};
}

NodeConverter GetMathComposeConverter() {
  return NodeConverter{
      .node_type = "math/compose",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name = absl::StrFormat("math/compose [Behavior node id %d]",
                                         node_data.index);

        if (node_data.values.size() != 3) {
          return absl::InvalidArgumentError(
              absl::StrFormat("Invalid number of inputs for compose transform"
                              ": %d. Must be three.",
                              node_data.values.size()));
        }

        MP_ASSIGN_OR_RETURN(
            ValueConnection translation,
            GetNodeValueByName("translation", node_data, converted_graph));
        MP_ASSIGN_OR_RETURN(
            ValueConnection rotation,
            GetNodeValueByName("rotation", node_data, converted_graph));
        MP_ASSIGN_OR_RETURN(
            ValueConnection scale,
            GetNodeValueByName("scale", node_data, converted_graph));

        CallExpression call_expression;
        call_expression.name = "ComposeTransform";
        call_expression.args.push_back(translation);
        call_expression.args.push_back(rotation);
        call_expression.args.push_back(scale);

        main_node.node = ValueNode{.value = call_expression};
        return absl::OkStatus();
      }};
}

NodeConverter GetMathDecomposeConverter() {
  return NodeConverter{
      .node_type = "math/decompose",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name = absl::StrFormat("math/decompose [Behavior node id %d]",
                                         node_data.index);

        if (node_data.values.size() != 1) {
          return absl::InvalidArgumentError(
              absl::StrFormat("Invalid number of inputs for decompose transform"
                              ": %d. Must be one.",
                              node_data.values.size()));
        }

        MP_ASSIGN_OR_RETURN(
            ValueConnection transform_matrix,
            ToValueConnection(node_data.values[0], converted_graph));

        CallExpression call_expression;
        call_expression.name = "DecomposeTransform";
        call_expression.args.push_back(transform_matrix);

        main_node.node = ValueNode{.value = call_expression};
        return absl::OkStatus();
      }};
}

NodeConverter GetMathInverseConverter() {
  return NodeConverter{
      .node_type = "math/inverse",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name = absl::StrFormat("math/inverse [Behavior node id %d]",
                                         node_data.index);

        if (node_data.values.size() != 1) {
          return absl::InvalidArgumentError(
              absl::StrFormat("%d inputs for inverse when 1 is expected",
                              node_data.values.size()));
        }

        MP_ASSIGN_OR_RETURN(
            ValueConnection matrix,
            ToValueConnection(node_data.values[0], converted_graph));

        CallExpression call_expression;
        call_expression.name = "InvertMatrix";
        call_expression.args.push_back(matrix);

        main_node.node = ValueNode{.value = call_expression};
        return absl::OkStatus();
      }};
}

NodeConverter GetMathMatMulConverter() {
  return NodeConverter{
      .node_type = "math/matmul",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & main_node,
                         converted_graph.GetNode(node_data.index));
        main_node.name = absl::StrFormat("math/matmul [Behavior node id %d]",
                                         node_data.index);

        if (node_data.values.size() != 2) {
          return absl::InvalidArgumentError(
              absl::StrFormat("%d inputs for inverse when 2 is expected",
                              node_data.values.size()));
        }

        MP_ASSIGN_OR_RETURN(
            ValueConnection mat_a,
            ToValueConnection(node_data.values[0], converted_graph));

        MP_ASSIGN_OR_RETURN(
            ValueConnection mat_b,
            ToValueConnection(node_data.values[1], converted_graph));

        CallExpression call_expression;
        call_expression.name = "MatrixMultiply";
        call_expression.args.push_back(mat_a);
        call_expression.args.push_back(mat_b);

        main_node.node = ValueNode{.value = call_expression};
        return absl::OkStatus();
      }};
}

NodeConverter GetFlowForLoopConverter() {
  return NodeConverter{
      .node_type = "flow/forLoop",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & flow_loop_node,
                         converted_graph.GetNode(node_data.index));
        flow_loop_node.name = absl::StrFormat(
            "flow/forLoop [Behavior node id %d]", node_data.index);

        MP_ASSIGN_OR_RETURN(
            ValueConnection start_index_connection,
            GetNodeValueByName("startIndex", node_data, converted_graph));
        MP_ASSIGN_OR_RETURN(
            ValueConnection end_index_connection,
            GetNodeValueByName("endIndex", node_data, converted_graph));
        MP_ASSIGN_OR_RETURN(
            ValueConnection increment_connection,
            GetNodeValueByName("increment", node_data, converted_graph));
        MP_ASSIGN_OR_RETURN(
            ExecutableNodeConnection loop_body,
            GetNodeFlowByName("loopBody", node_data, converted_graph));
        absl::StatusOr<ExecutableNodeConnection> completed =
            GetNodeFlowByName("completed", node_data, converted_graph);

        LoopStatement loop{.start_index = start_index_connection,
                           .end_index = end_index_connection,
                           .increment = increment_connection,
                           .looped_node = loop_body};

        if (completed.ok()) {
          loop.completed_node = *completed;
        }

        flow_loop_node.node = ExecutableNode{.statement = loop};

        return absl::OkStatus();
      }};
}

NodeConverter GetFlowSequenceConverter() {
  return NodeConverter{
      .node_type = "flow/sequence",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & flow_sequence_node,
                         converted_graph.GetNode(node_data.index));
        flow_sequence_node.name = absl::StrFormat(
            "flow/sequence [Behavior node id %d]", node_data.index);

        std::optional<BehaviorData::NodeData::ConfigurationData> configuration =
            FindConfiguration(BehaviorData::NodeData::ConfigurationType::
                                  NUMBER_OF_OUTPUT_FLOWS,
                              node_data.configuration, converted_graph);

        if (!configuration.has_value() ||
            !std::holds_alternative<int>(configuration->value)) {
          return absl::InvalidArgumentError(
              "Number of output flows not found in configuration.");
        }

        int number_of_output_flows = std::get<int>(configuration->value);

        if (node_data.flows.size() > number_of_output_flows) {
          return absl::InvalidArgumentError(absl::StrFormat(
              "Insufficient output flows for node id %d.", node_data.index));
        }

        SequenceStatement sequence_statement;
        for (int i = 0; i < node_data.flows.size(); ++i) {
          MP_ASSIGN_OR_RETURN(RecipeNode & next_node,
                           converted_graph.GetNode(node_data.flows[i].node));
          sequence_statement.next_nodes.push_back(
              ExecutableNodeConnection{.node_id = next_node.id});
        }

        flow_sequence_node.node =
            ExecutableNode{.statement = sequence_statement};

        return absl::OkStatus();
      }};
}

NodeConverter GetFlowDelayConverter() {
  return NodeConverter{
      .node_type = "flow/delay",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & flow_delay_node,
                         converted_graph.GetNode(node_data.index));
        flow_delay_node.name = absl::StrFormat(
            "flow/delay [Behavior node id %d]", node_data.index);

        MP_ASSIGN_OR_RETURN(
            ValueConnection duration_connection,
            GetNodeValueByName("duration", node_data, converted_graph));

        MP_ASSIGN_OR_RETURN(ExecutableNodeConnection next_node,
                         GetNodeFlowByName(kDefaultOutputFlowSocket, node_data,
                                           converted_graph));

        MP_ASSIGN_OR_RETURN(ExecutableNodeConnection done_node,
                         GetNodeFlowByName(kDefaultOutputAsyncDoneSocket,
                                           node_data, converted_graph));

        flow_delay_node.node = ExecutableNode{
            .statement = AsyncCallStatement{
                .expression = {.name = "Delay", .args = {duration_connection}},
                .next_node = next_node,
                .on_done_node = done_node}};

        return absl::OkStatus();
      }};
}

NodeConverter GetFlowBranchConverter() {
  return NodeConverter{
      .node_type = "flow/branch",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & flow_branch_node,
                         converted_graph.GetNode(node_data.index));
        flow_branch_node.name = absl::StrFormat(
            "flow/branch [Behavior node id %d]", node_data.index);

        MP_ASSIGN_OR_RETURN(
            ValueConnection condition_connection,
            GetNodeValueByName("condition", node_data, converted_graph));

        MP_ASSIGN_OR_RETURN(ExecutableNodeConnection true_next_node,
                         GetNodeFlowByName("true", node_data, converted_graph));

        MP_ASSIGN_OR_RETURN(
            ExecutableNodeConnection false_next_node,
            GetNodeFlowByName("false", node_data, converted_graph));

        flow_branch_node.node = ExecutableNode{
            .statement = BranchStatement{.test = condition_connection,
                                         .true_next_node = true_next_node,
                                         .false_next_node = false_next_node}};

        return absl::OkStatus();
      }};
}

NodeConverter GetFlowStopAudioConverter() {
  return NodeConverter{
      .node_type = "flow/stopAudio",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & stop_audio_node,
                         converted_graph.GetNode(node_data.index));
        stop_audio_node.name = absl::StrFormat(
            "flow/stopAudio [Behavior node id %d]", node_data.index);

        MP_ASSIGN_OR_RETURN(
            ValueConnection handle_connection,
            GetNodeValueByName("handle", node_data, converted_graph));

        MP_ASSIGN_OR_RETURN(ExecutableNodeConnection next_node,
                         GetNodeFlowByName("out", node_data, converted_graph));

        stop_audio_node.node = ExecutableNode{
            .statement = CallStatement{
                .expression = {.name = "StopGltfAudio",
                               .args =
                                   {
                                       GetNodeSelfConnection(converted_graph),
                                       handle_connection,
                                   }},
                .next_node = next_node}};
        return absl::OkStatus();
      }};
}

NodeConverter GetAsyncPlaySoundConverter() {
  return NodeConverter{
      .node_type = "async/playSound",
      .function = [](const BehaviorData::NodeData& node_data,
                     ConvertedGraph& converted_graph) -> absl::Status {
        MP_ASSIGN_OR_RETURN(RecipeNode & async_play_sound_node,
                         converted_graph.GetNode(node_data.index));
        async_play_sound_node.name = absl::StrFormat(
            "async/playSound [Behavior node id %d]", node_data.index);

        MP_ASSIGN_OR_RETURN(
            ValueConnection index_connection,
            GetNodeValueByName("sound", node_data, converted_graph));

        MP_ASSIGN_OR_RETURN(ExecutableNodeConnection next_node,
                         GetNodeFlowByName("out", node_data, converted_graph));

        MP_ASSIGN_OR_RETURN(ExecutableNodeConnection done_node,
                         GetNodeFlowByName("done", node_data, converted_graph));

        async_play_sound_node.node = ExecutableNode{
            .statement = AsyncCallStatement{
                .expression = {.name = "PlayGltfAudio",
                               .args =
                                   {
                                       GetNodeSelfConnection(converted_graph),
                                       index_connection,
                                   }},
                .next_node = next_node,
                .on_done_node = done_node}};
        return absl::OkStatus();
      }};
}

}  // namespace imp::gltf::behavior
