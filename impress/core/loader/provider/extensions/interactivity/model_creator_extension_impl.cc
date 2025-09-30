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

#include "core/loader/provider/extensions/interactivity/model_creator_extension_impl.h"

#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "core/loader/provider/extensions/interactivity/schemas/interactivity_generated.h"
#include "core/math/flatbuffer_support.h"
#include "core/math/vec.h"
#include "core/model/model_data.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::details {

namespace {

absl::StatusOr<model::ModelData::InteractivityData::VariableData>
ToVariableData(const schemas::InteractivityVariable* variable) {
  model::ModelData::InteractivityData::VariableData variable_data = {
      .id = variable->id()->str(),
  };

  switch (variable->value_type()) {
    case schemas::InteractivityValue::Bool: {
      variable_data.value = variable->value_as_Bool()->value();
      variable_data.type = model::ModelData::InteractivityData::ValueType::BOOL;
      break;
    }
    case schemas::InteractivityValue::Int: {
      variable_data.value = variable->value_as_Int()->value();
      variable_data.type = model::ModelData::InteractivityData::ValueType::INT;
      break;
    }
    case schemas::InteractivityValue::Float: {
      variable_data.value = variable->value_as_Float()->value();
      variable_data.type =
          model::ModelData::InteractivityData::ValueType::FLOAT;
      break;
    }
    case schemas::InteractivityValue::Float2: {
      const schemas::Float2* value = variable->value_as_Float2();
      variable_data.value = float2{value->x(), value->y()};
      variable_data.type =
          model::ModelData::InteractivityData::ValueType::FLOAT2;
      break;
    }
    case schemas::InteractivityValue::Float3: {
      const schemas::Float3* value = variable->value_as_Float3();
      variable_data.value = float3{value->x(), value->y(), value->z()};
      variable_data.type =
          model::ModelData::InteractivityData::ValueType::FLOAT3;
      break;
    }
    case schemas::InteractivityValue::Float4: {
      const schemas::Float4* value = variable->value_as_Float4();
      variable_data.value =
          float4{value->x(), value->y(), value->z(), value->w()};
      variable_data.type =
          model::ModelData::InteractivityData::ValueType::FLOAT4;
      break;
    }
    case schemas::InteractivityValue::Mat2f: {
      const schemas::Mat2f* mat = variable->value_as_Mat2f();
      variable_data.value = flatbuffers::UnPack(*mat);
      variable_data.type =
          model::ModelData::InteractivityData::ValueType::MAT2F;
      break;
    }
    case schemas::InteractivityValue::Mat3f: {
      const schemas::Mat3f* mat = variable->value_as_Mat3f();
      variable_data.value = flatbuffers::UnPack(*mat);
      variable_data.type =
          model::ModelData::InteractivityData::ValueType::MAT3F;
      break;
    }
    case schemas::InteractivityValue::Mat4f: {
      const schemas::Mat4f* mat = variable->value_as_Mat4f();
      variable_data.value = flatbuffers::UnPack(*mat);
      variable_data.type =
          model::ModelData::InteractivityData::ValueType::MAT4F;
      break;
    }
    case schemas::InteractivityValue::String: {
      variable_data.value = variable->value_as_String()->value()->str();
      variable_data.type =
          model::ModelData::InteractivityData::ValueType::STRING;
      break;
    }
    default:
      return absl::InternalError(
          absl::StrFormat("Invalid interactivity variable value type: %d",
                          variable->value_type()));
  }

  return variable_data;
}

}  // namespace

absl::StatusOr<model::ModelData::InteractivityData>
InteractivityModelCreatorExtensionImpl::DeserializeInteractivityData(
    const schemas::Interactivity* interactivity) {
  model::ModelData::InteractivityData interactivity_data;
  interactivity_data.graph_index = interactivity->graph();

  interactivity_data.graphs.reserve(interactivity->graphs()->size());

  for (const schemas::InteractivityGraph* graph : *interactivity->graphs()) {
    interactivity_data.graphs.push_back({});
    model::ModelData::InteractivityData::GraphData& graph_data =
        interactivity_data.graphs.back();

    graph_data.types.reserve(graph->types()->size());
    for (const schemas::InteractivityGraphTypeData* type : *graph->types()) {
      model::ModelData::InteractivityData::TypeData type_data;

      switch (type->type()) {
        case schemas::InteractivityVariableType::BOOL:
          type_data.type = model::ModelData::InteractivityData::ValueType::BOOL;
          break;
        case schemas::InteractivityVariableType::INT:
          type_data.type = model::ModelData::InteractivityData::ValueType::INT;
          break;
        case schemas::InteractivityVariableType::FLOAT:
          type_data.type =
              model::ModelData::InteractivityData::ValueType::FLOAT;
          break;
        case schemas::InteractivityVariableType::FLOAT2:
          type_data.type =
              model::ModelData::InteractivityData::ValueType::FLOAT2;
          break;
        case schemas::InteractivityVariableType::FLOAT3:
          type_data.type =
              model::ModelData::InteractivityData::ValueType::FLOAT3;
          break;
        case schemas::InteractivityVariableType::FLOAT4:
          type_data.type =
              model::ModelData::InteractivityData::ValueType::FLOAT4;
          break;
        case schemas::InteractivityVariableType::MAT2F:
          type_data.type =
              model::ModelData::InteractivityData::ValueType::MAT2F;
          break;
        case schemas::InteractivityVariableType::MAT3F:
          type_data.type =
              model::ModelData::InteractivityData::ValueType::MAT3F;
          break;
        case schemas::InteractivityVariableType::MAT4F:
          type_data.type =
              model::ModelData::InteractivityData::ValueType::MAT4F;
          break;
        default:
          // TODO: Add support for passing custom types through
          // loader.
          return absl::InternalError(absl::StrFormat(
              "Invalid interactivity graph type: %d", type->type()));
      }

      graph_data.types.push_back(std::move(type_data));
    }

    graph_data.nodes.reserve(graph->nodes()->size());
    for (const schemas::InteractivityNode* node : *graph->nodes()) {
      std::vector<model::ModelData::InteractivityData::NodeData::FlowData>
          flows;
      flows.reserve(node->flows()->size());
      for (const schemas::InteractivityNodeFlow* flow : *node->flows()) {
        flows.push_back({
            .id = flow->id()->str(),
            .node = static_cast<int>(flow->node()),
            .socket = flow->socket()->str(),
        });
      }

      std::vector<
          model::ModelData::InteractivityData::NodeData::ConfigurationData>
          configurations;
      configurations.reserve(node->configuration()->size());
      for (const schemas::InteractivityNodeConfiguration* config :
           *node->configuration()) {
        model::ModelData::InteractivityData::NodeData::ConfigurationData
            configuration_data;

        switch (config->id()) {
          case schemas::InteractivityNodeConfigurationType::
              NUMBER_OF_OUTPUT_FLOWS:
            configuration_data.id = model::ModelData::InteractivityData::
                NodeData::ConfigurationType::NUMBER_OF_OUTPUT_FLOWS;
            break;
          case schemas::InteractivityNodeConfigurationType::VARIABLE:
            configuration_data.id = model::ModelData::InteractivityData::
                NodeData::ConfigurationType::VARIABLE;
            break;
          case schemas::InteractivityNodeConfigurationType::POINTER:
            configuration_data.id = model::ModelData::InteractivityData::
                NodeData::ConfigurationType::POINTER;
            break;
          case schemas::InteractivityNodeConfigurationType::EVENT:
            configuration_data.id = model::ModelData::InteractivityData::
                NodeData::ConfigurationType::EVENT;
            break;
          case schemas::InteractivityNodeConfigurationType::NODE_INDEX:
            configuration_data.id = model::ModelData::InteractivityData::
                NodeData::ConfigurationType::NODE_INDEX;
            break;
          case schemas::InteractivityNodeConfigurationType::STOP_PROPAGATION:
            configuration_data.id = model::ModelData::InteractivityData::
                NodeData::ConfigurationType::STOP_PROPAGATION;
            break;
          case schemas::InteractivityNodeConfigurationType::EASING_TYPE:
            configuration_data.id = model::ModelData::InteractivityData::
                NodeData::ConfigurationType::EASING_TYPE;
            break;
          case schemas::InteractivityNodeConfigurationType::EASING_DURATION:
            configuration_data.id = model::ModelData::InteractivityData::
                NodeData::ConfigurationType::EASING_DURATION;
            break;
          case schemas::InteractivityNodeConfigurationType::CASES:
            configuration_data.id = model::ModelData::InteractivityData::
                NodeData::ConfigurationType::CASES;
            break;
          case schemas::InteractivityNodeConfigurationType::TYPE:
            configuration_data.id = model::ModelData::InteractivityData::
                NodeData::ConfigurationType::TYPE;
            break;
          case schemas::InteractivityNodeConfigurationType::
              NUMBER_OF_INPUT_FLOWS:
            configuration_data.id = model::ModelData::InteractivityData::
                NodeData::ConfigurationType::NUMBER_OF_INPUT_FLOWS;
            break;
          case schemas::InteractivityNodeConfigurationType::IS_RANDOM:
            configuration_data.id = model::ModelData::InteractivityData::
                NodeData::ConfigurationType::IS_RANDOM;
            break;
          case schemas::InteractivityNodeConfigurationType::IS_LOOP:
            configuration_data.id = model::ModelData::InteractivityData::
                NodeData::ConfigurationType::IS_LOOP;
            break;
          case schemas::InteractivityNodeConfigurationType::MESSAGE:
            configuration_data.id = model::ModelData::InteractivityData::
                NodeData::ConfigurationType::MESSAGE;
            break;
          case schemas::InteractivityNodeConfigurationType::INITIAL_INDEX:
            configuration_data.id = model::ModelData::InteractivityData::
                NodeData::ConfigurationType::INITIAL_INDEX;
            break;
        }

        // Second switch statement to reduce the amount of duplicated code
        // if they're combined
        switch (config->id()) {
          case schemas::InteractivityNodeConfigurationType::
              NUMBER_OF_OUTPUT_FLOWS:
          case schemas::InteractivityNodeConfigurationType::
              NUMBER_OF_INPUT_FLOWS:
          case schemas::InteractivityNodeConfigurationType::EVENT:
          case schemas::InteractivityNodeConfigurationType::NODE_INDEX:
          case schemas::InteractivityNodeConfigurationType::TYPE:
          case schemas::InteractivityNodeConfigurationType::INITIAL_INDEX:
            if (!config->value_as_Int()) {
              return absl::InternalError(absl::StrFormat(
                  "Interactivity node configuration indicates an int value is "
                  "needed for id %d, but it's unset",
                  config->id()));
            }
            configuration_data.value = config->value_as_Int()->value();
            break;
          case schemas::InteractivityNodeConfigurationType::VARIABLE:
          case schemas::InteractivityNodeConfigurationType::POINTER:
          case schemas::InteractivityNodeConfigurationType::EASING_TYPE:
          case schemas::InteractivityNodeConfigurationType::MESSAGE:
            if (!config->value_as_String() ||
                !config->value_as_String()->value()) {
              return absl::InternalError(absl::StrFormat(
                  "Interactivity node configuration indicates a string is "
                  "needed for id %d, but value is unset",
                  config->id()));
            }
            configuration_data.value =
                config->value_as_String()->value()->str();
            break;
          case schemas::InteractivityNodeConfigurationType::STOP_PROPAGATION:
          case schemas::InteractivityNodeConfigurationType::IS_RANDOM:
          case schemas::InteractivityNodeConfigurationType::IS_LOOP:
            if (!config->value_as_Bool()) {
              return absl::InternalError(absl::StrFormat(
                  "Interactivity node configuration indicates an bool value "
                  "is needed for id %d, but it's unset",
                  config->id()));
            }
            configuration_data.value = config->value_as_Bool()->value();
            break;
          case schemas::InteractivityNodeConfigurationType::EASING_DURATION:
            if (!config->value_as_Float()) {
              return absl::InternalError(absl::StrFormat(
                  "Interactivity node configuration indicates an float value "
                  "is needed for id %d, but it's unset",
                  config->id()));
            }
            configuration_data.value = config->value_as_Float()->value();
            break;
          case schemas::InteractivityNodeConfigurationType::CASES:
            if (!config->value_as_IntArray()) {
              return absl::InternalError(absl::StrFormat(
                  "Interactivity node configuration indicates an int array "
                  "value is needed for id %d, but it's unset",
                  config->id()));
            }
            configuration_data.value =
                std::vector<int>(config->value_as_IntArray()->values()->begin(),
                                 config->value_as_IntArray()->values()->end());
            break;
        }

        configurations.push_back(configuration_data);
      }

      std::vector<model::ModelData::InteractivityData::NodeData::ValueData>
          values;
      values.reserve(node->values()->size());
      for (const schemas::InteractivityNodeValue* value : *node->values()) {
        if (value->value_type() ==
            schemas::InteractivityNodeValueType::InteractivityVariable) {
          const schemas::InteractivityVariable* variable =
              value->value_as_InteractivityVariable();

          MP_ASSIGN_OR_RETURN(
              model::ModelData::InteractivityData::VariableData value_data,
              ToVariableData(variable));
          values.push_back(value_data);
        } else if (value->value_type() ==
                   schemas::InteractivityNodeValueType::InteractivityNodeFlow) {
          const schemas::InteractivityNodeFlow* flow =
              value->value_as_InteractivityNodeFlow();
          values.push_back(
              model::ModelData::InteractivityData::NodeData::FlowData{
                  .id = flow->id()->str(),
                  .node = static_cast<int>(flow->node()),
                  .socket = flow->socket()->str(),
              });
        } else {
          return absl::InternalError(absl::StrFormat(
              "Unhandled interactivity node value value type: %d",
              value->value_type()));
        }
      }

      graph_data.nodes.push_back({
          .type =
              (*graph->declarations())[node->declaration()] -> op() -> str(),
          .index = static_cast<int>(node->index()),
          .flows = std::move(flows),
          .configuration = std::move(configurations),
          .values = std::move(values),
      });
    }

    graph_data.variables.reserve(graph->variables()->size());
    for (const schemas::InteractivityVariable* variable : *graph->variables()) {
      MP_ASSIGN_OR_RETURN(
          model::ModelData::InteractivityData::VariableData variable_data,
          ToVariableData(variable));
      graph_data.variables.push_back(std::move(variable_data));
    }

    graph_data.events.reserve(graph->events()->size());
    for (const schemas::InteractivityEvent* event : *graph->events()) {
      std::vector<model::ModelData::InteractivityData::VariableData> values;
      values.reserve(event->values()->size());
      for (const schemas::InteractivityVariable* variable : *event->values()) {
        MP_ASSIGN_OR_RETURN(
            model::ModelData::InteractivityData::VariableData variable_data,
            ToVariableData(variable));
        values.push_back(std::move(variable_data));
      }
      graph_data.events.push_back({
          .id = event->id()->str(),
          .values = std::move(values),
      });
    }

    graph_data.declarations.reserve(graph->declarations()->size());
    for (const schemas::Declaration* declaration : *graph->declarations()) {
      graph_data.declarations.push_back({
          .op = declaration->op()->str(),
      });
    }
  }

  return interactivity_data;
}

}  // namespace imp::loader::details
