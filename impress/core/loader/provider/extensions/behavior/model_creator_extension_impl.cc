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

#include "core/loader/provider/extensions/behavior/model_creator_extension_impl.h"

#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/math/flatbuffer_support.h"
#include "core/math/vec.h"
#include "core/model/model_data.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::details {

namespace {

absl::StatusOr<model::ModelData::BehaviorData::VariableData> ToVariableData(
    const schemas::BehaviorVariable* variable) {
  model::ModelData::BehaviorData::VariableData variable_data = {
      .id = variable->id()->str(),
  };

  switch (variable->value_type()) {
    case schemas::BehaviorValue::Bool: {
      variable_data.value = variable->value_as_Bool()->value();
      variable_data.type = model::ModelData::BehaviorData::ValueType::BOOL;
      break;
    }
    case schemas::BehaviorValue::Int: {
      variable_data.value = variable->value_as_Int()->value();
      variable_data.type = model::ModelData::BehaviorData::ValueType::INT;
      break;
    }
    case schemas::BehaviorValue::Float: {
      variable_data.value = variable->value_as_Float()->value();
      variable_data.type = model::ModelData::BehaviorData::ValueType::FLOAT;
      break;
    }
    case schemas::BehaviorValue::Float2: {
      const schemas::Float2* value = variable->value_as_Float2();
      variable_data.value = float2{value->x(), value->y()};
      variable_data.type = model::ModelData::BehaviorData::ValueType::FLOAT2;
      break;
    }
    case schemas::BehaviorValue::Float3: {
      const schemas::Float3* value = variable->value_as_Float3();
      variable_data.value = float3{value->x(), value->y(), value->z()};
      variable_data.type = model::ModelData::BehaviorData::ValueType::FLOAT3;
      break;
    }
    case schemas::BehaviorValue::Float4: {
      const schemas::Float4* value = variable->value_as_Float4();
      variable_data.value =
          float4{value->x(), value->y(), value->z(), value->w()};
      variable_data.type = model::ModelData::BehaviorData::ValueType::FLOAT4;
      break;
    }
    case schemas::BehaviorValue::Mat4f: {
      const schemas::Mat4f* mat = variable->value_as_Mat4f();
      variable_data.value = flatbuffers::UnPack(*mat);
      variable_data.type = model::ModelData::BehaviorData::ValueType::MAT4F;
      break;
    }
    case schemas::BehaviorValue::String: {
      variable_data.value = variable->value_as_String()->value()->str();
      variable_data.type = model::ModelData::BehaviorData::ValueType::STRING;
      break;
    }
    default:
      return absl::InternalError(absl::StrFormat(
          "Invalid behavior variable value type: %d", variable->value_type()));
  }

  return variable_data;
}

}  // namespace

absl::StatusOr<model::ModelData::BehaviorData>
BehaviorModelCreatorExtensionImpl::DeserializeBehaviorData(
    const schemas::Behavior* behavior) {
  model::ModelData::BehaviorData behavior_data;

  behavior_data.nodes.reserve(behavior->nodes()->size());
  for (const schemas::BehaviorNode* node : *behavior->nodes()) {
    std::vector<model::ModelData::BehaviorData::NodeData::FlowData> flows;
    flows.reserve(node->flows()->size());
    for (const schemas::BehaviorNodeFlow* flow : *node->flows()) {
      flows.push_back({
          .id = flow->id()->str(),
          .node = static_cast<int>(flow->node()),
          .socket = flow->socket()->str(),
      });
    }

    std::vector<model::ModelData::BehaviorData::NodeData::ConfigurationData>
        configurations;
    configurations.reserve(node->configuration()->size());
    for (const schemas::BehaviorNodeConfiguration* config :
         *node->configuration()) {
      model::ModelData::BehaviorData::NodeData::ConfigurationData
          configuration_data;
      switch (config->type()) {
        case schemas::BehaviorVariableType::BOOL:
          configuration_data.value_type =
              model::ModelData::BehaviorData::ValueType::BOOL;
          break;
        case schemas::BehaviorVariableType::INT:
          configuration_data.value_type =
              model::ModelData::BehaviorData::ValueType::INT;
          break;
        case schemas::BehaviorVariableType::FLOAT:
          configuration_data.value_type =
              model::ModelData::BehaviorData::ValueType::FLOAT;
          break;
        case schemas::BehaviorVariableType::NIL:
          // Do nothing, as a nil type is indicated.
          break;
      }

      switch (config->id()) {
        case schemas::BehaviorNodeConfigurationType::NUMBER_OF_OUTPUT_FLOWS:
          configuration_data.id = model::ModelData::BehaviorData::NodeData::
              ConfigurationType::NUMBER_OF_OUTPUT_FLOWS;
          break;
        case schemas::BehaviorNodeConfigurationType::VARIABLE:
          configuration_data.id = model::ModelData::BehaviorData::NodeData::
              ConfigurationType::VARIABLE;
          break;
        case schemas::BehaviorNodeConfigurationType::PATH:
          configuration_data.id =
              model::ModelData::BehaviorData::NodeData::ConfigurationType::PATH;
          break;
        case schemas::BehaviorNodeConfigurationType::CUSTOM_EVENT:
          configuration_data.id = model::ModelData::BehaviorData::NodeData::
              ConfigurationType::CUSTOM_EVENT;
          break;
        case schemas::BehaviorNodeConfigurationType::NODE_INDEX:
          configuration_data.id = model::ModelData::BehaviorData::NodeData::
              ConfigurationType::NODE_INDEX;
          break;
        case schemas::BehaviorNodeConfigurationType::STOP_PROPAGATION:
          configuration_data.id = model::ModelData::BehaviorData::NodeData::
              ConfigurationType::STOP_PROPAGATION;
          break;
        case schemas::BehaviorNodeConfigurationType::EASING_TYPE:
          configuration_data.id = model::ModelData::BehaviorData::NodeData::
              ConfigurationType::EASING_TYPE;
          break;
        case schemas::BehaviorNodeConfigurationType::EASING_DURATION:
          configuration_data.id = model::ModelData::BehaviorData::NodeData::
              ConfigurationType::EASING_DURATION;
          break;
      }

      // Second switch statement to reduce the amount of duplicated code
      // if they're combined
      switch (config->id()) {
        case schemas::BehaviorNodeConfigurationType::NUMBER_OF_OUTPUT_FLOWS:
        case schemas::BehaviorNodeConfigurationType::CUSTOM_EVENT:
        case schemas::BehaviorNodeConfigurationType::NODE_INDEX:
          if (!config->value_as_Int()) {
            return absl::InternalError(absl::StrFormat(
                "Behavior node configuration indicates an int value is "
                "needed for id %d, but it's unset",
                config->id()));
          }
          configuration_data.value = config->value_as_Int()->value();
          break;
        case schemas::BehaviorNodeConfigurationType::VARIABLE:
        case schemas::BehaviorNodeConfigurationType::PATH:
        case schemas::BehaviorNodeConfigurationType::EASING_TYPE:
          if (!config->value_as_String() ||
              !config->value_as_String()->value()) {
            return absl::InternalError(absl::StrFormat(
                "Behavior node configuration indicates a string is "
                "needed for id %d, but value is unset",
                config->id()));
          }
          configuration_data.value = config->value_as_String()->value()->str();
          break;
        case schemas::BehaviorNodeConfigurationType::STOP_PROPAGATION:
          if (!config->value_as_Bool()) {
            return absl::InternalError(absl::StrFormat(
                "Behavior node configuration indicates an bool value "
                "is needed for id %d, but it's unset",
                config->id()));
          }
          configuration_data.value = config->value_as_Bool()->value();
          break;
        case schemas::BehaviorNodeConfigurationType::EASING_DURATION:
          if (!config->value_as_Float()) {
            return absl::InternalError(absl::StrFormat(
                "Behavior node configuration indicates an float value "
                "is needed for id %d, but it's unset",
                config->id()));
          }
          configuration_data.value = config->value_as_Float()->value();
          break;
      }

      configurations.push_back(configuration_data);
    }

    std::vector<model::ModelData::BehaviorData::NodeData::ValueData> values;
    values.reserve(node->values()->size());
    for (const schemas::BehaviorNodeValue* value : *node->values()) {
      if (value->value_type() ==
          schemas::BehaviorNodeValueType::BehaviorVariable) {
        const schemas::BehaviorVariable* variable =
            value->value_as_BehaviorVariable();

        MP_ASSIGN_OR_RETURN(
            model::ModelData::BehaviorData::VariableData value_data,
            ToVariableData(variable));
        values.push_back(value_data);
      } else if (value->value_type() ==
                 schemas::BehaviorNodeValueType::BehaviorNodeFlow) {
        const schemas::BehaviorNodeFlow* flow =
            value->value_as_BehaviorNodeFlow();
        values.push_back(model::ModelData::BehaviorData::NodeData::FlowData{
            .id = flow->id()->str(),
            .node = static_cast<int>(flow->node()),
            .socket = flow->socket()->str(),
        });
      } else {
        return absl::InternalError(
            absl::StrFormat("Unhandled behavior node value value type: %d",
                            value->value_type()));
      }
    }

    behavior_data.nodes.push_back({
        .type = node->type()->str(),
        .index = static_cast<int>(node->index()),
        .flows = std::move(flows),
        .configuration = std::move(configurations),
        .values = std::move(values),
    });
  }

  behavior_data.variables.reserve(behavior->variables()->size());
  for (const schemas::BehaviorVariable* variable : *behavior->variables()) {
    MP_ASSIGN_OR_RETURN(model::ModelData::BehaviorData::VariableData variable_data,
                     ToVariableData(variable));
    behavior_data.variables.push_back(std::move(variable_data));
  }

  behavior_data.custom_events.reserve(behavior->custom_events()->size());
  for (const schemas::BehaviorCustomEvent* custom_event :
       *behavior->custom_events()) {
    std::vector<model::ModelData::BehaviorData::VariableData> values;
    values.reserve(custom_event->values()->size());
    for (const schemas::BehaviorVariable* variable : *custom_event->values()) {
      MP_ASSIGN_OR_RETURN(
          model::ModelData::BehaviorData::VariableData variable_data,
          ToVariableData(variable));
      values.push_back(std::move(variable_data));
    }
    behavior_data.custom_events.push_back({
        .id = custom_event->id()->str(),
        .values = std::move(values),
    });
  }

  return behavior_data;
}

}  // namespace imp::loader::details
