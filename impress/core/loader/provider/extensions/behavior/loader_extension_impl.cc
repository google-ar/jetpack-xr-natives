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

#include "core/loader/provider/extensions/behavior/loader_extension_impl.h"

#include <cstdint>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/variant.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/string.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/paired_vector.h"
#include "core/common/schemas/math_generated.h"
#include "core/loader/provider/extensions/behavior/behavior.proto.imp.h"
#include "core/loader/provider/extensions/behavior/behavior_helpers.h"
#include "core/loader/provider/extensions/behavior/loader_extension.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/math/flatbuffer_support.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/model/model_data.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::details {

namespace {
using flatbuffers::FlatBufferBuilder;
using flatbuffers::Offset;
using flatbuffers::String;

using BehaviorNodeFlowId = model::ModelData::BehaviorNodeFlowId;
using BehaviorNodeFlowOffset = Offset<schemas::BehaviorNodeFlow>;
using BehaviorNodeFlowOffsets =
    PairedVector<BehaviorNodeFlowOffset, BehaviorNodeFlowId::ReferredType>;

using BehaviorNodeConfigurationId =
    model::ModelData::BehaviorNodeConfigurationId;
using BehaviorNodeConfigurationOffset =
    Offset<schemas::BehaviorNodeConfiguration>;
using BehaviorNodeConfigurationOffsets =
    PairedVector<BehaviorNodeConfigurationOffset,
                 BehaviorNodeConfigurationId::ReferredType>;

using BehaviorNodeValueId = model::ModelData::BehaviorNodeValueId;
using BehaviorNodeValueOffset = Offset<schemas::BehaviorNodeValue>;
using BehaviorNodeValueOffsets =
    PairedVector<BehaviorNodeValueOffset, BehaviorNodeValueId::ReferredType>;

using BehaviorNodeId = model::ModelData::BehaviorNodeId;
using BehaviorNodeOffset = Offset<schemas::BehaviorNode>;
using BehaviorNodeOffsets =
    PairedVector<BehaviorNodeOffset, BehaviorNodeId::ReferredType>;

using BehaviorVariableId = model::ModelData::BehaviorVariableId;
using BehaviorVariableOffset = Offset<schemas::BehaviorVariable>;
using BehaviorVariableOffsets =
    PairedVector<BehaviorVariableOffset, BehaviorVariableId::ReferredType>;

using BehaviorCustomEventId = model::ModelData::BehaviorCustomEventId;
using BehaviorCustomEventOffset = Offset<schemas::BehaviorCustomEvent>;
using BehaviorCustomEventOffsets =
    PairedVector<BehaviorCustomEventOffset,
                 BehaviorCustomEventId::ReferredType>;

using BehaviorValue = std::variant<absl::monostate, bool, int, float, float2,
                                   float3, float4, mat4f, absl::string_view>;
using BehaviorConfiguration = std::variant<int, absl::string_view, bool, float>;

BehaviorNodeFlowOffset CreateBehaviorNodeFlow(FlatBufferBuilder& fbb,
                                              absl::string_view id, int node,
                                              absl::string_view socket) {
  return schemas::CreateBehaviorNodeFlow(
      fbb, fbb.CreateString(id.data(), id.size()), static_cast<uint32_t>(node),
      fbb.CreateString(socket.data(), socket.size()));
}

BehaviorNodeConfigurationOffset CreateBehaviorNodeConfiguration(
    FlatBufferBuilder& fbb, schemas::BehaviorNodeConfigurationType id,
    schemas::BehaviorVariableType type, BehaviorConfiguration value) {
  struct Visitor {
    FlatBufferBuilder& fbb;
    schemas::BehaviorNodeConfigurationType id;
    schemas::BehaviorVariableType type;

    BehaviorNodeConfigurationOffset operator()(const int value) {
      return schemas::CreateBehaviorNodeConfiguration(
          fbb, id, type, schemas::BehaviorConfigurationValue::Int,
          fbb.CreateStruct(schemas::Int(value)).Union());
    }
    BehaviorNodeConfigurationOffset operator()(const bool value) {
      return schemas::CreateBehaviorNodeConfiguration(
          fbb, id, type, schemas::BehaviorConfigurationValue::Bool,
          fbb.CreateStruct(schemas::Bool(value)).Union());
    }
    BehaviorNodeConfigurationOffset operator()(const float value) {
      return schemas::CreateBehaviorNodeConfiguration(
          fbb, id, type, schemas::BehaviorConfigurationValue::Float,
          fbb.CreateStruct(schemas::Float(value)).Union());
    }
    BehaviorNodeConfigurationOffset operator()(const absl::string_view value) {
      // schemas::CreateString is creating a "String" table wrapping the
      // actual Flatbuffer string.
      return schemas::CreateBehaviorNodeConfiguration(
          fbb, id, type, schemas::BehaviorConfigurationValue::String,
          schemas::CreateString(fbb,
                                fbb.CreateString(value.data(), value.size()))
              .Union());
    }
  };

  return std::visit(Visitor{fbb, id, type}, value);
}

BehaviorNodeOffset CreateBehaviorNode(
    FlatBufferBuilder& fbb, absl::string_view type, int index,
    const BehaviorNodeFlowOffsets& flows,
    const BehaviorNodeConfigurationOffsets& configurations,
    const BehaviorNodeValueOffsets& values) {
  return schemas::CreateBehaviorNode(
      fbb, fbb.CreateString(type.data(), type.size()),
      static_cast<uint32_t>(index),
      CreateVector<schemas::BehaviorNodeFlow>(fbb, flows),
      CreateVector<schemas::BehaviorNodeConfiguration>(fbb, configurations),
      CreateVector<schemas::BehaviorNodeValue>(fbb, values));
}

BehaviorVariableOffset CreateBehaviorVariable(FlatBufferBuilder& fbb,
                                              absl::string_view id,
                                              BehaviorValue value) {
  struct Visitor {
    flatbuffers::FlatBufferBuilder& fbb;
    Offset<String> id;

    BehaviorVariableOffset operator()(const bool value) {
      return schemas::CreateBehaviorVariable(
          fbb, id, schemas::BehaviorValue::Bool,
          fbb.CreateStruct(schemas::Bool(value)).Union());
    }
    BehaviorVariableOffset operator()(const int value) {
      return schemas::CreateBehaviorVariable(
          fbb, id, schemas::BehaviorValue::Int,
          fbb.CreateStruct(schemas::Int(value)).Union());
    }
    BehaviorVariableOffset operator()(const float value) {
      return schemas::CreateBehaviorVariable(
          fbb, id, schemas::BehaviorValue::Float,
          fbb.CreateStruct(schemas::Float(value)).Union());
    }
    BehaviorVariableOffset operator()(const float2 value) {
      return schemas::CreateBehaviorVariable(
          fbb, id, schemas::BehaviorValue::Float2,
          fbb.CreateStruct(flatbuffers::Pack(value)).Union());
    }
    BehaviorVariableOffset operator()(const float3 value) {
      return schemas::CreateBehaviorVariable(
          fbb, id, schemas::BehaviorValue::Float3,
          fbb.CreateStruct(flatbuffers::Pack(value)).Union());
    }
    BehaviorVariableOffset operator()(const float4 value) {
      return schemas::CreateBehaviorVariable(
          fbb, id, schemas::BehaviorValue::Float4,
          fbb.CreateStruct(schemas::Float4(flatbuffers::Pack(value))).Union());
    }
    BehaviorVariableOffset operator()(const mat4f value) {
      return schemas::CreateBehaviorVariable(
          fbb, id, schemas::BehaviorValue::Mat4f,
          fbb.CreateStruct(schemas::Mat4f(flatbuffers::Pack(value))).Union());
    }
    BehaviorVariableOffset operator()(const absl::string_view value) {
      return schemas::CreateBehaviorVariable(
          fbb, id, schemas::BehaviorValue::String,
          schemas::CreateString(fbb,
                                fbb.CreateString(value.data(), value.size()))
              .Union());
    }
    BehaviorVariableOffset operator()(const absl::monostate value) { return 0; }
  };

  return std::visit(Visitor{fbb, fbb.CreateString(id.data(), id.size())},
                    value);
}

absl::Status CreateBehaviorNodeFlows(
    FlatBufferBuilder& fbb, std::vector<imp::gltf::Behavior::Node::Flow> flows,
    BehaviorNodeFlowOffsets& out_flow_offsets) {
  for (const auto& flow : flows) {
    out_flow_offsets.push_back(
        CreateBehaviorNodeFlow(fbb, flow.id, flow.node, flow.socket));
  }

  return absl::OkStatus();
}

// An out parameter is used instead of returning the Offset as it's a large copy
absl::Status CreateBehaviorNodeConfigurations(
    FlatBufferBuilder& fbb,
    std::vector<imp::gltf::Behavior::Node::Configuration> configurations,
    BehaviorNodeConfigurationOffsets& out_configuration_offsets) {
  for (const auto& configuration : configurations) {
    schemas::BehaviorVariableType configuration_type;
    switch (configuration.type) {
      case imp::gltf::Behavior::ValueType::BOOL:
        configuration_type = schemas::BehaviorVariableType::BOOL;
        break;
      case imp::gltf::Behavior::ValueType::INT:
        configuration_type = schemas::BehaviorVariableType::INT;
        break;
      case imp::gltf::Behavior::ValueType::FLOAT:
        configuration_type = schemas::BehaviorVariableType::FLOAT;
        break;
      default:
        configuration_type = schemas::BehaviorVariableType::NIL;
        break;
    }

    switch (configuration.id) {
      case imp::gltf::Behavior::Node::ConfigurationType::NUMBER_OF_OUTPUT_FLOWS:
        if (!std::get_if<int>(&configuration.value)) {
          return absl::InternalError(absl::StrFormat(
              "Behavior configuration id %d has no value", configuration.id));
        }
        out_configuration_offsets.push_back(CreateBehaviorNodeConfiguration(
            fbb, schemas::BehaviorNodeConfigurationType::NUMBER_OF_OUTPUT_FLOWS,
            configuration_type, std::get<int>(configuration.value)));
        break;
      case imp::gltf::Behavior::Node::ConfigurationType::VARIABLE:
        if (!std::get_if<std::string>(&configuration.value)) {
          return absl::InternalError(absl::StrFormat(
              "Behavior configuration id %d has no value", configuration.id));
        }
        out_configuration_offsets.push_back(CreateBehaviorNodeConfiguration(
            fbb, schemas::BehaviorNodeConfigurationType::VARIABLE,
            configuration_type, std::get<std::string>(configuration.value)));
        break;
      case imp::gltf::Behavior::Node::ConfigurationType::PATH:
        if (!std::get_if<std::string>(&configuration.value)) {
          return absl::InternalError(absl::StrFormat(
              "Behavior configuration id %d has no value", configuration.id));
        }
        out_configuration_offsets.push_back(CreateBehaviorNodeConfiguration(
            fbb, schemas::BehaviorNodeConfigurationType::PATH,
            configuration_type, std::get<std::string>(configuration.value)));
        break;
      case imp::gltf::Behavior::Node::ConfigurationType::CUSTOM_EVENT:
        if (!std::get_if<int>(&configuration.value)) {
          return absl::InternalError(absl::StrFormat(
              "Behavior configuration id %d has no value", configuration.id));
        }
        out_configuration_offsets.push_back(CreateBehaviorNodeConfiguration(
            fbb, schemas::BehaviorNodeConfigurationType::CUSTOM_EVENT,
            configuration_type, std::get<int>(configuration.value)));
        break;
      case imp::gltf::Behavior::Node::ConfigurationType::NODE_INDEX:
        if (!std::get_if<int>(&configuration.value)) {
          return absl::InternalError(absl::StrFormat(
              "Behavior configuration id %d has no value", configuration.id));
        }
        out_configuration_offsets.push_back(CreateBehaviorNodeConfiguration(
            fbb, schemas::BehaviorNodeConfigurationType::NODE_INDEX,
            configuration_type, std::get<int>(configuration.value)));
        break;
      case imp::gltf::Behavior::Node::ConfigurationType::STOP_PROPAGATION:
        if (!std::get_if<bool>(&configuration.value)) {
          return absl::InternalError(absl::StrFormat(
              "Behavior configuration id %d has no value", configuration.id));
        }
        out_configuration_offsets.push_back(CreateBehaviorNodeConfiguration(
            fbb, schemas::BehaviorNodeConfigurationType::STOP_PROPAGATION,
            configuration_type, std::get<bool>(configuration.value)));
        break;
      case imp::gltf::Behavior::Node::ConfigurationType::EASING_TYPE:
        if (!std::get_if<std::string>(&configuration.value)) {
          return absl::InternalError(absl::StrFormat(
              "Behavior configuration id %d has no value", configuration.id));
        }
        out_configuration_offsets.push_back(CreateBehaviorNodeConfiguration(
            fbb, schemas::BehaviorNodeConfigurationType::EASING_TYPE,
            configuration_type, std::get<std::string>(configuration.value)));
        break;
      case imp::gltf::Behavior::Node::ConfigurationType::EASING_DURATION:
        if (!std::get_if<float>(&configuration.value)) {
          return absl::InternalError(absl::StrFormat(
              "Behavior configuration id %d has no value", configuration.id));
        }
        out_configuration_offsets.push_back(CreateBehaviorNodeConfiguration(
            fbb, schemas::BehaviorNodeConfigurationType::EASING_DURATION,
            configuration_type, std::get<float>(configuration.value)));
        break;
      default:
        return absl::InternalError(absl::StrFormat(
            "Invalid behavior configuration id with id %d", configuration.id));
    }
  }

  return absl::OkStatus();
}

// An out parameter is used instead of returning the Offset as it's a large copy
absl::Status CreateBehaviorNodeValues(
    FlatBufferBuilder& fbb,
    std::vector<imp::gltf::Behavior::Node::Value> values,
    BehaviorNodeValueOffsets& out_value_offsets) {
  for (const auto& value : values) {
    struct Visitor {
      flatbuffers::FlatBufferBuilder& fbb;

      BehaviorNodeValueOffset operator()(
          const gltf::Behavior::Node::Flow& flow) {
        return schemas::CreateBehaviorNodeValue(
            fbb, schemas::BehaviorNodeValueType::BehaviorNodeFlow,
            CreateBehaviorNodeFlow(fbb, flow.id, flow.node, flow.socket)
                .Union());
      }
      BehaviorNodeValueOffset operator()(
          const gltf::Behavior::Variable& variable) {
        return schemas::CreateBehaviorNodeValue(
            fbb, schemas::BehaviorNodeValueType::BehaviorVariable,
            CreateBehaviorVariable(
                fbb, variable.id,
                absl::ConvertVariantTo<BehaviorValue>(variable.value))
                .Union());
      }
      BehaviorNodeValueOffset operator()(const absl::monostate monostate) {
        return 0;
      }
    };

    out_value_offsets.push_back(std::visit(Visitor{fbb}, value.value));
  }

  return absl::OkStatus();
}

// An out parameter is used instead of returning the Offset as it's a large copy
absl::Status CreateBehaviorVariables(
    FlatBufferBuilder& fbb,
    std::vector<imp::gltf::Behavior::Variable> variables,
    BehaviorVariableOffsets& out_variable_offsets) {
  for (const auto& variable : variables) {
    if (!extensions::IsValidVariable(variable)) {
      return absl::InternalError(absl::StrFormat(
          "Invalid behavior variable type with name %s", variable.id));
    }

    out_variable_offsets.push_back(CreateBehaviorVariable(
        fbb, variable.id,
        absl::ConvertVariantTo<BehaviorValue>(variable.value)));
  }

  return absl::OkStatus();
}

absl::StatusOr<BehaviorCustomEventOffset> CreateBehaviorCustomEvent(
    FlatBufferBuilder& fbb, gltf::Behavior::CustomEvent custom_event) {
  BehaviorVariableOffsets values_offset;
  MP_RETURN_IF_ERROR(
      CreateBehaviorVariables(fbb, custom_event.values, values_offset));

  return schemas::CreateBehaviorCustomEvent(
      fbb, fbb.CreateString(custom_event.id.data(), custom_event.id.size()),
      CreateVector<schemas::BehaviorVariable>(fbb, values_offset));
}

absl::Status CreateBehaviorCustomEvents(
    FlatBufferBuilder& fbb,
    std::vector<imp::gltf::Behavior::CustomEvent> custom_events,
    BehaviorCustomEventOffsets& out_custom_event_offsets) {
  for (const auto& custom_event : custom_events) {
    MP_ASSIGN_OR_RETURN(auto custom_event_offset,
                     CreateBehaviorCustomEvent(fbb, custom_event));
    out_custom_event_offsets.push_back(std::move(custom_event_offset));
  }
  return absl::OkStatus();
}

BehaviorOffset CreateBehaviorOffset(
    flatbuffers::FlatBufferBuilder& fbb,
    const BehaviorNodeOffsets& behavior_node_offsets,
    const BehaviorVariableOffsets& behavior_variable_offsets,
    const BehaviorCustomEventOffsets& behavior_custom_event_offsets) {
  return schemas::CreateBehavior(
      fbb, CreateVector<schemas::BehaviorNode>(fbb, behavior_node_offsets),
      CreateVector<schemas::BehaviorVariable>(fbb, behavior_variable_offsets),
      CreateVector<schemas::BehaviorCustomEvent>(
          fbb, behavior_custom_event_offsets));
}

}  // namespace

absl::StatusOr<BehaviorOffset> BehaviorLoaderExtensionImpl::AddBehavior(
    const imp::gltf::Behavior& behavior) {
  BehaviorNodeOffsets node_offsets;
  BehaviorVariableOffsets variable_offsets;
  BehaviorCustomEventOffsets custom_event_offsets;

  // Create Behavior Nodes
  for (const auto& node : behavior.nodes) {
    BehaviorNodeFlowOffsets node_flow_offsets;
    MP_RETURN_IF_ERROR(
        CreateBehaviorNodeFlows(fbb_, node.flows, node_flow_offsets));

    BehaviorNodeConfigurationOffsets node_configuration_offsets;
    MP_RETURN_IF_ERROR(CreateBehaviorNodeConfigurations(
        fbb_, node.configuration, node_configuration_offsets));

    BehaviorNodeValueOffsets node_value_offsets;
    MP_RETURN_IF_ERROR(
        CreateBehaviorNodeValues(fbb_, node.values, node_value_offsets));

    node_offsets.push_back(
        CreateBehaviorNode(fbb_, node.type, node.index, node_flow_offsets,
                           node_configuration_offsets, node_value_offsets));
  }

  // Create Behavior Variables
  MP_RETURN_IF_ERROR(
      CreateBehaviorVariables(fbb_, behavior.variables, variable_offsets));

  // Create Behavior Custom Events
  MP_RETURN_IF_ERROR(CreateBehaviorCustomEvents(fbb_, behavior.custom_events,
                                             custom_event_offsets));

  return CreateBehaviorOffset(fbb_, node_offsets, variable_offsets,
                              custom_event_offsets);
}

}  // namespace imp::loader::details
