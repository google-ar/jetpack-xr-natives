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

#include "core/loader/provider/extensions/interactivity/loader_extension_impl.h"

#include <sys/types.h>

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/variant.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/string.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/paired_vector.h"
#include "core/common/schemas/math_generated.h"
#include "core/loader/provider/extensions/interactivity/interactivity.proto.imp.h"
#include "core/loader/provider/extensions/interactivity/interactivity_helpers.h"
#include "core/loader/provider/extensions/interactivity/loader_extension.h"
#include "core/loader/provider/extensions/interactivity/schemas/interactivity_generated.h"
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

using InteractivityGraphId = model::ModelData::InteractivityGraphId;
using InteractivityGraphOffset = Offset<schemas::InteractivityGraph>;
using InteractivityGraphOffsets =
    PairedVector<InteractivityGraphOffset, InteractivityGraphId::ReferredType>;

using InteractivityNodeFlowId = model::ModelData::InteractivityNodeFlowId;
using InteractivityNodeFlowOffset = Offset<schemas::InteractivityNodeFlow>;
using InteractivityNodeFlowOffsets =
    PairedVector<InteractivityNodeFlowOffset,
                 InteractivityNodeFlowId::ReferredType>;

using InteractivityNodeConfigurationId =
    model::ModelData::InteractivityNodeConfigurationId;
using InteractivityNodeConfigurationOffset =
    Offset<schemas::InteractivityNodeConfiguration>;
using InteractivityNodeConfigurationOffsets =
    PairedVector<InteractivityNodeConfigurationOffset,
                 InteractivityNodeConfigurationId::ReferredType>;

using InteractivityNodeValueId = model::ModelData::InteractivityNodeValueId;
using InteractivityNodeValueOffset = Offset<schemas::InteractivityNodeValue>;
using InteractivityNodeValueOffsets =
    PairedVector<InteractivityNodeValueOffset,
                 InteractivityNodeValueId::ReferredType>;

using InteractivityNodeId = model::ModelData::InteractivityNodeId;
using InteractivityNodeOffset = Offset<schemas::InteractivityNode>;
using InteractivityNodeOffsets =
    PairedVector<InteractivityNodeOffset, InteractivityNodeId::ReferredType>;

using InteractivityDeclarationId = model::ModelData::InteractivityDeclarationId;
using InteractivityDeclarationOffset = Offset<schemas::Declaration>;
using InteractivityDeclarationOffsets =
    PairedVector<InteractivityDeclarationOffset,
                 InteractivityDeclarationId::ReferredType>;

using InteractivityVariableId = model::ModelData::InteractivityVariableId;
using InteractivityVariableOffset = Offset<schemas::InteractivityVariable>;
using InteractivityVariableOffsets =
    PairedVector<InteractivityVariableOffset,
                 InteractivityVariableId::ReferredType>;

using InteractivityEventId = model::ModelData::InteractivityEventId;
using InteractivityEventOffset = Offset<schemas::InteractivityEvent>;
using InteractivityEventOffsets =
    PairedVector<InteractivityEventOffset, InteractivityEventId::ReferredType>;

using InteractivityTypeId = model::ModelData::InteractivityTypeId;
using InteractivityGraphTypeDataOffset =
    Offset<schemas::InteractivityGraphTypeData>;
using InteractivityGraphTypeDataOffsets =
    PairedVector<InteractivityGraphTypeDataOffset,
                 InteractivityTypeId::ReferredType>;

using InteractivityValue =
    std::variant<absl::monostate, bool, int, float, float2, float3, float4,
                 mat2f, mat3f, mat4f, absl::string_view>;
using InteractivityConfiguration =
    std::variant<int, absl::string_view, bool, float, std::vector<int>>;

InteractivityNodeFlowOffset CreateInteractivityNodeFlow(
    FlatBufferBuilder& fbb, absl::string_view id, int node,
    absl::string_view socket) {
  return schemas::CreateInteractivityNodeFlow(
      fbb, fbb.CreateString(id.data(), id.size()), static_cast<uint32_t>(node),
      fbb.CreateString(socket.data(), socket.size()));
}

InteractivityNodeConfigurationOffset CreateInteractivityNodeConfiguration(
    FlatBufferBuilder& fbb, schemas::InteractivityNodeConfigurationType id,
    InteractivityConfiguration value) {
  struct Visitor {
    FlatBufferBuilder& fbb;
    schemas::InteractivityNodeConfigurationType id;

    InteractivityNodeConfigurationOffset operator()(const int value) {
      return schemas::CreateInteractivityNodeConfiguration(
          fbb, id, schemas::InteractivityConfigurationValue::Int,
          fbb.CreateStruct(schemas::Int(value)).Union());
    }
    InteractivityNodeConfigurationOffset operator()(const bool value) {
      return schemas::CreateInteractivityNodeConfiguration(
          fbb, id, schemas::InteractivityConfigurationValue::Bool,
          fbb.CreateStruct(schemas::Bool(value)).Union());
    }
    InteractivityNodeConfigurationOffset operator()(const float value) {
      return schemas::CreateInteractivityNodeConfiguration(
          fbb, id, schemas::InteractivityConfigurationValue::Float,
          fbb.CreateStruct(schemas::Float(value)).Union());
    }
    InteractivityNodeConfigurationOffset operator()(
        const absl::string_view value) {
      // schemas::CreateString is creating a "String" table wrapping the
      // actual Flatbuffer string.
      return schemas::CreateInteractivityNodeConfiguration(
          fbb, id, schemas::InteractivityConfigurationValue::String,
          schemas::CreateString(fbb,
                                fbb.CreateString(value.data(), value.size()))
              .Union());
    }
    InteractivityNodeConfigurationOffset operator()(
        const std::vector<int>& value) {
      return schemas::CreateInteractivityNodeConfiguration(
          fbb, id, schemas::InteractivityConfigurationValue::IntArray,
          schemas::CreateIntArray(fbb,
                                  fbb.CreateVector(value.data(), value.size()))
              .Union());
    }
  };

  return std::visit(Visitor{fbb, id}, value);
}

InteractivityNodeOffset CreateInteractivityNode(
    FlatBufferBuilder& fbb, int declaration, int index,
    const InteractivityNodeFlowOffsets& flows,
    const InteractivityNodeConfigurationOffsets& configurations,
    const InteractivityNodeValueOffsets& values) {
  return schemas::CreateInteractivityNode(
      fbb, static_cast<uint32_t>(declaration), static_cast<uint32_t>(index),
      CreateVector<schemas::InteractivityNodeFlow>(fbb, flows),
      CreateVector<schemas::InteractivityNodeConfiguration>(fbb,
                                                            configurations),
      CreateVector<schemas::InteractivityNodeValue>(fbb, values));
}

InteractivityDeclarationOffset CreateInteractivityDeclaration(
    FlatBufferBuilder& fbb, absl::string_view op) {
  return schemas::CreateDeclaration(fbb,
                                    fbb.CreateString(op.data(), op.size()));
}

InteractivityVariableOffset CreateInteractivityVariable(
    FlatBufferBuilder& fbb, absl::string_view id,
    gltf::Interactivity::Graph::Variable& variable) {
  struct Visitor {
    flatbuffers::FlatBufferBuilder& fbb;
    Offset<String> id;
    gltf::Interactivity::Graph::Variable& variable;

    InteractivityVariableOffset operator()(const bool value) {
      return schemas::CreateInteractivityVariable(
          fbb, id, schemas::InteractivityValue::Bool,
          fbb.CreateStruct(schemas::Bool(value)).Union());
    }
    InteractivityVariableOffset operator()(const int value) {
      return schemas::CreateInteractivityVariable(
          fbb, id, schemas::InteractivityValue::Int,
          fbb.CreateStruct(schemas::Int(value)).Union());
    }
    InteractivityVariableOffset operator()(const float value) {
      return schemas::CreateInteractivityVariable(
          fbb, id, schemas::InteractivityValue::Float,
          fbb.CreateStruct(schemas::Float(value)).Union());
    }
    InteractivityVariableOffset operator()(const float2 value) {
      return schemas::CreateInteractivityVariable(
          fbb, id, schemas::InteractivityValue::Float2,
          fbb.CreateStruct(flatbuffers::Pack(value)).Union());
    }
    InteractivityVariableOffset operator()(const float3 value) {
      return schemas::CreateInteractivityVariable(
          fbb, id, schemas::InteractivityValue::Float3,
          fbb.CreateStruct(flatbuffers::Pack(value)).Union());
    }
    InteractivityVariableOffset operator()(const float4 value) {
      return schemas::CreateInteractivityVariable(
          fbb, id, schemas::InteractivityValue::Float4,
          fbb.CreateStruct(schemas::Float4(flatbuffers::Pack(value))).Union());
    }
    InteractivityVariableOffset operator()(const mat2f value) {
      return schemas::CreateInteractivityVariable(
          fbb, id, schemas::InteractivityValue::Mat2f,
          fbb.CreateStruct(schemas::Mat2f(flatbuffers::Pack(value))).Union());
    }
    InteractivityVariableOffset operator()(const mat3f value) {
      return schemas::CreateInteractivityVariable(
          fbb, id, schemas::InteractivityValue::Mat3f,
          fbb.CreateStruct(schemas::Mat3f(flatbuffers::Pack(value))).Union());
    }
    InteractivityVariableOffset operator()(const mat4f value) {
      return schemas::CreateInteractivityVariable(
          fbb, id, schemas::InteractivityValue::Mat4f,
          fbb.CreateStruct(schemas::Mat4f(flatbuffers::Pack(value))).Union());
    }
    InteractivityVariableOffset operator()(const absl::string_view value) {
      return schemas::CreateInteractivityVariable(
          fbb, id, schemas::InteractivityValue::String,
          schemas::CreateString(fbb,
                                fbb.CreateString(value.data(), value.size()))
              .Union());
    }
    InteractivityVariableOffset operator()(const absl::monostate value) {
      extensions::interactivity::SetToDefaultValue(variable);
      return std::visit(Visitor{fbb, id, variable}, variable.value);
    }
  };

  return std::visit(
      Visitor{fbb, fbb.CreateString(id.data(), id.size()), variable},
      variable.value);
}

absl::Status CreateInteractivityNodeFlows(
    FlatBufferBuilder& fbb,
    const std::map<std::string, imp::gltf::Interactivity::Graph::Node::Flow>&
        flows,
    InteractivityNodeFlowOffsets& out_flow_offsets) {
  for (const auto& [socket_name, flow] : flows) {
    if (!flow.node.has_value()) {
      continue;
    }

    out_flow_offsets.push_back(
        CreateInteractivityNodeFlow(fbb, socket_name, *flow.node, flow.socket));
  }

  return absl::OkStatus();
}

// An out parameter is used instead of returning the Offset as it's a large copy
absl::Status CreateInteractivityNodeConfigurations(
    FlatBufferBuilder& fbb,
    const std::map<std::string,
                   imp::gltf::Interactivity::Graph::Node::Configuration>&
        configurations,
    InteractivityNodeConfigurationOffsets& out_configuration_offsets) {
  for (const auto& [_, configuration] : configurations) {
    switch (configuration.id) {
      case imp::gltf::Interactivity::Graph::Node::ConfigurationType::
          NUMBER_OF_OUTPUT_FLOWS:
        if (!std::get_if<int>(&configuration.value)) {
          return absl::InternalError(
              absl::StrFormat("Interactivity configuration id %d has no value",
                              configuration.id));
        }
        out_configuration_offsets.push_back(
            CreateInteractivityNodeConfiguration(
                fbb,
                schemas::InteractivityNodeConfigurationType::
                    NUMBER_OF_OUTPUT_FLOWS,
                std::get<int>(configuration.value)));
        break;
      case imp::gltf::Interactivity::Graph::Node::ConfigurationType::VARIABLE:
        if (!std::get_if<std::string>(&configuration.value)) {
          return absl::InternalError(
              absl::StrFormat("Interactivity configuration id %d has no value",
                              configuration.id));
        }
        out_configuration_offsets.push_back(
            CreateInteractivityNodeConfiguration(
                fbb, schemas::InteractivityNodeConfigurationType::VARIABLE,

                std::get<std::string>(configuration.value)));
        break;
      case imp::gltf::Interactivity::Graph::Node::ConfigurationType::POINTER:
        if (!std::get_if<std::string>(&configuration.value)) {
          return absl::InternalError(
              absl::StrFormat("Interactivity configuration id %d has no value",
                              configuration.id));
        }
        out_configuration_offsets.push_back(
            CreateInteractivityNodeConfiguration(
                fbb, schemas::InteractivityNodeConfigurationType::POINTER,

                std::get<std::string>(configuration.value)));
        break;
      case imp::gltf::Interactivity::Graph::Node::ConfigurationType::EVENT:
        if (!std::get_if<int>(&configuration.value)) {
          return absl::InternalError(
              absl::StrFormat("Interactivity configuration id %d has no value",
                              configuration.id));
        }
        out_configuration_offsets.push_back(
            CreateInteractivityNodeConfiguration(
                fbb, schemas::InteractivityNodeConfigurationType::EVENT,
                std::get<int>(configuration.value)));
        break;
      case imp::gltf::Interactivity::Graph::Node::ConfigurationType::NODE_INDEX:
        if (!std::get_if<int>(&configuration.value)) {
          return absl::InternalError(
              absl::StrFormat("Interactivity configuration id %d has no value",
                              configuration.id));
        }
        out_configuration_offsets.push_back(
            CreateInteractivityNodeConfiguration(
                fbb, schemas::InteractivityNodeConfigurationType::NODE_INDEX,
                std::get<int>(configuration.value)));
        break;
      case imp::gltf::Interactivity::Graph::Node::ConfigurationType::
          STOP_PROPAGATION:
        if (!std::get_if<bool>(&configuration.value)) {
          return absl::InternalError(
              absl::StrFormat("Interactivity configuration id %d has no value",
                              configuration.id));
        }
        out_configuration_offsets.push_back(
            CreateInteractivityNodeConfiguration(
                fbb,
                schemas::InteractivityNodeConfigurationType::STOP_PROPAGATION,
                std::get<bool>(configuration.value)));
        break;
      case imp::gltf::Interactivity::Graph::Node::ConfigurationType::
          EASING_TYPE:
        if (!std::get_if<std::string>(&configuration.value)) {
          return absl::InternalError(
              absl::StrFormat("Interactivity configuration id %d has no value",
                              configuration.id));
        }
        out_configuration_offsets.push_back(
            CreateInteractivityNodeConfiguration(
                fbb, schemas::InteractivityNodeConfigurationType::EASING_TYPE,

                std::get<std::string>(configuration.value)));
        break;
      case imp::gltf::Interactivity::Graph::Node::ConfigurationType::
          EASING_DURATION:
        if (!std::get_if<float>(&configuration.value)) {
          return absl::InternalError(
              absl::StrFormat("Interactivity configuration id %d has no value",
                              configuration.id));
        }
        out_configuration_offsets.push_back(
            CreateInteractivityNodeConfiguration(
                fbb,
                schemas::InteractivityNodeConfigurationType::EASING_DURATION,
                std::get<float>(configuration.value)));
        break;
      case imp::gltf::Interactivity::Graph::Node::ConfigurationType::CASES:
        if (!std::get_if<
                imp::gltf::Interactivity::Graph::Node::Configuration::IntArray>(
                &configuration.value)) {
          return absl::InternalError(
              absl::StrFormat("Interactivity configuration id %d has no value",
                              configuration.id));
        }

        out_configuration_offsets.push_back(
            CreateInteractivityNodeConfiguration(
                fbb, schemas::InteractivityNodeConfigurationType::CASES,

                std::get<imp::gltf::Interactivity::Graph::Node::Configuration::
                             IntArray>(configuration.value)
                    .values));
        break;
      case imp::gltf::Interactivity::Graph::Node::ConfigurationType::TYPE:
        if (!std::get_if<int>(&configuration.value)) {
          return absl::InternalError(
              absl::StrFormat("Interactivity configuration id %d has no value",
                              configuration.id));
        }
        out_configuration_offsets.push_back(
            CreateInteractivityNodeConfiguration(
                fbb, schemas::InteractivityNodeConfigurationType::TYPE,
                std::get<int>(configuration.value)));
        break;
      case imp::gltf::Interactivity::Graph::Node::ConfigurationType::
          NUMBER_OF_INPUT_FLOWS:
        if (!std::get_if<int>(&configuration.value)) {
          return absl::InternalError(absl::StrFormat(
              "Interactivity configuration id %d was not an int",
              configuration.id));
        }
        out_configuration_offsets.push_back(
            CreateInteractivityNodeConfiguration(
                fbb,
                schemas::InteractivityNodeConfigurationType::
                    NUMBER_OF_INPUT_FLOWS,
                std::get<int>(configuration.value)));
        break;
      case imp::gltf::Interactivity::Graph::Node::ConfigurationType::IS_RANDOM:
        if (!std::get_if<bool>(&configuration.value)) {
          return absl::InternalError(
              absl::StrFormat("Interactivity configuration id %d is not a bool",
                              configuration.id));
        }
        out_configuration_offsets.push_back(
            CreateInteractivityNodeConfiguration(
                fbb, schemas::InteractivityNodeConfigurationType::IS_RANDOM,
                std::get<bool>(configuration.value)));
        break;
      case imp::gltf::Interactivity::Graph::Node::ConfigurationType::IS_LOOP:
        if (!std::get_if<bool>(&configuration.value)) {
          return absl::InternalError(
              absl::StrFormat("Interactivity configuration id %d is not a bool",
                              configuration.id));
        }
        out_configuration_offsets.push_back(
            CreateInteractivityNodeConfiguration(
                fbb, schemas::InteractivityNodeConfigurationType::IS_LOOP,
                std::get<bool>(configuration.value)));
        break;
      case imp::gltf::Interactivity::Graph::Node::ConfigurationType::MESSAGE:
        if (!std::get_if<std::string>(&configuration.value)) {
          return absl::InternalError(
              absl::StrFormat("Interactivity configuration id %d has no value",
                              configuration.id));
        }
        out_configuration_offsets.push_back(
            CreateInteractivityNodeConfiguration(
                fbb, schemas::InteractivityNodeConfigurationType::MESSAGE,

                std::get<std::string>(configuration.value)));
        break;
      case imp::gltf::Interactivity::Graph::Node::ConfigurationType::
          INITIAL_INDEX:
        if (!std::get_if<int>(&configuration.value)) {
          return absl::InternalError(
              absl::StrFormat("Interactivity configuration id %d has no value",
                              configuration.id));
        }
        out_configuration_offsets.push_back(
            CreateInteractivityNodeConfiguration(
                fbb, schemas::InteractivityNodeConfigurationType::INITIAL_INDEX,
                std::get<int>(configuration.value)));
        break;
      default:
        return absl::InternalError(
            absl::StrFormat("Invalid interactivity configuration id with id %d",
                            configuration.id));
    }
  }

  return absl::OkStatus();
}

// An out parameter is used instead of returning the Offset as it's a large
// copy
absl::Status CreateInteractivityNodeValues(
    FlatBufferBuilder& fbb,
    std::map<std::string, ::imp::gltf::Interactivity::Graph::Node::Value>
        values,
    InteractivityNodeValueOffsets& out_value_offsets) {
  struct Visitor {
    flatbuffers::FlatBufferBuilder& fbb;
    std::string socket_name;

    InteractivityNodeValueOffset operator()(
        const gltf::Interactivity::Graph::Node::Flow& flow) {
      return schemas::CreateInteractivityNodeValue(
          fbb, schemas::InteractivityNodeValueType::InteractivityNodeFlow,
          CreateInteractivityNodeFlow(fbb, socket_name, *flow.node, flow.socket)
              .Union());
    }
    InteractivityNodeValueOffset operator()(
        gltf::Interactivity::Graph::Variable& variable) {
      return schemas::CreateInteractivityNodeValue(
          fbb, schemas::InteractivityNodeValueType::InteractivityVariable,
          CreateInteractivityVariable(fbb, socket_name, variable).Union());
    }
    InteractivityNodeValueOffset operator()(const absl::monostate monostate) {
      return 0;
    }
  };

  for (auto& [socket_name, value] : values) {
    out_value_offsets.push_back(
        std::visit(Visitor{fbb, socket_name}, value.value));
  }

  return absl::OkStatus();
}

// An out parameter is used instead of returning the Offset as it's a large
// copy
absl::Status CreateInteractivityVariables(
    FlatBufferBuilder& fbb,
    std::map<std::string, imp::gltf::Interactivity::Graph::Variable>& variables,
    InteractivityVariableOffsets& out_variable_offsets) {
  for (auto& [id, variable] : variables) {
    if (!extensions::interactivity::IsValidVariable(variable)) {
      return absl::InternalError(absl::StrFormat(
          "Invalid interactivity variable type with name %s", id));
    }

    out_variable_offsets.push_back(
        CreateInteractivityVariable(fbb, id, variable));
  }

  return absl::OkStatus();
}

// An out parameter is used instead of returning the Offset as it's a large
// copy
absl::Status CreateInteractivityVariables(
    FlatBufferBuilder& fbb,
    std::vector<imp::gltf::Interactivity::Graph::Variable> variables,
    InteractivityVariableOffsets& out_variable_offsets) {
  for (auto& variable : variables) {
    if (!extensions::interactivity::IsValidVariable(variable)) {
      return absl::InternalError(absl::StrFormat(
          "Invalid interactivity variable type with name %s", variable.id));
    }

    out_variable_offsets.push_back(
        CreateInteractivityVariable(fbb, variable.id, variable));
  }

  return absl::OkStatus();
}

absl::StatusOr<InteractivityEventOffset> CreateInteractivityEvent(
    FlatBufferBuilder& fbb, gltf::Interactivity::Graph::Event event) {
  InteractivityVariableOffsets values_offset;
  MP_RETURN_IF_ERROR(
      CreateInteractivityVariables(fbb, event.values, values_offset));

  return schemas::CreateInteractivityEvent(
      fbb, fbb.CreateString(event.id.data(), event.id.size()),
      CreateVector<schemas::InteractivityVariable>(fbb, values_offset));
}

absl::Status CreateInteractivityEvents(
    FlatBufferBuilder& fbb,
    const std::vector<imp::gltf::Interactivity::Graph::Event>& events,
    InteractivityEventOffsets& out_event_offsets) {
  for (const auto& event : events) {
    MP_ASSIGN_OR_RETURN(auto event_offset, CreateInteractivityEvent(fbb, event));
    out_event_offsets.push_back(std::move(event_offset));
  }
  return absl::OkStatus();
}

absl::StatusOr<InteractivityGraphTypeDataOffset>
CreateInteractivityGraphTypeDataOffset(FlatBufferBuilder& fbb,
                                       gltf::Interactivity::Graph::Type type) {
  if (type.signature == "bool") {
    return schemas::CreateInteractivityGraphTypeData(
        fbb, schemas::InteractivityVariableType::BOOL);
  } else if (type.signature == "int") {
    return schemas::CreateInteractivityGraphTypeData(
        fbb, schemas::InteractivityVariableType::INT);
  } else if (type.signature == "float") {
    return schemas::CreateInteractivityGraphTypeData(
        fbb, schemas::InteractivityVariableType::FLOAT);
  } else if (type.signature == "float2") {
    return schemas::CreateInteractivityGraphTypeData(
        fbb, schemas::InteractivityVariableType::FLOAT2);
  } else if (type.signature == "float3") {
    return schemas::CreateInteractivityGraphTypeData(
        fbb, schemas::InteractivityVariableType::FLOAT3);
  } else if (type.signature == "float4") {
    return schemas::CreateInteractivityGraphTypeData(
        fbb, schemas::InteractivityVariableType::FLOAT4);
  } else if (type.signature == "float2x2") {
    return schemas::CreateInteractivityGraphTypeData(
        fbb, schemas::InteractivityVariableType::MAT2F);
  } else if (type.signature == "float3x3") {
    return schemas::CreateInteractivityGraphTypeData(
        fbb, schemas::InteractivityVariableType::MAT3F);
  } else if (type.signature == "float4x4") {
    return schemas::CreateInteractivityGraphTypeData(
        fbb, schemas::InteractivityVariableType::MAT4F);
  }

  // TODO: Add support for passing custom types through loader.
  return absl::InternalError(
      absl::StrFormat("Unknown interactivity type: %s", type.signature));
}

absl::Status CreateInteractivityGraphTypeDataOffsets(
    FlatBufferBuilder& fbb,
    const std::vector<imp::gltf::Interactivity::Graph::Type>& types,
    InteractivityGraphTypeDataOffsets& out_type_offsets) {
  for (const auto& type : types) {
    MP_ASSIGN_OR_RETURN(auto type_offset,
                     CreateInteractivityGraphTypeDataOffset(fbb, type));
    out_type_offsets.push_back(std::move(type_offset));
  }
  return absl::OkStatus();
}

InteractivityGraphOffset CreateInteractivityGraphOffset(
    flatbuffers::FlatBufferBuilder& fbb,
    const InteractivityDeclarationOffsets& interactivity_declaration_offsets,
    const InteractivityNodeOffsets& interactivity_node_offsets,
    const InteractivityVariableOffsets& interactivity_variable_offsets,
    const InteractivityEventOffsets& interactivity_event_offsets,
    const InteractivityGraphTypeDataOffsets& interactivity_type_offsets) {
  return schemas::CreateInteractivityGraph(
      fbb,
      CreateVector<schemas::Declaration>(fbb,
                                         interactivity_declaration_offsets),
      CreateVector<schemas::InteractivityNode>(fbb, interactivity_node_offsets),
      CreateVector<schemas::InteractivityVariable>(
          fbb, interactivity_variable_offsets),
      CreateVector<schemas::InteractivityEvent>(fbb,
                                                interactivity_event_offsets),
      CreateVector<schemas::InteractivityGraphTypeData>(
          fbb, interactivity_type_offsets));
}

InteractivityOffset CreateInteractivityOffset(
    flatbuffers::FlatBufferBuilder& fbb,
    const InteractivityGraphOffsets& interactivity_graph_offsets,
    uint32_t graph_index) {
  return schemas::CreateInteractivity(fbb,
                                      CreateVector<schemas::InteractivityGraph>(
                                          fbb, interactivity_graph_offsets),
                                      graph_index);
}

}  // namespace

absl::StatusOr<InteractivityOffset>
InteractivityLoaderExtensionImpl::AddInteractivity(
    const imp::gltf::Interactivity& interactivity) {
  InteractivityGraphOffsets interactivity_graph_offsets;

  for (const auto& graph : interactivity.graphs) {
    InteractivityDeclarationOffsets declaration_offsets;
    InteractivityNodeOffsets node_offsets;
    InteractivityVariableOffsets variable_offsets;
    InteractivityEventOffsets event_offsets;
    InteractivityGraphTypeDataOffsets type_offsets;

    for (const auto& declaration : graph.declarations) {
      declaration_offsets.push_back(
          CreateInteractivityDeclaration(fbb_, declaration.op));
    }

    // Create Interactivity Nodes
    for (int i = 0; i < graph.nodes.size(); ++i) {
      const auto& node = graph.nodes[i];

      InteractivityNodeFlowOffsets node_flow_offsets;
      MP_RETURN_IF_ERROR(
          CreateInteractivityNodeFlows(fbb_, node.flows, node_flow_offsets));

      InteractivityNodeConfigurationOffsets node_configuration_offsets;
      MP_RETURN_IF_ERROR(CreateInteractivityNodeConfigurations(
          fbb_, node.configuration, node_configuration_offsets));

      InteractivityNodeValueOffsets node_value_offsets;
      MP_RETURN_IF_ERROR(
          CreateInteractivityNodeValues(fbb_, node.values, node_value_offsets));

      node_offsets.push_back(CreateInteractivityNode(
          fbb_, node.declaration, i, node_flow_offsets,
          node_configuration_offsets, node_value_offsets));
    }

    // Create Interactivity Variables
    MP_RETURN_IF_ERROR(
        CreateInteractivityVariables(fbb_, graph.variables, variable_offsets));

    // Create Interactivity Events
    MP_RETURN_IF_ERROR(
        CreateInteractivityEvents(fbb_, graph.events, event_offsets));

    // Create Interactivity Types
    MP_RETURN_IF_ERROR(CreateInteractivityGraphTypeDataOffsets(fbb_, graph.types,
                                                            type_offsets));

    interactivity_graph_offsets.push_back(CreateInteractivityGraphOffset(
        fbb_, declaration_offsets, node_offsets, variable_offsets,
        event_offsets, type_offsets));
  }

  return CreateInteractivityOffset(fbb_, interactivity_graph_offsets,
                                   interactivity.graph);
}

}  // namespace imp::loader::details
