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

#include "core/loader/provider/extensions/gltf_extension_interactivity.h"

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/loader/provider/extensions/interactivity/interactivity.proto.imp.h"
#include "core/loader/provider/extensions/interactivity/interactivity_helpers.h"
#include "core/loader/provider/extensions/interactivity/loader_extension.h"
#include "core/loader/provider/extensions/interactivity/loader_extension_impl.h"
#include "core/loader/provider/extensions/interactivity/model_creator_extension.h"
#include "core/loader/provider/extensions/interactivity/model_creator_extension_impl.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/proto/json_message_visitor.h"
#include "core/proto/json_reader.h"
#include "core/proto/proto_common.h"
#include "core/view/utils/string_map.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::extensions {

namespace {

// Below is a list of string identifiers of the various objects used in
// KHR_interactivity as they should appear in the interactivity json.

// Interactivity::Graph::Node::Configuration ID types
constexpr char kVariableNodeConfiguration[] = "variable";
constexpr char kNumberOfOutputFlowsNodeConfiguration[] = "numberOutputFlows";
constexpr char kPointerNodeConfiguration[] = "pointer";
constexpr char kEventNodeConfiguration[] = "event";
constexpr char kNodeIndexNodeConfiguration[] = "nodeIndex";
constexpr char kStopPropagationNodeConfiguration[] = "stopPropagation";
constexpr char kEasingTypeNodeConfiguration[] = "easingType";
constexpr char kEasingDurationNodeConfiguration[] = "easingDuration";
constexpr char kCasesNodeConfiguration[] = "cases";
constexpr char kTypeNodeConfiguration[] = "type";
constexpr char kNumberOfInputFlowsNodeConfiguration[] = "inputFlows";
constexpr char kIsRandomNodeConfiguration[] = "isRandom";
constexpr char kIsLoopNodeConfiguration[] = "isLoop";
constexpr char kMessageNodeConfiguration[] = "message";
constexpr char kInitialIndexNodeConfiguration[] = "initialIndex";
constexpr char kVariablesNodeConfiguration[] = "variables";
constexpr char kUseSlerpNodeConfiguration[] = "useSlerp";

// Interactivity::Graph::Variable types
constexpr char kBoolValueType[] = "bool";
constexpr char kIntValueType[] = "int";
constexpr char kFloatValueType[] = "float";
constexpr char kFloat2ValueType[] = "float2";
constexpr char kFloat3ValueType[] = "float3";
constexpr char kFloat4ValueType[] = "float4";
constexpr char kMat2fValueType[] = "float2x2";
constexpr char kMat3fValueType[] = "float3x3";
constexpr char kMat4fValueType[] = "float4x4";
constexpr char kStringValueType[] = "string";

// Obtain the field id of the variable field json name provided
constexpr int GetVariableFieldId(absl::string_view field_name) {
  for (int i = 0; i < gltf::Interactivity::Graph::Variable::kFieldsCount; ++i) {
    if (imp::gltf::Interactivity::Graph::Variable::kFieldJsonNames[i] ==
        field_name) {
      return gltf::Interactivity::Graph::Variable::kFieldIds[i];
    }
  }
  // Will never happen, as compilation will fail before reaching this.
  std::abort();
}

constexpr int GetInteractivityFieldId(absl::string_view field_name) {
  for (int i = 0; i < gltf::Interactivity::Graph::kFieldsCount; ++i) {
    if (imp::gltf::Interactivity::Graph::kFieldJsonNames[i] == field_name) {
      return gltf::Interactivity::Graph::kFieldIds[i];
    }
  }
  // Will never happen, as compilation will fail before reaching this.
  std::abort();
}

constexpr int GetConfigurationIdFieldId() {
  for (int i = 0;
       i < gltf::Interactivity::Graph::Node::Configuration::kFieldsCount; ++i) {
    if (imp::gltf::Interactivity::Graph::Node::Configuration::kFieldJsonNames
            [i] == "id") {
      return gltf::Interactivity::Graph::Node::Configuration::kFieldIds[i];
    }
  }
  // Will never happen, as compilation will fail before reaching this.
  std::abort();
}

constexpr int kVariableTypeFieldId = GetVariableFieldId("type");
constexpr int kVariableNameFieldId = GetVariableFieldId("id");
constexpr int kTypeArrayFieldId = GetInteractivityFieldId("types");
constexpr int kVariableArrayFieldId = GetInteractivityFieldId("variables");

template <typename VariantType, typename Variant>
absl::Status ConvertFloatArray(const std::vector<float>& values,
                               Variant& variant) {
  if constexpr (std::is_same_v<VariantType, float2>) {
    if (values.size() == 2) {
      variant.template emplace<float2>(float2{values[0], values[1]});
      return absl::OkStatus();
    }
  } else if constexpr (std::is_same_v<VariantType, float3>) {
    if (values.size() == 3) {
      variant.template emplace<float3>(float3{values[0], values[1], values[2]});
      return absl::OkStatus();
    }
  } else if constexpr (std::is_same_v<VariantType, float4>) {
    if (values.size() == 4) {
      variant.template emplace<float4>(
          float4{values[0], values[1], values[2], values[3]});
      return absl::OkStatus();
    }
  } else if constexpr (std::is_same_v<VariantType, mat2f>) {
    if (values.size() == 4) {
      variant.template emplace<mat2f>(
          mat2f{values[0], values[1], values[2], values[3]});
      return absl::OkStatus();
    }
  } else if constexpr (std::is_same_v<VariantType, mat3f>) {
    if (values.size() == 9) {
      variant.template emplace<mat3f>(mat3f{values[0], values[1], values[2],
                                            values[3], values[4], values[5],
                                            values[6], values[7], values[8]});
      return absl::OkStatus();
    }
  } else if constexpr (std::is_same_v<VariantType, mat4f>) {
    if (values.size() == 16) {
      variant.template emplace<mat4f>(mat4f{
          values[0], values[1], values[2], values[3], values[4], values[5],
          values[6], values[7], values[8], values[9], values[10], values[11],
          values[12], values[13], values[14], values[15]});
      return absl::OkStatus();
    }
  }

  return absl::InternalError(
      absl::StrFormat("Invalid number of float values: %d", values.size()));
}

template <typename VariantType, typename Variant>
void OnVisitVariant(Variant& variant, int field_id, proto::JsonReader& visitor,
                    const char* ptr, int token_type) {
  if (!std::holds_alternative<VariantType>(variant)) {
    variant.template emplace<VariantType>();
  }

  visitor.Visit<proto::GetFieldType<VariantType>()>(
      ptr, field_id, std::get_if<VariantType>(&variant),
      static_cast<VariantType*>(nullptr), token_type);
}

template <typename VariantType>
std::vector<VariantType> VisitArray(int field_id, proto::JsonReader& visitor,
                                    const char* ptr, int token_type) {
  std::vector<VariantType> values;

  visitor.Visit<proto::GetFieldType<VariantType>(),
                proto::RepeatedMergeStrategy::kOverwrite>(
      ptr, field_id, &values, static_cast<std::vector<VariantType>*>(nullptr),
      token_type);

  return values;
}

template <typename VariantType>
absl::StatusOr<VariantType> RetrieveValueFromSingleElementArray(
    int field_id, proto::JsonReader& visitor, const char* ptr, int token_type) {
  std::vector<VariantType> values =
      VisitArray<VariantType>(field_id, visitor, ptr, token_type);

  if (values.size() != 1) {
    return absl::InternalError(absl::StrFormat(
        "Expected a single element array for field %d", field_id));
  }

  return values[0];
}

template <typename VariantType, typename Variant>
absl::Status OnVisitVariantArrayWithSingleElement(Variant& variant,
                                                  int field_id,
                                                  proto::JsonReader& visitor,
                                                  const char* ptr,
                                                  int token_type) {
  if (!std::holds_alternative<VariantType>(variant)) {
    variant.template emplace<VariantType>();
  }

  MP_ASSIGN_OR_RETURN(variant, RetrieveValueFromSingleElementArray<VariantType>(
                                field_id, visitor, ptr, token_type));
  return absl::OkStatus();
}

template <typename VariantType, typename Variant>
absl::Status VisitImpType(Variant& variant, int field_id,
                          proto::JsonReader& visitor, const char* ptr,
                          int token_type) {
  std::vector<float> float_vector;
  visitor.Visit<proto::TYPE_FLOAT, proto::RepeatedMergeStrategy::kOverwrite>(
      ptr, field_id, &float_vector, static_cast<std::vector<float>*>(nullptr),
      token_type);
  MP_RETURN_IF_ERROR(ConvertFloatArray<VariantType>(float_vector, variant));

  if (!std::get_if<VariantType>(&variant)) {
    return absl::InternalError(
        absl::StrFormat("Parsed float array of size %d does not match the size "
                        "of requested variant type",
                        float_vector.size()));
  }

  return absl::OkStatus();
}

}  // namespace

// Implementation of the Interactivity extension of the glTF loader.
class InteractivityImpl : public Interactivity {
 public:
  InteractivityImpl();

  void AddHooks(proto::JsonMessageVisitor& json_message_visitor) override;

 private:
  absl::Status OnPreVisitInteractivityGraph(
      imp::gltf::Interactivity::Graph& graph, const proto::JsonReader& reader,
      const char* ptr);

  absl::Status OnPreVisitVariable(
      imp::gltf::Interactivity::Graph::Variable& variable,
      const proto::JsonReader& reader, const char* ptr);

  absl::Status OnPreVisitNodeValue(
      imp::gltf::Interactivity::Graph::Node::Value& value,
      const proto::JsonReader& reader, const char* ptr);

  // Visit and modifies the Interactivity Variable message.
  absl::StatusOr<bool> OnVisitVariable(
      imp::gltf::Interactivity::Graph::Variable& variable, int field_id,
      proto::JsonReader& visitor, const char* ptr, int token_type);

  absl::StatusOr<bool> OnVisitNodeConfiguration(
      imp::gltf::Interactivity::Graph::Node::Configuration& configuration,
      int field_id, proto::JsonReader& visitor, const char* ptr,
      int token_type);

  absl::StatusOr<bool> OnVisitValueType(
      imp::gltf::Interactivity::Graph::ValueType& value_type, int field_id,
      proto::JsonReader& visitor, const char* ptr, int token_type);

  std::vector<imp::gltf::Interactivity::Graph::Type> types_;
  std::vector<std::string> variable_names_;
  StringMap<gltf::Interactivity::Graph::Node::ConfigurationType>
      configuration_id_map_;
  StringMap<gltf::Interactivity::Graph::ValueType> value_type_map_;
};

std::unique_ptr<Interactivity> CreateInteractivityGltfExtension() {
  return std::make_unique<InteractivityImpl>();
}

std::unique_ptr<details::InteractivityLoaderExtension>
CreateInteractivityLoaderExtension(flatbuffers::FlatBufferBuilder& fbb) {
  return std::make_unique<details::InteractivityLoaderExtensionImpl>(fbb);
}

std::unique_ptr<details::InteractivityModelCreatorExtension>
CreateInteractivityModelCreatorExtension() {
  return std::make_unique<details::InteractivityModelCreatorExtensionImpl>();
}

InteractivityImpl::InteractivityImpl() {
  configuration_id_map_.emplace(
      kVariableNodeConfiguration,
      gltf::Interactivity::Graph::Node::ConfigurationType::VARIABLE);
  configuration_id_map_.emplace(kNumberOfOutputFlowsNodeConfiguration,
                                gltf::Interactivity::Graph::Node::
                                    ConfigurationType::NUMBER_OF_OUTPUT_FLOWS);
  configuration_id_map_.emplace(
      kPointerNodeConfiguration,
      gltf::Interactivity::Graph::Node::ConfigurationType::POINTER);
  configuration_id_map_.emplace(
      kEventNodeConfiguration,
      gltf::Interactivity::Graph::Node::ConfigurationType::EVENT);
  configuration_id_map_.emplace(
      kNodeIndexNodeConfiguration,
      gltf::Interactivity::Graph::Node::ConfigurationType::NODE_INDEX);
  configuration_id_map_.emplace(
      kStopPropagationNodeConfiguration,
      gltf::Interactivity::Graph::Node::ConfigurationType::STOP_PROPAGATION);
  configuration_id_map_.emplace(
      kEasingTypeNodeConfiguration,
      gltf::Interactivity::Graph::Node::ConfigurationType::EASING_TYPE);
  configuration_id_map_.emplace(
      kEasingDurationNodeConfiguration,
      gltf::Interactivity::Graph::Node::ConfigurationType::EASING_DURATION);
  configuration_id_map_.emplace(
      kCasesNodeConfiguration,
      gltf::Interactivity::Graph::Node::ConfigurationType::CASES);
  configuration_id_map_.emplace(
      kTypeNodeConfiguration,
      gltf::Interactivity::Graph::Node::ConfigurationType::TYPE);
  configuration_id_map_.emplace(kNumberOfInputFlowsNodeConfiguration,
                                gltf::Interactivity::Graph::Node::
                                    ConfigurationType::NUMBER_OF_INPUT_FLOWS);
  configuration_id_map_.emplace(
      kIsRandomNodeConfiguration,
      gltf::Interactivity::Graph::Node::ConfigurationType::IS_RANDOM);
  configuration_id_map_.emplace(
      kIsLoopNodeConfiguration,
      gltf::Interactivity::Graph::Node::ConfigurationType::IS_LOOP);
  configuration_id_map_.emplace(
      kMessageNodeConfiguration,
      gltf::Interactivity::Graph::Node::ConfigurationType::MESSAGE);
  configuration_id_map_.emplace(
      kInitialIndexNodeConfiguration,
      gltf::Interactivity::Graph::Node::ConfigurationType::INITIAL_INDEX);
  configuration_id_map_.emplace(
      kVariablesNodeConfiguration,
      gltf::Interactivity::Graph::Node::ConfigurationType::VARIABLES);
  configuration_id_map_.emplace(
      kUseSlerpNodeConfiguration,
      gltf::Interactivity::Graph::Node::ConfigurationType::USE_SLERP);

  value_type_map_.emplace(kBoolValueType,
                          gltf::Interactivity::Graph::ValueType::BOOL);
  value_type_map_.emplace(kIntValueType,
                          gltf::Interactivity::Graph::ValueType::INT);
  value_type_map_.emplace(kFloatValueType,
                          gltf::Interactivity::Graph::ValueType::FLOAT);
  value_type_map_.emplace(kFloat2ValueType,
                          gltf::Interactivity::Graph::ValueType::FLOAT2);
  value_type_map_.emplace(kFloat3ValueType,
                          gltf::Interactivity::Graph::ValueType::FLOAT3);
  value_type_map_.emplace(kFloat4ValueType,
                          gltf::Interactivity::Graph::ValueType::FLOAT4);
  value_type_map_.emplace(kMat2fValueType,
                          gltf::Interactivity::Graph::ValueType::MAT2F);
  value_type_map_.emplace(kMat3fValueType,
                          gltf::Interactivity::Graph::ValueType::MAT3F);
  value_type_map_.emplace(kMat4fValueType,
                          gltf::Interactivity::Graph::ValueType::MAT4F);
  value_type_map_.emplace(kStringValueType,
                          gltf::Interactivity::Graph::ValueType::STRING);
}

void InteractivityImpl::AddHooks(
    proto::JsonMessageVisitor& json_message_visitor) {
  // Registers a functor to do a first pass on the Interactivity message. This
  // lets it populate the Types array first, which is needed for parsing the
  // rest of the Interactivity message.
  json_message_visitor.OnPreVisit([this](imp::gltf::Interactivity::Graph& graph,
                                         const proto::JsonReader& reader,
                                         const char* ptr) {
    return OnPreVisitInteractivityGraph(graph, reader, ptr);
  });

  // Registers a functor to do a first pass on the Variable message. This lets
  // it populate the "type" field of the message, which is needed to determine
  // how to parse the "value" field.
  json_message_visitor.OnPreVisit(
      [this](imp::gltf::Interactivity::Graph::Variable& variable,
             const proto::JsonReader& reader, const char* ptr) {
        return OnPreVisitVariable(variable, reader, ptr);
      });

  json_message_visitor.OnPreVisit(
      [this](imp::gltf::Interactivity::Graph::Node::Value& value,
             const proto::JsonReader& reader, const char* ptr) {
        return OnPreVisitNodeValue(value, reader, ptr);
      });

  // Registers a functor to handle the Interactivity Variable message
  json_message_visitor.OnVisit(
      [this](imp::gltf::Interactivity::Graph::Variable& variable, int field_id,
             proto::JsonReader& visitor, const char* ptr, int token_type) {
        return OnVisitVariable(variable, field_id, visitor, ptr, token_type);
      });

  // Registers a functor to handle the Node Configuration message
  json_message_visitor.OnVisit(
      [this](
          imp::gltf::Interactivity::Graph::Node::Configuration& configuration,
          int field_id, proto::JsonReader& visitor, const char* ptr,
          int token_type) {
        return OnVisitNodeConfiguration(configuration, field_id, visitor, ptr,
                                        token_type);
      });

  // Registers a functor to handle the Node Value message
  json_message_visitor.OnVisit(
      [](imp::gltf::Interactivity::Graph::Node::Value& value, int field_id,
         proto::JsonReader& visitor, const char* ptr,
         int token_type) -> absl::StatusOr<bool> {
        visitor.Unknown(ptr, field_id, token_type);
        return true;
      });
}

absl::Status InteractivityImpl::OnPreVisitInteractivityGraph(
    imp::gltf::Interactivity::Graph& graph, const proto::JsonReader& reader,
    const char* ptr) {
  // Create a new JsonReader and parse the types array of the interactivity
  // json.
  proto::JsonMessageVisitor visitor;
  visitor.OnVisit([](imp::gltf::Interactivity::Graph& graph, int field_id,
                     proto::JsonReader& visitor, const char* ptr,
                     int token_type) -> absl::StatusOr<bool> {
    if (field_id == kTypeArrayFieldId || field_id == kVariableArrayFieldId) {
      // Let the default visit logic for types and the overridden logic for
      // variables below handle this, so return unhandled.
      return false;
    }

    // we're not interested in the other fields, skip them.
    visitor.Unknown(ptr, field_id, token_type);
    return true;
  });
  visitor.OnVisit([](imp::gltf::Interactivity::Graph::Variable& variable,
                     int field_id, proto::JsonReader& visitor, const char* ptr,
                     int token_type) -> absl::StatusOr<bool> {
    // we're only interested in the variable field, ignore parsing the rest.
    if (field_id != kVariableNameFieldId) {
      visitor.Unknown(ptr, field_id, token_type);
      return true;
    }

    // Let the default visit logic handle this, so return unhandled.
    return false;
  });

  proto::JsonReader sub(reader, &visitor);
  imp::gltf::Interactivity::Graph sub_graph;
  sub.ParseMsg(&sub_graph);
  types_ = sub_graph.types;
  variable_names_.resize(sub_graph.variables.size());
  for (int i = 0; i < sub_graph.variables.size(); i++) {
    variable_names_[i] = std::string(sub_graph.variables[i].id);
  }
  return absl::OkStatus();
}

absl::Status InteractivityImpl::OnPreVisitVariable(
    imp::gltf::Interactivity::Graph::Variable& variable,
    const proto::JsonReader& reader, const char* ptr) {
  // Create a new JsonReader and parse the type field of the variable json.
  proto::JsonMessageVisitor visitor;
  visitor.OnVisit([this](imp::gltf::Interactivity::Graph::Variable& variable,
                         int field_id, proto::JsonReader& visitor,
                         const char* ptr,
                         int token_type) -> absl::StatusOr<bool> {
    // we're only interested in the type field, ignore parsing the rest.
    if (field_id != kVariableTypeFieldId) {
      visitor.Unknown(ptr, field_id, token_type);
      return true;
    }

    return OnVisitVariable(variable, field_id, visitor, ptr, token_type);
  });

  proto::JsonReader sub(reader, &visitor);
  imp::gltf::Interactivity::Graph::Variable sub_variable;
  sub.ParseMsg(&sub_variable);
  variable.type = sub_variable.type;
  return absl::OkStatus();
}

absl::Status InteractivityImpl::OnPreVisitNodeValue(
    imp::gltf::Interactivity::Graph::Node::Value& value,
    const proto::JsonReader& reader, const char* ptr) {
  proto::JsonReader sub_var(reader, reader.GetMessageVisitor());
  imp::gltf::Interactivity::Graph::Variable variable;
  sub_var.ParseMsg(&variable);

  proto::JsonReader sub_flow(reader, reader.GetMessageVisitor());
  imp::gltf::Interactivity::Graph::Node::Flow flow;
  sub_flow.ParseMsg(&flow);

  if (interactivity::IsValidVariable(variable)) {
    value.value = variable;
  } else if (interactivity::IsValidFlow(flow)) {
    value.value = flow;
  } else {
    return absl::InternalError(absl::StrFormat(
        "Unable to determine a valid variable or flow in Node"));
  }
  return absl::OkStatus();
}

absl::StatusOr<bool> InteractivityImpl::OnVisitVariable(
    imp::gltf::Interactivity::Graph::Variable& variable, int field_id,
    proto::JsonReader& visitor, const char* ptr, int token_type) {
  switch (field_id) {
    case 1:
      // Let the default visit logic handle this, so return unhandled.
      return false;
      break;
    case 2: {
      return OnVisitValueType(variable.type, field_id, visitor, ptr,
                              token_type);
    }
    default: {
      switch (variable.type) {
        case imp::gltf::Interactivity::Graph::ValueType::BOOL:
          MP_RETURN_IF_ERROR(OnVisitVariantArrayWithSingleElement<bool>(
              variable.value, field_id, visitor, ptr, token_type));
          break;
        case imp::gltf::Interactivity::Graph::ValueType::INT:
          MP_RETURN_IF_ERROR(OnVisitVariantArrayWithSingleElement<int>(
              variable.value, field_id, visitor, ptr, token_type));
          break;
        case imp::gltf::Interactivity::Graph::ValueType::FLOAT:
          MP_RETURN_IF_ERROR(OnVisitVariantArrayWithSingleElement<float>(
              variable.value, field_id, visitor, ptr, token_type));
          break;
        case imp::gltf::Interactivity::Graph::ValueType::FLOAT2:
          MP_RETURN_IF_ERROR(VisitImpType<float2>(variable.value, field_id,
                                               visitor, ptr, token_type));
          break;
        case imp::gltf::Interactivity::Graph::ValueType::FLOAT3:
          MP_RETURN_IF_ERROR(VisitImpType<float3>(variable.value, field_id,
                                               visitor, ptr, token_type));
          break;
        case imp::gltf::Interactivity::Graph::ValueType::FLOAT4:
          MP_RETURN_IF_ERROR(VisitImpType<float4>(variable.value, field_id,
                                               visitor, ptr, token_type));
          break;
        case imp::gltf::Interactivity::Graph::ValueType::MAT2F:
          MP_RETURN_IF_ERROR(VisitImpType<mat2f>(variable.value, field_id, visitor,
                                              ptr, token_type));
          break;
        case imp::gltf::Interactivity::Graph::ValueType::MAT3F:
          MP_RETURN_IF_ERROR(VisitImpType<mat3f>(variable.value, field_id, visitor,
                                              ptr, token_type));
          break;
        case imp::gltf::Interactivity::Graph::ValueType::MAT4F:
          MP_RETURN_IF_ERROR(VisitImpType<mat4f>(variable.value, field_id, visitor,
                                              ptr, token_type));
          break;
        case imp::gltf::Interactivity::Graph::ValueType::STRING:
          OnVisitVariant<std::string>(variable.value, field_id, visitor, ptr,
                                      token_type);
          break;
        default:
          visitor.Unknown(ptr, field_id, token_type);
          return absl::InternalError(absl::StrFormat(
              "Unknown Interactivity variable type index: %d with id %s",
              variable.type, variable.id));
          break;
      }
    }
  }
  return true;
}

absl::StatusOr<bool> InteractivityImpl::OnVisitNodeConfiguration(
    imp::gltf::Interactivity::Graph::Node::Configuration& configuration,
    int field_id, proto::JsonReader& visitor, const char* ptr, int token_type) {
  switch (field_id) {
    case 1:
    case 2: {
      // Ignore id and type fields as they will be populated by the dictionary
      // key.
      visitor.Unknown(ptr, field_id, token_type);
      return true;
    }
    default: {
      // Retrieve the configuration id from the dictionary key.
      std::optional<absl::string_view> configuration_string =
          visitor.GetCurrentDictionaryKey();
      if (!configuration_string.has_value()) {
        visitor.Unknown(ptr, field_id, token_type);
        return absl::InternalError("Cannot determine configuration id.");
      }

      // Convert the configuration id to enum type.
      auto it = configuration_id_map_.find(configuration_string.value());
      if (it == configuration_id_map_.end()) {
        visitor.Unknown(ptr, field_id, token_type);
        return absl::InternalError(
            absl::StrFormat("Unknown Interactivity configuration id: %s",
                            configuration_string.value()));
      }

      configuration.id = it->second;

      switch (configuration.id) {
        case gltf::Interactivity::Graph::Node::ConfigurationType::EVENT:
        case gltf::Interactivity::Graph::Node::ConfigurationType::NODE_INDEX:
        case gltf::Interactivity::Graph::Node::ConfigurationType::
            NUMBER_OF_OUTPUT_FLOWS:
        case gltf::Interactivity::Graph::Node::ConfigurationType::TYPE:
        case gltf::Interactivity::Graph::Node::ConfigurationType::
            NUMBER_OF_INPUT_FLOWS:
        case gltf::Interactivity::Graph::Node::ConfigurationType::INITIAL_INDEX:
          MP_RETURN_IF_ERROR(OnVisitVariantArrayWithSingleElement<int>(
              configuration.value, field_id, visitor, ptr, token_type));
          break;
        case gltf::Interactivity::Graph::Node::ConfigurationType::VARIABLE: {
          MP_ASSIGN_OR_RETURN(uint32_t variable_index,
                           RetrieveValueFromSingleElementArray<uint32_t>(
                               field_id, visitor, ptr, token_type));

          if (variable_names_.empty()) {
            return absl::InternalError(
                absl::StrFormat("Interactivity variable names array is empty"));
          }

          if (variable_names_.size() <= variable_index) {
            return absl::InternalError(
                absl::StrFormat("Interactivity variable names does not contain "
                                "variable index %d",
                                variable_index));
          }

          configuration.value = variable_names_[variable_index];
        } break;
        case gltf::Interactivity::Graph::Node::ConfigurationType::POINTER:
        case gltf::Interactivity::Graph::Node::ConfigurationType::EASING_TYPE:
        case gltf::Interactivity::Graph::Node::ConfigurationType::MESSAGE:
          MP_RETURN_IF_ERROR(OnVisitVariantArrayWithSingleElement<std::string>(
              configuration.value, field_id, visitor, ptr, token_type));
          break;
        case gltf::Interactivity::Graph::Node::ConfigurationType::
            STOP_PROPAGATION:
        case gltf::Interactivity::Graph::Node::ConfigurationType::IS_RANDOM:
        case gltf::Interactivity::Graph::Node::ConfigurationType::IS_LOOP:
        case gltf::Interactivity::Graph::Node::ConfigurationType::USE_SLERP:
          MP_RETURN_IF_ERROR(OnVisitVariantArrayWithSingleElement<bool>(
              configuration.value, field_id, visitor, ptr, token_type));
          break;
        case gltf::Interactivity::Graph::Node::ConfigurationType::
            EASING_DURATION:
          MP_RETURN_IF_ERROR(OnVisitVariantArrayWithSingleElement<float>(
              configuration.value, field_id, visitor, ptr, token_type));
          break;
        case gltf::Interactivity::Graph::Node::ConfigurationType::VARIABLES:
        case gltf::Interactivity::Graph::Node::ConfigurationType::CASES: {
          std::vector<int> cases =
              VisitArray<int>(field_id, visitor, ptr, token_type);
          configuration.value =
              gltf::Interactivity::Graph::Node::Configuration::IntArray{
                  .values = cases};
        } break;
        default:
          visitor.Unknown(ptr, field_id, token_type);
          return absl::InternalError(
              absl::StrFormat("Unknown Interactivity configuration type id: %d",
                              configuration.id));
          break;
      }
    }
  }
  return true;
}

absl::StatusOr<bool> InteractivityImpl::OnVisitValueType(
    imp::gltf::Interactivity::Graph::ValueType& value_type, int field_id,
    proto::JsonReader& visitor, const char* ptr, int token_type) {
  uint32_t type_index;
  visitor.Visit<proto::TYPE_UINT32>(ptr, field_id, &type_index, nullptr,
                                    token_type);

  if (types_.empty() || types_.size() <= type_index) {
    return absl::InternalError(absl::StrFormat(
        "Unhandled Interactivity type of index %d", type_index));
  }

  absl::string_view type_string = types_[type_index].signature;
  if (auto it = value_type_map_.find(type_string);
      it != value_type_map_.end()) {
    value_type = it->second;
  } else {
    return absl::InternalError(
        absl::StrFormat("Invalid Interactivity value type: %s", type_string));
  }
  return true;
}

}  // namespace imp::loader::extensions
