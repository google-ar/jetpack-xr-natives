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

#include "core/loader/provider/extensions/gltf_extension_behavior.h"

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/loader/provider/extensions/behavior/behavior.proto.imp.h"
#include "core/loader/provider/extensions/behavior/behavior_helpers.h"
#include "core/loader/provider/extensions/behavior/loader_extension.h"
#include "core/loader/provider/extensions/behavior/loader_extension_impl.h"
#include "core/loader/provider/extensions/behavior/model_creator_extension.h"
#include "core/loader/provider/extensions/behavior/model_creator_extension_impl.h"
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
// KHR_behavior as they should appear in the behavior json.

// Behavior::Node::Configuration ID types
constexpr char kVariableNodeConfiguration[] = "variable";
constexpr char kNumberOfOutputFlowsNodeConfiguration[] = "numberOutputFlows";
constexpr char kPathNodeConfiguration[] = "path";
constexpr char kCustomEventNodeConfiguration[] = "customEvent";
constexpr char kNodeIndexNodeConfiguration[] = "nodeIndex";
constexpr char kStopPropagationNodeConfiguration[] = "stopPropagation";
constexpr char kEasingTypeNodeConfiguration[] = "easingType";
constexpr char kEasingDurationNodeConfiguration[] = "easingDuration";

// Behavior::Variable types
constexpr char kBoolValueType[] = "bool";
constexpr char kIntValueType[] = "int";
constexpr char kFloatValueType[] = "float";
constexpr char kFloat2ValueType[] = "float2";
constexpr char kFloat3ValueType[] = "float3";
constexpr char kFloat4ValueType[] = "float4";
constexpr char kMat4fValueType[] = "float4x4";
constexpr char kStringValueType[] = "string";
constexpr char kCustomValueType[] = "custom";

// Obtain the field id of the variable field json name provided
constexpr int GetVariableFieldId(absl::string_view field_name) {
  for (int i = 0; i < gltf::Behavior::Variable::kFieldsCount; ++i) {
    if (imp::gltf::Behavior::Variable::kFieldJsonNames[i] == field_name) {
      return gltf::Behavior::Variable::kFieldIds[i];
    }
  }
  // Will never happen, as compilation will fail before reaching this.
  std::abort();
}

constexpr int GetBehaviorFieldId(absl::string_view field_name) {
  for (int i = 0; i < gltf::Behavior::kFieldsCount; ++i) {
    if (imp::gltf::Behavior::kFieldJsonNames[i] == field_name) {
      return gltf::Behavior::kFieldIds[i];
    }
  }
  // Will never happen, as compilation will fail before reaching this.
  std::abort();
}

constexpr int GetConfigurationIdFieldId() {
  for (int i = 0; i < gltf::Behavior::Node::Configuration::kFieldsCount; ++i) {
    if (imp::gltf::Behavior::Node::Configuration::kFieldJsonNames[i] == "id") {
      return gltf::Behavior::Node::Configuration::kFieldIds[i];
    }
  }
  // Will never happen, as compilation will fail before reaching this.
  std::abort();
}

constexpr int kVariableTypeFieldId = GetVariableFieldId("type");
constexpr int kVariableNameFieldId = GetVariableFieldId("id");
constexpr int kTypeArrayFieldId = GetBehaviorFieldId("types");
constexpr int kVariableArrayFieldId = GetBehaviorFieldId("variables");
constexpr int kNodeConfigurationIdFieldId = GetConfigurationIdFieldId();

template <typename Variant>
absl::Status ConvertFloatArray(const std::vector<float>& values,
                               Variant& variant) {
  if (values.size() == 2) {
    variant.template emplace<float2>(float2{values[0], values[1]});
    return absl::OkStatus();
  } else if (values.size() == 3) {
    variant.template emplace<float3>(float3{values[0], values[1], values[2]});
    return absl::OkStatus();
  } else if (values.size() == 4) {
    variant.template emplace<float4>(
        float4{values[0], values[1], values[2], values[3]});
    return absl::OkStatus();
  } else if (values.size() == 16) {
    variant.template emplace<mat4f>(
        mat4f{values[0], values[1], values[2], values[3], values[4], values[5],
              values[6], values[7], values[8], values[9], values[10],
              values[11], values[12], values[13], values[14], values[15]});
    return absl::OkStatus();
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

template <typename VariantType, typename Variant>
absl::Status VisitImpType(Variant& variant, int field_id,
                          proto::JsonReader& visitor, const char* ptr,
                          int token_type) {
  std::vector<float> float_vector;
  visitor.Visit<proto::TYPE_FLOAT, proto::RepeatedMergeStrategy::kOverwrite>(
      ptr, field_id, &float_vector, static_cast<std::vector<float>*>(nullptr),
      token_type);
  MP_RETURN_IF_ERROR(ConvertFloatArray(float_vector, variant));

  if (!std::get_if<VariantType>(&variant)) {
    return absl::InternalError(
        absl::StrFormat("Parsed float array of size %d does not match the size "
                        "of requested variant type",
                        float_vector.size()));
  }

  return absl::OkStatus();
}

}  // namespace

// Implementation of the Behavior extension of the glTF loader.
class BehaviorImpl : public Behavior {
 public:
  BehaviorImpl();

  void AddHooks(proto::JsonMessageVisitor& json_message_visitor) override;

 private:
  absl::Status OnPreVisitBehavior(imp::gltf::Behavior& behavior,
                                  const proto::JsonReader& reader,
                                  const char* ptr);

  absl::Status OnPreVisitVariable(imp::gltf::Behavior::Variable& variable,
                                  const proto::JsonReader& reader,
                                  const char* ptr);

  absl::Status OnPreVisitNodeConfiguration(
      imp::gltf::Behavior::Node::Configuration& configuration,
      const proto::JsonReader& reader, const char* ptr);

  absl::Status OnPreVisitNodeValue(imp::gltf::Behavior::Node::Value& value,
                                   const proto::JsonReader& reader,
                                   const char* ptr);

  // Visit and modifies the Behavior Variable message.
  absl::StatusOr<bool> OnVisitVariable(imp::gltf::Behavior::Variable& variable,
                                       int field_id, proto::JsonReader& visitor,
                                       const char* ptr, int token_type);

  absl::StatusOr<bool> OnVisitNodeConfiguration(
      imp::gltf::Behavior::Node::Configuration& configuration, int field_id,
      proto::JsonReader& visitor, const char* ptr, int token_type);

  absl::StatusOr<bool> OnVisitValueType(
      imp::gltf::Behavior::ValueType& value_type, int field_id,
      proto::JsonReader& visitor, const char* ptr, int token_type);

  std::vector<imp::gltf::Behavior::Type> types_;
  std::vector<std::string> variable_names_;
  StringMap<gltf::Behavior::Node::ConfigurationType> configuration_id_map_;
  StringMap<gltf::Behavior::ValueType> value_type_map_;
};

std::unique_ptr<Behavior> CreateBehaviorGltfExtension() {
  return std::make_unique<BehaviorImpl>();
}

std::unique_ptr<details::BehaviorLoaderExtension> CreateBehaviorLoaderExtension(
    flatbuffers::FlatBufferBuilder& fbb) {
  return std::make_unique<details::BehaviorLoaderExtensionImpl>(fbb);
}

std::unique_ptr<details::BehaviorModelCreatorExtension>
CreateBehaviorModelCreatorExtension() {
  return std::make_unique<details::BehaviorModelCreatorExtensionImpl>();
}

BehaviorImpl::BehaviorImpl() {
  configuration_id_map_.emplace(
      kVariableNodeConfiguration,
      gltf::Behavior::Node::ConfigurationType::VARIABLE);
  configuration_id_map_.emplace(
      kNumberOfOutputFlowsNodeConfiguration,
      gltf::Behavior::Node::ConfigurationType::NUMBER_OF_OUTPUT_FLOWS);
  configuration_id_map_.emplace(kPathNodeConfiguration,
                                gltf::Behavior::Node::ConfigurationType::PATH);
  configuration_id_map_.emplace(
      kCustomEventNodeConfiguration,
      gltf::Behavior::Node::ConfigurationType::CUSTOM_EVENT);
  configuration_id_map_.emplace(
      kNodeIndexNodeConfiguration,
      gltf::Behavior::Node::ConfigurationType::NODE_INDEX);
  configuration_id_map_.emplace(
      kStopPropagationNodeConfiguration,
      gltf::Behavior::Node::ConfigurationType::STOP_PROPAGATION);
  configuration_id_map_.emplace(
      kEasingTypeNodeConfiguration,
      gltf::Behavior::Node::ConfigurationType::EASING_TYPE);
  configuration_id_map_.emplace(
      kEasingDurationNodeConfiguration,
      gltf::Behavior::Node::ConfigurationType::EASING_DURATION);

  value_type_map_.emplace(kBoolValueType, gltf::Behavior::ValueType::BOOL);
  value_type_map_.emplace(kIntValueType, gltf::Behavior::ValueType::INT);
  value_type_map_.emplace(kFloatValueType, gltf::Behavior::ValueType::FLOAT);
  value_type_map_.emplace(kFloat2ValueType, gltf::Behavior::ValueType::FLOAT2);
  value_type_map_.emplace(kFloat3ValueType, gltf::Behavior::ValueType::FLOAT3);
  value_type_map_.emplace(kFloat4ValueType, gltf::Behavior::ValueType::FLOAT4);
  value_type_map_.emplace(kMat4fValueType, gltf::Behavior::ValueType::MAT4F);
  value_type_map_.emplace(kStringValueType, gltf::Behavior::ValueType::STRING);
}

void BehaviorImpl::AddHooks(proto::JsonMessageVisitor& json_message_visitor) {
  // Registers a functor to do a first pass on the Behavior message. This lets
  // it populate the Types array first, which is needed for parsing the rest of
  // the Behavior message.
  json_message_visitor.OnPreVisit([this](imp::gltf::Behavior& behavior,
                                         const proto::JsonReader& reader,
                                         const char* ptr) {
    return OnPreVisitBehavior(behavior, reader, ptr);
  });

  // Registers a functor to do a first pass on the Variable message. This lets
  // it populate the "type" field of the message, which is needed to determine
  // how to parse the "value" field.
  json_message_visitor.OnPreVisit(
      [this](imp::gltf::Behavior::Variable& variable,
             const proto::JsonReader& reader, const char* ptr) {
        return OnPreVisitVariable(variable, reader, ptr);
      });

  json_message_visitor.OnPreVisit(
      [this](imp::gltf::Behavior::Node::Configuration& configuration,
             const proto::JsonReader& reader, const char* ptr) {
        return OnPreVisitNodeConfiguration(configuration, reader, ptr);
      });

  json_message_visitor.OnPreVisit(
      [this](imp::gltf::Behavior::Node::Value& value,
             const proto::JsonReader& reader, const char* ptr) {
        return OnPreVisitNodeValue(value, reader, ptr);
      });

  // Registers a functor to handle the Behavior Variable message
  json_message_visitor.OnVisit([this](imp::gltf::Behavior::Variable& variable,
                                      int field_id, proto::JsonReader& visitor,
                                      const char* ptr, int token_type) {
    return OnVisitVariable(variable, field_id, visitor, ptr, token_type);
  });

  // Registers a functor to handle the Node Configuration message
  json_message_visitor.OnVisit(
      [this](imp::gltf::Behavior::Node::Configuration& configuration,
             int field_id, proto::JsonReader& visitor, const char* ptr,
             int token_type) {
        return OnVisitNodeConfiguration(configuration, field_id, visitor, ptr,
                                        token_type);
      });

  // Registers a functor to handle the Node Value message
  json_message_visitor.OnVisit([](imp::gltf::Behavior::Node::Value& value,
                                  int field_id, proto::JsonReader& visitor,
                                  const char* ptr,
                                  int token_type) -> absl::StatusOr<bool> {
    visitor.Unknown(ptr, field_id, token_type);
    return true;
  });
}

absl::Status BehaviorImpl::OnPreVisitBehavior(imp::gltf::Behavior& behavior,
                                              const proto::JsonReader& reader,
                                              const char* ptr) {
  // Create a new JsonReader and parse the types array of the behavior json.
  proto::JsonMessageVisitor visitor;
  visitor.OnVisit([](imp::gltf::Behavior& behavior, int field_id,
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
  visitor.OnVisit([](imp::gltf::Behavior::Variable& variable, int field_id,
                     proto::JsonReader& visitor, const char* ptr,
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
  imp::gltf::Behavior sub_behavior;
  sub.ParseMsg(&sub_behavior);
  types_ = sub_behavior.types;
  variable_names_.resize(sub_behavior.variables.size());
  for (int i = 0; i < sub_behavior.variables.size(); i++) {
    variable_names_[i] = std::string(sub_behavior.variables[i].id);
  }
  return absl::OkStatus();
}

absl::Status BehaviorImpl::OnPreVisitVariable(
    imp::gltf::Behavior::Variable& variable, const proto::JsonReader& reader,
    const char* ptr) {
  // Create a new JsonReader and parse the type field of the variable json.
  proto::JsonMessageVisitor visitor;
  visitor.OnVisit([this](imp::gltf::Behavior::Variable& variable, int field_id,
                         proto::JsonReader& visitor, const char* ptr,
                         int token_type) -> absl::StatusOr<bool> {
    // we're only interested in the type field, ignore parsing the rest.
    if (field_id != kVariableTypeFieldId) {
      visitor.Unknown(ptr, field_id, token_type);
      return true;
    }

    return OnVisitVariable(variable, field_id, visitor, ptr, token_type);
  });

  proto::JsonReader sub(reader, &visitor);
  imp::gltf::Behavior::Variable sub_variable;
  sub.ParseMsg(&sub_variable);
  variable.type = sub_variable.type;
  return absl::OkStatus();
}

absl::Status BehaviorImpl::OnPreVisitNodeConfiguration(
    imp::gltf::Behavior::Node::Configuration& configuration,
    const proto::JsonReader& reader, const char* ptr) {
  // Create a new JsonReader and parse the type field of the configuration json.
  proto::JsonMessageVisitor visitor;
  visitor.OnVisit(
      [this](imp::gltf::Behavior::Node::Configuration& configuration,
             int field_id, proto::JsonReader& visitor, const char* ptr,
             int token_type) -> absl::StatusOr<bool> {
        // We want to avoid parsing the value field here, as it's dependent on
        // the data from the id field. Just parsing the id is sufficient.
        if (field_id != kNodeConfigurationIdFieldId) {
          visitor.Unknown(ptr, field_id, token_type);
          return true;
        }
        return OnVisitNodeConfiguration(configuration, field_id, visitor, ptr,
                                        token_type);
      });

  proto::JsonReader sub(reader, &visitor);
  imp::gltf::Behavior::Node::Configuration sub_configuration;
  sub.ParseMsg(&sub_configuration);
  configuration.id = sub_configuration.id;
  return absl::OkStatus();
}

absl::Status BehaviorImpl::OnPreVisitNodeValue(
    imp::gltf::Behavior::Node::Value& value, const proto::JsonReader& reader,
    const char* ptr) {
  proto::JsonReader sub_var(reader, reader.GetMessageVisitor());
  imp::gltf::Behavior::Variable variable;
  sub_var.ParseMsg(&variable);

  proto::JsonReader sub_flow(reader, reader.GetMessageVisitor());
  imp::gltf::Behavior::Node::Flow flow;
  sub_flow.ParseMsg(&flow);

  if (IsValidVariable(variable)) {
    value.value = variable;
  } else if (IsValidFlow(flow)) {
    value.value = flow;
  } else {
    return absl::InternalError(
        absl::StrFormat("Unable to determine a valid variable or flow in Node "
                        "Value with id: %s",
                        variable.id.empty() ? flow.id : variable.id));
  }
  return absl::OkStatus();
}

absl::StatusOr<bool> BehaviorImpl::OnVisitVariable(
    imp::gltf::Behavior::Variable& variable, int field_id,
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
        case imp::gltf::Behavior::ValueType::BOOL:
          OnVisitVariant<bool>(variable.value, field_id, visitor, ptr,
                               token_type);
          break;
        case imp::gltf::Behavior::ValueType::INT:
          OnVisitVariant<int>(variable.value, field_id, visitor, ptr,
                              token_type);
          break;
        case imp::gltf::Behavior::ValueType::FLOAT:
          OnVisitVariant<float>(variable.value, field_id, visitor, ptr,
                                token_type);
          break;
        case imp::gltf::Behavior::ValueType::FLOAT2:
          MP_RETURN_IF_ERROR(VisitImpType<float2>(variable.value, field_id,
                                               visitor, ptr, token_type));
          break;
        case imp::gltf::Behavior::ValueType::FLOAT3:
          MP_RETURN_IF_ERROR(VisitImpType<float3>(variable.value, field_id,
                                               visitor, ptr, token_type));
          break;
        case imp::gltf::Behavior::ValueType::FLOAT4:
          MP_RETURN_IF_ERROR(VisitImpType<float4>(variable.value, field_id,
                                               visitor, ptr, token_type));
          break;
        case imp::gltf::Behavior::ValueType::MAT4F:
          MP_RETURN_IF_ERROR(VisitImpType<mat4f>(variable.value, field_id, visitor,
                                              ptr, token_type));
          break;
        case imp::gltf::Behavior::ValueType::STRING:
          OnVisitVariant<std::string>(variable.value, field_id, visitor, ptr,
                                      token_type);
          break;
        default:
          visitor.Unknown(ptr, field_id, token_type);
          return absl::InternalError(absl::StrFormat(
              "Unknown Behavior variable type index: %d with id %s",
              variable.type, variable.id));
          break;
      }
    }
  }
  return true;
}

absl::StatusOr<bool> BehaviorImpl::OnVisitNodeConfiguration(
    imp::gltf::Behavior::Node::Configuration& configuration, int field_id,
    proto::JsonReader& visitor, const char* ptr, int token_type) {
  switch (field_id) {
    case 1: {
      std::string id_string;

      visitor.Visit<proto::TYPE_STRING>(ptr, field_id, &id_string, nullptr,
                                        token_type);
      if (auto it = configuration_id_map_.find(id_string);
          it != configuration_id_map_.end()) {
        configuration.id = it->second;
      } else {
        return absl::InternalError(
            absl::StrFormat("Invalid value type: %s", id_string));
      }
      break;
    }
    case 2: {
      return OnVisitValueType(configuration.type, field_id, visitor, ptr,
                              token_type);
    }
    default: {
      switch (configuration.id) {
        case gltf::Behavior::Node::ConfigurationType::CUSTOM_EVENT:
        case gltf::Behavior::Node::ConfigurationType::NODE_INDEX:
        case gltf::Behavior::Node::ConfigurationType::NUMBER_OF_OUTPUT_FLOWS:
          OnVisitVariant<int>(configuration.value, field_id, visitor, ptr,
                              token_type);
          break;
        case gltf::Behavior::Node::ConfigurationType::VARIABLE:
          uint32_t variable_index;
          visitor.Visit<proto::TYPE_UINT32>(ptr, field_id, &variable_index,
                                            nullptr, token_type);

          if (variable_names_.empty()) {
            return absl::InternalError(
                absl::StrFormat("Behavior variable names array is empty"));
          }

          if (variable_names_.size() <= variable_index) {
            return absl::InternalError(absl::StrFormat(
                "Behavior variable names does not contain variable index %d",
                variable_index));
          }

          configuration.value = variable_names_[variable_index];
          break;
        case gltf::Behavior::Node::ConfigurationType::PATH:
        case gltf::Behavior::Node::ConfigurationType::EASING_TYPE:
          OnVisitVariant<std::string>(configuration.value, field_id, visitor,
                                      ptr, token_type);
          break;
        case gltf::Behavior::Node::ConfigurationType::STOP_PROPAGATION:
          OnVisitVariant<bool>(configuration.value, field_id, visitor, ptr,
                               token_type);
          break;
        case gltf::Behavior::Node::ConfigurationType::EASING_DURATION:
          OnVisitVariant<float>(configuration.value, field_id, visitor, ptr,
                                token_type);
          break;
        default:
          visitor.Unknown(ptr, field_id, token_type);
          return absl::InternalError(
              absl::StrFormat("Unknown Behavior configuration value type: %d",
                              configuration.type));
          break;
      }
    }
  }
  return true;
}

absl::StatusOr<bool> BehaviorImpl::OnVisitValueType(
    imp::gltf::Behavior::ValueType& value_type, int field_id,
    proto::JsonReader& visitor, const char* ptr, int token_type) {
  uint32_t type_index;
  visitor.Visit<proto::TYPE_UINT32>(ptr, field_id, &type_index, nullptr,
                                    token_type);

  if (types_.empty() || types_.size() <= type_index) {
    return absl::InternalError(
        absl::StrFormat("Unhandled Behavior type of index %d", type_index));
  }

  absl::string_view type_string = types_[type_index].signature;
  if (type_string == kCustomValueType) {
    if (types_[type_index].extensions.behavior_string.has_value()) {
      value_type = gltf::Behavior::ValueType::STRING;
      return true;
    }
    return absl::InternalError(
        absl::StrFormat("Invalid Behavior custom type."));
  }

  if (auto it = value_type_map_.find(type_string);
      it != value_type_map_.end()) {
    value_type = it->second;
  } else {
    return absl::InternalError(
        absl::StrFormat("Invalid Behavior value type: %s", type_string));
  }
  return true;
}

}  // namespace imp::loader::extensions
