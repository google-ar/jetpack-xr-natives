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

#include "core/recipes/language/recipe_utils.h"

#include <sstream>
#include <string>
#include <variant>

#ifdef IMP_ENABLE_RECIPE_EXPERIMENTAL
#include <optional>
#endif

#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/variant.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/proto/any.proto.imp.h"
#include "core/proto/proto_writer.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/scene_handles/scene_handles.h"

namespace imp {

namespace recipe {

namespace {
struct RecipeTypeToStringVisitor {
  std::string operator()(int value) const {
    return absl::StrFormat("%d", value);
  }

  std::string operator()(float value) const {
    return absl::StrFormat("%.3f", value);
  }

  std::string operator()(double value) const {
    return absl::StrFormat("%.3lf", value);
  }

  std::string operator()(bool value) const { return value ? "true" : "false"; }

  std::string operator()(const std::string& value) const { return value; }

  std::string operator()(const float2& value) const {
    return absl::StrFormat("(%.3f, %.3f)", value[0], value[1]);
  }

  std::string operator()(const float3& value) const {
    return absl::StrFormat("(%.3f, %.3f, %.3f)", value[0], value[1], value[2]);
  }

  std::string operator()(const float4& value) const {
    return absl::StrFormat("(%.3f, %.3f, %.3f, %.3f)", value[0], value[1],
                           value[2], value[3]);
  }

  std::string operator()(const quatf& value) const {
    return absl::StrFormat("(%.3f, %.3f, %.3f, %.3f)", value.w, value.x,
                           value.y, value.z);
  }

  std::string operator()(const mat2f& value) const {
    return imp::ToString(value);
  }

  std::string operator()(const mat3f& value) const {
    return imp::ToString(value);
  }

  std::string operator()(const mat4f& value) const {
    return imp::ToString(value);
  }

  std::string operator()(const double3& value) const {
    return absl::StrFormat("(%.3f, %.3f, %.3f)", value[0], value[1], value[2]);
  }

  std::string operator()(const google::protobuf::imp_proto::Any& value) const {
    std::string data;
    proto::SerializeTo(&value, &data);
    return data;
  }

  std::string operator()(const NodeHandle& value) const {
    if (value.IsValid()) {
      return std::string(value->GetName());
    } else {
      return "Invalid NodeHandle";
    }
  }

  std::string operator()(const NodeSceneHandle& value) const {
    if (value.IsValid()) {
      return std::string(value->GetName());
    } else {
      return "Invalid NodeSceneHandle";
    }
  }

  std::string operator()(const LiteralArray& value) const {
    std::stringstream ss;
    ss << "[";
    bool first_pass = true;
    for (const auto& element : value.values) {
      if (!first_pass) {
        ss << ", ";
      }
      first_pass = false;
      ss << recipe::ToString(element.value);
    }
    ss << "]";
    return ss.str();
  }

  std::string operator()(const LiteralTuple& value) const {
    std::stringstream ss;
    ss << "(";
    bool first_pass = true;
    for (const auto& element : value.values) {
      if (!first_pass) {
        ss << ", ";
      }
      first_pass = false;
      ss << recipe::ToString(element.value);
    }
    ss << ")";
    return ss.str();
  }

  std::string operator()(const LiteralMap& value) const {
    std::stringstream ss;
    ss << "{";
    bool first_pass = true;
    for (const auto& [key, value] : value.values) {
      if (!first_pass) {
        ss << ", ";
      }
      first_pass = false;
      ss << key << ": " << recipe::ToString(value.value);
    }
    ss << "}";
    return ss.str();
  }

  std::string operator()(const RecipeRayHit& value) const {
    std::stringstream ss;
    ss << "[RecipeRayHit] distance: " << value.distance;
    if (value.node) {
      ss << " node: " << value.node->GetName();
    } else {
      ss << " node: Invalid";
    }
    ss << " world_point: " << imp::ToString(value.world_point);
    ss << " world_orientation: " << imp::ToString(value.world_orientation);
    if (value.world_normal.has_value()) {
      ss << " world_normal: " << imp::ToString(value.world_normal.value());
    }
    return ss.str();
  }

  std::string operator()(const std::monostate& value) const {
    return "[Unknown Type]";
  }
};

}  // namespace

std::string ToString(const Literal& literal) {
  return std::visit(RecipeTypeToStringVisitor{}, literal.value);
}

std::string ToString(const Variable& variable) {
  return std::visit(RecipeTypeToStringVisitor{}, variable);
}

std::string NodeIdToString(const NodeId& id) {
  return absl::StrFormat("%u", NodeIdHash()(id));
}

VariableDeclaration::Type ToType(const Variable& var) {
  switch (var.index()) {
    case Literal::kValue_IntValue:
      return VariableDeclaration::Type::INT;
    case Literal::kValue_FloatValue:
      return VariableDeclaration::Type::FLOAT;
    case Literal::kValue_DoubleValue:
      return VariableDeclaration::Type::DOUBLE;
    case Literal::kValue_BoolValue:
      return VariableDeclaration::Type::BOOL;
    case Literal::kValue_StringValue:
      return VariableDeclaration::Type::STRING;
    case Literal::kValue_Float2Value:
      return VariableDeclaration::Type::FLOAT2;
    case Literal::kValue_Float3Value:
      return VariableDeclaration::Type::FLOAT3;
    case Literal::kValue_Float4Value:
      return VariableDeclaration::Type::FLOAT4;
    case Literal::kValue_QuatfValue:
      return VariableDeclaration::Type::QUATF;
    case Literal::kValue_Mat2fValue:
      return VariableDeclaration::Type::MAT2F;
    case Literal::kValue_Mat3fValue:
      return VariableDeclaration::Type::MAT3F;
    case Literal::kValue_Mat4fValue:
      return VariableDeclaration::Type::MAT4F;
    case Literal::kValue_NodeValue:
      return VariableDeclaration::Type::NODE;
    case Literal::kValue_NodeSceneValue:
      return VariableDeclaration::Type::NODE_SCENE;
    case Literal::kValue_ProtoValue:
      return VariableDeclaration::Type::PROTO;
    case Literal::kValue_ArrayValue:
      return VariableDeclaration::Type::ARRAY;
    case Literal::kValue_TupleValue:
      return VariableDeclaration::Type::TUPLE;
    case Literal::kValue_MapValue:
      return VariableDeclaration::Type::MAP;
    case Literal::kValue_RayHitValue:
      return VariableDeclaration::Type::RAY_HIT;
    default:
      return VariableDeclaration::Type::UNKNOWN_VARIABLE_TYPE;
  }
}

VariableDeclaration::Type ToType(const Literal& literal) {
  return ToType(literal.value);
}

absl::string_view ToTypeName(const VariableDeclaration::Type& type) {
  return proto::EnumMetaData<VariableDeclaration::Type>::GetName(type);
}

absl::string_view ToTypeName(const Variable& var) {
  return ToTypeName(ToType(var));
}

absl::string_view ToTypeName(const Literal& literal) {
  return ToTypeName(ToType(literal));
}

void SetToDefault(const VariableDeclaration::Type& type, Variable& var) {
  switch (type) {
    case VariableDeclaration::Type::INT:
      var.emplace<int>(0);
      break;
    case VariableDeclaration::Type::FLOAT:
      var.emplace<float>(0);
      break;
    case VariableDeclaration::Type::DOUBLE:
      var.emplace<double>(0);
      break;
    case VariableDeclaration::Type::BOOL:
      var.emplace<bool>(false);
      break;
    case VariableDeclaration::Type::STRING:
      var.emplace<std::string>("");
      break;
    case VariableDeclaration::Type::FLOAT2:
      var.emplace<float2>();
      break;
    case VariableDeclaration::Type::FLOAT3:
      var.emplace<float3>();
      break;
    case VariableDeclaration::Type::FLOAT4:
      var.emplace<float4>();
      break;
    case VariableDeclaration::Type::QUATF:
      var.emplace<quatf>();
      break;
    case VariableDeclaration::Type::MAT2F:
      var.emplace<mat2f>();
      break;
    case VariableDeclaration::Type::MAT3F:
      var.emplace<mat3f>();
      break;
    case VariableDeclaration::Type::MAT4F:
      var.emplace<mat4f>();
      break;
    case VariableDeclaration::Type::NODE:
      var.emplace<NodeHandle>();
      break;
    case VariableDeclaration::Type::NODE_SCENE:
      var.emplace<NodeSceneHandle>();
      break;
    case VariableDeclaration::Type::PROTO:
      var.emplace<google::protobuf::imp_proto::Any>();
      break;
    case VariableDeclaration::Type::ARRAY:
      var.emplace<imp::LiteralArray>();
      break;
    case VariableDeclaration::Type::TUPLE:
      var.emplace<imp::LiteralTuple>();
      break;
    case VariableDeclaration::Type::MAP:
      var.emplace<imp::LiteralMap>();
      break;
    case VariableDeclaration::Type::RAY_HIT:
      var.emplace<imp::RecipeRayHit>();
      break;
    default:
      var.emplace<std::monostate>();
      break;
  }
}

std::string GetSocketVariableName(const NodeId& node_id,
                                  absl::string_view socket_name) {
  return absl::StrFormat("%s_%s_SOCKET_VALUE", recipe::NodeIdToString(node_id),
                         socket_name);
}

#if IMP_ENABLE_RECIPE_EXPERIMENTAL

std::string ToOpsSymbol(const BinaryExpression::BinaryOps& ops) {
  switch (ops) {
    case BinaryExpression::BinaryOps::ADD:
      return "+";
    case BinaryExpression::BinaryOps::SUBTRACT:
      return "-";
    case BinaryExpression::BinaryOps::MULTIPLY:
      return "*";
    case BinaryExpression::BinaryOps::DIVIDE:
      return "/";
    case BinaryExpression::BinaryOps::MOD:
      return "%";
    case BinaryExpression::BinaryOps::EQUALS:
      return "==";
    case BinaryExpression::BinaryOps::NOT_EQUALS:
      return "!=";
    case BinaryExpression::BinaryOps::GREATER_THAN:
      return ">";
    case BinaryExpression::BinaryOps::LESS_THAN:
      return "<";
    case BinaryExpression::BinaryOps::GREATER_THAN_OR_EQUAL:
      return ">=";
    case BinaryExpression::BinaryOps::LESS_THAN_OR_EQUAL:
      return "<=";
    case BinaryExpression::BinaryOps::AND:
      return "&&";
    case BinaryExpression::BinaryOps::OR:
      return "||";
    case BinaryExpression::BinaryOps::DOT:
      return ".";
    case BinaryExpression::BinaryOps::CROSS:
      return "cross";
    case BinaryExpression::BinaryOps::MIN:
      return "min";
    case BinaryExpression::BinaryOps::MAX:
      return "max";
    default:
      return "unknown";
  }
}

std::optional<bool> CoerceToBool(const Variable& var) {
  if (std::holds_alternative<bool>(var)) {
    return std::get<bool>(var);
  } else if (std::holds_alternative<int>(var)) {
    return std::get<int>(var);
  } else if (std::holds_alternative<float>(var)) {
    return std::get<float>(var);
  } else if (std::holds_alternative<double>(var)) {
    return std::get<double>(var);
  } else if (std::holds_alternative<float2>(var)) {
    return true;
  } else if (std::holds_alternative<float3>(var)) {
    return true;
  } else if (std::holds_alternative<float4>(var)) {
    return true;
  } else if (std::holds_alternative<quatf>(var)) {
    return true;
  } else if (std::holds_alternative<std::string>(var)) {
    auto str = std::get<std::string>(var);
    return !str.empty();
  } else if (std::holds_alternative<NodeHandle>(var)) {
    auto node = std::get<NodeHandle>(var);
    return node.IsValid();
  } else if (std::holds_alternative<NodeSceneHandle>(var)) {
    auto node = std::get<NodeSceneHandle>(var);
    return node.IsValid();
  } else if (std::holds_alternative<LiteralArray>(var)) {
    auto array = std::get<LiteralArray>(var);
    return !array.values.empty();
  } else if (std::holds_alternative<LiteralTuple>(var)) {
    auto tuple = std::get<LiteralTuple>(var);
    return !tuple.values.empty();
  } else if (std::holds_alternative<LiteralMap>(var)) {
    auto map = std::get<LiteralMap>(var);
    return !map.values.empty();
  } else if (std::holds_alternative<google::protobuf::imp_proto::Any>(var)) {
    return true;
  } else if (std::holds_alternative<RecipeRayHit>(var)) {
    return true;
  } else if (std::holds_alternative<std::monostate>(var)) {
    return false;
  }
  return std::nullopt;
}

std::optional<int> CoerceToInt(const Variable& var) {
  if (std::holds_alternative<int>(var)) {
    return std::get<int>(var);
  } else if (std::holds_alternative<float>(var)) {
    return std::get<float>(var);
  } else if (std::holds_alternative<double>(var)) {
    return std::get<double>(var);
  } else if (std::holds_alternative<bool>(var)) {
    return std::get<bool>(var);
  } else if (std::holds_alternative<std::monostate>(var)) {
    return 0;
  }
  return std::nullopt;
}

std::optional<float> CoerceToFloat(const Variable& var) {
  if (std::holds_alternative<int>(var)) {
    return std::get<int>(var);
  } else if (std::holds_alternative<float>(var)) {
    return std::get<float>(var);
  } else if (std::holds_alternative<double>(var)) {
    return std::get<double>(var);
  } else if (std::holds_alternative<bool>(var)) {
    return std::get<bool>(var);
  } else if (std::holds_alternative<std::monostate>(var)) {
    return 0.0f;
  }
  return std::nullopt;
}

std::optional<double> CoerceToDouble(const Variable& var) {
  if (std::holds_alternative<int>(var)) {
    return std::get<int>(var);
  } else if (std::holds_alternative<float>(var)) {
    return std::get<float>(var);
  } else if (std::holds_alternative<double>(var)) {
    return std::get<double>(var);
  } else if (std::holds_alternative<bool>(var)) {
    return std::get<bool>(var);
  } else if (std::holds_alternative<std::monostate>(var)) {
    return 0.0;
  }
  return std::nullopt;
}

std::optional<float3> CoerceToFloat3(const Variable& var) {
  if (std::holds_alternative<float3>(var)) {
    auto val = std::get<float3>(var);
    return val;
  } else if (std::holds_alternative<LiteralTuple>(var)) {
    auto tuple = std::get<LiteralTuple>(var);
    if (tuple.values.size() == 3) {
      auto x = CoerceToFloat(tuple.values[0].value);
      auto y = CoerceToFloat(tuple.values[1].value);
      auto z = CoerceToFloat(tuple.values[2].value);

      if (x.has_value() && y.has_value() && z.has_value()) {
        float3 rv{x.value(), y.value(), z.value()};
        return rv;
      }
    }
  }
  return std::nullopt;
}

std::optional<float4> CoerceToFloat4(const Variable& var) {
  if (std::holds_alternative<float4>(var)) {
    auto val = std::get<float4>(var);
    return val;
  } else if (std::holds_alternative<quatf>(var)) {
    auto val = std::get<quatf>(var);
    return float4{val.x, val.y, val.z, val.w};
  } else if (std::holds_alternative<LiteralTuple>(var)) {
    auto tuple = std::get<LiteralTuple>(var);
    if (tuple.values.size() == 4) {
      auto x = CoerceToFloat(tuple.values[0].value);
      auto y = CoerceToFloat(tuple.values[1].value);
      auto z = CoerceToFloat(tuple.values[2].value);
      auto w = CoerceToFloat(tuple.values[3].value);

      if (x.has_value() && y.has_value() && z.has_value() && w.has_value()) {
        float4 rv{x.value(), y.value(), z.value(), w.value()};
        return rv;
      }
    }
  }
  return std::nullopt;
}

std::optional<quatf> CoerceToQuatf(const Variable& var) {
  if (std::holds_alternative<quatf>(var)) {
    auto val = std::get<quatf>(var);
    return val;
  } else if (std::holds_alternative<float4>(var)) {
    auto val = std::get<float4>(var);
    return quatf{val.x, val.y, val.z, val.w};
  } else if (std::holds_alternative<LiteralTuple>(var)) {
    auto tuple = std::get<LiteralTuple>(var);
    if (tuple.values.size() == 4) {
      auto x = CoerceToFloat(tuple.values[0].value);
      auto y = CoerceToFloat(tuple.values[1].value);
      auto z = CoerceToFloat(tuple.values[2].value);
      auto w = CoerceToFloat(tuple.values[3].value);

      if (x.has_value() && y.has_value() && z.has_value() && w.has_value()) {
        quatf rv{x.value(), y.value(), z.value(), w.value()};
        return rv;
      }
    }
  }
  return std::nullopt;
}

NodeHandle CoerceToNode(const Variable& var) {
  if (std::holds_alternative<NodeHandle>(var)) {
    return std::get<NodeHandle>(var);
  } else if (std::holds_alternative<NodeSceneHandle>(var)) {
    NodeHandle scene_handle = std::get<NodeSceneHandle>(var);
    return scene_handle;
  }
  return NodeHandle();
}
#endif  // IMP_ENABLE_RECIPE_EXPERIMENTAL

}  // namespace recipe

}  // namespace imp
