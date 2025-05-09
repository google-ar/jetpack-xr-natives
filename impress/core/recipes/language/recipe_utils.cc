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

#include <iomanip>
#include <ios>
#include <sstream>
#include <string>
#include <variant>

#ifdef IMP_ENABLE_RECIPE_EXPERIMENTAL
#include <optional>
#endif

#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/variant.h"
#include "filament/filament/include/filament/Box.h"
#include "core/geometry/shapes/box.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/proto/any.proto.imp.h"
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
    return "[imp_proto::Any]";
  }

  std::string operator()(const NodeHandle& value) const {
    if (value.IsValid()) {
      return std::string(value->GetName());
    } else {
      return "Invalid NodeHandle";
    }
  }

  std::string operator()(const Box& value) const {
    return absl::StrFormat(
        "Center: (%.3f, %.3f, %.3f) HalfExtent: (%.3f, %.3f, %.3f)",
        value.center[0], value.center[1], value.center[2], value.halfExtent[0],
        value.halfExtent[1], value.halfExtent[2]);
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

  std::string operator()(const absl::monostate& value) const {
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
  if (std::holds_alternative<int>(var)) {
    return VariableDeclaration::Type::INT;
  } else if (std::holds_alternative<float>(var)) {
    return VariableDeclaration::Type::FLOAT;
  } else if (std::holds_alternative<double>(var)) {
    return VariableDeclaration::Type::DOUBLE;
  } else if (std::holds_alternative<bool>(var)) {
    return VariableDeclaration::Type::BOOL;
  } else if (std::holds_alternative<std::string>(var)) {
    return VariableDeclaration::Type::STRING;
  } else if (std::holds_alternative<float2>(var)) {
    return VariableDeclaration::Type::FLOAT2;
  } else if (std::holds_alternative<float3>(var)) {
    return VariableDeclaration::Type::FLOAT3;
  } else if (std::holds_alternative<float4>(var)) {
    return VariableDeclaration::Type::FLOAT4;
  } else if (std::holds_alternative<quatf>(var)) {
    return VariableDeclaration::Type::QUATF;
  } else if (std::holds_alternative<mat3f>(var)) {
    return VariableDeclaration::Type::MAT3F;
  } else if (std::holds_alternative<mat4f>(var)) {
    return VariableDeclaration::Type::MAT4F;
  } else if (std::holds_alternative<NodeHandle>(var)) {
    return VariableDeclaration::Type::NODE;
  } else if (std::holds_alternative<NodeSceneHandle>(var)) {
    return VariableDeclaration::Type::NODE_SCENE;
  } else if (std::holds_alternative<google::protobuf::imp_proto::Any>(var)) {
    return VariableDeclaration::Type::PROTO;
  } else if (std::holds_alternative<::filament::Box>(var)) {
    return VariableDeclaration::Type::BOX;
  } else if (std::holds_alternative<LiteralArray>(var)) {
    return VariableDeclaration::Type::ARRAY;
  } else if (std::holds_alternative<LiteralTuple>(var)) {
    return VariableDeclaration::Type::TUPLE;
  } else if (std::holds_alternative<LiteralMap>(var)) {
    return VariableDeclaration::Type::MAP;
  } else if (std::holds_alternative<RecipeRayHit>(var)) {
    return VariableDeclaration::Type::RAY_HIT;
  }
  return VariableDeclaration::Type::UNKNOWN_VARIABLE_TYPE;
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
    case VariableDeclaration::Type::BOX:
      var.emplace<::filament::Box>();
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
      var.emplace<absl::monostate>();
      break;
  }
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
  } else if (std::holds_alternative<Box>(var)) {
    auto box = std::get<Box>(var);
    return box.halfExtent == float3{0, 0, 0};
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
