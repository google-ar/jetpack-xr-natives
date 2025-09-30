/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_UTILS_H_
#define THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_UTILS_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#ifdef IMP_ENABLE_RECIPE_EXPERIMENTAL
#include <optional>
#endif

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/common/invocable.h"
#include "core/math/vec.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/view/utils/string_map.h"

namespace imp {

namespace recipe {

using VariableType = VariableDeclaration::Type;

using Variable = decltype(std::declval<Literal>().value);
using Variables = StringMap<Variable>;

using Args = std::vector<Variable>;
using NamedArgs = std::vector<std::pair<std::string, Variable>>;
using SocketReturnValue = std::variant<Variable, Future<recipe::Variable>>;

struct ReturnValueDeclaration {
  StringMap<SocketReturnValue> socket_values;
  std::optional<Future<absl::Status>> status_future;
};

struct ReturnValue {
  Variables values;
  std::optional<Future<Variables>> async_values;
};

using RecipeFunction = Invocable<absl::StatusOr<ReturnValue>(Args&)>;

inline constexpr absl::string_view kDefaultOutputSocketName = "out";
inline constexpr absl::string_view kDefaultInputSocketName = "in";

inline constexpr absl::string_view kDefaultArgPrefix = "arg_";

inline constexpr absl::string_view kDefaultAsyncExecutionIdSocketName =
    "async_execution_id";

// Hash function for NodeId.
struct NodeIdHash {
  size_t operator()(const NodeId& id) const {
    return absl::Hash<uint32_t>()(id.index);
  }
};

// Comparison function for NodeId.
struct NodeIdEqual {
  bool operator()(const NodeId& lhs, const NodeId& rhs) const {
    return lhs.index == rhs.index;
  }
};

template <typename T>
using NodeIdMap = absl::flat_hash_map<NodeId, T, NodeIdHash, NodeIdEqual>;

std::string NodeIdToString(const NodeId& id);

// Returns a string representation of the literal meant to display in the UI.
// Examples: 1, "Hello", "NodeName"
std::string ToString(const Literal& literal);

std::string ToString(const Variable& variable);

VariableDeclaration::Type ToType(const Variable& var);

VariableDeclaration::Type ToType(const Literal& literal);

absl::string_view ToTypeName(const VariableDeclaration::Type& type);

absl::string_view ToTypeName(const Variable& var);

absl::string_view ToTypeName(const Literal& literal);

void SetToDefault(const VariableDeclaration::Type& type, Variable& var);

#if IMP_ENABLE_RECIPE_EXPERIMENTAL

// Returns the symbol associated with for the given binary operator. i.e. "+"
std::string ToOpsSymbol(const BinaryExpression::BinaryOps& op);

// Coercion helpers - Each functions attempts to coerce the variable to the
// specified type. Returns std::nullopt if the variable cannot be coerced.
std::optional<bool> CoerceToBool(const Variable& var);

std::optional<int> CoerceToInt(const Variable& var);

std::optional<float> CoerceToFloat(const Variable& var);

std::optional<double> CoerceToDouble(const Variable& var);

std::optional<float3> CoerceToFloat3(const Variable& var);

std::optional<float4> CoerceToFloat4(const Variable& var);

std::optional<quatf> CoerceToQuatf(const Variable& var);

std::optional<std::string> CoerceToString(const Variable& var);

// Attempts to coerce the variable to a NodeHandle. Returns an invalid
// NodeHandle if the variable is not successful.
NodeHandle CoerceToNode(const Variable& var);

// Helper function to create a LiteralTuple from a sequence of Variables.
template <typename... Args>
LiteralTuple MakeTuple(const Args&... args) {
  LiteralTuple tuple;
  tuple.values.reserve(sizeof...(args));
  (tuple.values.push_back(Literal{.value = args}), ...);
  return tuple;
}

#endif  // IMP_ENABLE_RECIPE_EXPERIMENTAL

inline constexpr absl::string_view kNodeSelfVariableName = "NodeSelf";

inline constexpr absl::string_view kOnStartEventName = "OnStartEvent";

inline constexpr absl::string_view kOnUpdateEventName = "OnUpdateEvent";
inline constexpr absl::string_view kDeltaSecondsSocketName = "delta_seconds";
inline constexpr absl::string_view kElapsedSecondsSocketName =
    "elapsed_seconds";

inline constexpr absl::string_view kOnTapEventName = "OnTapEvent";
inline constexpr absl::string_view kTapTargetSocketName = "tap_target";
inline constexpr absl::string_view kTapPositionSocketName = "tap_position";
inline constexpr absl::string_view kTapRayHitSocketName = "tap_ray_hit";

inline constexpr absl::string_view kOnHoverBeginEventName = "OnHoverBeginEvent";
inline constexpr absl::string_view kOnHoverEndEventName = "OnHoverEndEvent";
inline constexpr absl::string_view kHoverTargetSocketName = "hover_target";

inline constexpr absl::string_view kEventNames[] = {
    kOnStartEventName, kOnUpdateEventName, kOnTapEventName,
    kOnHoverBeginEventName, kOnHoverEndEventName};

inline constexpr absl::string_view kControllerIndexSocketName =
    "controller_index";

inline constexpr absl::string_view kTimeSinceStart = "time_since_start";

inline constexpr absl::string_view kMathPi = "math_pi";
inline constexpr absl::string_view kMathE = "math_e";
inline constexpr absl::string_view kMathNan = "math_nan";
inline constexpr absl::string_view kMathInf = "math_inf";

template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
recipe::Variable Acos(T v) {
  return acos(v);
}

template <typename T>
TVec2<T> Acos(TVec2<T> v) {
  return TransformVector(acos, v);
}

template <typename T>
TVec3<T> Acos(TVec3<T> v) {
  return TransformVector(acos, v);
}

template <typename T>
TVec4<T> Acos(TVec4<T> v) {
  return TransformVector(acos, v);
}

template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
recipe::Variable Asin(T v) {
  return asin(v);
}

template <typename T>
TVec2<T> Asin(TVec2<T> v) {
  return TransformVector(asin, v);
}

template <typename T>
TVec3<T> Asin(TVec3<T> v) {
  return TransformVector(asin, v);
}

template <typename T>
TVec4<T> Asin(TVec4<T> v) {
  return TransformVector(asin, v);
}

template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
recipe::Variable Atan(T v) {
  return atan(v);
}

template <typename T>
TVec2<T> Atan(TVec2<T> v) {
  return TransformVector(atan, v);
}

template <typename T>
TVec3<T> Atan(TVec3<T> v) {
  return TransformVector(atan, v);
}

template <typename T>
TVec4<T> Atan(TVec4<T> v) {
  return TransformVector(atan, v);
}

template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
recipe::Variable Cos(T v) {
  return cos(v);
}

template <typename T>
TVec2<T> Cos(TVec2<T> v) {
  return TransformVector(cos, v);
}

template <typename T>
TVec3<T> Cos(TVec3<T> v) {
  return TransformVector(cos, v);
}

template <typename T>
TVec4<T> Cos(TVec4<T> v) {
  return TransformVector(cos, v);
}

template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
recipe::Variable Max(T left, T right) {
  return std::max(left, right);
}

template <typename T>
TVec2<T> Max(TVec2<T> left, TVec2<T> right) {
  return max(left, right);
}

template <typename T>
TVec3<T> Max(TVec3<T> left, TVec3<T> right) {
  return max(left, right);
}

template <typename T>
TVec4<T> Max(TVec4<T> left, TVec4<T> right) {
  return max(left, right);
}

template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
recipe::Variable Min(T left, T right) {
  return std::min(left, right);
}

template <typename T>
TVec2<T> Min(TVec2<T> left, TVec2<T> right) {
  return min(left, right);
}

template <typename T>
TVec3<T> Min(TVec3<T> left, TVec3<T> right) {
  return min(left, right);
}

template <typename T>
TVec4<T> Min(TVec4<T> left, TVec4<T> right) {
  return min(left, right);
}

// Returns the sign of the input value.
//    1 if value > 0.
//    0 if value == 0.
//    -1 if value < 0.
template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
T Sign(T v) {
  return v > T(0) ? T(1) : (v < T(0) ? T(-1) : T(0));
}

// Returns the sign of the input vector.
// Each component of the resulting vector will be the sign of its corresponding
// component.
template <typename T>
TVec2<T> Sign(TVec2<T> v) {
  return TVec2<T>(Sign(v.x), Sign(v.y));
}

// Returns the sign of the input vector.
// Each component of the resulting vector will be the sign of its corresponding
// component.
template <typename T>
TVec3<T> Sign(TVec3<T> v) {
  return TVec3<T>(Sign(v.x), Sign(v.y), Sign(v.z));
}

// Returns the sign of the input vector.
// Each component of the resulting vector will be the sign of its corresponding
// component.
template <typename T>
TVec4<T> Sign(TVec4<T> v) {
  return TVec4<T>(Sign(v.x), Sign(v.y), Sign(v.z), Sign(v.w));
}

template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
recipe::Variable Sin(T v) {
  return sin(v);
}

template <typename T>
TVec2<T> Sin(TVec2<T> v) {
  return TransformVector(sin, v);
}

template <typename T>
TVec3<T> Sin(TVec3<T> v) {
  return TransformVector(sin, v);
}

template <typename T>
TVec4<T> Sin(TVec4<T> v) {
  return TransformVector(sin, v);
}

template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
recipe::Variable Tan(T v) {
  return tan(v);
}

template <typename T>
TVec2<T> Tan(TVec2<T> v) {
  return TransformVector(tan, v);
}

template <typename T>
TVec3<T> Tan(TVec3<T> v) {
  return TransformVector(tan, v);
}

template <typename T>
TVec4<T> Tan(TVec4<T> v) {
  return TransformVector(tan, v);
}

}  // namespace recipe

inline LiteralArray operator+(const LiteralArray& lhs,
                              const LiteralArray& rhs) {
  LiteralArray result = lhs;
  for (const auto& literal : rhs.values) {
    result.values.push_back(literal);
  }
  return result;
}

inline LiteralArray& operator+=(LiteralArray& lhs, const LiteralArray& rhs) {
  for (const auto& literal : rhs.values) {
    lhs.values.push_back(literal);
  }
  return lhs;
}

template <typename T>
inline LiteralArray& operator+=(LiteralArray& lhs, const T& rhs) {
  lhs.values.push_back(Literal{.value = rhs});
  return lhs;
}

namespace output {

// Set this to true to enable debug logging for Recipe code.
constexpr bool kEnableRecipeLog = false;

// Used for debug logging in Recipe related code.
//
// This is used so that we can check in logs that are disabled at compile time
// by default but can be enabled for debugging.
template <class... Args>
void Recipe(const absl::FormatSpec<Args...>& format, Args&&... args) {
  if constexpr (kEnableRecipeLog) {
    IMP_LOG(imp::INFO) << absl::StrFormat(format, std::forward<Args>(args)...);
  }
}

}  // namespace output

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_UTILS_H_
