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

#include "core/loader/provider/extensions/interactivity/interactivity_helpers.h"

#include <limits>
#include <string>
#include <variant>

#include "core/loader/provider/extensions/interactivity/interactivity.proto.imp.h"
#include "core/math/mat.h"
#include "core/math/vec.h"

namespace imp::loader::extensions::interactivity {

bool IsValidVariable(const gltf::Interactivity::Graph::Variable& variable) {
  if (variable.type == imp::gltf::Interactivity::Graph::ValueType::UNKNOWN) {
    return false;
  }

  if (std::holds_alternative<std::monostate>(variable.value)) {
    // The specs allow for variables to be defined without a value. In this
    // case, a default value will be used according to its type.
    return true;
  }

  switch (variable.type) {
    case imp::gltf::Interactivity::Graph::ValueType::BOOL:
      return std::holds_alternative<bool>(variable.value);
      break;
    case imp::gltf::Interactivity::Graph::ValueType::INT:
      return std::holds_alternative<int>(variable.value);
      break;
    case imp::gltf::Interactivity::Graph::ValueType::FLOAT:
      return std::holds_alternative<float>(variable.value);
      break;
    case imp::gltf::Interactivity::Graph::ValueType::FLOAT2:
      return std::holds_alternative<float2>(variable.value);
      break;
    case imp::gltf::Interactivity::Graph::ValueType::FLOAT3:
      return std::holds_alternative<float3>(variable.value);
      break;
    case imp::gltf::Interactivity::Graph::ValueType::FLOAT4:
      return std::holds_alternative<float4>(variable.value);
      break;
    case imp::gltf::Interactivity::Graph::ValueType::MAT4F:
      return std::holds_alternative<mat4f>(variable.value);
      break;
    case imp::gltf::Interactivity::Graph::ValueType::STRING:
      return std::holds_alternative<std::string>(variable.value);
      break;
    default:
      return false;
  }
}

bool IsValidFlow(const gltf::Interactivity::Graph::Node::Flow& flow) {
  return !flow.socket.empty() && flow.node >= 0;
}

template <>
bool GetDefaultValue<bool>() {
  return false;
}

template <>
int GetDefaultValue<int>() {
  return 0;
}

template <>
float GetDefaultValue<float>() {
  return std::numeric_limits<float>::quiet_NaN();
}

template <>
float2 GetDefaultValue<float2>() {
  return float2(std::numeric_limits<float>::quiet_NaN());
}

template <>
float3 GetDefaultValue<float3>() {
  return float3(std::numeric_limits<float>::quiet_NaN());
}

template <>
float4 GetDefaultValue<float4>() {
  return float4(std::numeric_limits<float>::quiet_NaN());
}

template <>
mat4f GetDefaultValue<mat4f>() {
  return mat4f(float4(std::numeric_limits<float>::quiet_NaN()),
               float4(std::numeric_limits<float>::quiet_NaN()),
               float4(std::numeric_limits<float>::quiet_NaN()),
               float4(std::numeric_limits<float>::quiet_NaN()));
}

template <>
std::string GetDefaultValue<std::string>() {
  return "";
}

void SetToDefaultValue(gltf::Interactivity::Graph::Variable& variable) {
  switch (variable.type) {
    case imp::gltf::Interactivity::Graph::ValueType::BOOL:
      variable.value = GetDefaultValue<bool>();
      break;
    case imp::gltf::Interactivity::Graph::ValueType::INT:
      variable.value = GetDefaultValue<int>();
      break;
    case imp::gltf::Interactivity::Graph::ValueType::FLOAT:
      variable.value = GetDefaultValue<float>();
      break;
    case imp::gltf::Interactivity::Graph::ValueType::FLOAT2:
      variable.value = GetDefaultValue<float2>();
      break;
    case imp::gltf::Interactivity::Graph::ValueType::FLOAT3:
      variable.value = GetDefaultValue<float3>();
      break;
    case imp::gltf::Interactivity::Graph::ValueType::FLOAT4:
      variable.value = GetDefaultValue<float4>();
      break;
    case imp::gltf::Interactivity::Graph::ValueType::MAT4F:
      variable.value = GetDefaultValue<mat4f>();
      break;
    case imp::gltf::Interactivity::Graph::ValueType::STRING:
      variable.value = GetDefaultValue<std::string>();
      break;
    case imp::gltf::Interactivity::Graph::ValueType::UNKNOWN:
      break;
  }
}

}  // namespace imp::loader::extensions::interactivity
