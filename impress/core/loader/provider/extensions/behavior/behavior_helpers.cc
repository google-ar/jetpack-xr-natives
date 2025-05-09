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

#include "core/loader/provider/extensions/behavior/behavior_helpers.h"

#include <string>
#include <variant>

#include "core/loader/provider/extensions/behavior/behavior.proto.imp.h"
#include "core/math/mat.h"
#include "core/math/vec.h"

namespace imp::loader::extensions {

bool IsValidVariable(const gltf::Behavior::Variable& variable) {
  switch (variable.type) {
    case imp::gltf::Behavior::ValueType::BOOL:
      return std::holds_alternative<bool>(variable.value);
      break;
    case imp::gltf::Behavior::ValueType::INT:
      return std::holds_alternative<int>(variable.value);
      break;
    case imp::gltf::Behavior::ValueType::FLOAT:
      return std::holds_alternative<float>(variable.value);
      break;
    case imp::gltf::Behavior::ValueType::FLOAT2:
      return std::holds_alternative<float2>(variable.value);
      break;
    case imp::gltf::Behavior::ValueType::FLOAT3:
      return std::holds_alternative<float3>(variable.value);
      break;
    case imp::gltf::Behavior::ValueType::FLOAT4:
      return std::holds_alternative<float4>(variable.value);
      break;
    case imp::gltf::Behavior::ValueType::MAT4F:
      return std::holds_alternative<mat4f>(variable.value);
      break;
    case imp::gltf::Behavior::ValueType::STRING:
      return std::holds_alternative<std::string>(variable.value);
      break;
    default:
      return false;
  }
}

bool IsValidFlow(const gltf::Behavior::Node::Flow& flow) {
  return !flow.socket.empty() && flow.node >= 0;
}

}  // namespace imp::loader::extensions
