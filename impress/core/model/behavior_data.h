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

#ifndef THIRD_PARTY_IMPRESS_CORE_MODEL_BEHAVIOR_DATA_H_
#define THIRD_PARTY_IMPRESS_CORE_MODEL_BEHAVIOR_DATA_H_

#include <cstdint>
#include <string>
#include <vector>

#include "absl/types/optional.h"
#include "absl/types/variant.h"
#include "core/common/typed_id.h"
#include "core/math/math.h"

namespace imp::model {

// TODO Move this into a separate file to reduce bloat
struct BehaviorData {
  enum ValueType {
    BOOL,
    INT,
    FLOAT,
    FLOAT2,
    FLOAT3,
    FLOAT4,
    MAT4F,
    STRING,
  };

  struct VariableData {
    std::string id;
    ValueType type;
    std::variant<bool, int, float, float2, float3, float4, mat4f, std::string>
        value;
  };
  struct NodeData {
    enum ConfigurationType {
      VARIABLE,
      NUMBER_OF_OUTPUT_FLOWS,
      PATH,
      CUSTOM_EVENT,
      NODE_INDEX,
      STOP_PROPAGATION,
      EASING_TYPE,
      EASING_DURATION,
    };

    struct FlowData {
      std::string id;
      int node = 0;
      std::string socket;
    };

      // TODO  Possibly add some of the Configuration value types
      // as enums rather than raw strings?
      struct ConfigurationData {
        ConfigurationType id;
        std::optional<ValueType> value_type;
        std::variant<int, std::string, bool, float> value;
      };

      using ValueData = absl::variant<VariableData, FlowData>;

      std::string type;
      int index = 0;
      std::vector<FlowData> flows;
      std::vector<ConfigurationData> configuration;
      std::vector<ValueData> values;
  };

  struct CustomEventData {
    std::string id;
    std::vector<VariableData> values;
  };

  struct TypeData {
    std::string signature;
  };

  std::vector<NodeData> nodes;
  std::vector<VariableData> variables;
  std::vector<CustomEventData> custom_events;
  std::vector<TypeData> types;
};

using BehaviorNodeId = TypedIdWithSentinel<BehaviorData::NodeData, uint16_t>;
using BehaviorNodeFlowId =
    TypedIdWithSentinel<BehaviorData::NodeData::FlowData, uint16_t>;
using BehaviorNodeConfigurationId =
    TypedIdWithSentinel<BehaviorData::NodeData::ConfigurationData, uint16_t>;
using BehaviorNodeValueId =
    TypedIdWithSentinel<BehaviorData::NodeData::ValueData, uint16_t>;
using BehaviorVariableId =
    TypedIdWithSentinel<BehaviorData::VariableData, uint16_t>;
using BehaviorCustomEventId =
    TypedIdWithSentinel<BehaviorData::CustomEventData, uint16_t>;
using BehaviorTypeId = TypedIdWithSentinel<BehaviorData::TypeData, uint16_t>;

}  // namespace imp::model

#endif  // THIRD_PARTY_IMPRESS_CORE_MODEL_BEHAVIOR_DATA_H_
