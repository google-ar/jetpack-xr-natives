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

#ifndef THIRD_PARTY_IMPRESS_CORE_MODEL_INTERACTIVITY_DATA_H_
#define THIRD_PARTY_IMPRESS_CORE_MODEL_INTERACTIVITY_DATA_H_

#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "absl/types/variant.h"
#include "core/common/typed_id.h"
#include "core/math/math.h"

namespace imp::model {

struct InteractivityData {
  enum ValueType {
    BOOL,
    INT,
    FLOAT,
    FLOAT2,
    FLOAT3,
    FLOAT4,
    MAT2F,
    MAT3F,
    MAT4F,
    STRING,
  };

  using VariableValue = std::variant<bool, int, float, float2, float3, float4,
                                     mat2f, mat3f, mat4f, std::string>;

  struct VariableData {
    std::string id;
    ValueType type;
    VariableValue value;
  };

  struct NodeData {
    enum ConfigurationType {
      VARIABLE,
      NUMBER_OF_OUTPUT_FLOWS,
      POINTER,
      EVENT,
      NODE_INDEX,
      STOP_PROPAGATION,
      EASING_TYPE,
      EASING_DURATION,
      CASES,
      TYPE,
      NUMBER_OF_INPUT_FLOWS,
      IS_RANDOM,
      IS_LOOP,
      MESSAGE,
      INITIAL_INDEX,
      VARIABLES,
      USE_SLERP,
      SEVERITY,
    };

    struct FlowData {
      std::string id;
      int node = 0;
      std::string socket;
    };

    using ConfigurationValue =
        std::variant<int, std::string, bool, float, std::vector<int>>;

    struct ConfigurationData {
      ConfigurationType id;
      ConfigurationValue value;
    };

    using ValueData = std::variant<VariableData, FlowData>;

    std::string type;
    int index = 0;
    std::vector<FlowData> flows;
    std::vector<ConfigurationData> configuration;
    std::vector<ValueData> values;
  };

  struct EventData {
    std::string id;
    std::vector<VariableData> values;
  };

  struct TypeData {
    ValueType type;
  };

  struct DeclarationData {
    std::string op;
  };

  struct GraphData {
    std::vector<DeclarationData> declarations;
    std::vector<NodeData> nodes;
    std::vector<VariableData> variables;
    std::vector<EventData> events;
    std::vector<TypeData> types;
  };

  std::vector<GraphData> graphs;
  uint32_t graph_index = 0;
};

using InteractivityGraphId = TypedIdWithSentinel<InteractivityData, uint16_t>;

using InteractivityNodeId =
    TypedIdWithSentinel<InteractivityData::NodeData, uint16_t>;
using InteractivityNodeFlowId =
    TypedIdWithSentinel<InteractivityData::NodeData::FlowData, uint16_t>;
using InteractivityNodeConfigurationId =
    TypedIdWithSentinel<InteractivityData::NodeData::ConfigurationData,
                        uint16_t>;
using InteractivityNodeValueId =
    TypedIdWithSentinel<InteractivityData::NodeData::ValueData, uint16_t>;
using InteractivityVariableId =
    TypedIdWithSentinel<InteractivityData::VariableData, uint16_t>;
using InteractivityEventId =
    TypedIdWithSentinel<InteractivityData::EventData, uint16_t>;
using InteractivityTypeId =
    TypedIdWithSentinel<InteractivityData::TypeData, uint16_t>;
using InteractivityDeclarationId =
    TypedIdWithSentinel<InteractivityData::DeclarationData, uint16_t>;

}  // namespace imp::model

#endif  // THIRD_PARTY_IMPRESS_CORE_MODEL_INTERACTIVITY_DATA_H_
