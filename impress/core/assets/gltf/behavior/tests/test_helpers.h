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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_BEHAVIOR_TEST_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_BEHAVIOR_TEST_HELPERS_H_

#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "core/assets/gltf/behavior/converted_graph.h"
#include "core/assets/gltf/behavior/node_converters.h"
#include "core/assets/gltf/behavior/world_pointer.h"
#include "core/common/imp_matchers.h"
#include "core/common/registry.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/model/model_data.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_runtime_event.h"
#include "core/recipes/language/recipe_runtime_graph.h"
#include "core/recipes/language/recipe_scope.h"
#include "core/recipes/language/recipe_system.h"
#include "core/recipes/language/recipe_utils.h"
#include "core/view/base_view.h"

namespace imp::gltf_testing {

using ::testing::MockFunction;

using BehaviorData = ::imp::model::ModelData::BehaviorData;
using ConvertedGraph = ::imp::gltf::behavior::ConvertedGraph;
using NodeConverter = ::imp::gltf::behavior::NodeConverter;
using WorldPointer = ::imp::gltf::behavior::WorldPointer;

template <typename T>
::testing::Matcher<T> EqualOrAlmostEqual(const T& expected) {
  if constexpr (kIsAnyOf<T, float2, float3, float4, mat4f, quatf>) {
    return ::imp::testing::AlmostEqual(expected);
  } else if constexpr (kIsAnyOf<T, float>) {
    return ::testing::FloatEq(expected);
  } else {
    return ::testing::Eq(expected);
  }
}

template <typename ResultT>
struct NodeOutput {
  ResultT result;
  std::string socket_name = std::string(recipe::kDefaultOutputSocketName);
};

template <typename... T>
void EvaluateAndVerifyValueNode(BaseView* view,
                                BehaviorData::NodeData& test_node,
                                NodeConverter node_converter,
                                NodeOutput<T>... expected_outputs) {
  RecipeSystem& recipe_system =
      view->GetRegistry().GetOrCreate<RecipeSystem>(*view);

  int on_start_node_index = test_node.index + 1;
  int result_node_index = test_node.index + 2;
  BehaviorData::NodeData life_cycle_on_start_node;
  life_cycle_on_start_node.index = on_start_node_index;
  life_cycle_on_start_node.type = "lifecycle/onStart";
  life_cycle_on_start_node.flows.push_back(BehaviorData::NodeData::FlowData{
      .id = std::string(gltf::behavior::kDefaultOutputFlowSocket),
      .node = result_node_index,
      .socket = "in",
  });

  BehaviorData::NodeData result_node;
  result_node.index = result_node_index;

  BehaviorData behavior_data;
  behavior_data.nodes = {
      life_cycle_on_start_node,
      test_node,
      result_node,
  };
  ConvertedGraph converted_graph(behavior_data);

  MP_ASSERT_OK(gltf::behavior::GetLifeCycleOnStartConverter().function(
      life_cycle_on_start_node, converted_graph));
  MP_ASSERT_OK(node_converter.function(test_node, converted_graph));

  MP_ASSERT_OK_AND_ASSIGN(RecipeNode & recipe_test_node,
                       converted_graph.GetNode(test_node.index));
  MP_ASSERT_OK_AND_ASSIGN(RecipeNode & recipe_result_node,
                       converted_graph.GetNode(result_node.index));

  CallExpression expression{.name = "GetResultFunction"};
  (expression.args.push_back(ValueConnection{
       .connection =
           SocketConnection{.node_id = recipe_test_node.id,
                            .socket_name = expected_outputs.socket_name}}),
   ...);
  recipe_result_node.node =
      ExecutableNode{.statement = CallStatement{.expression = expression}};

  MockFunction<void(T...)> get_result_function;
  recipe_system.RegisterFunction("GetResultFunction",
                                 get_result_function.AsStdFunction());

  EXPECT_CALL(get_result_function,
              Call(EqualOrAlmostEqual(expected_outputs.result)...))
      .Times(1);

  RecipeScope scope;
  RecipeGraph recipe_graph{.recipe_nodes = converted_graph.GetRecipeNodes()};

  MP_ASSERT_OK_AND_ASSIGN(
      auto runtime_graph,
      RecipeRuntimeGraph::CreateRuntimeGraph(*view, recipe_graph));

  auto result = runtime_graph->TriggerEvent(
      RecipeRuntimeEvent{.name = std::string(recipe::kOnStartEventName)},
      &scope);

  EXPECT_OK(result);
}

}  // namespace imp::gltf_testing

#endif  // THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_BEHAVIOR_TEST_HELPERS_H_
