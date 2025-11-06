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

#include "core/assets/gltf/gltf_behavior_extension.h"

#include <cstdint>
#include <memory>
#include <stack>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "absl/types/variant.h"
#include "core/animation/property_animator.h"
#include "core/animation/property_animator_state.proto.imp.h"
#include "core/assets/gltf/behavior/converted_graph.h"
#include "core/assets/gltf/behavior/node_converter_constants.h"
#include "core/assets/gltf/behavior/node_converters.h"
#include "core/async/future.h"
#include "core/common/registry.h"
#include "core/common/robin_map.h"
#include "core/math/arrays.proto.imp.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/model/model_data.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_system.h"
#include "core/ncsb/node_handle.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_system.h"
#include "core/recipes/recipe_runner.h"
#include "core/recipes/recipe_runner_state.proto.imp.h"
#include "core/view/base_view.h"
#include "core/view/framework/animation/animation.proto.imp.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/gltf_scene.h"
#include "core/view/framework/scene/scene_system.h"

namespace imp {

using model::ModelData;
using BehaviorData = model::ModelData::BehaviorData;

namespace {

// WorldPointerPropertyAnimation stores information of a PropertyAnimation
// created from a world pointer.
struct WorldPointerPropertyAnimation {
  AnimationSampler sampler;
  std::unique_ptr<PropertyAnimation> animation;
};

template <typename Fn>
void WorldAnimateToFunction(NodeHandle root, NodeHandle node,
                            AnimationSampler sampler, Fn animation_function) {
  ComponentHandle<PropertyAnimator> property_animator =
      root->GetOrAddComponent<PropertyAnimator>();

  std::unique_ptr<WorldPointerPropertyAnimation> animation =
      std::make_unique<WorldPointerPropertyAnimation>();
  WorldPointerPropertyAnimation* animation_ptr = animation.get();
  animation->sampler = sampler;

  // Creates PropertyAnimation from the provided sampler and function.
  absl::StatusOr<std::unique_ptr<PropertyAnimation>> property_animation =
      property_animator->AddAnimation(&(animation->sampler),
                                      animation_function);
  if (!property_animation.ok()) {
    IMP_LOG(imp::WARNING) << "Gltf_WorldAnimateTo_Translation failed to play: "
                 << property_animation.status();
    return;
  }
  animation->animation = std::move(*property_animation);

  // Plays the animation.
  Future<absl::Status> play_future = animation_ptr->animation->PlayAsync();
  play_future.DependsOn(std::move(animation));
  play_future.KeptBy(node);
}

std::vector<int> GetTapNodes(ComponentHandle<GltfScene>& gltf_scene,
                             gltf::behavior::ConvertedGraph& converted_graph) {
  std::stack<NodeHandle> node_stack;
  for (int index : converted_graph.GetTapNodeIndices()) {
    node_stack.push(gltf_scene->GetOrCreateNodeFromGltfNodeIndex(index));
  }

  std::vector<int> tap_node_gltf_indices;
  absl::flat_hash_set<NodeHandle> visited;
  while (!node_stack.empty()) {
    NodeHandle current_node = node_stack.top();
    node_stack.pop();

    if (visited.find(current_node) != visited.end() ||
        !gltf_scene->HasEntityDataForNodeHandle(current_node)) {
      continue;
    }

    visited.emplace(current_node);

    uint64_t gltf_index =
        gltf_scene->GetEntityDataFromNodeHandle(current_node).original_index;
    tap_node_gltf_indices.push_back(static_cast<int>(gltf_index));
    for (const NodeHandle& child : current_node->GetChildren()) {
      node_stack.push(child);
    }
  }

  return tap_node_gltf_indices;
}

}  // namespace

Future<absl::Status> GltfBehaviorExtension::SetupInternal(
    ComponentHandle<GltfRenderer> gltf_renderer) {
  if (!gltf_renderer) {
    return Future<absl::Status>(
        absl::InvalidArgumentError("Invalid GltfRenderer handle."));
  }

  const ModelData& model_data = gltf_renderer->GetGltfAsset()->GetModelData();
  if (!model_data.Behavior()) {
    return Future<absl::Status>(absl::FailedPreconditionError(
        "No behavior extension information found."));
  }

  if (!gltf_renderer->GetModelRoot().IsValid()) {
    return Future<absl::Status>(
        absl::InvalidArgumentError("Invalid Model Root Node Handle."));
  }

  const BehaviorData& behavior_data = *model_data.Behavior();
  System& system = GetView()
                       .GetComponentManager()
                       .GetComponentSystem<GltfBehaviorExtension>();

  // Convert behavior graph to recipe graph
  RecipeGraph recipe_graph;

  // Create node index mapping.
  gltf::behavior::ConvertedGraph converted_graph(behavior_data);

  // Convert behavior nodes to recipe nodes
  for (const BehaviorData::NodeData& node_data : behavior_data.nodes) {
    absl::Status convert_status =
        system.ConvertBehaviorNodeAndAddToGraph(node_data, converted_graph);
    if (!convert_status.ok()) {
      // TODO: Handle unknown types properly instead of failing.
      return Future<absl::Status>(convert_status);
    }
  }

  recipe_graph.recipe_nodes = converted_graph.GetRecipeNodes();

  const std::vector<BehaviorData::VariableData>& variables =
      behavior_data.variables;
  for (const BehaviorData::VariableData& variable : variables) {
    VariableDeclaration variable_declaration;
    variable_declaration.name = variable.id;
    absl::StatusOr<VariableDeclaration::Type> recipe_type =
        system.GetRecipeType(variable.type);
    if (!recipe_type.ok()) {
      // TODO: Handle unknown types properly instead of failing.
      return Future<absl::Status>(absl::NotFoundError(
          absl::StrFormat("Unsupported behavior ValueType read: %d. Skipping.",
                          variable.type)));
    }

    variable_declaration.type = *recipe_type;
    variable_declaration.init_value =
        Literal{.value = absl::ConvertVariantTo<decltype(Literal::value)>(
                    variable.value)};
    recipe_graph.member_declarations.push_back(variable_declaration);
  }

  // Create RecipeRunner that's initially stopped. It will be started when
  // GltfRenderer's Setup finishes.
  absl::StatusOr<ComponentHandle<RecipeRunner>> add_result =
      GetNode()->AddComponentWithState<RecipeRunner>(
          RecipeRunnerState{.graph = recipe_graph, .start_on_load = false});

  if (!add_result.ok()) {
    return Future<absl::Status>(add_result.status());
  }

  recipe_runner_ = *add_result;

  ComponentHandle<GltfScene> gltf_scene =
      gltf_renderer->GetNode()->GetComponent<GltfScene>();
  tap_node_gltf_indices_ = GetTapNodes(gltf_scene, converted_graph);

  return Future<absl::Status>(absl::OkStatus());
}

bool GltfBehaviorExtension::IsValidFor(
    ComponentHandle<GltfRenderer> gltf_renderer) {
  const ModelData& model_data = gltf_renderer->GetGltfAsset()->GetModelData();
  return model_data.Behavior();
}

absl::Status GltfBehaviorExtension::Start() {
  if (recipe_runner_) {
    // Converts the tap node indices to NodeHandles.
    std::vector<NodeHandle> tap_targets;
    tap_targets.reserve(tap_node_gltf_indices_.size());
    for (int index : tap_node_gltf_indices_) {
      tap_targets.push_back(recipe_runner_->GetNode()
                                ->GetComponent<GltfScene>()
                                ->GetOrCreateNodeFromGltfNodeIndex(index));
    }
    // Sets the tap targets on the RecipeRunner.
    recipe_runner_->SetTapTargets(
        absl::Span<NodeHandle>(tap_targets.data(), tap_targets.size()));

    return recipe_runner_->Start();
  }

  return absl::FailedPreconditionError("No RecipeRunner found.");
}

GltfBehaviorExtension::System::System(BaseView* view)
    : ComponentSystem<GltfBehaviorExtension>(view) {
  view->GetSceneSystem().RegisterComponentIsfInfo<RecipeRunner>();

  RegisterBehaviorNodeConverter(gltf::behavior::GetLifeCycleOnStartConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetLifeCycleOnTickConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetVariableSetConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetVariableGetConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetCustomEventSendConverter());
  RegisterBehaviorNodeConverter(
      gltf::behavior::GetCustomEventReceiveConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetDebugConsoleConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetFlowForLoopConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetFlowSequenceConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetFlowDelayConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetFlowBranchConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetNodeOnSelectConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetWorldGetConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetWorldSetConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetWorldAnimateToConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathAddConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathSubConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathMulConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathDivConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathRemConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathGeConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathLeConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathGtConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathLtConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathEqConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathDotConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathCrossConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathMinConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathMaxConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathClampConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathAbsConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathSqrtConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathSinConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathCosConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathTanConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathAsinConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathAcosConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathAtanConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathAtanTwoConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathLogConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathSignConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathNormalizeConverter());
  RegisterBehaviorNodeConverter(
      gltf::behavior::GetTypeCastBoolToIntConverter());
  RegisterBehaviorNodeConverter(
      gltf::behavior::GetTypeCastBoolToFloatConverter());
  RegisterBehaviorNodeConverter(
      gltf::behavior::GetTypeCastIntToBoolConverter());
  RegisterBehaviorNodeConverter(
      gltf::behavior::GetTypeCastIntToFloatConverter());
  RegisterBehaviorNodeConverter(
      gltf::behavior::GetTypeCastFloatToBoolConverter());
  RegisterBehaviorNodeConverter(
      gltf::behavior::GetTypeCastFloatToIntConverter());
  RegisterBehaviorNodeConverter(
      gltf::behavior::GetWorldStartAnimationConverter());
  RegisterBehaviorNodeConverter(
      gltf::behavior::GetWorldStopAnimationConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathMakeVector2Converter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathMakeVector3Converter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathMakeVector4Converter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathPiConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathBreakVector2Converter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathBreakVector3Converter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathBreakVector4Converter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathComposeConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathDecomposeConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathInverseConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetMathMatMulConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetFlowStopAudioConverter());
  RegisterBehaviorNodeConverter(gltf::behavior::GetAsyncPlaySoundConverter());

  // TODO: Support more types.
  // TODO: We're not supporting registering String types for now,
  // due to security concerns. We can revisit this later.
  // Register default behavior types
  RegisterBehaviorType(BehaviorData::ValueType::BOOL,
                       VariableDeclaration::Type::BOOL);
  RegisterBehaviorType(BehaviorData::ValueType::INT,
                       VariableDeclaration::Type::INT);
  RegisterBehaviorType(BehaviorData::ValueType::FLOAT,
                       VariableDeclaration::Type::FLOAT);
  RegisterBehaviorType(BehaviorData::ValueType::FLOAT2,
                       VariableDeclaration::Type::FLOAT2);
  RegisterBehaviorType(BehaviorData::ValueType::FLOAT3,
                       VariableDeclaration::Type::FLOAT3);
  RegisterBehaviorType(BehaviorData::ValueType::FLOAT4,
                       VariableDeclaration::Type::FLOAT4);

  RecipeSystem& recipe_system =
      GetView().GetRegistry().GetOrCreate<RecipeSystem>(GetView());

  recipe_system.RegisterFunction(
      gltf::behavior::kGetNodeByIndexFunctionName,
      [](NodeHandle root, int index) {
        ComponentHandle<GltfScene> gltf_scene = root->GetComponent<GltfScene>();

        return gltf_scene->GetOrCreateNodeFromGltfNodeIndex(index);
      });

  recipe_system.RegisterFunction(
      gltf::behavior::kWorldAnimateToTranslationRecipeFunctionName,
      [](NodeHandle root, NodeHandle node, float3 target, float duration) {
        AnimationSampler sampler{
            .times_seconds = {0.0f, duration},
            .values_array =
                AnimationValues{
                    .type = Float3Array{.values = {node->GetLocalPosition(),
                                                   target}}},
            // TODO: Support other easing modes.
            .interpolation = InterpolationMode::INTERPOLATION_EASE_IN_OUT_CUBIC,
        };

        WorldAnimateToFunction(root, node, sampler, [node](float3 value) {
          if (node) {
            node->SetLocalPosition(value);
          }
        });
      });

  recipe_system.RegisterFunction(
      gltf::behavior::kWorldAnimateToRotationRecipeFunctionName,
      [](NodeHandle root, NodeHandle node, float4 target, float duration) {
        AnimationSampler sampler{
            .times_seconds = {0.0f, duration},
            .values_array =
                AnimationValues{
                    .type = QuatfArray{.values = {node->GetLocalRotation(),
                                                  quatf(target)}}},
            // TODO: Support other easing modes.
            .interpolation = InterpolationMode::INTERPOLATION_EASE_IN_OUT_CUBIC,
        };

        WorldAnimateToFunction(root, node, sampler, [node](quatf value) {
          if (node) {
            node->SetLocalRotation(value);
          }
        });
      });

  recipe_system.RegisterFunction(
      gltf::behavior::kWorldAnimateToScaleRecipeFunctionName,
      [](NodeHandle root, NodeHandle node, float3 target, float duration) {
        AnimationSampler sampler{
            .times_seconds = {0.0f, duration},
            .values_array =
                AnimationValues{
                    .type =
                        Float3Array{.values = {node->GetLocalScale(), target}}},
            // TODO: Support other easing modes.
            .interpolation = InterpolationMode::INTERPOLATION_EASE_IN_OUT_CUBIC,
        };

        WorldAnimateToFunction(root, node, sampler, [node](float3 value) {
          if (node) {
            node->SetLocalScale(value);
          }
        });
      });
}

void GltfBehaviorExtension::System::RegisterBehaviorNodeConverter(
    const gltf::behavior::NodeConverter& behavior_node_converter) {
  behavior_node_converters_[std::string(behavior_node_converter.node_type)] =
      behavior_node_converter;
}

absl::Status GltfBehaviorExtension::System::ConvertBehaviorNodeAndAddToGraph(
    const model::ModelData::BehaviorData::NodeData& node_data,
    gltf::behavior::ConvertedGraph& converted_graph) const {
  auto it = behavior_node_converters_.find(node_data.type);
  if (it == behavior_node_converters_.end()) {
    return absl::NotFoundError(absl::StrFormat(
        "No BehaviorNodeConverter for type %s was found.", node_data.type));
  }
  return it->second.function(node_data, converted_graph);
}

void GltfBehaviorExtension::System::RegisterBehaviorType(
    BehaviorData::ValueType behavior_type,
    VariableDeclaration::Type recipe_type) {
  registered_behavior_types_[behavior_type] = recipe_type;
}

absl::StatusOr<VariableDeclaration::Type>
GltfBehaviorExtension::System::GetRecipeType(
    BehaviorData::ValueType behavior_type) const {
  auto it = registered_behavior_types_.find(behavior_type);
  if (it == registered_behavior_types_.end()) {
    return absl::NotFoundError(absl::StrFormat(
        "No BehaviorType for type %d was found.", behavior_type));
  }
  return it->second;
}

}  // namespace imp
