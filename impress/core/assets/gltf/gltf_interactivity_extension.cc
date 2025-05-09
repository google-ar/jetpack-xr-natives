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

#include "core/assets/gltf/gltf_interactivity_extension.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "absl/types/variant.h"
#include "core/animation/property_animator.h"
#include "core/animation/property_animator_state.proto.imp.h"
#include "core/assets/gltf/interactivity/converted_graph.h"
#include "core/assets/gltf/interactivity/node_converter_constants.h"
#include "core/assets/gltf/interactivity/node_converters.h"
#include "core/assets/gltf/interactivity/node_converters/event/event.h"
#include "core/assets/gltf/interactivity/node_converters/math/math.h"
#include "core/async/future.h"
#include "core/common/registry.h"
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
using InteractivityData = model::ModelData::InteractivityData;

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

}  // namespace

Future<absl::Status> GltfInteractivityExtension::SetupInternal(
    ComponentHandle<GltfRenderer> gltf_renderer) {
  if (!gltf_renderer) {
    return Future<absl::Status>(
        absl::InvalidArgumentError("Invalid GltfRenderer handle."));
  }

  const ModelData& model_data = gltf_renderer->GetGltfAsset()->GetModelData();
  if (!model_data.Interactivity()) {
    return Future<absl::Status>(absl::FailedPreconditionError(
        "No interactivity extension information found."));
  }

  const InteractivityData& interactivity_data = *model_data.Interactivity();
  // TODO: Support loading non-default graphs.
  const InteractivityData::GraphData& graph_data =
      interactivity_data.graphs[interactivity_data.graph_index];

  System& system = GetView()
                       .GetComponentManager()
                       .GetComponentSystem<GltfInteractivityExtension>();

  // Convert interactivity graph to recipe graph
  RecipeGraph recipe_graph;

  // Create node index mapping.
  gltf::interactivity::ConvertedGraph converted_graph(graph_data);

  // Convert interactivity nodes to recipe nodes
  for (const InteractivityData::NodeData& node_data : graph_data.nodes) {
    absl::Status convert_status = system.ConvertInteractivityNodeAndAddToGraph(
        node_data, converted_graph);
    if (!convert_status.ok()) {
      // TODO: Handle unknown types properly instead of failing.
      return Future<absl::Status>(convert_status);
    }
  }

  recipe_graph.recipe_nodes = converted_graph.GetRecipeNodes();

  const std::vector<InteractivityData::VariableData>& variables =
      graph_data.variables;
  for (const InteractivityData::VariableData& variable : variables) {
    VariableDeclaration variable_declaration;
    variable_declaration.name = variable.id;
    absl::StatusOr<VariableDeclaration::Type> recipe_type =
        system.GetRecipeType(variable.type);
    if (!recipe_type.ok()) {
      // TODO: Handle unknown types properly instead of failing.
      return Future<absl::Status>(absl::NotFoundError(absl::StrFormat(
          "Unsupported interactivity ValueType read: %d. Skipping.",
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

  tap_node_gltf_indices_ = converted_graph.GetTapNodeIndices();

  return Future<absl::Status>(absl::OkStatus());
}

bool GltfInteractivityExtension::IsValidFor(
    ComponentHandle<GltfRenderer> gltf_renderer) {
  const ModelData& model_data = gltf_renderer->GetGltfAsset()->GetModelData();
  return model_data.Interactivity();
}

absl::Status GltfInteractivityExtension::Start() {
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

GltfInteractivityExtension::System::System(BaseView* view)
    : ComponentSystem<GltfInteractivityExtension>(view) {
  view->GetSceneSystem().RegisterComponentIsfInfo<RecipeRunner>();

  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetEventOnStartConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetEventOnTickConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetVariableSetConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetVariableGetConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetEventSendConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetEventReceiveConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetDebugConsoleConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetFlowForLoopConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetFlowSequenceConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetFlowDelayConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetFlowBranchConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetNodeOnSelectConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetWorldGetConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetWorldSetConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetWorldAnimateToConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathAddConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathSubConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathMulConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathDivConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathRemConverter());
  RegisterInteractivityNodeConverter(gltf::interactivity::GetMathGeConverter());
  RegisterInteractivityNodeConverter(gltf::interactivity::GetMathLeConverter());
  RegisterInteractivityNodeConverter(gltf::interactivity::GetMathGtConverter());
  RegisterInteractivityNodeConverter(gltf::interactivity::GetMathLtConverter());
  RegisterInteractivityNodeConverter(gltf::interactivity::GetMathEqConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathDotConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathCrossConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathMinConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathMaxConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathClampConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathAbsConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathSqrtConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathSinConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathCosConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathTanConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathAsinConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathAcosConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathAtanConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathAtanTwoConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathLogConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathSignConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathNormalizeConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetTypeCastBoolToIntConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetTypeCastBoolToFloatConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetTypeCastIntToBoolConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetTypeCastIntToFloatConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetTypeCastFloatToBoolConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetTypeCastFloatToIntConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetWorldStartAnimationConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetWorldStopAnimationConverter());
  RegisterInteractivityNodeConverter(gltf::interactivity::GetMathPiConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathComposeConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathDecomposeConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathInverseConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetMathMatMulConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetFlowStopAudioConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetAsyncPlaySoundConverter());

  RegisterInteractivityNodeConverters(
      gltf::interactivity::GetMathNodeConverters());

  // TODO: Support more types.
  // TODO: We're not supporting registering String types for now,
  // due to security concerns. We can revisit this later.
  // Register default interactivity types
  RegisterInteractivityType(InteractivityData::ValueType::BOOL,
                            VariableDeclaration::Type::BOOL);
  RegisterInteractivityType(InteractivityData::ValueType::INT,
                            VariableDeclaration::Type::INT);
  RegisterInteractivityType(InteractivityData::ValueType::FLOAT,
                            VariableDeclaration::Type::FLOAT);
  RegisterInteractivityType(InteractivityData::ValueType::FLOAT2,
                            VariableDeclaration::Type::FLOAT2);
  RegisterInteractivityType(InteractivityData::ValueType::FLOAT3,
                            VariableDeclaration::Type::FLOAT3);
  RegisterInteractivityType(InteractivityData::ValueType::FLOAT4,
                            VariableDeclaration::Type::FLOAT4);

  RecipeSystem& recipe_system =
      GetView().GetRegistry().GetOrCreate<RecipeSystem>(GetView());

  recipe_system.RegisterFunction(
      gltf::interactivity::kGetNodeByIndexFunctionName,
      [](NodeHandle root, int index) {
        ComponentHandle<GltfScene> gltf_scene = root->GetComponent<GltfScene>();

        return gltf_scene->GetOrCreateNodeFromGltfNodeIndex(index);
      });

  recipe_system.RegisterFunction(
      gltf::interactivity::kWorldAnimateToTranslationRecipeFunctionName,
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
      gltf::interactivity::kWorldAnimateToRotationRecipeFunctionName,
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
      gltf::interactivity::kWorldAnimateToScaleRecipeFunctionName,
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

void GltfInteractivityExtension::System::RegisterInteractivityNodeConverter(
    const gltf::interactivity::NodeConverter& node_converter) {
  node_converters_[std::string(node_converter.node_type)] = node_converter;
}

void GltfInteractivityExtension::System::RegisterInteractivityNodeConverters(
    absl::Span<const gltf::interactivity::NodeConverter> node_converters) {
  for (const gltf::interactivity::NodeConverter& node_converter :
       node_converters) {
    RegisterInteractivityNodeConverter(node_converter);
  }
}

absl::Status
GltfInteractivityExtension::System::ConvertInteractivityNodeAndAddToGraph(
    const model::ModelData::InteractivityData::NodeData& node_data,
    gltf::interactivity::ConvertedGraph& converted_graph) const {
  auto it = node_converters_.find(node_data.type);
  if (it == node_converters_.end()) {
    return absl::NotFoundError(
        absl::StrFormat("No InteractivityNodeConverter for type %s was found.",
                        node_data.type));
  }
  return it->second.function(node_data, converted_graph);
}

void GltfInteractivityExtension::System::RegisterInteractivityType(
    InteractivityData::ValueType value_type,
    VariableDeclaration::Type recipe_type) {
  registered_types_[value_type] = recipe_type;
}

absl::StatusOr<VariableDeclaration::Type>
GltfInteractivityExtension::System::GetRecipeType(
    InteractivityData::ValueType value_type) const {
  auto it = registered_types_.find(value_type);
  if (it == registered_types_.end()) {
    return absl::NotFoundError(absl::StrFormat(
        "No InteractivityType for type %d was found.", value_type));
  }
  return it->second;
}

}  // namespace imp
