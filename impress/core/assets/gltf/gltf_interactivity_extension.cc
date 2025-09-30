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

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stack>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "absl/types/variant.h"
#include "core/animation/property_animator.h"
#include "core/animation/property_animator_state.proto.imp.h"
#include "core/assets/gltf/interactivity/converted_graph.h"
#include "core/assets/gltf/interactivity/custom_statements/animation_start.h"
#include "core/assets/gltf/interactivity/custom_statements/animation_stop.h"
#include "core/assets/gltf/interactivity/custom_statements/animation_stop_at.h"
#include "core/assets/gltf/interactivity/custom_statements/cancel_delay.h"
#include "core/assets/gltf/interactivity/custom_statements/do_n.h"
#include "core/assets/gltf/interactivity/custom_statements/multi_gate.h"
#include "core/assets/gltf/interactivity/custom_statements/pointer_set.h"
#include "core/assets/gltf/interactivity/custom_statements/set_delay.h"
#include "core/assets/gltf/interactivity/custom_statements/throttle.h"
#include "core/assets/gltf/interactivity/custom_statements/variable_interpolate.h"
#include "core/assets/gltf/interactivity/custom_statements/wait_all.h"
#include "core/assets/gltf/interactivity/node_converter_constants.h"
#include "core/assets/gltf/interactivity/node_converters.h"
#include "core/assets/gltf/interactivity/node_converters/animation/animation.h"
#include "core/assets/gltf/interactivity/node_converters/debug/debug.h"
#include "core/assets/gltf/interactivity/node_converters/event/event.h"
#include "core/assets/gltf/interactivity/node_converters/flow/flow.h"
#include "core/assets/gltf/interactivity/node_converters/math/math.h"
#include "core/assets/gltf/interactivity/node_converters/pointer/pointer.h"
#include "core/assets/gltf/interactivity/node_converters/utils.h"
#include "core/assets/gltf/object_model/pointer_declarations/core_pointers.h"
#include "core/assets/gltf/object_model/pointer_parser.h"
#include "core/assets/gltf/object_model/property_pointer.h"
#include "core/async/future.h"
#include "core/collision/ray.h"
#include "core/common/registry.h"
#include "core/common/robin_map.h"
#include "core/math/arrays.proto.imp.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/model/entity_data.h"
#include "core/model/model_data.h"
#include "core/model/shared_data.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_system.h"
#include "core/ncsb/node_handle.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_system.h"
#include "core/recipes/language/recipe_types.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"
#include "core/recipes/recipe_runner.h"
#include "core/recipes/recipe_runner_state.proto.imp.h"
#include "core/view/base_view.h"
#include "core/view/framework/animation/animation.proto.imp.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/gltf_scene.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/framework/scene/scene_system.h"

namespace imp {

using model::ModelData;
using InteractivityData = model::ModelData::InteractivityData;
using PropertyPointer = gltf::PropertyPointer;
using PointerParser = gltf::PointerParser;

namespace {

enum InteractivityType : int { SELECTABILITY = 0, HOVERABILITY = 1 };

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

/**
 * Iterates through the entire glTF node tree and returns a list of indices
 * correlating to glTF nodes that are used for interactivity with the glTF Model
 *
 * Parameters:
 *  ModelData model_data - the model data for the glTF model
 *  ComponentHandle<GltfScene> gltf_scene - the reference to the GltfScene
 * object for the model
 *  NodeHandle root_node - the root node for the glTF Model
 *  InteractivityType interactivity_type -> the type of Interactivity nodes to
 * look for
 *
 * Returns:
 *  std::vector<int> - the list of indicies correlating to nodes that can
 * be interacted
 */
std::vector<int> GetInteractivityNodeIndices(
    const ModelData& model_data, const ComponentHandle<GltfScene>& gltf_scene,
    const NodeHandle& root_node, InteractivityType interactivity_type) {
  const auto& entities = model_data.Entities();
  RobinMap<NodeHandle, model::EntityId> node_to_entity_id_map;
  for (const auto entity_id : entities.Ids<model::EntityId>()) {
    NodeHandle node =
        gltf_scene->GetNodeFromBone(model_data.Entities()[entity_id].bone);
    if (!node.IsValid()) {
      continue;
    }
    node_to_entity_id_map.insert({node, entity_id});
  }

  if (node_to_entity_id_map.empty()) {
    return std::vector<int>{};
  }

  std::stack<NodeHandle> node_stack;
  for (const NodeHandle& child : root_node->GetChildren()) {
    node_stack.push(child);
  }

  std::vector<int> interactivity_node_gltf_indices;
  while (!node_stack.empty()) {
    NodeHandle current_node = node_stack.top();
    node_stack.pop();

    if (!node_to_entity_id_map.contains(current_node)) {
      continue;
    }

    auto entity_data = entities[node_to_entity_id_map.at(current_node)];
    bool is_interactivity_node = false;
    switch (interactivity_type) {
      case InteractivityType::HOVERABILITY: {
        std::optional<model::NodeHoverability> hoverable =
            entity_data.node_hoverability;
        is_interactivity_node = !hoverable.has_value() ||
                                !hoverable->hoverable.has_value() ||
                                hoverable->hoverable.value();
        break;
      }
      case InteractivityType::SELECTABILITY: {
        std::optional<model::NodeSelectability> selectable =
            entity_data.node_selectability;
        is_interactivity_node = !selectable.has_value() ||
                                !selectable->selectable.has_value() ||
                                selectable->selectable.value();
        break;
      }
    }

    if (is_interactivity_node) {
      uint64_t gltf_index = entity_data.original_index;
      interactivity_node_gltf_indices.push_back(static_cast<int>(gltf_index));
      for (const NodeHandle& child : current_node->GetChildren()) {
        node_stack.push(child);
      }
    }
  }

  return interactivity_node_gltf_indices;
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

  if (!gltf_renderer->GetModelRoot().IsValid()) {
    return Future<absl::Status>(
        absl::InvalidArgumentError("Invalid Model Root Node Handle."));
  }

  // Get a list of glTF node indices that are "hoverable" as defined in the
  // KHR_node_hoverability spec
  //
  // Link to KHR_node_hoverability extension spec:
  // https://github.com/KhronosGroup/glTF/blob/355fdb80fe4601eefa687e3380b615a524d4e00a/extensions/2.0/Khronos/KHR_node_hoverability/README.md
  //
  // TODO: add support to update this lists when the mutable
  // pointer property defined in the KHR_node_hoverability extension on a node
  // is changed
  hover_node_gltf_indicies_ = GetInteractivityNodeIndices(
      model_data, gltf_renderer->GetNode()->GetComponent<GltfScene>(),
      gltf_renderer->GetModelRoot(), InteractivityType::HOVERABILITY);

  // Get a list of glTF node indices that are "selectable" as defined in the
  // KHR_node_selectability spec
  //
  // Link to KHR_node_selectability extension spec:
  // https://github.com/KhronosGroup/glTF/blob/cb871bb5b4d5b3d0aa7f2211d2f1b8efa4024a77/extensions/2.0/Khronos/KHR_node_selectability/README.md
  //
  // TODO: add support to update this lists when the mutable
  // pointer property defined in the KHR_node_selectability extension on a node
  // is changed
  tap_node_gltf_indices_ = GetInteractivityNodeIndices(
      model_data, gltf_renderer->GetNode()->GetComponent<GltfScene>(),
      gltf_renderer->GetModelRoot(), InteractivityType::SELECTABILITY);

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

  recipe_graph.member_declarations.push_back(VariableDeclaration{
      .name = std::string(
          gltf::interactivity::AnimationStopAtCustomStatement::kStopTimeMap),
      .type = VariableDeclaration::MAP,
      .init_value = Literal{LiteralMap{.values = {}}}});

  recipe_graph.member_declarations.push_back(VariableDeclaration{
      .name =
          std::string(gltf::interactivity::VariableInterpolateCustomStatement::
                          kVariableInterpolateMap),
      .type = VariableDeclaration::MAP,
      .init_value = Literal{LiteralMap{.values = {}}}});

  // Create RecipeRunner that's initially stopped. It will be started when
  // GltfRenderer's Setup finishes.
  absl::StatusOr<ComponentHandle<RecipeRunner>> add_result =
      GetNode()->AddComponentWithState<RecipeRunner>(
          RecipeRunnerState{.graph = recipe_graph, .start_on_load = false});

  if (!add_result.ok()) {
    return Future<absl::Status>(add_result.status());
  }

  recipe_runner_ = *add_result;

  return Future<absl::Status>(absl::OkStatus());
}

bool GltfInteractivityExtension::IsValidFor(
    ComponentHandle<GltfRenderer> gltf_renderer) {
  const ModelData& model_data = gltf_renderer->GetGltfAsset()->GetModelData();
  return model_data.Interactivity();
}

absl::Status GltfInteractivityExtension::Start() {
  if (recipe_runner_) {
    ComponentHandle<GltfScene> gltf_scene =
        recipe_runner_->GetNode()->GetComponent<GltfScene>();

    // Converts the tap node indices to NodeHandles.
    std::vector<NodeHandle> tap_targets;
    tap_targets.reserve(tap_node_gltf_indices_.size());
    for (int index : tap_node_gltf_indices_) {
      tap_targets.push_back(
          gltf_scene->GetOrCreateNodeFromGltfNodeIndex(index));
    }
    // Sets the tap targets on the RecipeRunner.
    recipe_runner_->SetTapTargets(
        absl::Span<NodeHandle>(tap_targets.data(), tap_targets.size()));

    // Converts the hover node indices to NodeHandles.
    std::vector<NodeHandle> hover_targets;
    hover_targets.reserve(hover_node_gltf_indicies_.size());
    for (int index : hover_node_gltf_indicies_) {
      hover_targets.push_back(
          gltf_scene->GetOrCreateNodeFromGltfNodeIndex(index));
    }
    // Sets the hover targets on the RecipeRunner
    recipe_runner_->SetHoverTargets(
        absl::Span<NodeHandle>(hover_targets.data(), hover_targets.size()));

    return recipe_runner_->Start();
  }

  return absl::FailedPreconditionError("No RecipeRunner found.");
}

GltfInteractivityExtension::System::System(BaseView* view)
    : ComponentSystem<GltfInteractivityExtension>(view) {
  view->GetSceneSystem().RegisterComponentIsfInfo<RecipeRunner>();

  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetVariableSetConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetVariableGetConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetDebugConsoleConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetFlowDelayConverter());
  RegisterInteractivityNodeConverter(
      gltf::interactivity::GetFlowBranchConverter());
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

  // (broken link) start
  RegisterInteractivityNodeConverters(
      gltf::interactivity::GetAnimationNodeConverters());
  RegisterInteractivityNodeConverters(
      gltf::interactivity::GetDebugNodeConverters());
  RegisterInteractivityNodeConverters(
      gltf::interactivity::GetEventNodeConverters());
  RegisterInteractivityNodeConverters(
      gltf::interactivity::GetFlowNodeConverters());
  RegisterInteractivityNodeConverters(
      gltf::interactivity::GetMathNodeConverters());
  RegisterInteractivityNodeConverters(
      gltf::interactivity::GetPointerConverters());
  // (broken link) end

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
  RegisterInteractivityType(InteractivityData::ValueType::MAT2F,
                            VariableDeclaration::MAT2F);
  RegisterInteractivityType(InteractivityData::ValueType::MAT3F,
                            VariableDeclaration::MAT3F);
  RegisterInteractivityType(InteractivityData::ValueType::MAT4F,
                            VariableDeclaration::MAT4F);

  // Register core pointer declarations to the pointer parser.
  GetPointerParser().RegisterPointerDeclarations(
      gltf::GetCorePointerDeclarations());

  RecipeSystem& recipe_system =
      GetView().GetRegistry().GetOrCreate<RecipeSystem>(GetView());

  recipe_system.RegisterCustomStatementType<
      gltf::interactivity::AnimationStartCustomStatement>();
  recipe_system.RegisterCustomStatementType<
      gltf::interactivity::AnimationStopCustomStatement>();
  recipe_system.RegisterCustomStatementType<
      gltf::interactivity::AnimationStopAtCustomStatement>();
  recipe_system.RegisterCustomStatementType<
      gltf::interactivity::CancelDelayCustomStatement>();
  recipe_system
      .RegisterCustomStatementType<gltf::interactivity::DoNCustomStatement>();
  recipe_system.RegisterCustomStatementType<
      gltf::interactivity::MultiGateCustomStatement>();
  recipe_system.RegisterCustomStatementType<
      gltf::interactivity::SetDelayCustomStatement>();
  recipe_system.RegisterCustomStatementType<
      gltf::interactivity::ThrottleCustomStatement>();
  recipe_system.RegisterCustomStatementType<
      gltf::interactivity::VariableInterpolateCustomStatement>();
  recipe_system.RegisterCustomStatementType<
      gltf::interactivity::WaitAllCustomStatement>();
  recipe_system.RegisterCustomStatementType<
      gltf::interactivity::PointerSetCustomStatement>();

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

  recipe_system.RegisterFunction(
      gltf::interactivity::kGetHoverEventDataFunctionName,
      [](NodeHandle hovered_node, int controllerIndex) -> recipe::Variables {
        ComponentHandle<GltfMesh> mesh = hovered_node->GetComponent<GltfMesh>();
        uint64_t gltf_index =
            mesh.IsValid() ? mesh->GetOriginalGltfIndex() : -1;
        const std::string hover_node_index_socket_name =
            std::string(gltf::interactivity::kHoverNodeIndexOutputValueSocket);
        const std::string controller_index_socket_name =
            std::string(gltf::interactivity::kControllerIndexOutputValueSocket);
        recipe::Variables arguments;
        arguments[hover_node_index_socket_name] = static_cast<int>(gltf_index);
        arguments[controller_index_socket_name] = controllerIndex;
        return arguments;
      });

  recipe_system.RegisterFunction(
      gltf::interactivity::kPointerGetFunctionName,
      [this](recipe::Args args) -> absl::StatusOr<recipe::Variables> {
        // Arguments:
        // 0 - NodeHandle : The gltf model node.
        // 1 - default_value : The default value to return if the pointer is
        // not valid for the model.
        // 2 - pointer_path_0 : The first component of the pointer path.
        // 3 - pointer_path_1 : The second component of the pointer path.
        // ...
        recipe::Variables return_values;
        return_values["isValid"] = false;

        if (args.size() <= 2) {
          return absl::InvalidArgumentError(
              "Expected at least 3 arguments for pointer/get.");
        }

        if (!std::holds_alternative<NodeHandle>(args[0])) {
          return absl::InvalidArgumentError(
              "The first argument to pointer/get must be a NodeHandle.");
        }
        NodeHandle gltf_model = std::get<NodeHandle>(args[0]);

        return_values["value"] = args[1];

        std::string pointer_path;
        // The first two arguments are the default value and the gltf model node
        // handle, so we skip them when constructing the pointer path.
        for (int i = 2; i < args.size(); ++i) {
          absl::StrAppend(&pointer_path, recipe::ToString(args[i]));
        }

        std::optional<PropertyPointer> pointer =
            GetPointerParser().TryParse(pointer_path);
        if (!pointer.has_value()) {
          // Returns error if pointer is invalid.
          return absl::NotFoundError(
              absl::StrFormat("Invalid pointer: %s.", pointer_path));
        }

        absl::StatusOr<PropertyPointer::PointerValue> value =
            pointer->GetValue(gltf_model);
        if (!value.ok()) {
          // Returns default values if pointer is not valid for the model.
          return return_values;
        }

        return_values["value"] =
            absl::ConvertVariantTo<recipe::Variable>(value.value());
        return_values["isValid"] = true;

        return return_values;
      });

  recipe_system.RegisterFunction(
      gltf::interactivity::kGetSelectEventDataFunctionName,
      [&view = GetView()](NodeHandle selected_node, int controllerIndex,
                          float2 screen_pos,
                          RecipeRayHit selection_ray_hit) -> recipe::Variables {
        ComponentHandle<GltfMesh> mesh =
            selected_node->GetComponent<GltfMesh>();
        uint64_t gltf_index =
            mesh.IsValid() ? mesh->GetOriginalGltfIndex() : -1;
        const std::string selected_node_index_socket_name =
            std::string(gltf::interactivity::kSelectNodeIndexOutputValueSocket);
        const std::string controller_index_socket_name =
            std::string(gltf::interactivity::kControllerIndexOutputValueSocket);
        const std::string selection_point_socket_name =
            std::string(gltf::interactivity::kSelectSelectionPointValueSocket);
        const std::string selection_ray_origin_socket_name = std::string(
            gltf::interactivity::kSelectSelectionRayOriginValueSocket);

        float3 selection_point{std::numeric_limits<float>::quiet_NaN()};
        float3 ray_origin{std::numeric_limits<float>::quiet_NaN()};
        if (selection_ray_hit.node.IsValid()) {
          selection_point = selection_ray_hit.world_point;

          Ray world_ray =
              view.GetCameraManager().GetCamera()->WorldRayFromPixelPoint(
                  screen_pos);
          ray_origin = world_ray.origin;
        }

        recipe::Variables arguments;
        arguments[selected_node_index_socket_name] =
            static_cast<int>(gltf_index);
        arguments[controller_index_socket_name] = controllerIndex;
        arguments[selection_point_socket_name] = selection_point;
        arguments[selection_ray_origin_socket_name] = ray_origin;
        return arguments;
      });

  recipe_system.RegisterFunction(
      "Switch",
      [](recipe::Args input_args) -> absl::StatusOr<recipe::Variable> {
        if (input_args.size() < 3) {
          return absl::InvalidArgumentError(
              "Switch requires at least 3 arguments, namely the selection "
              "value, the "
              "default value, and the case to value connection offset map.");
        }
        recipe::Variable input_selection = input_args[0];
        recipe::Variable input_default = input_args[1];
        recipe::Variable input_case_to_value_offset = input_args[2];

        // Check the validity and get the actual value of "selection".
        if (!std::holds_alternative<int>(input_selection)) {
          return absl::InvalidArgumentError(
              "Switch requires the 1st parameter (selection value) to be an "
              "integer.");
        }
        int selection = std::get<int>(input_selection);

        // Check the validity of the case to value connection offset map.
        if (!std::holds_alternative<LiteralMap>(input_case_to_value_offset)) {
          return absl::InvalidArgumentError(
              "Switch requires the 3rd parameter (case to value connection "
              "offset) "
              "to be a LiteralMap.");
        }
        LiteralMap case_to_value_offset_map =
            std::get<LiteralMap>(input_case_to_value_offset);
        for (const auto& [key, value] : case_to_value_offset_map.values) {
          if (!std::holds_alternative<int>(value.value)) {
            return absl::InvalidArgumentError(
                "Switch requires the 3rd parameter (case to value connection "
                "offset "
                "map) to have integer values in the map.");
          }
        }

        // KHR_Interactivity spec requires all the value connections matched to
        // "cases" to have the same type as the default value connection. Here
        // we check that.
        int value_type_index = input_default.index();
        for (int i = 3; i < input_args.size(); i++) {
          if (input_args[i].index() != value_type_index) {
            return absl::InvalidArgumentError(
                "Switch requires all the value connections matched to cases to "
                "have "
                "the same type as the default value connection.");
          }
        }

        if (case_to_value_offset_map.values.find(std::to_string(selection)) ==
            case_to_value_offset_map.values.end()) {
          return input_default;
        }
        int offset = std::get<int>(
            case_to_value_offset_map.values[std::to_string(selection)].value);
        if (offset + 3 >= input_args.size()) {
          return absl::InvalidArgumentError(absl::StrFormat(
              "case %d has an offset of %d, which is out of range.", selection,
              offset));
        }
        return input_args[offset + 3];
      });

  recipe_system.RegisterFunction(gltf::interactivity::kGltfAsrFunctionName,
                                 [](int value, int shift_amount) -> int {
                                   return (value >> (shift_amount & 0x1F));
                                 });

  recipe_system.RegisterFunction(gltf::interactivity::kGltfLslFunctionName,
                                 [](int value, int shift_amount) -> int {
                                   return (value << (shift_amount & 0x1F));
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

PointerParser& GltfInteractivityExtension::System::GetPointerParser() {
  return GetView().GetRegistry().GetOrCreate<PointerParser>();
}

}  // namespace imp
