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
#include "core/assets/gltf/interactivity/converted_graph.h"
#include "core/assets/gltf/interactivity/custom_statements/animation_start.h"
#include "core/assets/gltf/interactivity/custom_statements/animation_stop.h"
#include "core/assets/gltf/interactivity/custom_statements/animation_stop_at.h"
#include "core/assets/gltf/interactivity/custom_statements/cancel_delay.h"
#include "core/assets/gltf/interactivity/custom_statements/debug_log.h"
#include "core/assets/gltf/interactivity/custom_statements/do_n.h"
#include "core/assets/gltf/interactivity/custom_statements/multi_gate.h"
#include "core/assets/gltf/interactivity/custom_statements/on_hover.h"
#include "core/assets/gltf/interactivity/custom_statements/on_select.h"
#include "core/assets/gltf/interactivity/custom_statements/pointer_interpolate.h"
#include "core/assets/gltf/interactivity/custom_statements/pointer_set.h"
#include "core/assets/gltf/interactivity/custom_statements/set_delay.h"
#include "core/assets/gltf/interactivity/custom_statements/throttle.h"
#include "core/assets/gltf/interactivity/custom_statements/variable_interpolate.h"
#include "core/assets/gltf/interactivity/custom_statements/variable_set.h"
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
#include "core/assets/gltf/interactivity/node_converters/variable/variable.h"
#include "core/assets/gltf/object_model/pointer_declarations/core_pointers.h"
#include "core/assets/gltf/object_model/pointer_parser.h"
#include "core/assets/gltf/object_model/property_pointer.h"
#include "core/async/future.h"
#include "core/common/registry.h"
#include "core/common/robin_map.h"
#include "core/common/robin_set.h"
#include "core/common/typed_set_vector.h"
#include "core/common/variant.h"
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
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/gltf_scene.h"
#include "core/view/framework/scene/scene_system.h"
#include "core/view/utils/string_map.h"

namespace imp {

using model::ModelData;
using InteractivityData = model::ModelData::InteractivityData;
using PropertyPointer = gltf::PropertyPointer;
using PointerParser = gltf::PointerParser;

namespace {

enum InteractivityType : int { SELECTABILITY = 0, HOVERABILITY = 1 };

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
RobinSet<int> GetInteractivityNodeIndices(
    const ComponentHandle<GltfRenderer>& gltf_renderer,
    InteractivityType interactivity_type) {
  std::stack<NodeHandle> node_stack;
  for (const NodeHandle& child : gltf_renderer->GetModelRoot()->GetChildren()) {
    node_stack.push(child);
  }

  const TypedSetVector<model::EntityData>& entities =
      gltf_renderer->GetGltfAsset()->GetModelData().Entities();
  RobinSet<int> interactivity_node_gltf_indices;
  while (!node_stack.empty()) {
    NodeHandle current_node = node_stack.top();
    node_stack.pop();

    std::optional<model::EntityId> entity_id =
        gltf_renderer->GetEntityIdFromNodeHandle(current_node);
    if (!entity_id.has_value()) {
      continue;
    }

    bool is_interactivity_node = false;
    switch (interactivity_type) {
      case InteractivityType::HOVERABILITY: {
        std::optional<model::NodeHoverability> hoverable =
            entities[*entity_id].node_hoverability;
        is_interactivity_node = !hoverable.has_value() ||
                                !hoverable->hoverable.has_value() ||
                                hoverable->hoverable.value();
        break;
      }
      case InteractivityType::SELECTABILITY: {
        std::optional<model::NodeSelectability> selectable =
            entities[*entity_id].node_selectability;
        is_interactivity_node = !selectable.has_value() ||
                                !selectable->selectable.has_value() ||
                                selectable->selectable.value();
        break;
      }
    }

    if (is_interactivity_node) {
      uint64_t gltf_index = entities[*entity_id].original_index;
      interactivity_node_gltf_indices.insert(static_cast<int>(gltf_index));
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
      gltf_renderer, InteractivityType::HOVERABILITY);

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
      gltf_renderer, InteractivityType::SELECTABILITY);

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
  absl::StatusOr<std::unique_ptr<gltf::interactivity::ConvertedGraph>> result =
      gltf::interactivity::ConvertedGraph::Create(graph_data);
  if (!result.ok()) {
    return Future<absl::Status>(
        absl::InvalidArgumentError("Unable to create ConvertedGraph."));
  }
  gltf::interactivity::ConvertedGraph& converted_graph = **result;

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

  for (const RecipeNode& node : recipe_graph.recipe_nodes) {
    // Create default socket values as member variables of the graph.
    std::optional<StringMap<Literal>> default_socket_values =
        converted_graph.GetDefaultSocketValues(node.id);
    if (default_socket_values.has_value()) {
      for (const auto& [socket_name, socket_value] : *default_socket_values) {
        VariableDeclaration variable_declaration;
        variable_declaration.name =
            recipe::GetSocketVariableName(node.id, socket_name);
        variable_declaration.type = recipe::ToType(socket_value);
        variable_declaration.init_value = socket_value;
        recipe_graph.member_declarations.push_back(variable_declaration);
      }
    }
  }

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

  recipe_graph.member_declarations.push_back(VariableDeclaration{
      .name =
          std::string(gltf::interactivity::kPointerInterpolateMapValueSocket),
      .type = VariableDeclaration::MAP,
      .init_value = Literal{LiteralMap{.values = {}}}});

  // TODO: Look into converting these Stop Propagation Map
  // variables into constant Literal Nodes
  recipe_graph.member_declarations.push_back(VariableDeclaration{
      .name = std::string(gltf::interactivity::kHoverInNodesMap),
      .type = VariableDeclaration::MAP,
      .init_value = Literal{converted_graph.GetOnHoverInStopPropagationMap()}});

  recipe_graph.member_declarations.push_back(VariableDeclaration{
      .name = std::string(gltf::interactivity::kHoverOutNodesMap),
      .type = VariableDeclaration::MAP,
      .init_value =
          Literal{converted_graph.GetOnHoverOutStopPropagationMap()}});

  recipe_graph.member_declarations.push_back(VariableDeclaration{
      .name = std::string(gltf::interactivity::kSelectNodesMap),
      .type = VariableDeclaration::MAP,
      .init_value = Literal{converted_graph.GetTapStopPropagationMap()}});

  ComponentHandle<GltfScene> gltf_scene = GetNode()->GetComponent<GltfScene>();
  if (!gltf_scene) {
    return Future<absl::Status>(
        absl::FailedPreconditionError("No GltfScene found."));
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

  // Converts the tap node indices to NodeHandles.
  std::vector<NodeHandle> tap_targets;
  tap_targets.reserve(converted_graph.GetTapStopPropagationMap().values.size());
  for (const auto& [index, stop_propagation] :
       converted_graph.GetTapStopPropagationMap().values) {
    int gltf_node_index = std::stoi(index);
    if (tap_node_gltf_indices_.contains(gltf_node_index)) {
      tap_targets.push_back(
          gltf_scene->GetOrCreateNodeFromGltfNodeIndex(gltf_node_index));
    }
  }

  // Sets the tap targets on the RecipeRunner.
  recipe_runner_->SetTapTargets(
      absl::Span<NodeHandle>(tap_targets.data(), tap_targets.size()));

  // Converts the hover node indices to NodeHandles.
  std::vector<NodeHandle> hover_targets;
  hover_targets.reserve(
      converted_graph.GetOnHoverInStopPropagationMap().values.size() +
      converted_graph.GetOnHoverOutStopPropagationMap().values.size());
  for (const auto& [index, stop_propagation] :
       converted_graph.GetOnHoverInStopPropagationMap().values) {
    int gltf_node_index = std::stoi(index);
    if (hover_node_gltf_indicies_.contains(gltf_node_index)) {
      hover_targets.push_back(
          gltf_scene->GetOrCreateNodeFromGltfNodeIndex(gltf_node_index));
    }
  }
  for (const auto& [index, stop_propagation] :
       converted_graph.GetOnHoverOutStopPropagationMap().values) {
    int gltf_node_index = std::stoi(index);
    if (hover_node_gltf_indicies_.contains(gltf_node_index)) {
      hover_targets.push_back(
          gltf_scene->GetOrCreateNodeFromGltfNodeIndex(gltf_node_index));
    }
  }

  // Sets the hover targets on the RecipeRunner.
  recipe_runner_->SetHoverTargets(
      absl::Span<NodeHandle>(hover_targets.data(), hover_targets.size()));

  return Future<absl::Status>(absl::OkStatus());
}

bool GltfInteractivityExtension::IsValidFor(
    ComponentHandle<GltfRenderer> gltf_renderer) {
  const ModelData& model_data = gltf_renderer->GetGltfAsset()->GetModelData();
  return model_data.Interactivity();
}

absl::Status GltfInteractivityExtension::Start() {
  if (recipe_runner_) {
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
  RegisterInteractivityNodeConverters(
      gltf::interactivity::GetVariableNodeConverters());
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
      gltf::interactivity::OnHoverCustomStatement>();
  recipe_system.RegisterCustomStatementType<
      gltf::interactivity::OnSelectCustomStatement>();
  recipe_system.RegisterCustomStatementType<
      gltf::interactivity::SetDelayCustomStatement>();
  recipe_system.RegisterCustomStatementType<
      gltf::interactivity::ThrottleCustomStatement>();
  recipe_system.RegisterCustomStatementType<
      gltf::interactivity::VariableInterpolateCustomStatement>();
  recipe_system.RegisterCustomStatementType<
      gltf::interactivity::WaitAllCustomStatement>();
  recipe_system.RegisterCustomStatementType<
      gltf::interactivity::PointerInterpolateCustomStatement>();
  recipe_system.RegisterCustomStatementType<
      gltf::interactivity::PointerSetCustomStatement>();
  recipe_system.RegisterCustomStatementType<
      gltf::interactivity::VariableSetCustomStatement>();
  recipe_system.RegisterCustomStatementType<
      gltf::interactivity::DebugLogCustomStatement>();

  recipe_system.RegisterFunction(
      gltf::interactivity::kGetNodeByIndexFunctionName,
      [](NodeHandle root, int index) {
        ComponentHandle<GltfScene> gltf_scene = root->GetComponent<GltfScene>();

        return gltf_scene->GetOrCreateNodeFromGltfNodeIndex(index);
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

        return_values[recipe::kDefaultOutputSocketName.data()] = args[1];

        std::string pointer_path;
        // The first two arguments are the default value and the gltf model node
        // handle, so we skip them when constructing the pointer path.
        for (int i = 2; i < args.size(); ++i) {
          absl::StrAppend(&pointer_path, recipe::ToString(args[i]));
        }

        std::optional<PropertyPointer> pointer =
            GetPointerParser().TryParse(pointer_path);
        if (!pointer.has_value()) {
          // Returns default values if pointer fails to parse.
          return return_values;
        }

        absl::StatusOr<PropertyPointer::PointerValue> pointer_value =
            pointer->GetValue(gltf_model);
        if (!pointer_value.ok()) {
          // Returns default values if pointer is not valid for the model.
          return return_values;
        }

        absl::StatusOr<recipe::Variable> value =
            TryConvertVariantTo<recipe::Variable>(pointer_value.value());
        if (!value.ok()) {
          return_values["isValid"] = false;
          return return_values;
        }
        return_values[std::string(recipe::kDefaultOutputSocketName)] =
            value.value();
        return_values["isValid"] = true;

        return return_values;
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
