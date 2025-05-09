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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_GLTF_INTERACTIVITY_EXTENSION_H_
#define THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_GLTF_INTERACTIVITY_EXTENSION_H_

#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "core/assets/gltf/interactivity/converted_graph.h"
#include "core/assets/gltf/interactivity/node_converters.h"
#include "core/async/future.h"
#include "core/common/robin_map.h"
#include "core/model/model_data.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_system.h"
#include "core/ncsb/system.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/recipe_runner.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_extension.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/utils/string_map.h"

namespace imp {

// The entrypoint for the KHR_interactivity extension from the glTF loader.
//
// Upon creation, it converts the KHR_interactivity information stored in the
// attached GltfRenderer into a an equivalent RecipeGraph.
class GltfInteractivityExtension : public GltfExtension {
 public:
  class System : public ComponentSystem<GltfInteractivityExtension> {
   public:
    explicit System(BaseView* view);

    // Register a InteractivityNodeConverter for the given node type.
    //
    // If the type is already registered, it will replace the existing one.
    void RegisterInteractivityNodeConverter(
        const gltf::interactivity::NodeConverter& node_converter);

    // Register a set of InteractivityNodeConverters.
    //
    // For any node converters in the span, if the type is already registered,
    // it will replace the existing one.
    void RegisterInteractivityNodeConverters(
        absl::Span<const gltf::interactivity::NodeConverter> node_converters);

    // Convert a KHR_interactivity node into a RecipeNode.
    //
    // If the type is not registered, absl::NotFoundError will be returned.
    // This will also return any error occurred during the conversion.
    absl::Status ConvertInteractivityNodeAndAddToGraph(
        const model::ModelData::InteractivityData::NodeData& node_data,
        gltf::interactivity::ConvertedGraph& converted_graph) const;

    // Register mapping from KHR_interactivity type to Recipe type.
    //
    // If the type is already registered, it will replace the existing one.
    void RegisterInteractivityType(
        model::ModelData::InteractivityData::ValueType value_type,
        VariableDeclaration::Type recipe_type);

    // Convert KHR_interactivity type signature into Recipe variable type.
    //
    // If the type is not registered, absl::NotFoundError will be returned.
    absl::StatusOr<VariableDeclaration::Type> GetRecipeType(
        model::ModelData::InteractivityData::ValueType value_type) const;

   private:
    StringMap<gltf::interactivity::NodeConverter> node_converters_;
    RobinMap<model::ModelData::InteractivityData::ValueType,
             VariableDeclaration::Type>
        registered_types_;
  };

  // Invoked via SFINAE to test if this extension is valid for the gltf
  // renderer passed in.
  static bool IsValidFor(ComponentHandle<GltfRenderer> gltf_renderer);

  absl::Status Start() override;

 private:
  Future<absl::Status> SetupInternal(
      ComponentHandle<GltfRenderer> gltf_renderer) override;

  std::vector<int> tap_node_gltf_indices_;
  ComponentHandle<RecipeRunner> recipe_runner_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_GLTF_INTERACTIVITY_EXTENSION_H_
