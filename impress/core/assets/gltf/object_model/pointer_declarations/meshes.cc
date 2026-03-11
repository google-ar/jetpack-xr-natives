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

#include "core/assets/gltf/object_model/pointer_declarations/meshes.h"

#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "core/assets/gltf/object_model/property_pointer.h"
#include "core/assets/gltf/object_model/token_parser.h"
#include "core/common/robin_set.h"
#include "core/model/shared_data.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/gltf_scene.h"

namespace imp::gltf {

using PointerValue = PropertyPointer::PointerValue;

namespace {
using PointerDeclaration = PropertyPointer::PointerDeclaration;

constexpr absl::string_view kMeshesLengthToken = "meshes.length";
constexpr absl::string_view kMeshesToken = "meshes";
constexpr absl::string_view kWeightsLengthToken = "weights.length";
constexpr absl::string_view kWeightsToken = "weights";
constexpr absl::string_view kPrimitivesLengthToken = "primitives.length";
constexpr absl::string_view kPrimitivesToken = "primitives";
constexpr absl::string_view kMaterialToken = "material";

}  // namespace

std::vector<TokenParser> MeshesLengthPointerDeclaration::GetTokenParsers()
    const {
  return {std::string(kMeshesLengthToken)};
}

absl::StatusOr<PointerValue> MeshesLengthPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  ComponentHandle<GltfRenderer> gltf_renderer =
      gltf_model->GetComponent<GltfRenderer>();
  if (!gltf_renderer) {
    return absl::InternalError("No GltfRenderer found on the glTF model node.");
  }
  return gltf_renderer->GetMeshCount();
}

absl::Status MeshesLengthPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PointerValue value) const {
  return absl::FailedPreconditionError(
      "Setting meshes length is not supported.");
}

std::vector<TokenParser>
MeshesWeightsLengthPointerDeclaration::GetTokenParsers() const {
  return {std::string(kMeshesToken), GetIntTokenParser(),
          std::string(kWeightsLengthToken)};
}

absl::StatusOr<PointerValue> MeshesWeightsLengthPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  if (!std::holds_alternative<int>(parsed_tokens[1])) {
    return absl::FailedPreconditionError(
        "The second parsed component is expected to be an int.");
  }
  int mesh_index = std::get<int>(parsed_tokens[1]);

  ComponentHandle<GltfRenderer> gltf_renderer =
      gltf_model->GetComponent<GltfRenderer>();
  if (!gltf_renderer) {
    return absl::InternalError("No GltfRenderer found on the glTF model node.");
  }
  const RobinSet<NodeHandle>* nodes =
      gltf_renderer->GetNodesFromOriginalMeshIndex(mesh_index);
  if (!nodes) {
    return absl::InternalError(
        "This mesh does not exist or is not used in the scene.");
  }

  // Get the weights length from any of the nodes that use this mesh.
  return static_cast<int>(
      (*nodes->begin())->GetComponent<GltfMesh>()->GetMorphTargetCount());
}

absl::Status MeshesWeightsLengthPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PointerValue value) const {
  return absl::FailedPreconditionError(
      "Setting weights length values is not supported.");
}

std::vector<TokenParser> MeshesWeightsPointerDeclaration::GetTokenParsers()
    const {
  return {std::string(kMeshesToken), GetIntTokenParser(),
          std::string(kWeightsToken)};
}

absl::StatusOr<PointerValue> MeshesWeightsPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  if (!std::holds_alternative<int>(parsed_tokens[1])) {
    return absl::FailedPreconditionError(
        "The second parsed component is expected to be an int.");
  }
  int mesh_index = std::get<int>(parsed_tokens[1]);

  ComponentHandle<GltfRenderer> gltf_renderer =
      gltf_model->GetComponent<GltfRenderer>();
  if (!gltf_renderer) {
    return absl::InternalError("No GltfRenderer found on the glTF model node.");
  }

  return gltf_renderer->GetMeshMorphTargetWeights(mesh_index);
}

absl::Status MeshesWeightsPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PointerValue value) const {
  if (!std::holds_alternative<std::vector<float>>(value)) {
    return absl::FailedPreconditionError(
        "A std::vector<float> value is expected.");
  }
  if (!std::holds_alternative<int>(parsed_tokens[1])) {
    return absl::FailedPreconditionError(
        "The second parsed component is expected to be an int.");
  }
  int mesh_index = std::get<int>(parsed_tokens[1]);

  ComponentHandle<GltfRenderer> gltf_renderer =
      gltf_model->GetComponent<GltfRenderer>();
  if (!gltf_renderer) {
    return absl::InternalError("No GltfRenderer found on the glTF model node.");
  }
  gltf_renderer->SetMeshMorphTargetWeights(std::get<std::vector<float>>(value),
                                           mesh_index);
  return absl::OkStatus();
}

std::vector<TokenParser> MeshesWeightPointerDeclaration::GetTokenParsers()
    const {
  return {std::string(kMeshesToken), GetIntTokenParser(),
          std::string(kWeightsToken), GetIntTokenParser()};
}

absl::StatusOr<PointerValue> MeshesWeightPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  if (!std::holds_alternative<int>(parsed_tokens[1])) {
    return absl::FailedPreconditionError(
        "The second parsed component is expected to be an int.");
  }
  int mesh_index = std::get<int>(parsed_tokens[1]);

  if (!std::holds_alternative<int>(parsed_tokens[3])) {
    return absl::FailedPreconditionError(
        "The fourth parsed component is expected to be an int.");
  }
  int target_index = std::get<int>(parsed_tokens[3]);

  ComponentHandle<GltfRenderer> gltf_renderer =
      gltf_model->GetComponent<GltfRenderer>();
  if (!gltf_renderer) {
    return absl::InternalError("No GltfRenderer found on the glTF model node.");
  }
  std::optional<float> weight =
      gltf_renderer->GetMeshMorphTargetWeight(mesh_index, target_index);
  if (!weight.has_value()) {
    return absl::InternalError(
        "No mesh weight found for the given mesh and morph target index.");
  }
  return weight.value();
}

absl::Status MeshesWeightPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PointerValue value) const {
  if (!std::holds_alternative<float>(value)) {
    return absl::FailedPreconditionError(
        "A float value is expected for the weight.");
  }
  float weight = std::get<float>(value);

  if (!std::holds_alternative<int>(parsed_tokens[1])) {
    return absl::FailedPreconditionError(
        "The second parsed component is expected to be an int.");
  }
  int mesh_index = std::get<int>(parsed_tokens[1]);

  if (!std::holds_alternative<int>(parsed_tokens[3])) {
    return absl::FailedPreconditionError(
        "The fourth parsed component is expected to be an int.");
  }
  int target_index = std::get<int>(parsed_tokens[3]);

  ComponentHandle<GltfRenderer> gltf_renderer =
      gltf_model->GetComponent<GltfRenderer>();
  if (!gltf_renderer) {
    return absl::InternalError("No GltfRenderer found on the glTF model node.");
  }

  return gltf_renderer->SetMeshMorphTargetWeight(weight, mesh_index,
                                                 target_index);
}

std::vector<TokenParser> MeshesPrimitivesLengthDeclaration::GetTokenParsers()
    const {
  return {std::string(kMeshesToken), GetIntTokenParser(),
          std::string(kPrimitivesLengthToken)};
}

absl::StatusOr<PointerValue> MeshesPrimitivesLengthDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  if (!std::holds_alternative<int>(parsed_tokens[1])) {
    return absl::FailedPreconditionError(
        "The second parsed component is expected to be an int.");
  }
  int mesh_index = std::get<int>(parsed_tokens[1]);

  ComponentHandle<GltfRenderer> gltf_renderer =
      gltf_model->GetComponent<GltfRenderer>();
  if (!gltf_renderer) {
    return absl::InternalError("No GltfRenderer found on the glTF model node.");
  }
  const RobinSet<NodeHandle>* nodes =
      gltf_renderer->GetNodesFromOriginalMeshIndex(mesh_index);
  if (!nodes) {
    return absl::InternalError(
        "This mesh does not exist or is not used in the scene.");
  }

  // Get the primitive count from any of the nodes that use this mesh.
  return static_cast<int>(
      (*nodes->begin())->GetComponent<GltfMesh>()->GetPrimitiveCount());
}

absl::Status MeshesPrimitivesLengthDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PointerValue value) const {
  return absl::FailedPreconditionError(
      "Setting primitives length values is not supported.");
}

std::vector<TokenParser>
MeshesPrimitivesMaterialPointerDeclaration::GetTokenParsers() const {
  return {std::string(kMeshesToken), GetIntTokenParser(),
          std::string(kPrimitivesToken), GetIntTokenParser(),
          std::string(kMaterialToken)};
}

absl::StatusOr<PointerValue>
MeshesPrimitivesMaterialPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  if (!std::holds_alternative<int>(parsed_tokens[1])) {
    return absl::FailedPreconditionError(
        "The second parsed component is expected to be an int.");
  }
  int mesh_index = std::get<int>(parsed_tokens[1]);

  if (!std::holds_alternative<int>(parsed_tokens[3])) {
    return absl::FailedPreconditionError(
        "The fourth parsed component is expected to be an int.");
  }
  int primitive_index = std::get<int>(parsed_tokens[3]);

  ComponentHandle<GltfRenderer> gltf_renderer =
      gltf_model->GetComponent<GltfRenderer>();
  if (!gltf_renderer) {
    return absl::InternalError("No GltfRenderer found on the glTF model node.");
  }
  const RobinSet<NodeHandle>* node_handles =
      gltf_renderer->GetNodesFromOriginalMeshIndex(mesh_index);
  if (!node_handles) {
    return absl::InternalError(
        "This mesh does not exist or is not used in the scene.");
  }

  // Use the first node that uses this mesh to get the original material index.
  std::optional<model::EntityId> entity_id =
      gltf_renderer->GetEntityIdFromNodeHandle(*node_handles->begin());
  if (!entity_id.has_value()) {
    return absl::InternalError("No entity id found for the given node handle.");
  }
  return gltf_renderer->GetOriginalMaterialIndex(entity_id.value(),
                                                 primitive_index);
}

absl::Status MeshesPrimitivesMaterialPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PointerValue value) const {
  return absl::FailedPreconditionError(
      "Setting primitives material values is not supported.");
}

}  // namespace imp::gltf
