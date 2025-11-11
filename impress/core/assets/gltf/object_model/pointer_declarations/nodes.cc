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

#include "core/assets/gltf/object_model/pointer_declarations/nodes.h"

#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "core/assets/gltf/object_model/property_pointer.h"
#include "core/assets/gltf/object_model/token_parser.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/model/skeleton_data.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/path_manager.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/gltf_scene.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::gltf {

using PointerValue = PropertyPointer::PointerValue;

namespace {
using PointerDeclaration = PropertyPointer::PointerDeclaration;

constexpr absl::string_view kNodesToken = "nodes";
constexpr absl::string_view kTranslationToken = "translation";
constexpr absl::string_view kRotationToken = "rotation";
constexpr absl::string_view kScaleToken = "scale";
constexpr absl::string_view kMatrixToken = "matrix";
constexpr absl::string_view kGlobalMatrixToken = "globalMatrix";
constexpr absl::string_view kNodesLengthToken = "nodes.length";
constexpr absl::string_view kChildrenLengthToken = "children.length";
constexpr absl::string_view kChildrenToken = "children";
constexpr absl::string_view kParentToken = "parent";
constexpr absl::string_view kMeshToken = "mesh";
constexpr absl::string_view kWeightsLengthToken = "weights.length";
constexpr absl::string_view kWeightsToken = "weights";

struct NodeHandles {
  NodeHandle gltf_root;
  NodeHandle node;
};

absl::StatusOr<NodeHandles> GetNodeHandles(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) {
  if (!std::holds_alternative<int>(parsed_tokens[1])) {
    return absl::FailedPreconditionError(
        "The second parsed component is expected to be an int.");
  }

  int node_index = std::get<int>(parsed_tokens[1]);
  ComponentHandle<GltfScene> gltf_scene = gltf_model->GetComponent<GltfScene>();
  NodeHandle node = gltf_scene->GetOrCreateNodeFromGltfNodeIndex(node_index);
  if (!node) {
    return absl::InternalError(absl::StrFormat(
        "Node index %d not found in GltfScene of glTF root node %s.",
        node_index, gltf_model->GetName()));
  }
  return NodeHandles{.gltf_root = gltf_scene->GetRoot(), .node = node};
};

}  // namespace

std::vector<TokenParser> NodesTranslationPointerDeclaration::GetTokenParsers()
    const {
  return {std::string(kNodesToken), GetIntTokenParser(),
          std::string(kTranslationToken)};
}

absl::StatusOr<PointerValue> NodesTranslationPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  MP_ASSIGN_OR_RETURN(NodeHandles node_handles,
                   GetNodeHandles(gltf_model, parsed_tokens));
  return node_handles.node->GetLocalPosition();
}

absl::Status NodesTranslationPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PointerValue value) const {
  MP_ASSIGN_OR_RETURN(NodeHandles node_handles,
                   GetNodeHandles(gltf_model, parsed_tokens));
  if (!std::holds_alternative<float3>(value)) {
    return absl::FailedPreconditionError("A float3 value is expected.");
  }
  node_handles.node->SetLocalPosition(std::get<float3>(value));
  return absl::OkStatus();
}

std::vector<TokenParser> NodesRotationPointerDeclaration::GetTokenParsers()
    const {
  return {std::string(kNodesToken), GetIntTokenParser(),
          std::string(kRotationToken)};
}

absl::StatusOr<PointerValue> NodesRotationPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  MP_ASSIGN_OR_RETURN(NodeHandles node_handles,
                   GetNodeHandles(gltf_model, parsed_tokens));
  return node_handles.node->GetLocalRotation().xyzw;
}

absl::Status NodesRotationPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PointerValue value) const {
  MP_ASSIGN_OR_RETURN(NodeHandles node_handles,
                   GetNodeHandles(gltf_model, parsed_tokens));
  if (!std::holds_alternative<float4>(value)) {
    return absl::FailedPreconditionError("A float4 value is expected.");
  }
  quatf rotation;
  rotation.xyzw = std::get<float4>(value);
  node_handles.node->SetLocalRotation(rotation);
  return absl::OkStatus();
}

std::vector<TokenParser> NodesScalePointerDeclaration::GetTokenParsers() const {
  return {std::string(kNodesToken), GetIntTokenParser(),
          std::string(kScaleToken)};
}

absl::StatusOr<PointerValue> NodesScalePointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  MP_ASSIGN_OR_RETURN(NodeHandles node_handles,
                   GetNodeHandles(gltf_model, parsed_tokens));
  return node_handles.node->GetLocalScale();
}

absl::Status NodesScalePointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PointerValue value) const {
  MP_ASSIGN_OR_RETURN(NodeHandles node_handles,
                   GetNodeHandles(gltf_model, parsed_tokens));
  if (!std::holds_alternative<float3>(value)) {
    return absl::FailedPreconditionError("A float3 value is expected.");
  }
  node_handles.node->SetLocalScale(std::get<float3>(value));
  return absl::OkStatus();
}

std::vector<TokenParser> NodesMatrixPointerDeclaration::GetTokenParsers()
    const {
  return {std::string(kNodesToken), GetIntTokenParser(),
          std::string(kMatrixToken)};
}

absl::StatusOr<PointerValue> NodesMatrixPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  MP_ASSIGN_OR_RETURN(NodeHandles node_handles,
                   GetNodeHandles(gltf_model, parsed_tokens));
  return node_handles.node->GetLocalTransform().AsMat4();
}

absl::Status NodesMatrixPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PointerValue value) const {
  return absl::FailedPreconditionError(
      "Setting matrix values is not supported.");
}

std::vector<TokenParser> NodesGlobalMatrixPointerDeclaration::GetTokenParsers()
    const {
  return {std::string(kNodesToken), GetIntTokenParser(),
          std::string(kGlobalMatrixToken)};
}

absl::StatusOr<PointerValue> NodesGlobalMatrixPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  MP_ASSIGN_OR_RETURN(NodeHandles node_handles,
                   GetNodeHandles(gltf_model, parsed_tokens));
  return node_handles.node->GetView().GetPathManager().GetRelativeTransform(
      node_handles.gltf_root, node_handles.node);
}

absl::Status NodesGlobalMatrixPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PointerValue value) const {
  return absl::FailedPreconditionError(
      "Setting global matrix values is not supported.");
}

std::vector<TokenParser> NodesLengthPointerDeclaration::GetTokenParsers()
    const {
  return {std::string(kNodesLengthToken)};
}

absl::StatusOr<PointerValue> NodesLengthPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  ComponentHandle<GltfScene> gltf_scene = gltf_model->GetComponent<GltfScene>();
  if (!gltf_scene) {
    return absl::InternalError("No GltfScene found on the glTF model node.");
  }

  return static_cast<int>(gltf_scene->GetNumBones());
}

absl::Status NodesLengthPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PointerValue value) const {
  return absl::FailedPreconditionError(
      "Setting nodes length values is not supported.");
}

std::vector<TokenParser>
NodesChildrenLengthPointerDeclaration::GetTokenParsers() const {
  return {std::string(kNodesToken), GetIntTokenParser(),
          std::string(kChildrenLengthToken)};
}

absl::StatusOr<PointerValue> NodesChildrenLengthPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  MP_ASSIGN_OR_RETURN(NodeHandles node_handles,
                   GetNodeHandles(gltf_model, parsed_tokens));
  return static_cast<int>(node_handles.node->GetChildren().size());
}

absl::Status NodesChildrenLengthPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PointerValue value) const {
  return absl::FailedPreconditionError(
      "Setting children length values is not supported.");
}

std::vector<TokenParser> NodesChildPointerDeclaration::GetTokenParsers() const {
  return {std::string(kNodesToken), GetIntTokenParser(),
          std::string(kChildrenToken), GetIntTokenParser()};
}

absl::StatusOr<PointerValue> NodesChildPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  if (!std::holds_alternative<int>(parsed_tokens[1])) {
    return absl::FailedPreconditionError(
        "The second parsed component is expected to be an int.");
  }
  int node_index = std::get<int>(parsed_tokens[1]);

  if (!std::holds_alternative<int>(parsed_tokens[3])) {
    return absl::FailedPreconditionError(
        "The fourth parsed component is expected to be an int.");
  }
  int child_index = std::get<int>(parsed_tokens[3]);

  ComponentHandle<GltfScene> gltf_scene = gltf_model->GetComponent<GltfScene>();
  if (!gltf_scene) {
    return absl::InternalError("No GltfScene found on the glTF model node.");
  }

  ComponentHandle<GltfRenderer> gltf_renderer =
      gltf_model->GetComponent<GltfRenderer>();
  if (!gltf_renderer) {
    return absl::InternalError("No GltfRenderer found on the glTF model node.");
  }

  const model::SkeletonData& skeleton =
      gltf_renderer->GetGltfAsset()->GetModelData().Skeleton();

  std::optional<model::BoneId> bone_id =
      gltf_scene->GetBoneIdFromGltfNodeIndex(node_index);
  if (!bone_id.has_value()) {
    return absl::InternalError("No bone id found.");
  }

  if (child_index >= skeleton.bones[bone_id.value()].num_children) {
    return absl::InternalError("Child index is out of bounds.");
  }

  model::BoneChildId first_child_id =
      skeleton.bones[bone_id.value()].first_child;
  model::BoneChildId child_id = first_child_id;
  for (int i = 0; i < child_index; ++i) {
    child_id = skeleton.bones[child_id].next_sibling;
  }
  return skeleton.bones[child_id].node_index;
}

absl::Status NodesChildPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PointerValue value) const {
  return absl::FailedPreconditionError(
      "Setting child values is not supported.");
}

std::vector<TokenParser> NodesParentPointerDeclaration::GetTokenParsers()
    const {
  return {std::string(kNodesToken), GetIntTokenParser(),
          std::string(kParentToken)};
}

absl::StatusOr<PointerValue> NodesParentPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  ComponentHandle<GltfScene> gltf_scene = gltf_model->GetComponent<GltfScene>();
  if (!gltf_scene) {
    return absl::InternalError("No GltfScene found on the glTF model node.");
  }

  ComponentHandle<GltfRenderer> gltf_renderer =
      gltf_model->GetComponent<GltfRenderer>();
  if (!gltf_renderer) {
    return absl::InternalError("No GltfRenderer found on the glTF model node.");
  }

  const model::SkeletonData& skeleton =
      gltf_renderer->GetGltfAsset()->GetModelData().Skeleton();

  int node_index = std::get<int>(parsed_tokens[1]);
  std::optional<model::BoneId> bone_id =
      gltf_scene->GetBoneIdFromGltfNodeIndex(node_index);
  if (!bone_id.has_value()) {
    return absl::InternalError("No bone id found.");
  }

  model::BoneParentId parent_id = skeleton.bones[bone_id.value()].parent;
  return skeleton.bones[parent_id].node_index;
}

absl::Status NodesParentPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PointerValue value) const {
  return absl::FailedPreconditionError(
      "Setting parent values is not supported.");
}

std::vector<TokenParser> NodesMeshPointerDeclaration::GetTokenParsers() const {
  return {std::string(kNodesToken), GetIntTokenParser(),
          std::string(kMeshToken)};
}

absl::StatusOr<PointerValue> NodesMeshPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  MP_ASSIGN_OR_RETURN(NodeHandles node_handles,
                   GetNodeHandles(gltf_model, parsed_tokens));
  ComponentHandle<GltfMesh> gltf_mesh =
      node_handles.node->GetComponent<GltfMesh>();
  if (!gltf_mesh) {
    return absl::InternalError("No GltfMesh found on the node.");
  }
  return static_cast<int>(gltf_mesh->GetOriginalGltfMeshIndex());
}

absl::Status NodesMeshPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PointerValue value) const {
  return absl::FailedPreconditionError("Setting mesh values is not supported.");
}

std::vector<TokenParser> NodesWeightsLengthPointerDeclaration::GetTokenParsers()
    const {
  return {std::string(kNodesToken), GetIntTokenParser(),
          std::string(kWeightsLengthToken)};
}

absl::StatusOr<PointerValue> NodesWeightsLengthPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  MP_ASSIGN_OR_RETURN(NodeHandles node_handles,
                   GetNodeHandles(gltf_model, parsed_tokens));
  ComponentHandle<GltfMesh> gltf_mesh =
      node_handles.node->GetComponent<GltfMesh>();
  if (!gltf_mesh) {
    return absl::InternalError("No GltfMesh found on the node.");
  }
  return static_cast<int>(gltf_mesh->GetMorphTargetCount());
}

absl::Status NodesWeightsLengthPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PointerValue value) const {
  return absl::FailedPreconditionError(
      "Setting weights length values is not supported.");
}

std::vector<TokenParser> NodesWeightsPointerDeclaration::GetTokenParsers()
    const {
  return {std::string(kNodesToken), GetIntTokenParser(),
          std::string(kWeightsToken)};
}

absl::StatusOr<PointerValue> NodesWeightsPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  MP_ASSIGN_OR_RETURN(NodeHandles node_handles,
                   GetNodeHandles(gltf_model, parsed_tokens));
  ComponentHandle<GltfMesh> gltf_mesh =
      node_handles.node->GetComponent<GltfMesh>();
  if (!gltf_mesh) {
    return absl::InternalError("No GltfMesh found on the node.");
  }
  return gltf_mesh->GetMorphTargetWeights();
}

absl::Status NodesWeightsPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PointerValue value) const {
  if (!std::holds_alternative<std::vector<float>>(value)) {
    return absl::FailedPreconditionError(
        "A std::vector<float> value is expected.");
  }
  MP_ASSIGN_OR_RETURN(NodeHandles node_handles,
                   GetNodeHandles(gltf_model, parsed_tokens));
  ComponentHandle<GltfMesh> gltf_mesh =
      node_handles.node->GetComponent<GltfMesh>();
  if (!gltf_mesh) {
    return absl::InternalError("No GltfMesh found on the node.");
  }
  gltf_mesh->SetMorphTargetWeights(std::get<std::vector<float>>(value));
  return absl::OkStatus();
}

std::vector<TokenParser> NodesWeightPointerDeclaration::GetTokenParsers()
    const {
  return {std::string(kNodesToken), GetIntTokenParser(),
          std::string(kWeightsToken), GetIntTokenParser()};
}

absl::StatusOr<PointerValue> NodesWeightPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  MP_ASSIGN_OR_RETURN(NodeHandles node_handles,
                   GetNodeHandles(gltf_model, parsed_tokens));
  ComponentHandle<GltfMesh> gltf_mesh =
      node_handles.node->GetComponent<GltfMesh>();
  if (!gltf_mesh) {
    return absl::InternalError("No GltfMesh found on the node.");
  }
  int index = std::get<int>(parsed_tokens[3]);
  if (index >= gltf_mesh->GetMorphTargetCount()) {
    return absl::InternalError("Index is out of bounds for morph targets.");
  }
  return gltf_mesh->GetMorphTargetWeight(index);
}

absl::Status NodesWeightPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PointerValue value) const {
  if (!std::holds_alternative<float>(value)) {
    return absl::FailedPreconditionError("A float value is expected.");
  }
  MP_ASSIGN_OR_RETURN(NodeHandles node_handles,
                   GetNodeHandles(gltf_model, parsed_tokens));
  ComponentHandle<GltfMesh> gltf_mesh =
      node_handles.node->GetComponent<GltfMesh>();
  if (!gltf_mesh) {
    return absl::InternalError("No GltfMesh found on the node.");
  }
  int index = std::get<int>(parsed_tokens[3]);
  if (index >= gltf_mesh->GetMorphTargetCount()) {
    return absl::InternalError("Index is out of bounds for morph targets.");
  }
  gltf_mesh->SetMorphTargetWeight(index, std::get<float>(value));
  return absl::OkStatus();
}

}  // namespace imp::gltf
