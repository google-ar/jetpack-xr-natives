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

#include "core/assets/gltf/object_model/pointer_declarations/skins.h"

#include <cstdint>
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
#include "core/common/typed_set_vector.h"
#include "core/common/typed_vector.h"
#include "core/model/entity_data.h"
#include "core/model/joint_data.h"
#include "core/model/shared_data.h"
#include "core/model/skin_data.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/gltf_scene.h"

namespace imp::gltf {

namespace {
using PointerDeclaration = PropertyPointer::PointerDeclaration;

inline constexpr absl::string_view kJointsLengthToken = "joints.length";
inline constexpr absl::string_view kJointsToken = "joints";
inline constexpr absl::string_view kSkinsLengthToken = "skins.length";
inline constexpr absl::string_view kSkinsToken = "skins";
inline constexpr absl::string_view kSkeletonToken = "skeleton";

absl::StatusOr<model::SkinId> GetSkinId(
    ComponentHandle<GltfRenderer> gltf_renderer, NodeHandle gltf_model,
    absl::Span<const ParsedToken> parsed_tokens) {
  if (!std::holds_alternative<int>(parsed_tokens[1])) {
    return absl::InvalidArgumentError("Expected parsed token to be type int");
  }

  model::SkinId skin_id{static_cast<int16_t>(std::get<int>(parsed_tokens[1]))};

  if (!gltf_renderer->GetGltfAsset()->GetModelData().Skins().IsValid(skin_id)) {
    return absl::NotFoundError("Could not find skin_id in glTF Model Data");
  }

  return skin_id;
}

}  // namespace

std::vector<TokenParser> SkinsJointsLengthPointerDeclaration::GetTokenParsers()
    const {
  return {std::string(kSkinsToken), GetIntTokenParser(),
          std::string(kJointsLengthToken)};
};

absl::StatusOr<PropertyPointer::PointerValue>
SkinsJointsLengthPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  ComponentHandle<GltfRenderer> gltf_renderer =
      gltf_model->GetComponent<GltfRenderer>();
  if (!gltf_renderer.IsValid()) {
    return absl::InvalidArgumentError("Could not find GltfRenderer Component");
  }

  absl::StatusOr<model::SkinId> skin_id =
      GetSkinId(gltf_renderer, gltf_model, parsed_tokens);
  if (!skin_id.ok()) {
    return skin_id.status();
  }

  return static_cast<int>(gltf_renderer->GetGltfAsset()
                              ->GetModelData()
                              .Skins()[*skin_id]
                              .sampled_joints.size());
}

absl::Status SkinsJointsLengthPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PropertyPointer::PointerValue value) const {
  return absl::FailedPreconditionError(
      "\"/skins/joints.length\" is read-only.");
}

std::vector<TokenParser> SkinsJointNodePointerDeclaration::GetTokenParsers()
    const {
  return {std::string(kSkinsToken), GetIntTokenParser(),
          std::string(kJointsToken), GetIntTokenParser()};
}

absl::StatusOr<PropertyPointer::PointerValue>
SkinsJointNodePointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  ComponentHandle<GltfRenderer> gltf_renderer =
      gltf_model->GetComponent<GltfRenderer>();
  if (!gltf_renderer.IsValid()) {
    return absl::InvalidArgumentError("Could not find GltfRenderer Component");
  }

  ComponentHandle<GltfScene> gltf_scene = gltf_model->GetComponent<GltfScene>();
  if (!gltf_scene.IsValid()) {
    return absl::InvalidArgumentError("Could not find GltfScene Component");
  }

  absl::StatusOr<model::SkinId> skin_id =
      GetSkinId(gltf_renderer, gltf_model, parsed_tokens);
  if (!skin_id.ok()) {
    return skin_id.status();
  }

  if (!std::holds_alternative<int>(parsed_tokens[3])) {
    return absl::InvalidArgumentError("Expected parsed token to be type int");
  }

  const TypedVector<model::SkinData>& skins =
      gltf_renderer->GetGltfAsset()->GetModelData().Skins();

  model::SampledJointId sampled_joint_id{
      static_cast<uint8_t>(std::get<int>(parsed_tokens[3]))};
  if (!skins[*skin_id].sampled_joints.IsValid(sampled_joint_id)) {
    return absl::NotFoundError(
        "Could not find joint in list of joints for skin");
  }

  model::JointId joint_id =
      skins[*skin_id].sampled_joints[sampled_joint_id].joint;
  if (!skins[*skin_id].joints.IsValid(joint_id)) {
    return absl::NotFoundError(
        "Could not find joint in list of joints for skin");
  }

  std::optional<uint16_t> gltf_node_index =
      gltf_scene->GetGltfNodeIndexFromBoneId(
          skins[*skin_id].joints[joint_id].source);
  if (!gltf_node_index.has_value()) {
    return absl::NotFoundError("Could not find bone in skeleton");
  }

  return *gltf_node_index;
}

absl::Status SkinsJointNodePointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PropertyPointer::PointerValue value) const {
  return absl::FailedPreconditionError("\"/skins/{}/joints/{}\" is read-only.");
}

std::vector<TokenParser> SkinsLengthPointerDeclaration::GetTokenParsers()
    const {
  return {std::string(kSkinsLengthToken)};
}

absl::StatusOr<PropertyPointer::PointerValue>
SkinsLengthPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  ComponentHandle<GltfRenderer> gltf_renderer =
      gltf_model->GetComponent<GltfRenderer>();
  if (!gltf_renderer.IsValid()) {
    return absl::InvalidArgumentError("Could not find GltfRenderer Component");
  }

  return static_cast<int>(
      gltf_renderer->GetGltfAsset()->GetModelData().Skins().size());
}

absl::Status SkinsLengthPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PropertyPointer::PointerValue value) const {
  return absl::FailedPreconditionError("\"/skins.length\" is read-only.");
}

std::vector<TokenParser> SkinsSkeletonPointerDeclaration::GetTokenParsers()
    const {
  return {std::string(kSkinsToken), GetIntTokenParser(),
          std::string(kSkeletonToken)};
}

absl::StatusOr<PropertyPointer::PointerValue>
SkinsSkeletonPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  ComponentHandle<GltfRenderer> gltf_renderer =
      gltf_model->GetComponent<GltfRenderer>();
  if (!gltf_renderer.IsValid()) {
    return absl::InvalidArgumentError("Could not find GltfRenderer Component");
  }

  absl::StatusOr<model::SkinId> skin_id =
      GetSkinId(gltf_renderer, gltf_model, parsed_tokens);
  if (!skin_id.ok()) {
    return skin_id.status();
  }

  model::WeakEntityId pose_root_id =
      gltf_renderer->GetGltfAsset()->GetModelData().Skins()[*skin_id].pose_root;

  const TypedSetVector<model::EntityData>& entities =
      gltf_renderer->GetGltfAsset()->GetModelData().Entities();
  if (!entities.IsValid(pose_root_id)) {
    return absl::NotFoundError(
        "Could not find skeleton root node in Model Data");
  }

  return entities[pose_root_id].original_index;
}

absl::Status SkinsSkeletonPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PropertyPointer::PointerValue value) const {
  return absl::FailedPreconditionError("\"/skins/{}/skeleton\" is read-only.");
}

}  // namespace imp::gltf
