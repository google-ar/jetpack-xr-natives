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

#include "core/scene_handles/scene_handles.h"

#include <string>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/variant.h"
#include "core/ncsb/node_handle.h"
#include "core/scene_handles/scene_handle_helper.h"
#include "core/scene_handles/scene_handle_interface.h"
#include "core/view/framework/assets/gltf_scene.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

NodeSceneHandle::NodeSceneHandle() {}

NodeSceneHandle::NodeSceneHandle(SceneHandleInterface::Identifier identifier)
    : scene_handle_helper_(identifier) {}

NodeSceneHandle::NodeSceneHandle(SceneHandleInterface::Identifier identifier,
                                 NodeHandle node)
    : NodeHandle(node), scene_handle_helper_(identifier) {}

absl::Status NodeSceneHandle::AssignSceneHandleForIdentifier(
    NodeHandle identified_node, NodeHandle attached_node) {
  if (absl::holds_alternative<absl::monostate>(
          scene_handle_helper_.GetIdentifier())) {
    // No identifier assigned.
    return absl::OkStatus();
  }

  MP_RETURN_IF_ERROR(scene_handle_helper_.RequireIdentifiedNode(identified_node));
  *this =
      NodeSceneHandle(scene_handle_helper_.GetIdentifier(), identified_node);
  return absl::OkStatus();
}

const SceneHandleInterface::Identifier& NodeSceneHandle::GetIdentifier() {
  return ResolveIdentifier();
}

SceneHandleInterface::Identifier& NodeSceneHandle::ResolveIdentifier() {
  scene_handle_helper_.UpdateIdentifier(GetSceneNode());
  return scene_handle_helper_.GetIdentifier();
}

std::string NodeSceneHandle::ToSceneHandleString() const {
  return scene_handle_helper_.GetIdentifierString();
}

std::string NodeSceneHandle::GetTypeName() const { return "Node"; }

NodeHandle NodeSceneHandle::GetSceneNode() const { return *this; }

void NodeSceneHandle::AssignSceneHandleForNode(NodeHandle node) {
  if (!node) {
    *this = NodeSceneHandle();
    return;
  }

  *this = NodeSceneHandle(
      SceneHandleInterface::Identifier(std::string(node->GetName())), node);
}

bool NodeSceneHandle::CanAssignSceneHandleForNode(NodeHandle node) const {
  if (!node) {
    return false;
  }

  if (node->GetName().empty()) {
    return false;
  }

  return true;
}

GltfNodeSceneHandle::GltfNodeSceneHandle() {}

GltfNodeSceneHandle::GltfNodeSceneHandle(absl::string_view gltf_node_name)
    : gltf_node_name_(gltf_node_name) {}

GltfNodeSceneHandle::GltfNodeSceneHandle(
    SceneHandleInterface::Identifier identifier,
    absl::string_view gltf_node_name)
    : scene_handle_helper_(identifier), gltf_node_name_(gltf_node_name) {}

GltfNodeSceneHandle::GltfNodeSceneHandle(
    SceneHandleInterface::Identifier identifier,
    absl::string_view gltf_node_name, NodeHandle node)
    : NodeHandle(node),
      scene_handle_helper_(identifier),
      gltf_node_name_(gltf_node_name) {}

std::string GltfNodeSceneHandle::GetGltfNodeName() const {
  return gltf_node_name_;
}

absl::Status GltfNodeSceneHandle::AssignSceneHandleForIdentifier(
    NodeHandle identified_node, NodeHandle attached_node) {
  NodeHandle target_node = attached_node;
  bool has_identifier = !absl::holds_alternative<absl::monostate>(
      scene_handle_helper_.GetIdentifier());
  bool has_target_gltf_node_name = !gltf_node_name_.empty();

  // If we have neither an identifier nor a target name, then this
  // GltfNodeSceneHandle is simply unassigned.
  if (!has_identifier && !has_target_gltf_node_name) {
    return absl::OkStatus();
  }

  if (!absl::holds_alternative<absl::monostate>(
          scene_handle_helper_.GetIdentifier())) {
    MP_RETURN_IF_ERROR(
        scene_handle_helper_.RequireIdentifiedNode(identified_node));
    target_node = identified_node;
  }

  if (!has_target_gltf_node_name) {
    return absl::UnavailableError("GltfNodeSceneHandle has no gltf_node_name.");
  }

  auto gltf_scene = target_node->GetComponent<GltfScene>();
  if (!gltf_scene) {
    return absl::UnavailableError(
        absl::StrFormat("Can't assign GltfNodeSceneHandle, target node %s does "
                        "not contain a glTF scene.",
                        ToString(target_node)));
  }

  NodeHandle gltf_node = gltf_scene->GetOrCreateNode(gltf_node_name_);
  if (!gltf_node) {
    return absl::UnavailableError(absl::StrFormat(
        "GltfNodeSceneHandle cannot find gltf node named %s", gltf_node_name_));
  }

  *this = GltfNodeSceneHandle(scene_handle_helper_.GetIdentifier(),
                              gltf_node_name_, gltf_node);
  return absl::OkStatus();
}

const SceneHandleInterface::Identifier& GltfNodeSceneHandle::GetIdentifier() {
  return scene_handle_helper_.GetIdentifier();
}
std::string GltfNodeSceneHandle::ToSceneHandleString() const {
  return scene_handle_helper_.GetIdentifierString();
}

std::string GltfNodeSceneHandle::GetTypeName() const { return "glTF Node"; }

NodeHandle GltfNodeSceneHandle::GetSceneNode() const { return *this; }

void GltfNodeSceneHandle::AssignSceneHandleForNode(NodeHandle node) {
  // TODO: Add support for GltfNodeSceneHandle in the editor.
  // This is trickier than the other scene handles because if the identifier is
  // unset then the node the component is attached to is used to find the
  // GltfScene, and then there is an additional field for picking which glTF
  // node within the scene to map to. This means that the node isn't enough
  // information by itself to determine the GltfNodeSceneHandle.
  IMP_LOG(imp::FATAL)
      << "GltfNodeSceneHandle::AssignSceneHandleForNode is not implemented.";
}

bool GltfNodeSceneHandle::CanAssignSceneHandleForNode(NodeHandle node) const {
  // TODO: Add support for GltfNodeSceneHandle in the editor.
  // This is trickier than the other scene handles because if the identifier is
  // unset then the node the component is attached to is used to find the
  // GltfScene, and then there is an additional field for picking which glTF
  // node within the scene to map to. This means that the node isn't enough
  // information by itself to determine the GltfNodeSceneHandle.
  return false;
}

}  // namespace imp
