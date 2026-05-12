// Copyright 2026 Google LLC
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

#include "apibindings/node_manager.h"

#include <cstdint>
#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "apibindings/impress_api_view.h"
#include "apibindings/model_manager.h"
#include "core/math/transform.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/assets/gltf_scene.h"

namespace imp {

namespace {

class NodeManagerImpl : public NodeManager {
 public:
  explicit NodeManagerImpl(ImpressApiView& view);
  ~NodeManagerImpl() override = default;

  int32_t CreateImpressNode() override;
  absl::Status DestroyImpressNode(int32_t node) override;
  absl::Status SetImpressNodeParent(int32_t child, int32_t parent) override;
  absl::StatusOr<int32_t> GetImpressNodeParent(int32_t node_id) override;
  absl::StatusOr<int32_t> GetImpressNodeChildCount(int32_t node_id) override;
  absl::StatusOr<int32_t> GetImpressNodeChildAt(int32_t node_id,
                                                int32_t index) override;
  absl::StatusOr<absl::string_view> GetImpressNodeName(
      int32_t node_id) override;
  absl::StatusOr<imp::Transform<float>> GetImpressNodeLocalTransform(
      int32_t node_id) override;
  absl::Status SetImpressNodeLocalTransform(
      int32_t node_id, const imp::Transform<float>& transform) override;

 private:
  ImpressApiView& view_;
};

}  // namespace

NodeManagerImpl::NodeManagerImpl(ImpressApiView& view) : view_(view) {}

int32_t NodeManagerImpl::CreateImpressNode() {
  return view_.CreateNode().GetEntity().getId();
}

absl::Status NodeManagerImpl::DestroyImpressNode(int32_t node) {
  // If the node is animating, be sure to remove it from the Animation map.
  // Otherwise we hit an assert on the next update.
  auto unused = view_.GetModelManager().StopGltfModelAnimation(node);

  NodeHandle node_handle(utils::Entity::import(node));
  if (node_handle) {
    view_.DestroyNode(node_handle);
    return absl::OkStatus();
  }
  return absl::InvalidArgumentError("Node is not valid.");
}

absl::Status NodeManagerImpl::SetImpressNodeParent(int32_t child,
                                                   int32_t parent) {
  NodeHandle child_handle(utils::Entity::import(child));
  if (!child_handle) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Child node is not valid: %d.", child));
  }
  NodeHandle parent_handle(utils::Entity::import(parent));
  if (!parent_handle) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Parent node is not valid: %d.", parent));
  }
  child_handle->SetParent(parent_handle);
  return absl::OkStatus();
}

absl::StatusOr<int32_t> NodeManagerImpl::GetImpressNodeParent(int32_t node_id) {
  NodeHandle node_handle(utils::Entity::import(node_id));
  if (!node_handle) return absl::InvalidArgumentError("Invalid node");
  auto result = node_handle->GetParent()->GetEntity().getId();
  return result;
}

absl::StatusOr<int32_t> NodeManagerImpl::GetImpressNodeChildCount(
    int32_t node_id) {
  NodeHandle node_handle(utils::Entity::import(node_id));
  if (!node_handle) return absl::InvalidArgumentError("Invalid node");
  ComponentHandle<GltfScene> gltf_scene =
      node_handle->GetComponent<GltfScene>();
  if (!gltf_scene)
    return absl::FailedPreconditionError(
        "Node does not have a GltfScene component.");
  return gltf_scene->GetNumBones();
}

absl::StatusOr<int32_t> NodeManagerImpl::GetImpressNodeChildAt(int32_t node_id,
                                                               int32_t index) {
  NodeHandle node_handle(utils::Entity::import(node_id));
  if (!node_handle) return absl::InvalidArgumentError("Invalid node");
  ComponentHandle<GltfScene> gltf_scene =
      node_handle->GetComponent<GltfScene>();
  if (!gltf_scene)
    return absl::FailedPreconditionError(
        "Node does not have a GltfScene component.");
  utils::Entity entity =
      gltf_scene->GetOrCreateNodeFromGltfNodeIndex(index)->GetEntity();
  if (!entity) return absl::InvalidArgumentError("Invalid node index.");
  return entity.getId();
}

absl::StatusOr<absl::string_view> NodeManagerImpl::GetImpressNodeName(
    int32_t node_id) {
  NodeHandle node_handle(utils::Entity::import(node_id));
  if (!node_handle) return absl::InvalidArgumentError("Invalid node");
  auto result = node_handle->GetName();
  return result;
}

absl::StatusOr<imp::Transform<float>>
NodeManagerImpl::GetImpressNodeLocalTransform(int32_t node_id) {
  NodeHandle node_handle(utils::Entity::import(node_id));
  if (!node_handle) return absl::InvalidArgumentError("Invalid node");
  return node_handle->GetLocalTransform();
}

absl::Status NodeManagerImpl::SetImpressNodeLocalTransform(
    int32_t node_id, const imp::Transform<float>& transform) {
  NodeHandle node_handle(utils::Entity::import(node_id));
  if (!node_handle) return absl::InvalidArgumentError("Invalid node");
  node_handle->SetLocalTransform(transform);
  return absl::OkStatus();
}

std::unique_ptr<NodeManager> CreateNodeManager(ImpressApiView& view) {
  return std::make_unique<NodeManagerImpl>(view);
}

}  // namespace imp
