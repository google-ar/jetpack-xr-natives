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

#include "core/view/framework/assets/gltf_scene.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "absl/types/variant.h"
#include "core/common/filament_helpers.h"
#include "core/common/robin_map.h"
#include "core/common/typed_set_vector.h"
#include "core/geometry/shapes/box.h"
#include "core/math/mat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/model/model_data.h"
#include "core/model/skeleton_data.h"
#include "core/ncsb/base_node.h"
#include "core/ncsb/node.h"

namespace imp {

// The name of the node created underneath this node to contain the hierarchy.
static constexpr absl::string_view kModelRootName = "model_root";

void GltfScene::Setup(AssetPtr<GltfAsset> gltf_asset) {
  gltf_asset_ = gltf_asset;

  root_ = FindChildWithName(GetNode(), kModelRootName);
  if (!root_) {
    root_ = GetView().CreateNode();
    root_->SetName(kModelRootName);
    root_->SetParent(GetNode());
  }

  const model::SkeletonData& skeleton = gltf_asset_->GetModelData().Skeleton();
  // Populates runtime_bones_ with initial data in the bind pose.
  // First, searches for a preexisting node in the hierarchy to use as the
  // runtime bone. If one doesn't exist, then the bone is represented
  // "virtually" as a matrix. Later, "virtual bones" can be turned into full
  // nodes if GetOrCreateNode, GetOrCreateNodeFromBone, or CreateAllNodes is
  // called.
  for (BoneId bone_id : skeleton.bones.Ids<BoneId>()) {
    BoneParentId parent_id = skeleton.bones[bone_id].parent;
    bone_id_lookup_.insert({skeleton.bones[bone_id].node_index, bone_id});

    // Try to find this bone's preexisting parent node if one exists.
    NodeHandle existing_parent_node;
    if (parent_id) {
      existing_parent_node = GetNodeFromBone(parent_id);
    } else {
      // If the bone has no parent, just use the root as the preexisting
      // parent.
      existing_parent_node = root_;
    }

    // Try to find the preexisting node that is a child of the preexisting
    // parent if it exists.
    NodeHandle existing_node;
    if (existing_parent_node) {
      const std::string& name = skeleton.bones[bone_id].name;
      existing_node = FindChildWithName(existing_parent_node, name);
    }

    PreciseTransform local_transform = skeleton.bones[bone_id].local_transform;
    // If there is a preexisting node, use that as the runtime bone. Otherwise,
    // use the bind pose matrix as a "virtual bone".
    if (existing_node) {
      if (GetView().IsPreciseTranslationEnabled()) {
        existing_node->SetLocalTransformPrecise(
            Transform<double>(local_transform));
      } else {
        existing_node->SetLocalTransform(Transform<float>(local_transform));
      }
      runtime_bones_.push_back(existing_node);
    } else {
      runtime_bones_.push_back(
          VirtualBone{local_transform, mat4f(local_transform.AsMat4())});
    }
  }
}

void GltfScene::Cleanup() { GetView().DestroyNode(root_); }

NodeHandle GltfScene::GetRoot() const { return root_; }

NodeHandle GltfScene::GetOrCreateNode(absl::string_view name) {
  absl::optional<BoneId> opt_id = GetBoneFromName(name);
  if (opt_id) {
    return GetOrCreateNodeFromBone(opt_id.value());
  }

  return NodeHandle();
}

NodeHandle GltfScene::GetNodeFromBone(BoneId bone_id) const {
  if (!runtime_bones_.IsValid(bone_id)) {
    return NodeHandle();
  }

  auto& runtime_bone_variant = runtime_bones_[bone_id];
  if (absl::holds_alternative<NodeHandle>(runtime_bone_variant)) {
    return absl::get<NodeHandle>(runtime_bone_variant);
  }

  return NodeHandle();
}

Transform<float> GltfScene::GetLocalTransformFromBone(BoneId bone_id) const {
  if (!runtime_bones_.IsValid(bone_id)) {
    return {};
  };

  auto& runtime_bone_variant = runtime_bones_[bone_id];
  if (absl::holds_alternative<NodeHandle>(runtime_bone_variant)) {
    return absl::get<NodeHandle>(runtime_bone_variant)->GetLocalTransform();
  } else {
    return Transform<float>(
        absl::get<VirtualBone>(runtime_bone_variant).local_transform);
  }
}

const mat4f& GltfScene::GetLocalTransformMatFromBone(BoneId bone_id) const {
  if (!runtime_bones_.IsValid(bone_id)) {
    return kIdentityMat4f;
  };

  auto& runtime_bone_variant = runtime_bones_[bone_id];
  if (absl::holds_alternative<NodeHandle>(runtime_bone_variant)) {
    return absl::get<NodeHandle>(runtime_bone_variant)->GetLocalTrs();
  } else {
    return absl::get<VirtualBone>(runtime_bone_variant).local_transform_mat;
  }
}

void GltfScene::SetLocalTransformFromBone(
    BoneId bone_id, const Transform<float>& local_transform) {
  if (!runtime_bones_.IsValid(bone_id)) {
    return;
  }

  auto& runtime_bone_variant = runtime_bones_[bone_id];
  if (absl::holds_alternative<NodeHandle>(runtime_bone_variant)) {
    NodeHandle node = absl::get<NodeHandle>(runtime_bone_variant);
    node->SetLocalTransform(local_transform);
  } else {
    runtime_bone_variant = VirtualBone{PreciseTransform(local_transform),
                                       local_transform.AsMat4()};
  }

  local_trs_updated_ = true;
}

absl::optional<GltfScene::BoneId> GltfScene::GetBoneFromName(
    absl::string_view name) const {
  const model::SkeletonData& skeleton = gltf_asset_->GetModelData().Skeleton();
  auto itr = skeleton.first_bone_from_hash.find(Hash(name));
  if (itr == skeleton.first_bone_from_hash.end()) {
    return absl::nullopt;
  }

  return itr->second;
}

NodeHandle GltfScene::GetOrCreateNodeFromBone(BoneId bone_id) {
  // Bone Id is invalid.
  if (!runtime_bones_.IsValid(bone_id)) {
    return NodeHandle();
  }

  auto& runtime_bone_variant = runtime_bones_[bone_id];

  // Node already created, return it.
  if (absl::holds_alternative<NodeHandle>(runtime_bone_variant)) {
    return absl::get<NodeHandle>(runtime_bone_variant);
  }

  // By default, the parent of the node is the root.
  NodeHandle parent = root_;

  // If the bone has a parent, get or create the node for the parent bone
  // recursively. A node cannot have a virtual bone as its parent.
  const model::SkeletonData& skeleton = gltf_asset_->GetModelData().Skeleton();
  if (BoneParentId bone_parent_id = skeleton.bones[bone_id].parent) {
    parent = GetOrCreateNodeFromBone(bone_parent_id);
  }

  // Create the actual node and assign its properties.
  const std::string& name = skeleton.bones[bone_id].name;
  NodeHandle node = GetView().CreateNode();
  node->SetName(name);
  node->SetParent(parent);

  // Set its transform depending on if precise translation is enabled.
  auto& virtual_bone = absl::get<VirtualBone>(runtime_bone_variant);
  if (GetView().IsPreciseTranslationEnabled()) {
    node->SetLocalTransformPrecise(
        Transform<double>(virtual_bone.local_transform));
  } else {
    node->SetLocalTransform(Transform<float>(virtual_bone.local_transform));
  }

  // Assign the newly created node back into the variant, replacing the virtual
  // bone.
  runtime_bone_variant = node;

  return node;
}

NodeHandle GltfScene::GetOrCreateNodeFromGltfNodeIndex(
    uint16_t gltf_node_index) {
  auto it = bone_id_lookup_.find(gltf_node_index);
  if (it != bone_id_lookup_.end()) {
    return GetOrCreateNodeFromBone(it->second);
  }
  return NodeHandle();
}

void GltfScene::CreateAllNodes() {
  // Loop through each bone in the scene and create an instantiated node from
  // it if it doesn't already exist.
  const model::SkeletonData& skeleton = gltf_asset_->GetModelData().Skeleton();
  const TypedSetVector<imp::model::BoneData>& bones = skeleton.bones;
  for (auto bone_id : bones.Ids<imp::model::BoneId>()) {
    GetOrCreateNodeFromBone(bone_id);
  }
}

size_t GltfScene::GetNumBones() const { return runtime_bones_.size(); }

Box GltfScene::GetLocalBoneBounds() const {
  Box result = NilBounds();

  const model::SkeletonData& skeleton = gltf_asset_->GetModelData().Skeleton();

  const mat4f& root_trs = root_->GetLocalTrs();

  PairedVector<mat4f, model::BoneData> bone_transforms_in_node_space;
  bone_transforms_in_node_space.resize(skeleton.bones.size());

  // Iterate through each bone and get it's transform in the coordinate space
  // of this node. Bones are are guaranteed to be ordered so that parents come
  // before children.
  for (BoneId bone_id : skeleton.bones.Ids<BoneId>()) {
    const mat4f& local_trs = GetLocalTransformMatFromBone(bone_id);
    if (BoneParentId bone_parent_id = skeleton.bones[bone_id].parent) {
      bone_transforms_in_node_space[bone_id] =
          root_trs * bone_transforms_in_node_space[bone_parent_id] * local_trs;
    } else {
      bone_transforms_in_node_space[bone_id] = root_trs * local_trs;
    }
  }

  // Make the box including the translation from each transform.
  for (const mat4f& bone_node_space_transform : bone_transforms_in_node_space) {
    result.unionSelf(Box{.center = bone_node_space_transform[3].xyz});
  }

  return result;
}

Box GltfScene::GetWorldBoneBounds() const {
  return TransformBounds(GetLocalBoneBounds(), GetNode()->GetWorldTrs());
}

std::optional<uint16_t> GltfScene::GetGltfNodeIndexFromBoneId(
    BoneId bone_id) const {
  const model::SkeletonData& skeleton = gltf_asset_->GetModelData().Skeleton();

  if (!skeleton.bones.IsValid(bone_id)) {
    return std::nullopt;
  }

  return skeleton.bones[bone_id].node_index;
}

std::optional<model::BoneId> GltfScene::GetBoneIdFromGltfNodeIndex(
    uint16_t gltf_node_index) const {
  auto it = bone_id_lookup_.find(gltf_node_index);
  if (it != bone_id_lookup_.end()) {
    return it->second;
  }
  return std::nullopt;
}

bool GltfScene::GetLocalTrsUpdated() {
  bool result = local_trs_updated_;
  local_trs_updated_ = false;
  return result;
}

NodeHandle GltfScene::FindChildWithName(NodeHandle parent,
                                        absl::string_view name) {
  NodeHandle result;
  for (NodeHandle child : parent->GetChildren()) {
    if (child->GetName() == name) {
      if (!result) {
        result = child;
      } else {
        IMP_LOG(imp::FATAL)
            << "Attempt to pre-create model hierarchy in ISF with non-unique "
               "child names is unsupported: "
            << name;
      }
    }
  }
  return result;
}

}  // namespace imp
