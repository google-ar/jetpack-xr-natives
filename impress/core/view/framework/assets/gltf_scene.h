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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_GLTF_SCENE_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_GLTF_SCENE_H_

#include <cstddef>
#include <cstdint>
#include <optional>

#include "absl/strings/string_view.h"
#include "absl/types/variant.h"
#include "core/assets/asset_ptr.h"
#include "core/common/robin_map.h"
#include "core/geometry/shapes/box.h"
#include "core/math/mat.h"
#include "core/math/transform.h"
#include "core/model/entity_data.h"
#include "core/model/shared_data.h"
#include "core/model/skeleton_data.h"
#include "core/ncsb/component.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/assets/gltf_asset.h"
#include "robin_map/include/tsl/robin_map.h"

namespace imp {

class GltfRenderer;

// Tracks and controls the runtime representation of bones from a GltfAsset.
//
// Each bone corresponds to one of the nodes from the hierarchy of the glTF file
// that the GltfAsset represents. This includes glTF nodes used for skinning,
// meshes, lights, and leaf nodes.
//
// GltfRenderer uses GltfScene to apply skinning, and attach GltfMesh and
// LightComponent to the bones from the GltfAsset.
//
// GltfAnimator uses GltfScene to animate the translation, rotation, and scale
// of the bones from the GltfAsset.
//
// When first created, all bones in the GltfScene are considered 'virtual'. That
// means that they don't exist in the Impress scene graph as NodeHandles, and
// instead GltfScene tracks their translation, rotation, and scale internally.
//
// A 'virtual' bone can be turned into an 'instantiated' bone that exists in the
// Impress scene graph as a NodeHandle at any time by calling GetOrCreateNode or
// GetOrCreateNodeFromBone.
//
// These nodes will be named to match the name of the
// bone in the GltfAsset. They can be used like normal nodes by being
// transformed dynamically (impacting skinning), adding components to them, or
// adding children to them. Note, they should not be re-parented because that
// will break skinning.
//
// Bones are only instantiated in the scene graph as-needed for performance, it
// is more expensive to update skinning for instantiated bones. GltfRenderer
// will automatically instantiate impress nodes for all bones with a mesh or a
// light attached.
class GltfScene : public Component {
 public:
  using BoneId = model::BoneId;
  using BoneParentId = model::BoneParentId;

  // Creates the root Node.
  void Setup(AssetPtr<GltfAsset> gltf_asset);

  // Destroys the root Node and therefore all its children as well.
  void Cleanup();

  // Returns the root Node that holds all created Nodes. This represents
  // "global" space for the model, and is used by GltfRenderer for pivoting.
  //
  // Note: This should *not* be reparented out of the hierarchy under
  // this component, or GltfRenderer's pivoting may break.
  // TODO: Consider getting rid of the 'root' entirely in favor of
  // just treating the node GltfScene is attached to as the root.
  NodeHandle GetRoot() const;

  // Returns the previously created Node with this name if it exists, or creates
  // it and all parents up to the root. If there
  // is no bone with name, it returns an invalid node.
  //
  // Note: These nodes should *not* be reparented, it will cause skinning to
  // break.
  NodeHandle GetOrCreateNode(absl::string_view name);

  // Returns the previously created Node for this bone if it exists. Otherwise,
  // returns an invalid NodeHandle.
  //
  // Note: These nodes should *not* be reparented, it will cause skinning to
  // break.
  NodeHandle GetNodeFromBone(BoneId bone_id) const;

  // Returns the previously created Node for this bone if it exists, or creates
  // it and all parents up to the root. If the bone id is invalid, then this
  // returns an empty NodeHandle.
  //
  // Note: These nodes should *not* be reparented, it will cause skinning to
  // break.
  NodeHandle GetOrCreateNodeFromBone(BoneId bone_id);

  // Returns the previously created Node for the bone that corresponds to the
  // given gltf node index if it exists, or creates it and all parents up to the
  // root. An invalid NodeHandle will be returned if there's no matching node.
  //
  // Note: These nodes should *not* be reparented, it will cause skinning to
  // break.
  NodeHandle GetOrCreateNodeFromGltfNodeIndex(uint16_t gltf_node_index);

  // Converts all 'virtual' bones into 'instantiated' bones that exist in the
  // Impress scene graph as a NodeHandle.
  void CreateAllNodes();

  // Returns the local trs for the bone for this bone if it exists, or
  // kIdentityMat4f otherwise.
  Transform<float> GetLocalTransformFromBone(BoneId bone_id) const;

  const mat4f& GetLocalTransformMatFromBone(BoneId bone_id) const;

  // Returns the BoneId for the node within the gltf asset with the given name.
  // If the gltf has multiple nodes with different names, which one is returned
  // is unspecified.
  absl::optional<BoneId> GetBoneFromName(absl::string_view name) const;

  // Sets the local trs for this bone if it exists.
  void SetLocalTransformFromBone(BoneId bone_id,
                                 const Transform<float>& local_transform);

  // Returns the total number of all bones.
  size_t GetNumBones() const;

  // Gets the bounds of the bones in the Gltf scene in the coordinate space of
  // this node.
  //
  // The bounds includes the positions of all bones, regardless of if they
  // contain a mesh or not. It does not include the extents of any meshes in the
  // gltf. To get the mesh bounds, use GltfRenderer::GetLocalBounds.
  Box GetLocalBoneBounds() const;

  // Gets the bounds of the bones in the Gltf scene in the world coordinate
  // space.
  //
  // The bounds includes the positions of all bones, regardless of if they
  // contain a mesh or not. It does not include the extents of any meshes in the
  // gltf. To get the mesh bounds, use GltfRenderer::GetWorldBounds.
  Box GetWorldBoneBounds() const;

  // Traverses through all created Nodes. Fn should have the signature
  // "void(NodeHandle node)".
  template <typename Fn>
  void ForAllNodes(const Fn& fn) const;

  // Returns true if there exists any cached EntityData from the glTF Model for
  // a given Impress Scene Node.
  bool HasEntityDataForNodeHandle(NodeHandle node) const;

  // Returns the cached EntityData from the glTF Model for a given Impress Scene
  // Node.
  model::EntityData::Proxy GetEntityDataFromNodeHandle(NodeHandle node) const;

  // Returns the corresponding glTF Scene Node Index for a Bone from the
  // skeleton in the glTF Model at a given BoneId; will return std::nullopt if
  // the BoneId passed into this method does not exist in the glTF Model's
  // skeleton.
  std::optional<uint16_t> GetGltfNodeIndexFromBoneId(BoneId bone_id) const;

  // Returns the BoneId of the bone that corresponds to the given gltf node
  // index in the original glTF file; will return std::nullopt if the gltf node
  // index cannot be found in the glTF model's skeleton data.
  std::optional<BoneId> GetBoneIdFromGltfNodeIndex(
      uint16_t gltf_node_index) const;

 private:
  // Represents a runtime bone. Bone is 'virtual' if it's just represented as a
  // mat4f, and 'instantiated' if it's represented as a full NodeHandle.
  // To save memory, we only store a single precision mat4f to be used for
  // skinning (glTF animations only support single precision, per the spec), but
  // we keep the PreciseTransform around in case we need to instantiate the bone
  // into a NodeHandle with double precision.
  struct VirtualBone {
    PreciseTransform local_transform;
    mat4f local_transform_mat;
  };
  using RuntimeBoneVariant = absl::variant<VirtualBone, NodeHandle>;

  // Returns whether local trs has been updated since the last time this was
  // called, then resets it to false. Should only be called by GltfRenderer.
  bool GetLocalTrsUpdated();

  // Finds a direct child of the given parent with the given name in the set of
  // pre-existing nodes or returns an invalid NodeHandle.
  NodeHandle FindChildWithName(NodeHandle parent, absl::string_view name);

  NodeHandle root_;

  bool local_trs_updated_ = false;

  AssetPtr<GltfAsset> gltf_asset_;
  mutable model::BoneLookup<RuntimeBoneVariant> runtime_bones_;

  // Maps from gltf node index to bone id.
  tsl::robin_map<uint16_t, BoneId> bone_id_lookup_;

  // Maps from node handle to the entity data id.
  RobinMap<NodeHandle, model::EntityId> node_to_entitiy_id_map_;

  friend class GltfRenderer;
};

template <typename Fn>
void GltfScene::ForAllNodes(const Fn& fn) const {
  for (auto& runtime_bone_variant : runtime_bones_) {
    if (absl::holds_alternative<NodeHandle>(runtime_bone_variant)) {
      fn(absl::get<NodeHandle>(runtime_bone_variant));
    }
  }
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_GLTF_SCENE_H_
