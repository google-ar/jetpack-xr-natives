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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_GLTF_RENDERER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_GLTF_RENDERER_H_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/hash/hash.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/animation/gltf_animation.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/common/bit_vector.h"
#include "core/common/invocable.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/common/paired_vector.h"
#include "core/common/robin_map.h"
#include "core/common/typed_id.h"
#include "core/geometry/shapes/box.h"
#include "core/graph/dependency_graph.h"
#include "core/material_library/generic_material.h"
#include "core/material_library/material_param_value.h"
#include "core/materials/material.h"
#include "core/math/math.h"
#include "core/model/model_data.h"
#include "core/model/shared_data.h"
#include "core/model/skeleton_data.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/component_system.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/update_phase.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_asset.h"
#include "core/view/framework/assets/gltf_extension.h"
#include "core/view/framework/assets/gltf_state.proto.imp.h"
#include "core/view/framework/assets/gltf_traits.h"
#include "core/view/framework/lighting/light_component.h"
#include "core/view/utils/asset.h"
#include "core/view/utils/frame_time.h"
#include "robin_map/include/tsl/robin_map.h"

namespace imp {

// Forward declare un-included types
class GltfMesh;

// Renders & manages the contents of a GltfAsset.
//
// When loading a model, the resulting node is guaranteed to have a GltfRenderer
// and a GltfScene attached to it.
//
// A GltfRenderer can also be manually added to a node, which will automatically
// add a GltfScene as well.
//
// Together, GltfRenderer and GltfScene create a representation of the glTF's
// contents as children that mirror the hierarchy and properties of the glTF
// file as close as possible. GltfScene is responsible for the hierarchy of
// nodes. GltfRenderer is responsible for attaching and managing GltfMesh
// components to the node's that contain meshes in the glTF file.
//
// Additionally, GltfRenderer is responsible for applying skinning from the
// bones to the final rendering. The method GltfRenderer::ScheduleSkinningUpdate
// must be called to indicate that skinning should be updated at the end of the
// frame. This is done to avoid the performance overhead of doing it every frame
// or detecting when the bones have changed. Animating a glTF with GltfAnimator
// automatically calls ScheduleSkinningUpdate. However, ScheduleSkinningUpdate
// must be called manually when the bones are only adjusted dynamically.
//
// GltfRenderer will also (by default) add a GltfCollider component to each node
// containing a GltfMesh. This make it so intersection tests can be performed
// against the bounds of the meshes in the glTF. Intersection tests are
// performed automatically in response to pointer input. This makes it possible
// to do things like detect when a glTF is tapped. As is typical in Impress,
// these events bubble up through the node hierarchy so it can be used to
// determine which specific node within the glTF was tapped, or when the glTF as
// a whole was tapped.
//
// Please note that GltfRenderer will also clean up the colliders when it's
// removed. If the collider mode is BOX_COLLIDER, then the BoxCollider will be
// removed. If the collider mode is not GLTF_COLLIDER_*, then all of the
// GltfCollider within GltfScene will be removed.
//
// TODO: Add ability to replace many materials with single call.
class GltfRenderer : public Component {
 public:
  // If set, disables changes to skinning system wide, without affecting
  // individual component state or simulation.
  enum class SkinningSystemOverride {
    kSkinningSystemEnabled,
    kSkinningSystemDisabled
  };
  class System : public ComponentSystem<GltfRenderer> {
   public:
    explicit System(BaseView* view) : ComponentSystem(view) {}

    // Registers an extension of type T, where T is derived from GltfExtension.
    template <typename T>
    void RegisterExtension();

    // Registers an extension of type T, where T is derived from GltfExtension,
    // that depends on an extension of type K.
    template <typename T, typename K>
    void RegisterExtensionWithDependency();

    // Removes an extension of type T, where T is derived from GltfExtension,
    // from the extension registry.
    template <typename T>
    void UnregisterExtension();

    // Gets the number of extensions currently registered.
    size_t GetExtensionCount() const;

    using ExtensionCreationFunction =
        Invocable<Future<absl::Status>(ComponentHandle<GltfRenderer>)>;

    using ExtensionValidityFunction =
        Invocable<bool(ComponentHandle<GltfRenderer>)>;

    Future<absl::Status> SetupExtensionsForRenderer(
        ComponentHandle<GltfRenderer> renderer);

    inline void SetSkinningSystemOverride(
        SkinningSystemOverride skinning_system_override) {
      skinning_system_override_ = skinning_system_override;
    }
    inline SkinningSystemOverride GetSkinningSystemOverride() const {
      return skinning_system_override_;
    }

    void AfterComponentAdded(GltfRenderer& renderer) override;

   private:
    struct ExtensionInfo {
      ExtensionCreationFunction creation_function;
      ExtensionValidityFunction is_valid;
    };

    RobinMap<ComponentId, ExtensionInfo> extension_info_map_;
    DependencyGraph<ComponentId> extension_dependency_graph_;
    SkinningSystemOverride skinning_system_override_ =
        SkinningSystemOverride::kSkinningSystemEnabled;
  };

  // Provides information for instancing of Renderables
  struct InstanceInfo {
    int instance_count = 0;
    std::vector<mat4f> instance_transforms;
  };

  void Setup(AssetPtr<GltfAsset> gltf_asset,
             std::optional<GltfAsset::LoadOptions> options = absl::nullopt);

  void Cleanup();

  Future<absl::Status> Setup();
  Future<absl::Status> Setup(
      const AssetDefinition& asset_definition,
      std::optional<GltfAsset::LoadOptions> options = absl::nullopt);
  Future<absl::Status> Setup(
      absl::string_view asset_url,
      std::optional<GltfAsset::LoadOptions> options = absl::nullopt);
  Future<absl::Status> Setup(
      absl::Cord contents, absl::string_view asset_url,
      std::optional<GltfAsset::LoadOptions> options = absl::nullopt);

  void Update(const FrameTime& frame_time);

  // Called when the component is deactivated or activated. This turns off or on
  // the render by setting the layer mask of every entity in the model.
  void OnActiveStatusChanged(bool is_active);

  // Returns the cached EntityId from the glTF Model for a given Impress Scene
  // Node
  std::optional<model::EntityId> GetEntityIdFromNodeHandle(
      NodeHandle node) const;

  AssetPtr<GltfAsset> GetGltfAsset() const;

  NodeHandle GetModelRoot() const;

  // Gets the bounds of the meshes in the Gltf in the coordinate space of this
  // node.
  //
  // These are the bounds used for collision with the GltfRenderer when using
  // the BOX_COLLIDER collider mode.
  //
  // If there are no meshes, then this will return an invalid Box.
  Box GetLocalBounds() const;

  // Gets the bounds of the meshes in the Gltf in the world coordinate space.
  //
  // If there are no meshes, then this will return an invalid Box.
  Box GetWorldBounds() const;

  // Gets the bounds of both the meshes and the bones in the Gltf in the
  // coordinate space of this node.
  //
  // This is the union of GltfRenderer::GetLocalBounds and
  // GltfScene::GetLocalBoneBounds.
  Box GetLocalFullBounds() const;

  // Gets the bounds of both the meshes and the bones in the Gltf in the world
  // coordinate space.
  //
  // This is the union of GltfRenderer::GetWorldBounds and
  // GltfScene::GetWorldBoneBounds.
  Box GetWorldFullBounds() const;

  void SetRenderBounds(const Box& local_bounds);
  absl::string_view GetAssetUrl() const { return state_.asset; }

  // Returns both the original materials and the current material
  // overrides in the GltfAsset.
  ABSL_DEPRECATED(
      "Use imp::BorrowedMaterialPtr overload instead. See "
      "(broken link).")
  std::vector<Material*> GetMaterials() const;

  // Returns both the original materials and the current material
  // overrides in the GltfAsset, but only if they were added through the
  // Borrowed or OwnedMaterialPtr overrides. It will not include any materials
  // added as raw pointers.
  std::vector<BorrowedMaterialPtr> GetBorrowedMaterials() const;

  Material* GetMaterialByIndex(uint16_t material_index) const;
  // Returns the generic material that is actually used by this model by index.
  // If sharing mode is DUPLICATED_DEFAULT, returns the duplicated material for
  // this instance. If sharing mode is SHARED, a non-const material cannot be
  // returned so an error is returned.
  absl::StatusOr<GenericMaterial*> GetGenericMaterialByIndex(
      uint16_t material_index) const;

  // Returns the current material overrides in this GltfRenderer.
  ABSL_DEPRECATED(
      "Use imp::BorrowedMaterialPtr overload instead. See "
      "(broken link).")
  std::vector<Material*> GetMaterialOverrides() const;

  // Returns the current material overrides in this GltfRenderer, but only if
  // they were added through the Borrowed or OwnedMaterialPtr overrides. It will
  // not include any materials added as raw pointers.
  std::vector<BorrowedMaterialPtr> GetBorrowedMaterialOverrides() const;

  // Return a light component according to a light punctual id
  ComponentHandle<LightComponent> GetLightComponentById(
      model::ModelData::LightPunctualId id);

  // Returns the node with this name from the GltfAsset, creating it if it
  // didn't exist yet. This allows modifying bone positions, after which
  // ScheduleSkinningUpdate() needs to be called.
  NodeHandle GetOrCreateNode(absl::string_view name);

  // Schedules a skinning update that will happen once a frame. This needs to be
  // called whenever the bones change, such as moving nodes from
  // GetOrCreateNode(). Animations already do this automatically so it doesn't
  // need to be called for that case.
  //
  // Skinning updates occur at the end of the frame, so changes to bones that
  // occur after ScheduleSkinningUpdate is called but before the end of the
  // frame will still apply.
  // TODO See if we can remove the need for this.
  void ScheduleSkinningUpdate();

  // Get the transformation of a joint in target space.
  const PairedVector<mat4f, model::ModelData::SampledJointData>&
  GetSampledTransforms(model::ModelData::SkinId skin_id,
                       model::ModelData::EntityId entity_id) const;

  // 'Pivot', here, is the term of art from UI layout applied to 3D.  It's the
  // 'Anchor' or 'Origin' phrased relative to the rendered bounds.  A pivot of
  // (0.5, 0.5, 0.5) will tumble about the geometric center.  A pivot of (0.5,
  // 0, 0.5) will tumble around the center of the bottom bounds.  The pivot
  // is initially computed from the imported data (and may exceed the range
  // [0..1] in one or more dimensions).  Visually speaking, setting the pivot
  // moves geometry and keeps the origin static.
  float3 GetPivot() const;
  void SetPivot(float3 pivot);

  // Sets which materials variant to use to render this glTF based on the glTF
  // extension KHR_materials_variants.
  //
  // The MaterialsVariantsId maps to the index of the materials variant in the
  // glTF file.
  //   - If absl::nullopt is passed in, then the active variant is unset and the
  //     default materials are used for all primitives.
  //   - If a primitive previously had its material overridden by calling
  //     GltfMesh::SetMaterialOverride, then this method will not impact the
  //     material assigned to that primitive.
  //   - If an invalid id is passed in, this method does nothing.
  void SetMaterialsVariant(std::optional<model::ModelData::MaterialsVariantsId>
                               materials_variant_id);

  // Same as the above, except doesn't take over ownership of the material.
  ABSL_DEPRECATED(
      "Use imp::BorrowedMaterialPtr overload instead. See "
      "(broken link).")
  void SetMaterialOverrideByIndex(Material* new_material,
                                  size_t material_index);

  // Overrides the material of all GltfMeshes by material index.
  void SetMaterialOverrideByIndex(OwnedMaterialPtr new_material,
                                  size_t material_index);

  // Same as the above, except doesn't take over ownership of the material.
  void SetMaterialOverrideByIndex(BorrowedMaterialPtr new_material,
                                  size_t material_index);

  // Returns the overridden material by material index if there is one.
  // Otherwise, returns nullptr;
  ABSL_DEPRECATED(
      "Use imp::BorrowedMaterialPtr overload instead. See "
      "(broken link).")
  Material* GetMaterialOverrideByIndex(size_t material_index) const;

  // Returns the overridden material by material index if there is one, and it
  // was set using the Borrowed or OwnedMaterialPtr overrides.
  // If the override was set using a raw pointer or no override exists, returns
  // nullptr.
  BorrowedMaterialPtr GetBorrowedMaterialOverrideByIndex(
      size_t material_index) const;

  // Returns the node entity and data vector.
  const PairedVector<utils::Entity, model::ModelData::EntityData>&
  GetNodeEntities() {
    return node_entities_;
  }
  Future<absl::Status> GetExtensionFuture() { return extension_setup_future_; }

  const RobinSet<NodeHandle>* GetNodesFromOriginalMeshIndex(
      int16_t original_mesh_index) const;

  absl::StatusOr<int16_t> GetOriginalMaterialIndex(
      model::EntityId entity_id, size_t primitive_index) const;

  int GetMeshCount() const { return mesh_index_to_nodes_.size(); };

  // Returns the per mesh record of morph target weights.
  std::vector<float> GetMeshMorphTargetWeights(size_t mesh_index) const;

  // Returns an individual morph target's weight in the per mesh record if it
  // exists, otherwise returns std::nullopt.
  std::optional<float> GetMeshMorphTargetWeight(size_t mesh_index,
                                                size_t target_index) const;

  // Sets the per mesh record of morph target weights and update the real-time
  // weights on Filament for all nodes that use this mesh if the per node
  // weights are not set for those nodes.
  void SetMeshMorphTargetWeights(const std::vector<float>& weights,
                                 size_t mesh_index);

  // Sets an individual morph target's weight in the per mesh record and update
  // the real-time weights on Filament for all nodes that use this mesh if the
  // per node weights are not set for those nodes.
  absl::Status SetMeshMorphTargetWeight(float weight, size_t mesh_index,
                                        size_t target_index);

  // Returns true if skinning is scheduled to be updated.
  bool IsSkinningScheduled() const { return skinning_scheduled_; }

 private:
  using Bone = model::BoneData;
  using BoneId = model::BoneId;
  using BoneTargetId = animation::GltfAnimation::BoneTargetId;
  using ModelData = model::ModelData;
  using MaterialId = ModelData::MaterialId;
  using LightPunctualId = ModelData::LightPunctualId;
  using EntityData = ModelData::EntityData;
  using EntityId = ModelData::EntityId;
  using Joint = ModelData::JointData;
  using JointId = ModelData::JointId;
  using SampledJoint = ModelData::SampledJointData;
  using SampledJointId = ModelData::SampledJointId;
  using SkinId = ModelData::SkinId;
  using SkinnedEntityId = ModelData::SkinnedEntityId;
  using SkinnedEntityData = ModelData::SkinnedEntityData;
  using WeakEntityId = TypedId<EntityData, int32_t>;
  using ItemId = WeakEntityId;

  struct RuntimeSkinnedEntity {
    SkinnedEntityId skinned_entity;
    PairedVector<mat4f, SampledJoint> sampled_xforms;
  };
  struct RuntimeSkin {
    SkinId skin;
    PairedVector<RuntimeSkinnedEntity, SkinnedEntityData> skinned_entities;
  };
  RobinMap<LightPunctualId, ComponentHandle<LightComponent>>
      light_punctual_lookup_;

  // Maps the original mesh index from the glTF file to the set of Impress nodes
  // that hold that mesh.
  RobinMap<int16_t, RobinSet<NodeHandle>> mesh_index_to_nodes_;

  void InitializeSkinning();
  void UpdateSkinning();

  GltfState state_;

  AssetPtr<GltfAsset> gltf_asset_;
  std::optional<model::ModelData::MaterialsVariantsId>
      active_materials_variant_id_;

  // If state_.material_sharing_mode is DUPLICATED_DEFAULT ,
  // duplicated_materials_ will be returned. If state_.material_sharing_mode is
  // SHARED, the shared materials from GltfAsset will be returned.
  const GenericMaterialListing& GetMaterialsInternal() const;

  GenericMaterialListing duplicated_materials_;

  // For compactness, we refer to our children by entity.
  PairedVector<utils::Entity, EntityData> node_entities_;

  PairedVector<std::vector<float>, EntityData> node_morph_target_weights_;

  RobinMap<int, std::vector<float>> mesh_morph_target_weights_;

  std::vector<RuntimeSkin> runtime_skins_;
  // Below fields are used to track the transforms of joints in the coordinate
  // space of the root node when calculating skinning. It isn't part of the
  // skin_target so that it can be re-used each frame.
  PairedVector<mat4f, Bone> root_xforms_;
  PairedBitVector<Bone> root_xforms_updated_;

  bool skinning_scheduled_ = true;

  // Returns the material currently used for rendering.
  //
  // If a material override has been set this will return the override instead
  // of the original Material.
  Material* GetMaterial(EntityId entity_id, size_t primitive_index) const;

  // Overrides the material used on a primitive in this GltfMesh.
  //
  // This changes the underlying assignment for rendering, but does
  // not change the material assigned in the underlying GltfAsset. If material
  // is null, the original material from the models data will be used.
  void SetMaterialOverride(Material* raw_new_material, EntityId entity_id,
                           size_t primitive_index = 0);

  void SetMaterialOverride(OwnedOrBorrowedPtr<Material> new_material,
                           EntityId entity_id, size_t primitive_index = 0);

  // Returns the material override if set or null if none is set.
  Material* GetMaterialOverride(EntityId entity_id,
                                size_t primitive_index = 0) const;

  // Sets the value of morph target weights used for real-time rendering on
  // Filament.
  void SetMorphTargetWeights(const std::vector<float>& weights,
                             model::ModelData::EntityId entity_id);

  // Returns number of morph targets.
  size_t GetMorphTargetCount(model::ModelData::EntityId entity_id) const;

  // Returns the current morph target weights, namely the latest values passed
  // to Filament.
  std::vector<float> GetMorphTargetWeights(model::ModelData::EntityId entity_id,
                                           int original_mesh_index) const;

  // Sets the per node record of morph target weights and update the real-time
  // weights on Filament.
  void SetNodeMorphTargetWeights(const std::vector<float>& weights,
                                 model::ModelData::EntityId entity_id);

  void SetMaterialOverrideByIndexInternal(Material* raw_new_material,
                                          size_t material_index);

  void SetMaterialOverrideByIndexInternal(
      OwnedOrBorrowedPtr<Material> new_material, size_t material_index);

  GltfState::ColliderMode GetColliderMode() const;

  GltfState::MaterialSharingMode GetMaterialSharingMode() const;

  // Allows a primitive from the GltfAsset to be indexed and hashed.
  //
  // A primitive is an array of triangles with the same material.
  // Multiple primitives compose a mesh and might also be called a part or
  // submesh in other systems.
  struct GltfPrimitive {
    // EntityId to index the mesh/node.
    EntityId entity_id;
    // Index of the primitive within the mesh.
    size_t primitive_index;

    bool operator==(const GltfPrimitive& rhs) const {
      return entity_id == rhs.entity_id &&
             primitive_index == rhs.primitive_index;
    }

    struct Hash {
      size_t operator()(GltfPrimitive const& id) const {
        return absl::Hash<std::pair<EntityId::ValueType, size_t>>()(
            std::pair<EntityId::ValueType, size_t>(id.entity_id,
                                                   id.primitive_index));
      }
    };
  };

  //  Lookup table of per-primitive temporary materials in use with this model.
  tsl::robin_map<GltfPrimitive, OwnedOrBorrowedPtr<Material>,
                 GltfPrimitive::Hash>
      owned_or_borrowed_material_primitive_overrides_;

  // Used for backwards compatibility for deprecated API.
  tsl::robin_map<GltfPrimitive, Material*, GltfPrimitive::Hash>
      raw_material_primitive_overrides_;

  //  Array of per-index temporary materials in use with this model.
  PairedVector<OwnedOrBorrowedPtr<Material>, GenericMaterialPtr>
      owned_or_borrowed_material_index_overrides_;

  // Used for backwards compatibility for deprecated API.
  PairedVector<Material*, GenericMaterialPtr> raw_material_index_overrides_;

  InstanceInfo instance_info_{};

  Future<absl::Status> extension_setup_future_;
  std::vector<ComponentHandle<GltfExtension>> extensions_;

  // The load options used to load this asset. It won't make sense to modify
  // this after Setup.
  GltfAsset::LoadOptions load_options_;

  // Map of Impress Scene Nodes to EntityIds
  RobinMap<NodeHandle, model::EntityId> node_to_entity_id_map_;

  friend class GltfMesh;

 public:
  using IsfInfo = IsfInfo<&GltfRenderer::state_>;
  static constexpr UpdatePhase kUpdatePhase = UpdatePhase::kEnd;
  static constexpr bool kRunInEditMode = true;
};

template <typename T>
void GltfRenderer::System::RegisterExtension() {
  static_assert(std::is_base_of_v<GltfExtension, T>,
                "Extension must be derived from GltfExtension.");

  ComponentId extension_id = GetComponentTypeId<T>();

  if (extension_info_map_.find(extension_id) != extension_info_map_.end()) {
    // Extension is already registered, do nothing.
    return;
  }

  extension_info_map_.insert(
      {extension_id,
       ExtensionInfo{
           .creation_function = [](ComponentHandle<GltfRenderer> renderer)
               -> Future<absl::Status> {
             if (!renderer) {
               return Future<absl::Status>(
                   absl::InvalidArgumentError("Invalid gltf renderer"));
             }

             return renderer->GetNode()->AddComponent<T>(renderer).Then(
                 [renderer](ComponentHandle<T> component) mutable {
                   ComponentHandle<GltfExtension> extension =
                       Component::GetHandle<GltfExtension>(component.Get());
                   renderer->extensions_.push_back(extension);
                 });
           },
           .is_valid = [](ComponentHandle<GltfRenderer> renderer) -> bool {
             if constexpr (kHasValidForFunc<T, GltfRenderer>) {
               return T::IsValidFor(renderer);
             }
             // By default, if an extension does not have the IsValidFor
             // function, it'll always be valid i.e. will always be added to the
             // node.
             return true;
           }}});

  extension_dependency_graph_.AddNode(extension_id);
}

template <typename T, typename K>
void GltfRenderer::System::RegisterExtensionWithDependency() {
  static_assert(std::is_base_of_v<GltfExtension, T>,
                "Extension must be derived from GltfExtension.");
  static_assert(std::is_base_of_v<GltfExtension, K>,
                "Dependency extension must be derived from GltfExtension.");
  ComponentId extension_id = GetComponentTypeId<T>();
  ComponentId dependent_extension_id = GetComponentTypeId<K>();

  RegisterExtension<T>();
  if (extension_info_map_.find(dependent_extension_id) ==
      extension_info_map_.end()) {
    RegisterExtension<K>();
  }

  if (extension_id == dependent_extension_id) {
    IMP_LOG(imp::WARNING) << "Dependent extension is the same as the registered "
                    "extension. Ignoring the dependency.";
    return;
  }

  extension_dependency_graph_.AddDependency(extension_id,
                                            dependent_extension_id);
}

template <typename T>
void GltfRenderer::System::UnregisterExtension() {
  static_assert(std::is_base_of_v<GltfExtension, T>,
                "Extension must be derived from GltfExtension.");
  ComponentId extension_id = GetComponentTypeId<T>();
  extension_info_map_.erase(extension_id);
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_GLTF_RENDERER_H_
