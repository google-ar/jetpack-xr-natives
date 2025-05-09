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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_IMPL_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_IMPL_H_

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Material.h"
#include "filament/filament/include/filament/MorphTargetBuffer.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/entity_absl_hasher.h"
#include "core/common/robin_map.h"
#include "core/common/robin_set.h"
#include "core/config.h"
#include "core/geometry/shapes/capsule.h"
#include "core/geometry/shapes/sphere.h"
#include "core/lighting/image_based_lighting_types.h"
#include "core/material_library/generic_material_spec.h"
#include "core/material_library/material_param_value.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/model/mesh/base_mesh_builder.h"
#include "core/ncsb/update_id.h"
#include "core/ncsb/update_phase.h"
#include "core/ncsb/update_system.h"
#include "core/split_engine/split_engine_mesh_serializer.h"
#include "core/split_engine/split_engine_texture_serializer.h"
#if IMP_PLATFORM(ANDROID)
#include "core/render/android/android_defines.h"
#include "core/render/android/platform_android_external_texture_surface.h"
#endif
#include "core/render/base_renderable_manager.h"
#include "core/render/base_texture_builder.h"
#include "core/split_engine/materials/split_engine_material.h"
#include "core/split_engine/split_engine_bridge_sender.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"
#include "split_engine/schemas/split_engine_data_generated.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"

namespace imp::split_engine {

// The SplitEngineSerializerImpl is responsible for serializing the data
// required to render the scene into a flatbuffer format that can be sent to
// the SplitEngineRenderer on the backend.
//
// It implements the SplitEngineSerializer interface and also inherits from
// BaseRenderableManager so that it can intercept all the calls made to the main
// RenderableManagerWrapper by normal Impress code.
//
// The Update() method of this class is responsible for serializing all the
// data into flatbuffers and sending them to the SplitEngineRenderer and runs
// at the very end of the frame.
//
// Note that a Cleanup() is not required because the renderer side will clear
// all data for an app when the bridge is released.
//
// TODO: Split the BaseRenderableManager code to a separate class.
class SplitEngineSerializerImpl
    : public SplitEngineSerializer,
      public BaseRenderableManager,
      public UpdateSystem::Updater<SplitEngineSerializerImpl> {
 public:
  // Update at the very end so that serialization occurs after all other frame
  // logic is done.
  static constexpr UpdatePhase kUpdatePhase = UpdatePhase::kEnd;
  using UpdateDependencies = UpdateIds<SplitEngineMaterialUpdater>;

  // Constructs a SplitEngineSerializerImpl.
  // The bridge_sender is used to send serialized data to the render using
  // a shared channel, which is initialized with a fixed size buffer of
  // bridge_buffer_size_bytes. The one_shot_bridge_sender sends larger
  // serialized data on a separate channel, allowing optimization of
  // underlying resources for serialization of larger and infrequent data.
  SplitEngineSerializerImpl(
      BaseView& view, std::unique_ptr<SplitEngineBridgeSender> bridge_sender,
      std::unique_ptr<SplitEngineBridgeSender> one_shot_bridge_sender,
      size_t bridge_buffer_size_bytes);

  SplitEngineSerializerImpl(const SplitEngineSerializerImpl&) = delete;
  SplitEngineSerializerImpl(SplitEngineSerializerImpl&&) = delete;

  SplitEngineSerializerImpl& operator=(const SplitEngineSerializerImpl&) =
      delete;
  SplitEngineSerializerImpl& operator=(SplitEngineSerializerImpl&&) = delete;

  void Update(const FrameTime& frame_time) override;

  void SetSpy(BaseRenderableManager& spy) override;

  filament::RenderableManager::Instance GetInstance(
      utils::Entity e) const override;
  bool HasComponent(utils::Entity e) const override;
  void Destroy(utils::Entity e) override;
  size_t GetPrimitiveCount(
      filament::RenderableManager::Instance instance) const override;
  const Box& GetAxisAlignedBoundingBox(
      filament::RenderableManager::Instance instance) const override;
  void SetMaterialInstanceAt(
      filament::RenderableManager::Instance instance, size_t primitiveIndex,
      const filament::MaterialInstance* material_instance) override;
  void SetGeometryAt(filament::RenderableManager::Instance instance,
                     size_t primitiveIndex,
                     filament::backend::PrimitiveType type,
                     filament::VertexBuffer* vertices,
                     filament::IndexBuffer* indices, size_t offset,
                     size_t count) override;
  void SetAxisAlignedBoundingBox(filament::RenderableManager::Instance instance,
                                 const Box& aabb) override;
  void SetPriority(filament::RenderableManager::Instance instance,
                   uint8_t priority) override;
  void SetChannel(filament::RenderableManager::Instance instance,
                  uint8_t channel) override;
  uint8_t GetLayerMask(
      filament::RenderableManager::Instance instance) const override;
  void SetLayerMask(filament::RenderableManager::Instance instance,
                    uint8_t select, uint8_t values) override;
  void SetBlendOrderAt(filament::RenderableManager::Instance instance,
                       size_t primitiveIndex, uint16_t order) override;
  void SetGlobalBlendOrderEnabledAt(
      filament::RenderableManager::Instance instance, size_t primitiveIndex,
      bool enabled) override;
  bool IsShadowCaster(
      filament::RenderableManager::Instance instance) const override;
  void SetCastShadows(filament::RenderableManager::Instance instance,
                      bool enable) override;
  bool IsShadowReceiver(
      filament::RenderableManager::Instance instance) const override;
  void SetReceiveShadows(filament::RenderableManager::Instance instance,
                         bool enable) override;
  void SetFogEnabled(filament::RenderableManager::Instance instance,
                     bool enable) override;
  size_t GetMorphTargetCount(
      filament::RenderableManager::Instance instance) const override;
  void SetMorphWeights(filament::RenderableManager::Instance instance,
                       float const* weights, size_t count,
                       size_t offset) override;

  std::unique_ptr<BaseRenderableManager::Builder> NewBuilder(
      size_t count) override;

  void AddMaterial(const filament::Material* material,
                   const BufferAccess& data) override;
  void RemoveMaterial(const filament::Material* material) override;
  void AddMaterialInstance(const filament::Material* material,
                           const filament::MaterialInstance* instance) override;
  void DuplicateMaterialInstance(
      const filament::MaterialInstance* instance,
      const filament::MaterialInstance* copy) override;
  void RemoveMaterialInstance(
      const filament::MaterialInstance* instance) override;
  void SetMaterialParameter(const filament::MaterialInstance* material,
                            absl::string_view name,
                            const MaterialParamValue& value) override;
  void SetMaterialParameter(const filament::MaterialInstance* material,
                            absl::string_view name,
                            const filament::Texture* texture,
                            const filament::TextureSampler& sampler) override;
  void CreateNode(utils::Entity entity) override;
  void DestroyNode(utils::Entity entity) override;
  void SetEnabled(utils::Entity entity, bool enabled) override;
  void SetName(utils::Entity entity, absl::string_view name) override;
  void SetParent(utils::Entity entity, utils::Entity parent) override;
  void SetLocalTransform(utils::Entity entity, const mat4f& transform) override;
  void SetLocalTransform(utils::Entity entity, const mat4& transform) override;
  void AssignUserId(utils::Entity entity, uint32_t user_id) override;
  std::unique_ptr<BaseTextureBuilder> CreateTextureBuilder() override;
  std::unique_ptr<BaseMeshBuilder> CreateMeshBuilder() override;
#if IMP_PLATFORM(ANDROID)
  std::unique_ptr<PlatformAndroidExternalTextureSurface>
  CreateAndroidExternalTextureSurface(
      ContentSecurityLevel security_level,
      absl::Span<const SurfaceViewType> view_types) override;
#endif
  void SetBoxCollider(utils::Entity entity, const Box& box,
                      bool enabled) override;
  void SetMeshCollider(utils::Entity entity, bool enabled) override;
  void SetSphereCollider(utils::Entity entity, const Sphere& sphere,
                         bool enabled) override;
  void SetCapsuleCollider(utils::Entity entity, const Capsule& capsul,
                          bool enabled) override;
  void ClearCollider(utils::Entity entity, ColliderType collider_type) override;
  void AddTexture(
      filament::Texture& texture,
      SplitEngineTextureSerializer& split_engine_texture_serializer) override;
  void RemoveTexture(filament::Texture& texture) override;
  void SerializeMesh(
      SplitEngineMeshSerializer& split_engine_mesh_serializer) override;
  Future<GenericMaterialPtr> CreateGenericMaterial(
      const GenericMaterialSpec& spec) override;
  void SetBuiltInMaterialParameters(
      const filament::MaterialInstance* material,
      android_xr::schemas::BuiltInMaterialParameters type,
      SerializeBuiltInMaterialParametersFunc serialize_func) override;
  void SerializeImageBasedLightingAsset(
      filament::Texture& reflection_texture,
      const SphericalHarmonics& spherical_harmonics,
      const ImageBasedLightingAssetCubemapImages& cubemap_images) override;
  void RemoveImageBasedLightingAsset(
      filament::Texture& reflection_texture) override;
  void SetPreferredEnvironmentIblAsset(filament::Texture& reflection_texture,
                                       float intensity,
                                       const float3& tint) override;
  void ClearPreferredEnvironmentIblAsset() override;
  void RemoveMorphTargetBuffer(
      filament::MorphTargetBuffer* morph_target_buffer) override;
  void RemoveVertexBuffer(filament::VertexBuffer* vertex_buffer) override;
  void RemoveIndexBuffer(filament::IndexBuffer* index_buffer) override;

 protected:
  void SetBonesInternal(filament::RenderableManager::Instance instance,
                        filament::RenderableManager::Bone const* transforms,
                        size_t boneCount, size_t offset) override;
  void SetBonesInternal(filament::RenderableManager::Instance instance,
                        mat4f const* transforms, size_t boneCount,
                        size_t offset) override;

 private:
  // Returns the FlatBufferBuilder for the current frame that corresponds to the
  // given request type.
  flatbuffers::FlatBufferBuilder* GetFlatBufferBuilderFor(
      android_xr::schemas::CommandTypes command_type);

  size_t EstimateImageBasedLightingAssetBufferSize(
      const SphericalHarmonics& spherical_harmonics,
      const ImageBasedLightingAssetCubemapImages& cubemap_images);
  void SerializeMeshIndicesAndVertices(
      SplitEngineMeshSerializer& split_engine_mesh_serializer);
  void SerializeMeshMorphTargets(
      SplitEngineMeshSerializer& split_engine_mesh_serializer);

  using ResourceId = std::uint64_t;
  using FlatBufferBuilderPtr = std::unique_ptr<flatbuffers::FlatBufferBuilder>;
  using IdAndFlatBufferBuilderPtr = std::pair<ResourceId, FlatBufferBuilderPtr>;

  // The type of data to be accumulated by material updates (see below).
  enum class MaterialUpdateBuilderType {
    kMaterialParameters,
    kBuiltInMaterialParameters,
    kMaterialDuplicates,
  };

  // Returns the FlatBufferBuilderPtr for the given type of material update.
  // If there is no current builder, a new one will be created.
  // If the current builder is of the same type, it will be returned.
  // If the current builder is of a different type, it will be committed and a
  // new builder will be created and returned.
  //
  // Explanation: there is no correct "static" ordering between setting material
  // parameters and duplicating materials.
  // Originally, it was always set parameters then duplicate, which worked for
  // the main use-case of duplicating the main glTF generic materials. This
  // meant that the duplicates would happen after setting the parameters on the
  // backend so that the duplicate would have all the same parameters.
  // With KHR_animation_pointer support, though, the material is duplicated and
  // then NEW PARAMS are set on the duplicate on the same frame on which it was
  // duplicated. This caused a crash, essentially, because the code tried to set
  // params on a material instance that didn't exist because the duplicate
  // command hadn't yet been processed.
  // This code preserves the app-side sequencing. This allows for a sequence
  // such as this to occur all on the same frame:
  //   Create Material 1
  //   Set Params on 1
  //   Duplicate 1 -> 2
  //   Set Params on 2
  //
  // The new code basically commits all the params / duplicates whenever the
  // app switches to a new operation but still preserves that parameters are
  // accumulated as much as possible rather than creating a command instantly
  // for every change, i.e. if an app sets many parameters each frame and
  // doesn't duplicate any materials, only one SetMaterialParameters command is
  // issued for the frame.
  FlatBufferBuilderPtr& GetMaterialUpdateBuilder(
      MaterialUpdateBuilderType type);

  // Helper to create a vector of flatbuffers::Offset<T>.
  template <typename T>
  using VectorOffset = std::vector<flatbuffers::Offset<T>>;

  // Helper for creating a map where an entity is the key.
  template <typename T>
  using EntityMap = RobinMap<utils::Entity, T, EntityHasher>;
  using EntitySet = RobinSet<utils::Entity, EntityHasher>;

  // Forward declare builder class for renderables. Defined in the .cc file. The
  // NewBuilder() method returns an instance of this type.
  class RenderableBuilder;

  // Below are data structures used to track the changes to the scene graph that
  // have occurred within a frame. These data structures are then serialized
  // into flatbuffer requests at the end of the frame and sent to the
  // SplitEngineRenderer through the SplitEngineBridge
  //
  // Some of these data structures are entirely C++ structures that match the
  // schemas in split_engine_data.fbs. Some of these structures directly store
  // the flatbuffer offset of the data that will be sent to the
  // SplitEngineRenderer which allows the data to be serialized without an
  // additional copy.
  //
  // It is intentional that the structures use the above mixed approach.
  //
  // This is because once data is written to a flatbuffer offset it cannot be
  // mutated again. Therefore, using a flatbuffer offset is preferred for large
  // data that aren't expected to change once written (i.e. vertex buffers,
  // index buffers, bones). However, for smaller data that could reasonably
  // change multiple times in a frame (i.e. node transforms, blend order), it is
  // better to hold the change in an intermediate structure before writing it to
  // the flatbuffer.
  //
  // All of these structures are then cleared at the end of each frame after
  // serialization.

  struct MeshCollider {};

  struct MaterialParameters {
    VectorOffset<android_xr::schemas::MaterialParamInfo> params;
    VectorOffset<android_xr::schemas::MaterialTextureParameter> texture_params;
  };

  struct UpdateNodeInfo {
    std::optional<std::string> name;
    std::optional<android_xr::schemas::Bool> enabled;
    std::optional<std::variant<mat4f, mat4>> transform;
    std::optional<utils::Entity> parent;
  };

  struct MorphTargetInfo {
    uint64_t morph_target_buffer_offset = 0;
    uint64_t morph_target_buffer_count = 0;
  };

  struct RenderableFlags {
    std::optional<android_xr::schemas::Bool> culling_enabled;
  };

  struct MorphTargetData {
    std::optional<android_xr::schemas::UInt64> morph_target_buffer_id;
    std::vector<MorphTargetInfo> morph_target_info;
  };

  struct AddRenderableInfo {
    uint32_t primitive_count = 0;
    std::optional<android_xr::schemas::UInt32> skinning_bone_count;
    std::optional<MorphTargetData> morph_target_data;
    std::optional<RenderableFlags> renderable_flags;
  };

  struct GeometryUpdateInfo {
    uint64_t vertex_buffer_id = 0;
    uint64_t index_buffer_id = 0;
    uint32_t offset = 0;
    uint32_t count = 0;
    uint8_t primitive_type = 0;
  };

  struct PrimitiveUpdateInfo {
    uint32_t primitive_index = 0;
    std::optional<GeometryUpdateInfo> geometry;
    std::optional<android_xr::schemas::UInt64> material_instance_id;
    std::optional<android_xr::schemas::UInt16> blend_order;
    std::optional<android_xr::schemas::Bool> global_blend_order_enabled;
  };

  struct LayerMask {
    uint8_t select;
    uint8_t values;
  };

  struct UpdateRenderableInfo {
    // Index is the primitive index.
    RobinMap<uint32_t, PrimitiveUpdateInfo> primitives;
    std::optional<Box> bounds;
    std::optional<LayerMask> layer_mask;
    flatbuffers::Offset<android_xr::schemas::Bones> bones;
    flatbuffers::Offset<android_xr::schemas::MorphWeights> morph_weights;
    std::optional<android_xr::schemas::UInt8> priority;
  };

  struct EnvironmentLightParams {
    uint64_t image_based_lighting_asset_id;
    float intensity;
    float3 tint;
  };

  struct AddOrUpdateColliderInfo {
    using ColliderVariant = std::variant<Box, MeshCollider, Sphere, Capsule>;
    ColliderVariant collider;
    std::optional<android_xr::schemas::Bool> enabled;
  };

  utils::Entity GetEntity(
      const filament::RenderableManager::Instance& instance) const;

  FlatBufferBuilderPtr CreateFlatBufferBuilder();
  FlatBufferBuilderPtr CreateFlatBufferBuilder(size_t size_bytes);

  void SendMessageGroup(std::vector<FlatBufferBuilderPtr>& messages);

  BaseView& view_;
  std::unique_ptr<SplitEngineBridgeSender> bridge_sender_;

  // Use an independent sender for larger requests so that we can send them as
  // throw away buffers of arbitrary size.
  std::unique_ptr<SplitEngineBridgeSender> one_shot_bridge_sender_;

  size_t bridge_buffer_size_bytes_;

  // A builder for adding and removing materials and material instances.
  FlatBufferBuilderPtr materials_builder_;
  // Vectors to accumulate materials and instances to add this frame.
  VectorOffset<android_xr::schemas::Material> materials_to_add_;
  std::unordered_map<ResourceId, ResourceId> material_instances_to_add_;
  // Vectors to accumulate materials and instances to remove this frame.
  std::vector<ResourceId> materials_to_remove_;
  std::vector<ResourceId> material_instances_to_remove_;

  // Commits the current material update builder to a Command if it is not null.
  void CommitCurrentMaterialUpdateBuilder();
  // Commits and clears all accumulated material parameters to a Command.
  void CommitMaterialParameters(FlatBufferBuilderPtr& fbb);
  // Commits and clears all accumulated built-in material params to a Command.
  void CommitBuiltInMaterialParameters(FlatBufferBuilderPtr& fbb);
  // Commits and clears all accumulated material duplicates to a Command.
  void CommitMaterialDuplicates(FlatBufferBuilderPtr& fbb);

  // A list of FlatBufferBuilders for material parameters, built-in material
  // parameters, and material duplicates. These builders are interleaved in the
  // order they are requested by the app in order to maintain the relative order
  // of material parameters and duplicates.
  std::vector<FlatBufferBuilderPtr> material_update_commands_;
  MaterialUpdateBuilderType current_material_update_builder_type_ =
      MaterialUpdateBuilderType::kMaterialParameters;

  // Data for accumulating updates to materials over the course of a frame.
  std::unordered_map<ResourceId, MaterialParameters> material_params_;
  std::vector<flatbuffers::Offset<
      android_xr::schemas::BuiltInMaterialInstanceParameters>>
      built_in_material_parameters_;
  std::unordered_map<ResourceId, ResourceId> material_instances_to_duplicate_;

  // A list of textures to remove this frame. Note: there is no list of textures
  // to add because textures are serialized via individual one-off, custom-sized
  // memory regions. This is done because large assets like textures may exceed
  // our standard arena size for normal serialized command data.
  std::vector<ResourceId> textures_to_remove_;

  // Node requests for this frame.
  EntitySet add_nodes_;
  EntityMap<UpdateNodeInfo> node_updates_;
  EntitySet remove_nodes_;
  EntityMap<uint32_t> user_id_assignments_;

  // Renderable requests for this frame.
  FlatBufferBuilderPtr renderable_updates_builder_;
  EntityMap<AddRenderableInfo> add_renderables_;
  EntityMap<UpdateRenderableInfo> renderable_updates_;
  EntitySet remove_renderables_;

  // Collider requests for this frame.
  EntityMap<AddOrUpdateColliderInfo> collider_add_or_updates_;
  EntityMap<android_xr::schemas::ColliderType> collider_removals_;

  // ImageBasedLightingAsset requests for this frame.
  std::vector<uint64_t> image_based_lighting_assets_to_remove_;

  // Preferred environment IBL asset request for this frame.
  std::optional<EnvironmentLightParams> preferred_environment_ibl_asset_id_;

  // Morph target buffer ids to remove for this frame.
  std::vector<uint64_t> morph_target_buffers_to_remove_;
  // Vertex buffer ids to remove for this frame.
  std::vector<uint64_t> vertex_buffers_to_remove_;
  // Index buffer ids to remove for this frame.
  std::vector<uint64_t> index_buffers_to_remove_;

  void SerializeRemoveImageBasedLightingAssets(
      std::vector<FlatBufferBuilderPtr>& command_queue);
  void SerializeSetPreferredEnvironmentIblAsset(
      std::vector<FlatBufferBuilderPtr>& command_queue);
  void SerializeAddMaterials(std::vector<FlatBufferBuilderPtr>& command_queue);
  void SerializeRemoveMaterials(
      std::vector<FlatBufferBuilderPtr>& command_queue);
  void SerializeAddMaterialInstances(
      std::vector<FlatBufferBuilderPtr>& command_queue);
  void SerializeRemoveMaterialInstances(
      std::vector<FlatBufferBuilderPtr>& command_queue);
  void SerializeAddNodes(std::vector<FlatBufferBuilderPtr>& command_queue);
  void SerializeRemoveNodes(std::vector<FlatBufferBuilderPtr>& command_queue);
  void SerializeAssignUserIdToNodes(
      std::vector<FlatBufferBuilderPtr>& command_queue);
  void SerializeUpdateNodes(std::vector<FlatBufferBuilderPtr>& command_queue);
  void SerializeRemoveRenderables(
      std::vector<FlatBufferBuilderPtr>& command_queue);
  void SerializeAddRenderables(
      std::vector<FlatBufferBuilderPtr>& command_queue);
  void SerializeUpdateRenderables(
      std::vector<FlatBufferBuilderPtr>& command_queue);
  void SerializeRemoveTextures(
      std::vector<FlatBufferBuilderPtr>& command_queue);
  void SerializeAddOrUpdateColliders(
      std::vector<FlatBufferBuilderPtr>& command_queue);
  void SerializeRemoveColliders(
      std::vector<FlatBufferBuilderPtr>& command_queue);
  void SerializeRemoveMorphTargetBuffers(
      std::vector<FlatBufferBuilderPtr>& command_queue);
  void SerializeRemoveMeshData(
      std::vector<FlatBufferBuilderPtr>& command_queue);
  // When all commands have been serialized, this method will send the commands
  // to the SplitEngineRenderer and assert if any data structure is not empty.
  void SendCommandQueue(std::vector<FlatBufferBuilderPtr>& command_queue);
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_IMPL_H_
