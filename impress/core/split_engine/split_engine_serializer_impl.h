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

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include "absl/base/nullability.h"
#include "absl/container/flat_hash_map.h"
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
#include "core/split_engine/android/split_engine_android_bridge.h"
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
// TODO: (broken link) - Split the BaseRenderableManager code to a separate class.
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
      BaseView& view, std::unique_ptr<SplitEngineAndroidBridge> bridge,
      std::unique_ptr<SplitEngineBridgeSender> bridge_sender,
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
      utils::Entity entity) const override;
  bool HasComponent(utils::Entity entity) const override;
  void Destroy(utils::Entity entity) override;
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

  SplitEngineAndroidBridge& GetBridge() override;

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
  size_t EstimateImageBasedLightingAssetBufferSize(
      const SphericalHarmonics& spherical_harmonics,
      const ImageBasedLightingAssetCubemapImages& cubemap_images);
  void SerializeMeshIndicesAndVertices(
      SplitEngineMeshSerializer& split_engine_mesh_serializer);
  void SerializeMeshMorphTargets(
      SplitEngineMeshSerializer& split_engine_mesh_serializer);

  using ResourceId = std::uint64_t;
  using FlatBufferBuilderPtr = std::unique_ptr<flatbuffers::FlatBufferBuilder>;

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
    ResourceId vertex_buffer_id = 0;
    ResourceId index_buffer_id = 0;
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
    ResourceId image_based_lighting_asset_id;
    float intensity;
    float3 tint;
  };

  struct AddOrUpdateColliderInfo {
    using ColliderVariant = std::variant<Box, MeshCollider, Sphere, Capsule>;
    ColliderVariant collider;
    std::optional<android_xr::schemas::Bool> enabled;
  };

  struct RemoveMeshBuffers {
    std::vector<ResourceId> vertex_buffers;
    std::vector<ResourceId> index_buffers;
  };

  utils::Entity GetEntity(
      const filament::RenderableManager::Instance& instance) const;

  FlatBufferBuilderPtr CreateFlatBufferBuilder();
  FlatBufferBuilderPtr CreateFlatBufferBuilder(size_t size_bytes);

  BaseView& view_;
  // Hold the split engine bridge and ensures the lifetime of the bridge is the
  // lifetime of the serializer.
  // NOTE: it is critical that the bridge is destroyed after the senders are
  // destroyed, so the order of these fields is important.
  std::unique_ptr<SplitEngineAndroidBridge> bridge_;

  // The main bridge sender used for sending messages to the split engine
  // renderer.
  std::unique_ptr<SplitEngineBridgeSender> bridge_sender_;

  // Use an independent sender for larger requests so that we can send them as
  // throw away buffers of arbitrary size.
  std::unique_ptr<SplitEngineBridgeSender> one_shot_bridge_sender_;

  size_t bridge_buffer_size_bytes_;

  // A batch is a collection of commands that will be serialized together. All
  // commands within the same batch need to be of the same type, because the
  // flatbuffer root is Command.
  struct CommandBatchBase {
    // The index of a batch is its place in the execution order. The batch with
    // index 0 will be executed first.
    CommandBatchBase(android_xr::schemas::CommandTypes type, int index)
        : type(type), index(index) {}
    virtual ~CommandBatchBase() = default;
    virtual void Serialize(flatbuffers::FlatBufferBuilder& fbb) = 0;

    const android_xr::schemas::CommandTypes type;
    int index = 0;
  };

  // Each command type requires a different type of data. This struct selects
  // the correct type for a given command type.
  template <android_xr::schemas::CommandTypes CommandT>
  struct DataSelector {};
  template <>
  struct DataSelector<android_xr::schemas::CommandTypes::AddRenderables> {
    using data_type = EntityMap<AddRenderableInfo>;
  };
  template <>
  struct DataSelector<android_xr::schemas::CommandTypes::RemoveRenderables> {
    using data_type = EntitySet;
  };
  template <>
  struct DataSelector<android_xr::schemas::CommandTypes::UpdateRenderables> {
    using data_type = EntityMap<UpdateRenderableInfo>;
  };
  template <>
  struct DataSelector<android_xr::schemas::CommandTypes::RemoveMeshData> {
    using data_type = RemoveMeshBuffers;
  };
  template <>
  struct DataSelector<
      android_xr::schemas::CommandTypes::RemoveMorphTargetBuffers> {
    using data_type = std::vector<ResourceId>;
  };
  template <>
  struct DataSelector<
      android_xr::schemas::CommandTypes::SetPreferredEnvironmentIblAsset> {
    using data_type = std::optional<EnvironmentLightParams>;
  };
  template <>
  struct DataSelector<
      android_xr::schemas::CommandTypes::RemoveImageBasedLightingAssets> {
    using data_type = std::vector<ResourceId>;
  };
  template <>
  struct DataSelector<android_xr::schemas::CommandTypes::AddMaterials> {
    using data_type = VectorOffset<android_xr::schemas::Material>;
  };
  template <>
  struct DataSelector<
      android_xr::schemas::CommandTypes::SetMaterialParameters> {
    using data_type = std::unordered_map<ResourceId, MaterialParameters>;
  };
  template <>
  struct DataSelector<android_xr::schemas::CommandTypes::RemoveMaterials> {
    using data_type = std::vector<ResourceId>;
  };
  template <>
  struct DataSelector<android_xr::schemas::CommandTypes::AddMaterialInstances> {
    using data_type = std::unordered_map<ResourceId, ResourceId>;
  };
  template <>
  struct DataSelector<
      android_xr::schemas::CommandTypes::DuplicateMaterialInstances> {
    using data_type = std::unordered_map<ResourceId, ResourceId>;
  };
  template <>
  struct DataSelector<
      android_xr::schemas::CommandTypes::SetBuiltInMaterialParameters> {
    using data_type =
        VectorOffset<android_xr::schemas::BuiltInMaterialInstanceParameters>;
  };
  template <>
  struct DataSelector<
      android_xr::schemas::CommandTypes::RemoveMaterialInstances> {
    using data_type = std::vector<ResourceId>;
  };
  template <>
  struct DataSelector<android_xr::schemas::CommandTypes::RemoveTextures> {
    using data_type = std::vector<ResourceId>;
  };
  template <>
  struct DataSelector<android_xr::schemas::CommandTypes::AddNodes> {
    using data_type = EntitySet;
  };
  template <>
  struct DataSelector<android_xr::schemas::CommandTypes::RemoveNodes> {
    using data_type = EntitySet;
  };
  template <>
  struct DataSelector<android_xr::schemas::CommandTypes::UpdateNodes> {
    using data_type = EntityMap<UpdateNodeInfo>;
  };
  template <>
  struct DataSelector<android_xr::schemas::CommandTypes::AssignUserIdToNodes> {
    using data_type = EntityMap<uint32_t>;
  };
  template <>
  struct DataSelector<android_xr::schemas::CommandTypes::AddOrUpdateColliders> {
    using data_type = EntityMap<AddOrUpdateColliderInfo>;
  };
  template <>
  struct DataSelector<android_xr::schemas::CommandTypes::RemoveColliders> {
    using data_type = EntityMap<android_xr::schemas::ColliderType>;
  };

  template <android_xr::schemas::CommandTypes CommandT>
  struct Batch : public CommandBatchBase {
    using T = typename DataSelector<CommandT>::data_type;

   public:
    Batch(int i) : CommandBatchBase(CommandT, i) {}
    void Serialize(flatbuffers::FlatBufferBuilder& fbb) override;
    T data;
  };

  // Returns the FlatBufferBuilderPtr for the given batch.
  // If there is no current builder, a new one will be created.
  // TODO: (broken link) - Use OwnedPtr and BorrowedPtr for CommandBatchBase when
  // they support implicit upcast and static_cast.
  flatbuffers::FlatBufferBuilder* /*absl_nonnull*/ GetFlatBufferBuilderFor(
      CommandBatchBase& batch);

  // Note: In the best case scenario we would only create a flatbuffer builder
  // when we want to send a message to the bridge, in which case we could work
  // with a local variable instead of storing it. However, since some commands
  // (like SetMaterialParameters) store their data as a flatbuffer, we have to
  // create the builder when the command is first called, and store it until the
  // message is sent.
  // We do not store the builder as part of the Batch class to emphasize that
  // the Batch data should be independent of the builder.
  // The batch pointers are owned by batch_queue_.
  absl::flat_hash_map<CommandBatchBase* /*absl_nonnull*/,
                      /*absl_nonnull*/ FlatBufferBuilderPtr>
      fbb_;

  // Stores batches of commands in the order they were created. This ensures
  // that commands are executed in the general order intended by the app.
  // NOTE: In theory, we don't need to execute commands in the order they came
  // in, as long as we run commands affecting the same dependency in the correct
  // order. However, running commands in this order appears more correct.
  std::queue</*absl_nonnull*/ std::unique_ptr<CommandBatchBase>> batch_queue_;

  // Queue of batches that are executed at the end of the frame. This is used
  // for removing resources, as it is unsafe to remove resources while they are
  // potentially still in use.
  enum class RemoveResourceChannel : uint8_t {
    kMesh = 0,
    kMaterialInstance = 1,
    kMaterial = 2,
    kTexture = 3,
    kCount = 4,
  };
  static constexpr size_t kRemoveResourceChannelCount =
      static_cast<size_t>(RemoveResourceChannel::kCount);
  std::array<std::unique_ptr<CommandBatchBase>, kRemoveResourceChannelCount>
      end_of_frame_batches_;

  // This data structure allows for quick lookup of batches by type.
  absl::flat_hash_map<android_xr::schemas::CommandTypes,
                      std::vector<CommandBatchBase* /*absl_nonnull*/>>
      batches_;

  // Stores the index of the last batch that affected a given entity.
  // This is used to ensure the order of dependent commands.
  EntityMap<int /*batch_index*/> last_batch_idx_affecting_entity_;

  // We need to store resource IDs separately, because they do not use the same
  // ID system as entities. While ID collisions do not break our algorithm, they
  // do lead to the creation of more batches than necessary.
  absl::flat_hash_map<ResourceId, int /*batch_index*/>
      last_batch_idx_affecting_resource_;

  // When a command is added to a batch, we use this helper method to store that
  // this is the last batch that affected the given entities and resources.
  void StoreAffectedDependenciesBatchIdx(
      const std::vector<utils::Entity>& entity_dependencies,
      const std::vector<ResourceId>& resource_dependencies, int batch_idx);

  // Returns the first batch of the given type that runs after the last batch
  // that affected the given dependencies. Returns nullptr if no such batch
  // exists.
  CommandBatchBase* /*absl_nullable*/ FindBatch(
      android_xr::schemas::CommandTypes command,
      const std::vector<utils::Entity>& entity_dependencies = {},
      const std::vector<ResourceId>& resource_dependencies = {});

  // Adds the given batch to the queue and returns a pointer to it.
  CommandBatchBase* /*absl_nonnull*/ AddBatch(
      /*absl_nonnull*/ std::unique_ptr<CommandBatchBase> batch,
      const std::vector<utils::Entity>& entity_dependencies = {},
      const std::vector<ResourceId>& resource_dependencies = {});

  // Returns either an existing batch that keeps dependent commands in order, or
  // a new batch if no such batch exists.
  template <android_xr::schemas::CommandTypes CommandT>
  Batch<CommandT>& GetOrCreateBatch(
      const std::vector<utils::Entity>& entity_dependencies = {},
      const std::vector<ResourceId>& resource_dependencies = {});

  // Returns a batch for the given command type.
  template <android_xr::schemas::CommandTypes CommandT>
  Batch<CommandT>& GetOrCreateEndOfFrameBatch(RemoveResourceChannel channel);

  // Sends a Flatbuffer for the given batch to the bridge.
  void SendMessage(CommandBatchBase* /*absl_nonnull*/ batch_base);

  // Sends all batches in the queue to the bridge and cleans up.
  void SendAllBatches();
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_IMPL_H_
