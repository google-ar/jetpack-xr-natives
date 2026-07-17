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
#include <vector>

#include "absl/base/nullability.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Material.h"
#include "filament/filament/include/filament/MorphTargetBuffer.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/entity_absl_hasher.h"
#include "core/common/invocable.h"
#include "core/common/owned_ptr.h"
#include "core/common/robin_map.h"
#include "core/common/robin_set.h"
#include "core/config.h"
#include "core/geometry/shapes/capsule.h"
#include "core/geometry/shapes/sphere.h"
#include "core/lighting/image_based_lighting_types.h"
#include "core/material_library/generic_material_spec.h"
#include "core/material_library/material_param_value.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/model/mesh/base_mesh_builder.h"
#include "core/ncsb/update_id.h"
#include "core/ncsb/update_phase.h"
#include "core/ncsb/update_system.h"
#include "core/render_passes/texture_pipeline_renderer_state.proto.imp.h"
#include "core/split_engine/android/split_engine_android_bridge.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_mesh_serializer.h"
#include "core/split_engine/split_engine_serializer_batch_manager.h"
#include "core/split_engine/split_engine_serializer_data_types.h"
#include "core/split_engine/split_engine_serializer_transport.h"
#include "core/split_engine/split_engine_texture_serializer.h"
#if IMP_PLATFORM(ANDROID)
#include "core/render/android/android_defines.h"
#include "core/render/android/platform_android_external_texture_surface.h"
#endif
#include "core/render/base_renderable_manager.h"
#include "core/render/base_texture_builder.h"
#include "core/split_engine/materials/split_engine_builtin_material.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"

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
  using UpdateDependencies = UpdateIds<SplitEngineBuiltinMaterialUpdater>;

  // The maximum number of frames that can be in-flight before we stop sending
  // frames to the renderer and wait for the system to catch up.
  static constexpr size_t kMaxInFlightFrames = 10;

  // Constructs a SplitEngineSerializerImpl.
  // The bridge_sender is used to send serialized data to the render using
  // a shared channel, which is initialized with a fixed size buffer of
  // bridge_buffer_size_bytes. The one_shot_bridge_sender sends larger
  // serialized data on a separate channel, allowing optimization of
  // underlying resources for serialization of larger and infrequent data.
  SplitEngineSerializerImpl(
      BaseView& view, int32_t api_level,
      imp::OwnedPtr<SplitEngineSerializerTransport> transport,
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
  void ClearMaterialInstanceAt(filament::RenderableManager::Instance instance,
                               size_t primitiveIndex) override;
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
  bool GetFogEnabled(
      filament::RenderableManager::Instance instance) const override;
  void SetFogEnabled(filament::RenderableManager::Instance instance,
                     bool enable) override;
  size_t GetMorphTargetCount(
      filament::RenderableManager::Instance instance) const override;
  void SetMorphWeights(filament::RenderableManager::Instance instance,
                       float const* weights, size_t count,
                       size_t offset) override;
  bool IsCullingEnabled(
      filament::RenderableManager::Instance instance) const override;

  std::unique_ptr<BaseRenderableManager::Builder> NewBuilder(
      size_t count) override;

  int32_t GetApiLevel() const override;

  SplitEngineAndroidBridge& GetBridge() override;

  bool ReadyForNextFrame() const override;

  void AddMaterial(
      const filament::Material* material, const BufferAccess& data,
      const MaterialPreCompileOptions& material_pre_compile_options) override;
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
  void DestroyNode(utils::Entity entity,
                   const std::vector<utils::Entity>& dependencies) override;
  void SetEnabled(utils::Entity entity, bool enabled) override;
  void SetName(utils::Entity entity, absl::string_view name) override;
  void SetParent(utils::Entity entity, utils::Entity parent) override;
  void SetLocalTransform(utils::Entity entity, const mat4f& transform) override;
  void SetLocalTransform(utils::Entity entity, const mat4& transform) override;
  void SetGroups(utils::Entity entity,
                 absl::Span<const absl::string_view> groups) override;
  void AssignUserId(utils::Entity entity, uint32_t user_id) override;
  std::unique_ptr<BaseTextureBuilder> CreateTextureBuilder() override;
  std::unique_ptr<BaseMeshBuilder> CreateMeshBuilder() override;
#if IMP_PLATFORM(ANDROID)
  Future<std::unique_ptr<PlatformAndroidExternalTextureSurface>>
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
  void SerializeTexture(std::unique_ptr<const SplitEngineTextureSerializer>
                            split_engine_texture_serializer,
                        imp::Invocable<void()> on_done) override;
  void RemoveTexture(filament::Texture& texture) override;
  void SerializeMesh(std::unique_ptr<const SplitEngineMeshSerializer>
                         split_engine_mesh_serializer) override;
  Future<GenericMaterialPtr> CreateGenericMaterial(
      const GenericMaterialSpec& spec) override;
  MaterialPtr CreateCustomMaterial(MaterialPtr material) override;
  Future<absl::Status> RequestCustomFilamentMaterial(
      absl::string_view material_source, filament::Material* filament_material,
      const MaterialPreCompileOptions& precompile_options) override;
  void SetBuiltInMaterialParameters(
      const filament::MaterialInstance* material,
      BuiltInMaterialParameters type,
      SerializeBuiltInMaterialParametersFunc serialize_func) override;
  void SerializeImageBasedLightingAsset(
      filament::Texture& reflection_texture,
      std::unique_ptr<SphericalHarmonics> /*absl_nullable*/  spherical_harmonics,
      ImageBasedLightingAssetCubemapImages cubemap_images) override;
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

  // Texture Pipeline Renderer
  void AddTexturePipelineRenderer(
      utils::Entity entity, const TexturePipelineRendererState& state) override;
  void RemoveTexturePipelineRenderer(utils::Entity entity) override;
  void SetTexturePipelineRendererPassesEnabled(
      utils::Entity entity, const std::vector<bool>& enabled_passes) override;
  void SetTexturePipelineRendererProjectionQuad(
      utils::Entity entity,
      const std::optional<TexturePipelineRendererProjectionQuad>& quad)
      override;
  void RegisterNamedTexture(const filament::Texture& texture,
                            absl::string_view name) override;
  void UnregisterNamedTexture(const filament::Texture& texture) override;

 protected:
  void SetBonesInternal(filament::RenderableManager::Instance instance,
                        filament::RenderableManager::Bone const* transforms,
                        size_t boneCount, size_t offset) override;
  void SetBonesInternal(filament::RenderableManager::Instance instance,
                        mat4f const* transforms, size_t boneCount,
                        size_t offset) override;

 private:
  size_t EstimateImageBasedLightingAssetBufferSize(
      const SphericalHarmonics* /*absl_nullable*/  spherical_harmonics,
      const ImageBasedLightingAssetCubemapImages& cubemap_images);
  void SerializeMeshIndicesAndVertices(
      const SplitEngineMeshSerializer& split_engine_mesh_serializer);
  void SerializeMeshMorphTargets(
      std::unique_ptr<const SplitEngineMeshSerializer>
          split_engine_mesh_serializer);

  using ResourceId = std::uint64_t;

  // Helper to create a vector of flatbuffers::Offset<T>.
  template <typename T>
  using VectorOffset = std::vector<flatbuffers::Offset<T>>;

  // Helper for creating a map where an entity is the key.
  template <typename T>
  using EntityMap = RobinMap<utils::Entity, T, EntityHasher>;
  using EntitySet = RobinSet<utils::Entity, EntityHasher>;

  utils::Entity GetEntity(
      const filament::RenderableManager::Instance& instance) const;

  imp::OwnedPtr<SplitEngineSerializerTransport::MessageBuilder>
  CreateFlatBufferBuilder();
  imp::OwnedPtr<SplitEngineSerializerTransport::MessageBuilder>
  CreateFlatBufferBuilder(size_t size_bytes);

  BaseView& view_;

  // The maximum API level that the serializer is allowed to serialize.
  const int32_t api_level_;

  // The transport used for sending messages to the split engine renderer.
  const imp::OwnedPtr<SplitEngineSerializerTransport> transport_;

  const size_t bridge_buffer_size_bytes_;

  SplitEngineSerializerBatchManager batch_manager_;

  // Borrows a FlatbufferBuilder associated with the given batch.
  //
  // TODO: (broken link) - Use OwnedPtr and BorrowedPtr for CommandBatchBase when
  // they support implicit upcast and static_cast.
  imp::BorrowedPtr<SplitEngineSerializerTransport::MessageBuilder>
  BorrowFlatBufferBuilder(SerializerDataTypes::CommandBatchBase& batch);

  // Transfers ownership of a FlatbufferBuilder associated with the given batch
  // to the caller.
  //
  imp::OwnedPtr<SplitEngineSerializerTransport::MessageBuilder>
  ReleaseFlatBufferBuilder(SerializerDataTypes::CommandBatchBase& batch);

  // Note: In the best case scenario we would only create a flatbuffer builder
  // when we want to send a message to the bridge, in which case we could work
  // with a local variable instead of storing it. However, since some commands
  // (like SetMaterialParameters) store their data as a flatbuffer, we have to
  // create the builder when the command is first called, and store it until the
  // message is sent.
  // We do not store the builder as part of the Batch class to emphasize that
  // the Batch data should be independent of the builder.
  // The batch pointers are owned by batch_queue_.
  absl::flat_hash_map<
      SerializerDataTypes::CommandBatchBase* /*absl_nonnull*/ ,
      imp::OwnedPtr<SplitEngineSerializerTransport::MessageBuilder>>
      fbb_;

  // Sends a Flatbuffer for the given batch to the bridge.
  void SendMessage(
      SerializerDataTypes::CommandBatchBase* /*absl_nonnull*/  batch_base);

  // Sends all batches in the queue to the bridge and cleans up.
  void SendAllBatches();

  // The group ID for the commands in the current frame update.
  // One shot updates uses their own group IDs.
  std::optional<MessageGroupId> frame_update_group_id_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_IMPL_H_
