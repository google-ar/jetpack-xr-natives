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

#ifndef THIRD_PARTY_SPLIT_ENGINE_CLIENT_SPLIT_ENGINE_IPC_CLIENT_H_
#define THIRD_PARTY_SPLIT_ENGINE_CLIENT_SPLIT_ENGINE_IPC_CLIENT_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "absl/functional/any_invocable.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "split_engine/client/batch.h"
#include "split_engine/client/batch_manager.h"
#include "split_engine/client/transport.h"
#include "split_engine/client/types.h"
#include "split_engine/schemas/split_engine_data_generated.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"
#include "split_engine/schemas/split_engine_render_passes_generated.h"

namespace split_engine {

// The SplitEngineIpcClient is responsible for serializing the data required to
// render the scene into a flatbuffer format that can be sent to the
// SplitEngineRenderer on the backend.
class SplitEngineIpcClient {
 public:
  // Initializes the client with the supported IPC API level, the transport
  // layer for sending flatbuffer messages to the remote renderer, a material
  // requester for handling material asset dependencies, and the size of the
  // bridge buffer allocated for IPC transactions.
  SplitEngineIpcClient(int32_t api_level, std::unique_ptr<Transport> transport);

  SplitEngineIpcClient(const SplitEngineIpcClient&) = delete;
  SplitEngineIpcClient(SplitEngineIpcClient&&) = delete;

  SplitEngineIpcClient& operator=(const SplitEngineIpcClient&) = delete;
  SplitEngineIpcClient& operator=(SplitEngineIpcClient&&) = delete;

  // Returns the maximum negotiated IPC API level supported by the remote
  // renderer.
  int32_t GetApiLevel() const;

  // Allocates a renderable component associated with the specified scene node
  // on the remote renderer.
  void AddRenderable(
      EntityId node_id, uint32_t primitive_count, uint32_t skinning_bone_count,
      std::unique_ptr<android_xr::schemas::MorphTargetDataT> morph_target_data,
      std::unique_ptr<android_xr::schemas::RenderableFlagsT> renderable_flags);

  // Batches a command to remove the renderable component associated with the
  // specified scene node on the remote renderer.
  void DestroyRenderable(EntityId node_id);

  // Updates the material instance bound to a specific primitive index of a
  // renderable node on the remote renderer.
  void SetMaterialInstanceAt(EntityId node_id, size_t primitive_index,
                             ResourceId material_instance_id);

  // Configures the geometric mesh data (vertices, indices, drawing primitive
  // type, and draw range) for a specific primitive index of a renderable node
  // on the remote renderer.
  void SetGeometryAt(EntityId node_id, size_t primitiveIndex, uint8_t type,
                     ResourceId vertex_buffer_id, ResourceId index_buffer_id,
                     uint32_t offset, uint32_t count);

  // Sets the local axis-aligned bounding box for a renderable node, used for
  // culling and bounds calculations on the remote renderer.
  void SetAxisAlignedBoundingBox(EntityId node_id,
                                 android_xr::schemas::Box aabb);

  // Sets the rendering priority for a renderable node to control draw ordering
  // among coplanar or overlapping elements.
  void SetPriority(EntityId node_id, uint8_t priority);

  // Configures the visibility layer mask for a renderable node, determining
  // which camera views or render passes can see this object.
  void SetLayerMask(EntityId node_id, uint8_t select, uint8_t values);

  // Defines the explicit sorting order for a transparent primitive within a
  // renderable node, overriding default depth sorting.
  void SetBlendOrderAt(EntityId node_id, size_t primitiveIndex, uint16_t order);

  // Enables or disables global blend ordering for a primitive, allowing its
  // blend order to be sorted against all other transparent objects in the
  // scene rather than just locally.
  void SetGlobalBlendOrderEnabledAt(EntityId node_id, size_t primitiveIndex,
                                    bool enabled);

  // Sets the morph target weights for a skinned renderable node.
  void SetMorphWeights(EntityId node_id, std::vector<float> weights,
                       size_t offset);

  // Updates the skeletal animation bone transformation matrices for a skinned
  // renderable node.
  void SetBones(EntityId node_id,
                std::vector<android_xr::schemas::Mat4f> transforms);

  // Batches a command to remove a previously registered material asset from the
  // remote renderer's resource cache.
  void RemoveMaterial(ResourceId material_id);

  // Instantiates a material instance from a parent material asset on the remote
  // renderer.
  void AddMaterialInstance(ResourceId material_id, ResourceId instance_id);

  // Creates an exact duplicate of an existing material instance on the remote
  // renderer, preserving all previously configured parameter values.
  void DuplicateMaterialInstance(ResourceId instance_id, ResourceId copy_id);

  // Batches a command to destroy a material instance on the remote renderer.
  void RemoveMaterialInstance(ResourceId instance_id);

  // Sets a scalar or vector parameter value by name on a specific material
  // instance.
  void SetMaterialParameter(ResourceId material_id, std::string name,
                            android_xr::schemas::MaterialParamValueUnion value);

  // Binds a texture resource and its sampling configuration (filtering, wrap
  // modes) to a parameter name on a material instance.
  void SetMaterialParameter(
      ResourceId material_id, std::string name, ResourceId texture_id,
      std::unique_ptr<android_xr::schemas::TextureSamplerT> texture_sampler);

  // Allocates a new scene graph node on the remote renderer corresponding to a
  // local entity.
  void CreateNode(EntityId node_id);

  // Batches a command to destroy a scene graph node on the remote renderer,
  // ensuring all dependent child nodes or attached components are processed in
  // the correct order.
  void DestroyNode(EntityId node_id, const std::vector<EntityId>& dependencies);

  // Activates or deactivates a scene graph node on the remote renderer,
  // toggling visibility and processing for it and its subtree.
  void SetEnabled(EntityId node_id, bool enabled);

  // Assigns a debugging name to a scene graph node on the remote renderer.
  void SetName(EntityId node_id, std::string name);

  // Attaches a scene graph node as a child of another node on the remote
  // renderer, establishing hierarchical transform inheritance.
  void SetParent(EntityId node_id, EntityId parent);

  // Sets the single-precision local transformation matrix (relative to its
  // parent) for a scene graph node.
  void SetLocalTransform(EntityId node_id,
                         android_xr::schemas::Mat4f transform);

  // Sets the double-precision local transformation matrix (relative to its
  // parent) for a scene graph node.
  void SetLocalTransform(EntityId node_id, android_xr::schemas::Mat4 transform);

  // Assigns categorical group labels to a scene graph node on the remote
  // renderer for filtering and querying.
  void SetGroups(EntityId node_id, std::vector<std::string> groups);

  // Associates an application-defined user identifier with a scene graph node
  // for app-level tracking on the remote renderer.
  void AssignUserId(EntityId node_id, uint32_t user_id);

  // Attaches or updates an oriented box collision geometry component on a scene
  // graph node.
  void SetBoxCollider(EntityId node_id, android_xr::schemas::Box box,
                      bool enabled);

  // Attaches or updates a mesh-based collision geometry component on a scene
  // graph node.
  void SetMeshCollider(EntityId node_id, bool enabled);

  // Attaches or updates a spherical collision geometry component on a scene
  // graph node.
  void SetSphereCollider(EntityId node_id,
                         android_xr::schemas::SphereColliderT sphere,
                         bool enabled);

  // Attaches or updates a capsule collision geometry component on a scene graph
  // node.
  void SetCapsuleCollider(EntityId node_id,
                          android_xr::schemas::CapsuleColliderT capsule,
                          bool enabled);

  // Removes a specific collision geometry component type from a scene graph
  // node on the remote renderer.
  void ClearCollider(EntityId node_id,
                     android_xr::schemas::ColliderType collider_type);

  // Packages and transmits texture asset data buffers to the remote renderer,
  // executing a callback upon successful IPC transfer.
  void AddTextures(AddTexturesInfo add_textures,
                   absl::AnyInvocable<void()> on_done);

  // Batches a command to release and destroy a texture asset on the remote
  // renderer.
  void RemoveTexture(ResourceId texture_id);

  // Transmits unpacked vertex and index buffer geometry definitions to the
  // remote renderer for creating mesh assets.
  void AddMeshData(AddMeshDataInfo mesh_data);

  // Sends sparse updates to existing vertex and/or index buffers on the remote
  // renderer.
  void UpdateMeshData(std::vector<VertexBufferUpdateInfo> vertex_buffer_updates,
                      std::vector<IndexBufferUpdateInfo> index_buffer_updates);

  // Transmits vertex morph target buffer definitions to the remote renderer for
  // facial or blend-shape animations.
  void AddMorphTargetBuffers(AddMorphTargetBuffersInfo morph_target_buffers);

  // Registers a compiled Filament material asset and its precompile constant
  // options with the remote renderer using a unique tracking identifier.
  void AddCompiledFilamentMaterial(android_xr::schemas::MaterialT material);

  // Asynchronously requests the creation of a standard built-in material
  // instance on the remote renderer. Returns a future representing the
  // completion status of the IPC request.
  void RequestBuiltInMaterialInstance(
      android_xr::schemas::BuiltInMaterialRequestT request,
      absl::AnyInvocable<void(absl::Status)> response_callback);

  // Asynchronously requests the compilation and creation of a customized
  // material asset on the remote renderer. Returns a future representing the
  // completion status of the IPC request.
  void AddCustomMaterial(
      android_xr::schemas::AddCustomMaterialRequestT request,
      absl::AnyInvocable<void(absl::Status)> response_callback);

  // Customizes the parameter payload for a built-in material instance by
  // executing a caller-provided flatbuffer serialization function.
  void SetBuiltInMaterialParameters(
      android_xr::schemas::BuiltInMaterialInstanceParametersT parameters);

  // Serializes environment cubemaps and spherical harmonics data to create
  // image-based lighting (IBL) assets on the remote renderer.
  void AddImageBasedLightingAsset(
      android_xr::schemas::AddImageBasedLightingAssetsT add_assets);

  // Batches a command to remove an image-based lighting asset from the remote
  // renderer.
  void RemoveImageBasedLightingAsset(ResourceId reflection_texture_id);

  // Sets the active environmental image-based lighting asset, intensity scale,
  // and color tint for the rendered scene.
  void SetPreferredEnvironmentIblAsset(ResourceId reflection_texture_id,
                                       float intensity,
                                       android_xr::schemas::Float3 tint);

  // Unbinds the active environmental image-based lighting asset, resetting
  // scene illumination to default.
  void ClearPreferredEnvironmentIblAsset();

  // Batches a command to release a morph target animation buffer on the remote
  // renderer.
  void RemoveMorphTargetBuffer(ResourceId morph_target_buffer_id);

  // Batches a command to release a vertex geometry buffer on the remote
  // renderer.
  void RemoveVertexBuffer(ResourceId vertex_buffer_id);

  // Batches a command to release an index geometry buffer on the remote
  // renderer.
  void RemoveIndexBuffer(ResourceId index_buffer_id);

  // Attaches a texture pipeline renderer component to a scene node on the
  // remote renderer, configuring its initial rendering state.
  void AddTexturePipelineRenderer(
      EntityId node_id,
      std::unique_ptr<android_xr::schemas::TexturePipelineRendererT> state);

  // Batches a command to remove a texture pipeline renderer component from a
  // scene node on the remote renderer.
  void RemoveTexturePipelineRenderer(EntityId node_id);

  // Updates the active execution state of specific render passes within a
  // texture pipeline renderer component.
  void SetTexturePipelineRendererPassesEnabled(
      EntityId node_id, std::vector<bool> enabled_passes);

  // Configures or updates the projection quad geometry used by a texture
  // pipeline renderer component for rendering.
  void SetTexturePipelineRendererProjectionQuad(
      EntityId node_id,
      std::unique_ptr<android_xr::schemas::ProjectionQuadT> quad);

  // Registers a symbolic name for a texture ID, enabling the remote renderer to
  // resolve and bind textures generated dynamically by backend pipelines.
  void RegisterNamedTexture(ResourceId texture_id, std::string name);

  // Removes a previously registered named texture mapping from the remote
  // renderer.
  void UnregisterNamedTexture(ResourceId texture_id);

  // Sends all batches in the queue to the bridge and cleans up.
  void SendAllBatches();

  // Returns the number of currently active/in-flight frame updates.
  absl::StatusOr<int32_t> GetActiveFrameUpdatesCount() const;

 private:
  // The maximum API level that the serializer is allowed to serialize.
  const int32_t api_level_;

  // The transport used for sending messages to the split engine renderer.
  const std::unique_ptr<Transport> transport_;

  SplitEngineSerializerBatchManager batch_manager_;

  android_xr::schemas::UpdateRenderableT&
  GetOrCreateUpdateRenderableCmdForEntity(
      EntityId node_id,
      const std::vector<ResourceId>& resource_dependencies = {});

  android_xr::schemas::UpdateNodeT& GetOrCreateUpdateNodeCmdForEntity(
      EntityId node_id, const std::vector<EntityId>& entity_dependencies = {},
      const std::vector<ResourceId>& resource_dependencies = {});

  android_xr::schemas::AddOrUpdateColliderT&
  GetOrCreateAddOrUpdateColliderCmdForEntity(EntityId node_id);

  // Serializes the given batch on the transport.
  void SerializeBatch(MessageGroupId message_group_id, BatchBase& batch_base);
};

}  // namespace split_engine

#endif  // THIRD_PARTY_SPLIT_ENGINE_CLIENT_SPLIT_ENGINE_IPC_CLIENT_H_
