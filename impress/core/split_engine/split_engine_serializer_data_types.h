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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_DATA_TYPES_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_DATA_TYPES_H_

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include "filament/libs/utils/include/utils/Entity.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/common/entity_absl_hasher.h"
#include "core/common/robin_map.h"
#include "core/common/robin_set.h"
#include "core/geometry/shapes/capsule.h"
#include "core/geometry/shapes/sphere.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/render_passes/texture_pipeline_renderer_projection_quad.h"
#include "core/render_passes/texture_pipeline_renderer_state.proto.imp.h"
#include "split_engine/schemas/split_engine_data_generated.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"

namespace imp::split_engine {

namespace SerializerDataTypes {

using ResourceId = std::uint64_t;

// Helper to create a vector of flatbuffers::Offset<T>.
template <typename T>
using VectorOffset = std::vector<flatbuffers::Offset<T>>;

// Helper for creating a map where an entity is the key.
template <typename T>
using EntityMap = imp::RobinMap<utils::Entity, T, EntityHasher>;
using EntitySet = imp::RobinSet<utils::Entity, EntityHasher>;

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
  std::optional<std::vector<std::string>> groups;
};

struct MorphTargetInfo {
  uint64_t morph_target_buffer_offset = 0;
  uint64_t morph_target_buffer_count = 0;
};

struct RenderableFlags {
  std::optional<android_xr::schemas::Bool> culling_enabled;
};

struct AddTexturePipelineRendererInfo {
  TexturePipelineRendererState state;
};
struct UpdateTexturePipelineRendererInfo {
  std::optional<std::vector<bool>> enabled_passes;
  std::optional<std::optional<TexturePipelineRendererProjectionQuad>>
      projection_quad;
};
struct RegisterNamedTextureInfo {
  std::string name;
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
  imp::RobinMap<uint32_t, PrimitiveUpdateInfo> primitives;
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

struct DuplicateMaterialInstanceInfo {
  ResourceId copy_id;
  ResourceId instance_id;
};

// A batch is a collection of commands that will be serialized together. All
// commands within the same batch need to be of the same type, because the
// flatbuffer root is Command.
struct CommandBatchBase {
  CommandBatchBase(android_xr::schemas::CommandTypes type, int index)
      : type(type), index(index) {}
  virtual ~CommandBatchBase() = default;
  virtual std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
  Serialize(flatbuffers::FlatBufferBuilder& fbb) = 0;

  const android_xr::schemas::CommandTypes type;

  // The index of a batch is its place in the execution order. The batch with
  // index 0 will be executed first.
  int index = 0;
};

// Each command type requires a different type of data. This struct selects
// the correct type for a given command type.
template <android_xr::schemas::CommandTypes CommandT>
struct DataSelector {};

template <>
struct DataSelector<android_xr::schemas::CommandTypes::AddRenderables> {
  using data_type =
      SerializerDataTypes::EntityMap<SerializerDataTypes::AddRenderableInfo>;
};
template <>
struct DataSelector<android_xr::schemas::CommandTypes::RemoveRenderables> {
  using data_type = SerializerDataTypes::EntitySet;
};
template <>
struct DataSelector<android_xr::schemas::CommandTypes::UpdateRenderables> {
  using data_type =
      SerializerDataTypes::EntityMap<SerializerDataTypes::UpdateRenderableInfo>;
};
template <>
struct DataSelector<android_xr::schemas::CommandTypes::RemoveMeshData> {
  using data_type = SerializerDataTypes::RemoveMeshBuffers;
};
template <>
struct DataSelector<
    android_xr::schemas::CommandTypes::RemoveMorphTargetBuffers> {
  using data_type = std::vector<SerializerDataTypes::ResourceId>;
};
template <>
struct DataSelector<
    android_xr::schemas::CommandTypes::SetPreferredEnvironmentIblAsset> {
  using data_type = std::optional<SerializerDataTypes::EnvironmentLightParams>;
};
template <>
struct DataSelector<
    android_xr::schemas::CommandTypes::RemoveImageBasedLightingAssets> {
  using data_type = std::vector<SerializerDataTypes::ResourceId>;
};
template <>
struct DataSelector<android_xr::schemas::CommandTypes::AddMaterials> {
  using data_type =
      SerializerDataTypes::VectorOffset<android_xr::schemas::Material>;
};
template <>
struct DataSelector<android_xr::schemas::CommandTypes::SetMaterialParameters> {
  using data_type = std::unordered_map<SerializerDataTypes::ResourceId,
                                       SerializerDataTypes::MaterialParameters>;
};
template <>
struct DataSelector<android_xr::schemas::CommandTypes::RemoveMaterials> {
  using data_type = std::vector<SerializerDataTypes::ResourceId>;
};
template <>
struct DataSelector<android_xr::schemas::CommandTypes::AddMaterialInstances> {
  using data_type = std::unordered_map<SerializerDataTypes::ResourceId,
                                       SerializerDataTypes::ResourceId>;
};
template <>
struct DataSelector<
    android_xr::schemas::CommandTypes::DuplicateMaterialInstances> {
  using data_type =
      std::vector<SerializerDataTypes::DuplicateMaterialInstanceInfo>;
};
template <>
struct DataSelector<
    android_xr::schemas::CommandTypes::SetBuiltInMaterialParameters> {
  using data_type = SerializerDataTypes::VectorOffset<
      android_xr::schemas::BuiltInMaterialInstanceParameters>;
};
template <>
struct DataSelector<
    android_xr::schemas::CommandTypes::RemoveMaterialInstances> {
  using data_type = std::vector<SerializerDataTypes::ResourceId>;
};
template <>
struct DataSelector<android_xr::schemas::CommandTypes::RemoveTextures> {
  using data_type = std::vector<SerializerDataTypes::ResourceId>;
};
template <>
struct DataSelector<android_xr::schemas::CommandTypes::AddNodes> {
  using data_type = SerializerDataTypes::EntitySet;
};
template <>
struct DataSelector<android_xr::schemas::CommandTypes::RemoveNodes> {
  using data_type = SerializerDataTypes::EntitySet;
};
template <>
struct DataSelector<android_xr::schemas::CommandTypes::UpdateNodes> {
  using data_type =
      SerializerDataTypes::EntityMap<SerializerDataTypes::UpdateNodeInfo>;
};
template <>
struct DataSelector<android_xr::schemas::CommandTypes::AssignUserIdToNodes> {
  using data_type = SerializerDataTypes::EntityMap<uint32_t>;
};
template <>
struct DataSelector<android_xr::schemas::CommandTypes::AddOrUpdateColliders> {
  using data_type = SerializerDataTypes::EntityMap<
      SerializerDataTypes::AddOrUpdateColliderInfo>;
};
template <>
struct DataSelector<android_xr::schemas::CommandTypes::RemoveColliders> {
  using data_type =
      SerializerDataTypes::EntityMap<android_xr::schemas::ColliderType>;
};
template <>
struct DataSelector<
    android_xr::schemas::CommandTypes::AddTexturePipelineRenderers> {
  using data_type = SerializerDataTypes::EntityMap<
      SerializerDataTypes::AddTexturePipelineRendererInfo>;
};
template <>
struct DataSelector<
    android_xr::schemas::CommandTypes::RemoveTexturePipelineRenderers> {
  using data_type = std::vector<utils::Entity>;
};
template <>
struct DataSelector<
    android_xr::schemas::CommandTypes::UpdateTexturePipelineRenderers> {
  using data_type = SerializerDataTypes::EntityMap<
      SerializerDataTypes::UpdateTexturePipelineRendererInfo>;
};
template <>
struct DataSelector<android_xr::schemas::CommandTypes::RegisterNamedTextures> {
  using data_type =
      imp::RobinMap<uint64_t, SerializerDataTypes::RegisterNamedTextureInfo>;
};
template <>
struct DataSelector<
    android_xr::schemas::CommandTypes::UnregisterNamedTextures> {
  using data_type = std::vector<SerializerDataTypes::ResourceId>;
};

// Batch uses the DataSelector to determine the type of data to store for a
// given command type.
template <android_xr::schemas::CommandTypes CommandT>
struct Batch : public CommandBatchBase {
 public:
  Batch(int i) : CommandBatchBase(CommandT, i) {}
  std::optional<flatbuffers::Offset<android_xr::schemas::Command>> Serialize(
      flatbuffers::FlatBufferBuilder& fbb) override;
  typename DataSelector<CommandT>::data_type data;
};
}  // namespace SerializerDataTypes

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_DATA_TYPES_H_
