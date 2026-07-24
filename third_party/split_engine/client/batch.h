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

#ifndef THIRD_PARTY_SPLIT_ENGINE_CLIENT_BATCH_H_
#define THIRD_PARTY_SPLIT_ENGINE_CLIENT_BATCH_H_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/types/span.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "split_engine/client/types.h"
#include "split_engine/schemas/split_engine_data_generated.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_render_passes_generated.h"

namespace split_engine {

// Buffer handle that will call the release callback when it is destroyed.
using BufferHandle = WithDestructionCallback<absl::Span<const uint8_t>>;

struct VertexBlockUpdateInfo {
  uint8_t block_index;
  uint32_t offset;
  BufferHandle buffer;
};

struct VertexBufferUpdateInfo {
  ResourceId id;
  std::vector<VertexBlockUpdateInfo> block_updates;
};

struct IndexBufferUpdateInfo {
  ResourceId id;
  uint32_t offset;
  BufferHandle buffer;
};

struct UpdateMeshDataInfo {
  std::vector<VertexBufferUpdateInfo> vertex_buffers;
  std::vector<IndexBufferUpdateInfo> index_buffers;
};

struct VertexBlockInfoCreateInfo {
  std::vector<android_xr::schemas::VertexAttributeInfo> attributes;
  BufferHandle buffer;
  uint32_t stride = 0;
};

struct VertexBufferInfoCreateInfo {
  std::vector<VertexBlockInfoCreateInfo> blocks;
  uint64_t vertex_count = 0;
  bool advanced_skinning = false;
};

struct VertexBufferCreateInfo {
  ResourceId id = 0;
  VertexBufferInfoCreateInfo buffer;
  android_xr::schemas::VertexAccessFlags vertex_access_flags =
      android_xr::schemas::VertexAccessFlags::NONE;
};

struct IndexBufferInfoCreateInfo {
  android_xr::schemas::IndexType type = android_xr::schemas::IndexType::USHORT;
  BufferHandle buffer;
};

struct IndexBufferCreateInfo {
  ResourceId id = 0;
  IndexBufferInfoCreateInfo buffer;
  bool store_index_data = false;
};

struct AddMeshDataInfo {
  std::vector<VertexBufferCreateInfo> vertex_buffers;
  std::vector<IndexBufferCreateInfo> index_buffers;
};

struct MorphTargetAttributeInfoCreateInfo {
  BufferHandle positions;
  BufferHandle tangents;
};

struct MorphTargetBufferInfoCreateInfo {
  std::vector<MorphTargetAttributeInfoCreateInfo> targets;
  uint64_t vertex_count = 0;
  bool enable_custom_morphing = false;
};

struct MorphTargetBufferCreateInfo {
  ResourceId id = 0;
  MorphTargetBufferInfoCreateInfo buffer;
};

struct AddMorphTargetBuffersInfo {
  std::vector<MorphTargetBufferCreateInfo> morph_target_buffers;
};

struct PixelBufferCreateInfo {
  BufferHandle buffer;
};

struct TextureCreateInfo {
  ResourceId id = 0;
  uint32_t width = 0;
  uint32_t height = 0;
  android_xr::schemas::TextureFormat format =
      android_xr::schemas::TextureFormat::R8;
  uint8_t levels = 0;
  android_xr::schemas::SamplerType sampler =
      android_xr::schemas::SamplerType::SAMPLER_2D;
  bool mips = false;
  std::vector<android_xr::schemas::ImageParamsT> image_params;
  std::vector<PixelBufferCreateInfo> pixel_buffers;
  std::string name;
};

struct AddTexturesInfo {
  std::vector<TextureCreateInfo> textures;
};

// Helper for creating a map where an entity is the key.
template <typename T>
using EntityMap = absl::flat_hash_map<EntityId, T>;

using EntitySet = absl::flat_hash_set<EntityId>;

template <typename T>
using ResourceMap = absl::flat_hash_map<ResourceId, T>;

// A batch is a collection of commands that will be serialized together. All
// commands within the same batch need to be of the same type, because the
// flatbuffer root is Command.
struct BatchBase {
  BatchBase(android_xr::schemas::CommandTypes type, int index)
      : type(type), index(index) {}
  virtual ~BatchBase() = default;
  virtual std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
  Serialize(flatbuffers::FlatBufferBuilder& fbb) = 0;

  virtual size_t GetSerializedSize() const = 0;

  const android_xr::schemas::CommandTypes type;

  // The index of a batch is its place in the execution order. The batch with
  // index 0 will be executed first.
  int index = 0;
};

// Each command type requires a different type of data. This struct selects
// the correct type for a given command type.
template <android_xr::schemas::CommandTypes CommandT>
struct BatchDataType {};
template <>
struct BatchDataType<android_xr::schemas::CommandTypes::AddRenderables> {
  using type = std::vector<android_xr::schemas::AddRenderableT>;
};
template <>
struct BatchDataType<android_xr::schemas::CommandTypes::RemoveRenderables> {
  using type = EntitySet;
};
template <>
struct BatchDataType<android_xr::schemas::CommandTypes::UpdateRenderables> {
  using type = EntityMap<android_xr::schemas::UpdateRenderableT>;
};
template <>
struct BatchDataType<android_xr::schemas::CommandTypes::RemoveMeshData> {
  using type = android_xr::schemas::RemoveMeshDataT;
};
template <>
struct BatchDataType<android_xr::schemas::CommandTypes::UpdateMeshData> {
  using type = UpdateMeshDataInfo;
};
template <>
struct BatchDataType<
    android_xr::schemas::CommandTypes::RemoveMorphTargetBuffers> {
  using type = std::vector<ResourceId>;
};
template <>
struct BatchDataType<
    android_xr::schemas::CommandTypes::SetPreferredEnvironmentIblAsset> {
  using type = android_xr::schemas::SetPreferredEnvironmentIblAssetT;
};
template <>
struct BatchDataType<
    android_xr::schemas::CommandTypes::RemoveImageBasedLightingAssets> {
  using type = std::vector<ResourceId>;
};
template <>
struct BatchDataType<android_xr::schemas::CommandTypes::AddMaterials> {
  using type = std::vector<android_xr::schemas::MaterialT>;
};
template <>
struct BatchDataType<android_xr::schemas::CommandTypes::SetMaterialParameters> {
  using type = ResourceMap<android_xr::schemas::MaterialParametersT>;
};
template <>
struct BatchDataType<android_xr::schemas::CommandTypes::RemoveMaterials> {
  using type = std::vector<ResourceId>;
};
template <>
struct BatchDataType<android_xr::schemas::CommandTypes::AddMaterialInstances> {
  using type = ResourceMap<ResourceId>;
};
template <>
struct BatchDataType<
    android_xr::schemas::CommandTypes::DuplicateMaterialInstances> {
  using type = std::vector<android_xr::schemas::DuplicateMaterialInstanceT>;
};
template <>
struct BatchDataType<
    android_xr::schemas::CommandTypes::SetBuiltInMaterialParameters> {
  using type =
      std::vector<android_xr::schemas::BuiltInMaterialInstanceParametersT>;
};
template <>
struct BatchDataType<
    android_xr::schemas::CommandTypes::RemoveMaterialInstances> {
  using type = std::vector<ResourceId>;
};
template <>
struct BatchDataType<android_xr::schemas::CommandTypes::RemoveTextures> {
  using type = std::vector<ResourceId>;
};
template <>
struct BatchDataType<android_xr::schemas::CommandTypes::AddNodes> {
  using type = EntitySet;
};
template <>
struct BatchDataType<android_xr::schemas::CommandTypes::RemoveNodes> {
  using type = EntitySet;
};
template <>
struct BatchDataType<android_xr::schemas::CommandTypes::UpdateNodes> {
  using type = EntityMap<android_xr::schemas::UpdateNodeT>;
};
template <>
struct BatchDataType<android_xr::schemas::CommandTypes::AssignUserIdToNodes> {
  using type = EntityMap<uint32_t>;
};
template <>
struct BatchDataType<android_xr::schemas::CommandTypes::AddOrUpdateColliders> {
  using type = EntityMap<android_xr::schemas::AddOrUpdateColliderT>;
};
template <>
struct BatchDataType<android_xr::schemas::CommandTypes::RemoveColliders> {
  using type = EntityMap<android_xr::schemas::ColliderType>;
};
template <>
struct BatchDataType<
    android_xr::schemas::CommandTypes::AddTexturePipelineRenderers> {
  using type = EntityMap<android_xr::schemas::AddTexturePipelineRendererT>;
};
template <>
struct BatchDataType<
    android_xr::schemas::CommandTypes::RemoveTexturePipelineRenderers> {
  using type = std::vector<EntityId>;
};
template <>
struct BatchDataType<
    android_xr::schemas::CommandTypes::UpdateTexturePipelineRenderers> {
  using type = EntityMap<android_xr::schemas::UpdateTexturePipelineRendererT>;
};
template <>
struct BatchDataType<android_xr::schemas::CommandTypes::RegisterNamedTextures> {
  using type = EntityMap<android_xr::schemas::RegisterNamedTextureT>;
};
template <>
struct BatchDataType<
    android_xr::schemas::CommandTypes::UnregisterNamedTextures> {
  using type = std::vector<ResourceId>;
};

// Batch uses the BatchDataType to determine the type of data to store for a
// given command type.
template <android_xr::schemas::CommandTypes CommandT>
struct Batch : public BatchBase {
 public:
  Batch(int i) : BatchBase(CommandT, i) {}
  std::optional<flatbuffers::Offset<android_xr::schemas::Command>> Serialize(
      flatbuffers::FlatBufferBuilder& fbb) override;
  size_t GetSerializedSize() const override;
  typename BatchDataType<CommandT>::type data;
};

// Helper functions for zero-copy Flatbuffer packing and size estimation.
size_t GetSerializedSize(const AddMeshDataInfo& info);
flatbuffers::Offset<android_xr::schemas::AddMeshData> Pack(
    flatbuffers::FlatBufferBuilder& fbb, const AddMeshDataInfo& info);

size_t GetSerializedSize(const AddMorphTargetBuffersInfo& info);
flatbuffers::Offset<android_xr::schemas::AddMorphTargetBuffers> Pack(
    flatbuffers::FlatBufferBuilder& fbb, const AddMorphTargetBuffersInfo& info);

size_t GetSerializedSize(const AddTexturesInfo& info, int32_t api_level);
flatbuffers::Offset<android_xr::schemas::AddTextures> Pack(
    flatbuffers::FlatBufferBuilder& fbb, const AddTexturesInfo& info,
    int32_t api_level);

}  // namespace split_engine

#endif  // THIRD_PARTY_SPLIT_ENGINE_CLIENT_BATCH_H_
