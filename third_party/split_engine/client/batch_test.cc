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

#include "split_engine/client/batch.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "absl/types/span.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "split_engine/client/types.h"
#include "split_engine/schemas/split_engine_data_generated.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_render_passes_generated.h"

namespace split_engine {
namespace {

using android_xr::schemas::Command;
using android_xr::schemas::CommandTypes;

// Helper to serialize and finish a batch, returning the size of the flatbuffer.
template <CommandTypes CommandT>
size_t SerializeBatch(Batch<CommandT>& batch) {
  flatbuffers::FlatBufferBuilder fbb;
  auto offset = batch.Serialize(fbb);
  if (!offset.has_value()) {
    return 0;
  }
  fbb.Finish(*offset);
  return fbb.GetSize();
}

TEST(BatchTest, SetMaterialParameters) {
  Batch<CommandTypes::SetMaterialParameters> batch(0);
  batch.data[1] = android_xr::schemas::MaterialParametersT();

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, SetBuiltInMaterialParameters) {
  Batch<CommandTypes::SetBuiltInMaterialParameters> batch(0);
  batch.data.push_back(
      android_xr::schemas::BuiltInMaterialInstanceParametersT());

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, DuplicateMaterialInstances) {
  Batch<CommandTypes::DuplicateMaterialInstances> batch(0);
  batch.data.push_back(android_xr::schemas::DuplicateMaterialInstanceT());

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, AddOrUpdateColliders) {
  Batch<CommandTypes::AddOrUpdateColliders> batch(0);
  batch.data[1] = android_xr::schemas::AddOrUpdateColliderT();

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, RemoveColliders) {
  Batch<CommandTypes::RemoveColliders> batch(0);
  batch.data[1] = android_xr::schemas::ColliderType::BoxCollider;

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, RemoveMorphTargetBuffers) {
  Batch<CommandTypes::RemoveMorphTargetBuffers> batch(0);
  batch.data.push_back(1);

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, RemoveMeshData) {
  Batch<CommandTypes::RemoveMeshData> batch(0);
  batch.data.index_buffer_ids.push_back(1);

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, UpdateMeshData) {
  Batch<CommandTypes::UpdateMeshData> batch(0);
  IndexBufferUpdateInfo ib;
  ib.id = 1;
  ib.offset = 0;
  std::array<uint8_t, 4> ib_data = {1, 2, 3, 4};
  ib.buffer = BufferHandle(ib_data, [](absl::Span<const uint8_t>) {});
  batch.data.index_buffers.push_back(std::move(ib));

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, RemoveImageBasedLightingAssets) {
  Batch<CommandTypes::RemoveImageBasedLightingAssets> batch(0);
  batch.data.push_back(1);

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, SetPreferredEnvironmentIblAsset) {
  Batch<CommandTypes::SetPreferredEnvironmentIblAsset> batch(0);
  batch.data.image_based_lighting_asset_id = 42;

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, AddMaterials) {
  Batch<CommandTypes::AddMaterials> batch(0);
  batch.data.push_back(android_xr::schemas::MaterialT());

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, RemoveMaterials) {
  Batch<CommandTypes::RemoveMaterials> batch(0);
  batch.data.push_back(1);

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, AddMaterialInstances) {
  Batch<CommandTypes::AddMaterialInstances> batch(0);
  batch.data[1] = 2;

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, RemoveMaterialInstances) {
  Batch<CommandTypes::RemoveMaterialInstances> batch(0);
  batch.data.push_back(1);

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, RemoveTextures) {
  Batch<CommandTypes::RemoveTextures> batch(0);
  batch.data.push_back(1);

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, AddNodes) {
  Batch<CommandTypes::AddNodes> batch(0);
  batch.data.insert(1);

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, RemoveNodes) {
  Batch<CommandTypes::RemoveNodes> batch(0);
  batch.data.insert(1);

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, AssignUserIdToNodes) {
  Batch<CommandTypes::AssignUserIdToNodes> batch(0);
  batch.data[1] = 42;

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, UpdateNodes) {
  Batch<CommandTypes::UpdateNodes> batch(0);
  batch.data[1] = android_xr::schemas::UpdateNodeT();

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, AddRenderables) {
  Batch<CommandTypes::AddRenderables> batch(0);
  batch.data.push_back(android_xr::schemas::AddRenderableT());

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, RemoveRenderables) {
  Batch<CommandTypes::RemoveRenderables> batch(0);
  batch.data.insert(1);

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, UpdateRenderables) {
  Batch<CommandTypes::UpdateRenderables> batch(0);
  batch.data[1] = android_xr::schemas::UpdateRenderableT();

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, AddTexturePipelineRenderers) {
  Batch<CommandTypes::AddTexturePipelineRenderers> batch(0);
  batch.data[1] = android_xr::schemas::AddTexturePipelineRendererT();

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, RemoveTexturePipelineRenderers) {
  Batch<CommandTypes::RemoveTexturePipelineRenderers> batch(0);
  batch.data.push_back(1);

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, RegisterNamedTextures) {
  Batch<CommandTypes::RegisterNamedTextures> batch(0);
  batch.data[1] = android_xr::schemas::RegisterNamedTextureT();

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, UnregisterNamedTextures) {
  Batch<CommandTypes::UnregisterNamedTextures> batch(0);
  batch.data.push_back(1);

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, UpdateTexturePipelineRenderers) {
  Batch<CommandTypes::UpdateTexturePipelineRenderers> batch(0);
  batch.data[1] = android_xr::schemas::UpdateTexturePipelineRendererT();

  size_t estimated_size = batch.GetSerializedSize();
  size_t actual_size = SerializeBatch(batch);

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, GetSerializedSize_AddMeshDataInfo) {
  AddMeshDataInfo info;
  VertexBufferCreateInfo vb;
  vb.id = 1;
  std::array<uint8_t, 4> vb_data = {1, 2, 3, 4};
  vb.buffer.blocks.push_back(
      {{}, BufferHandle(vb_data, [](absl::Span<const uint8_t>) {}), 4});
  info.vertex_buffers.push_back(std::move(vb));

  IndexBufferCreateInfo ib;
  ib.id = 2;
  std::array<uint8_t, 2> ib_data = {5, 6};
  ib.buffer.buffer = BufferHandle(ib_data, [](absl::Span<const uint8_t>) {});
  info.index_buffers.push_back(std::move(ib));

  size_t estimated_size = GetSerializedSize(info);

  flatbuffers::FlatBufferBuilder fbb;
  auto command_offset = android_xr::schemas::CreateCommand(
      fbb, CommandTypes::AddMeshData, Pack(fbb, info).Union());
  fbb.Finish(command_offset);
  size_t actual_size = fbb.GetSize();

  EXPECT_EQ(estimated_size, actual_size);
}

TEST(BatchTest, GetSerializedSize_AddMorphTargetBuffersInfo) {
  AddMorphTargetBuffersInfo info;
  MorphTargetBufferCreateInfo buf;
  buf.id = 1;
  std::array<uint8_t, 2> pos_data = {1, 2};
  std::array<uint8_t, 2> tan_data = {3, 4};
  buf.buffer.targets.push_back(MorphTargetAttributeInfoCreateInfo{
      .positions = BufferHandle(pos_data, [](absl::Span<const uint8_t>) {}),
      .tangents = BufferHandle(tan_data, [](absl::Span<const uint8_t>) {})});
  info.morph_target_buffers.push_back(std::move(buf));

  size_t estimated_size = GetSerializedSize(info);

  flatbuffers::FlatBufferBuilder fbb;
  auto command_offset = android_xr::schemas::CreateCommand(
      fbb, CommandTypes::AddMorphTargetBuffers, Pack(fbb, info).Union());
  fbb.Finish(command_offset);
  size_t actual_size = fbb.GetSize();

  EXPECT_EQ(estimated_size, actual_size - 24);
}

TEST(BatchTest, GetSerializedSize_AddTexturesInfo) {
  int32_t api_level = 1;
  AddTexturesInfo info;
  TextureCreateInfo tex;
  tex.id = 1;
  std::array<uint8_t, 4> data = {1, 2, 3, 4};
  tex.pixel_buffers.push_back(
      {BufferHandle(data, [](absl::Span<const uint8_t>) {})});
  info.textures.push_back(std::move(tex));

  size_t estimated_size = GetSerializedSize(info, api_level);

  flatbuffers::FlatBufferBuilder fbb;
  auto command_offset = android_xr::schemas::CreateCommand(
      fbb, CommandTypes::AddTextures, Pack(fbb, info, api_level).Union());
  fbb.Finish(command_offset);
  size_t actual_size = fbb.GetSize();

  EXPECT_EQ(estimated_size, actual_size - 8);
}

}  // namespace
}  // namespace split_engine
