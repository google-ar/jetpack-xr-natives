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

#include "split_engine/client/split_engine_ipc_client.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "absl/functional/any_invocable.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/status_matchers.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "split_engine/client/batch.h"
#include "split_engine/client/transport.h"
#include "split_engine/client/types.h"
#include "split_engine/schemas/split_engine_data_generated.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"
#include "split_engine/schemas/split_engine_render_passes_generated.h"

namespace split_engine {
namespace {

class FakeSplitEngineSerializerTransport : public Transport {
 public:
  struct SentMessage {
    MessageGroupId group_id;
    std::vector<uint8_t> payload;
  };

  absl::StatusOr<MessageGroupId> BeginFrameUpdate(size_t max_size) override {
    if (!begin_frame_update_status_.ok()) {
      return begin_frame_update_status_;
    }
    active_frame_updates_++;
    return next_group_id_++;
  }

  absl::StatusOr<MessageGroupId> BeginOneShot(size_t max_size) override {
    active_one_shots_++;
    return next_group_id_++;
  }

  OwnedOrBorrowedFlatBufferBuilder CreateBuilder(MessageGroupId id,
                                                 size_t size) override {
    return OwnedOrBorrowedFlatBufferBuilder(
        std::make_unique<flatbuffers::FlatBufferBuilder>(size));
  }

  OwnedOrBorrowedFlatBufferBuilder CreateRequestBuilder(size_t size) override {
    return OwnedOrBorrowedFlatBufferBuilder(
        std::make_unique<flatbuffers::FlatBufferBuilder>(size));
  }

  absl::Status AddMessage(
      MessageGroupId id, OwnedOrBorrowedFlatBufferBuilder builder,
      const flatbuffers::Offset<android_xr::schemas::Command>& offset)
      override {
    builder->Finish(offset);
    uint8_t* buf = builder->GetBufferPointer();
    size_t size = builder->GetSize();
    sent_messages_.push_back({id, std::vector<uint8_t>(buf, buf + size)});
    return absl::OkStatus();
  }

  absl::Status AddMessage(MessageGroupId id,
                          OwnedOrBorrowedFlatBufferBuilder builder,
                          OffsetProducer offset_fn) override {
    auto offset = offset_fn(*builder);
    builder->Finish(offset);
    uint8_t* buf = builder->GetBufferPointer();
    size_t size = builder->GetSize();
    sent_messages_.push_back({id, std::vector<uint8_t>(buf, buf + size)});
    return absl::OkStatus();
  }

  absl::Status End(MessageGroupId id) override {
    if (!end_status_.ok()) {
      return end_status_;
    }
    if (active_frame_updates_ > 0)
      active_frame_updates_--;
    else if (active_one_shots_ > 0)
      active_one_shots_--;
    return absl::OkStatus();
  }

  void SendRequest(
      OwnedOrBorrowedFlatBufferBuilder builder,
      const flatbuffers::Offset<android_xr::schemas::Request>& offset,
      absl::AnyInvocable<
          void(absl::StatusOr<android_xr::schemas::ResponseTypesUnion>)>
          response_handler) override {
    builder->Finish(offset);
    uint8_t* buf = builder->GetBufferPointer();
    size_t size = builder->GetSize();
    sent_requests_.emplace_back(buf, buf + size);
    response_handler(android_xr::schemas::ResponseTypesUnion());
  }

  void Schedule(absl::AnyInvocable<absl::Status()> fn) override {
    scheduled_tasks_.push_back(std::move(fn));
  }

  absl::StatusOr<int32_t> GetActiveFrameUpdatesCount() const override {
    return active_frame_updates_;
  }
  absl::StatusOr<int32_t> GetActiveOneShotsCount() const override {
    return active_one_shots_;
  }
  void ClearReleasedMessageGroups() override {}

  void DrainScheduledTasks() {
    for (auto& task : scheduled_tasks_) {
      
    }
    scheduled_tasks_.clear();
  }

  const std::vector<SentMessage>& GetSentMessages() const {
    return sent_messages_;
  }

  const std::vector<std::vector<uint8_t>>& GetSentRequests() const {
    return sent_requests_;
  }

  void SetBeginFrameUpdateStatus(absl::Status status) {
    begin_frame_update_status_ = status;
  }
  void SetEndStatus(absl::Status status) { end_status_ = status; }

 private:
  int32_t active_frame_updates_ = 0;
  int32_t active_one_shots_ = 0;
  MessageGroupId next_group_id_ = 1;
  std::vector<SentMessage> sent_messages_;
  std::vector<std::vector<uint8_t>> sent_requests_;
  std::vector<absl::AnyInvocable<absl::Status()>> scheduled_tasks_;
  absl::Status begin_frame_update_status_ = absl::OkStatus();
  absl::Status end_status_ = absl::OkStatus();
};

class SplitEngineIpcClientTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto transport = std::make_unique<FakeSplitEngineSerializerTransport>();
    fake_transport_ = transport.get();

    serializer_ =
        std::make_unique<SplitEngineIpcClient>(kApiLevel, std::move(transport));
  }

  static constexpr int32_t kApiLevel = 1;

  FakeSplitEngineSerializerTransport* fake_transport_;
  std::unique_ptr<SplitEngineIpcClient> serializer_;
};

TEST_F(SplitEngineIpcClientTest, GetApiLevel_ReturnsConfiguredApiLevel) {
  EXPECT_EQ(serializer_->GetApiLevel(), kApiLevel);
}

TEST_F(SplitEngineIpcClientTest,
       AddTextures_ExecutesOneShotTransportAndInvokesCallback) {
  bool callback_invoked = false;
  bool pb_callback_invoked = false;
  AddTexturesInfo add_textures;
  TextureCreateInfo tex;
  tex.id = 1;
  std::array<uint8_t, 4> data = {1, 2, 3, 4};
  tex.pixel_buffers.push_back({BufferHandle(
      data, [&](absl::Span<const uint8_t>) { pb_callback_invoked = true; })});
  add_textures.textures.push_back(std::move(tex));

  serializer_->AddTextures(std::move(add_textures),
                           [&]() { callback_invoked = true; });

  EXPECT_EQ(*fake_transport_->GetActiveOneShotsCount(), 0);
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 1);
  EXPECT_FALSE(callback_invoked);
  EXPECT_TRUE(pb_callback_invoked);

  fake_transport_->DrainScheduledTasks();
  EXPECT_TRUE(callback_invoked);
}

TEST_F(SplitEngineIpcClientTest, AddMeshData_ExecutesOneShotTransport) {
  bool vb_callback_invoked = false;
  bool ib_callback_invoked = false;
  AddMeshDataInfo mesh_data;

  VertexBufferCreateInfo vb;
  vb.id = 1;
  std::array<uint8_t, 4> vb_data = {1, 2, 3, 4};
  vb.buffer.blocks.push_back({{},
                              BufferHandle(vb_data,
                                           [&](absl::Span<const uint8_t>) {
                                             vb_callback_invoked = true;
                                           }),
                              4});
  mesh_data.vertex_buffers.push_back(std::move(vb));

  IndexBufferCreateInfo ib;
  ib.id = 2;
  std::array<uint8_t, 2> ib_data = {5, 6};
  ib.buffer.buffer = BufferHandle(
      ib_data, [&](absl::Span<const uint8_t>) { ib_callback_invoked = true; });
  mesh_data.index_buffers.push_back(std::move(ib));

  serializer_->AddMeshData(std::move(mesh_data));

  EXPECT_EQ(*fake_transport_->GetActiveOneShotsCount(), 0);
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 1);
  EXPECT_TRUE(vb_callback_invoked);
  EXPECT_TRUE(ib_callback_invoked);
}

TEST_F(SplitEngineIpcClientTest,
       AddMorphTargetBuffers_ExecutesOneShotTransport) {
  bool pos_callback_invoked = false;
  bool tan_callback_invoked = false;
  AddMorphTargetBuffersInfo buffers;
  MorphTargetBufferCreateInfo buf;
  buf.id = 1;

  std::array<uint8_t, 2> pos_data = {1, 2};
  std::array<uint8_t, 2> tan_data = {3, 4};

  buf.buffer.targets.push_back(MorphTargetAttributeInfoCreateInfo{
      .positions = BufferHandle{pos_data,
                                [&](absl::Span<const uint8_t>) {
                                  pos_callback_invoked = true;
                                }},
      .tangents = BufferHandle{tan_data, [&](absl::Span<const uint8_t>) {
                                 tan_callback_invoked = true;
                               }}});
  buffers.morph_target_buffers.push_back(std::move(buf));

  serializer_->AddMorphTargetBuffers(std::move(buffers));

  EXPECT_EQ(*fake_transport_->GetActiveOneShotsCount(), 0);
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 1);
  EXPECT_TRUE(pos_callback_invoked);
  EXPECT_TRUE(tan_callback_invoked);
}

TEST_F(SplitEngineIpcClientTest,
       AddImageBasedLightingAsset_ExecutesOneShotTransport) {
  android_xr::schemas::AddImageBasedLightingAssetsT assets;

  serializer_->AddImageBasedLightingAsset(std::move(assets));

  EXPECT_EQ(*fake_transport_->GetActiveOneShotsCount(), 0);
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 1);
}

TEST_F(SplitEngineIpcClientTest, RequestBuiltInMaterialInstance_Request) {
  android_xr::schemas::BuiltInMaterialRequestT request;
  request.material_instance_id = 123;

  absl::Status status;
  serializer_->RequestBuiltInMaterialInstance(
      std::move(request), [&](absl::Status s) { status = s; });

  EXPECT_OK(status);
  EXPECT_EQ(fake_transport_->GetSentRequests().size(), 1);
}

TEST_F(SplitEngineIpcClientTest, AddCustomMaterial_Request) {
  android_xr::schemas::AddCustomMaterialRequestT request;
  request.material_id = 456;

  absl::Status status;
  serializer_->AddCustomMaterial(std::move(request),
                                 [&](absl::Status s) { status = s; });

  EXPECT_OK(status);
  EXPECT_EQ(fake_transport_->GetSentRequests().size(), 1);
}

TEST_F(SplitEngineIpcClientTest, CreateNode_BatchesCommand) {
  serializer_->CreateNode(10);

  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 0);

  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 1);
}

TEST_F(SplitEngineIpcClientTest, SetNodeProperties_BatchesUpdates) {
  EntityId node_id = 20;
  serializer_->SetName(node_id, "TestNode");
  serializer_->SetParent(node_id, 5);
  serializer_->SetEnabled(node_id, true);
  serializer_->SetLocalTransform(node_id, android_xr::schemas::Mat4f());
  serializer_->SetGroups(node_id, {"group1"});
  serializer_->AssignUserId(node_id, 999);

  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 0);

  serializer_->SendAllBatches();
  EXPECT_GT(fake_transport_->GetSentMessages().size(), 0);
}

TEST_F(SplitEngineIpcClientTest, DestroyNode_BatchesCommandWithDependencies) {
  serializer_->DestroyNode(30, {10, 20});

  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 0);

  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 1);
}

TEST_F(SplitEngineIpcClientTest, SetGeometryAt_BatchesUpdate) {
  serializer_->SetGeometryAt(1, 0, 1, 100, 101, 0, 10);

  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 0);

  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 1);
}

TEST_F(SplitEngineIpcClientTest, SetMaterialInstanceAt_BatchesUpdate) {
  serializer_->SetMaterialInstanceAt(1, 0, 200);

  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 0);

  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 1);
}

TEST_F(SplitEngineIpcClientTest, SetRenderableProperties_BatchesUpdates) {
  EntityId node_id = 40;
  serializer_->SetAxisAlignedBoundingBox(node_id, android_xr::schemas::Box());
  serializer_->SetPriority(node_id, 2);
  serializer_->SetLayerMask(node_id, 1, 1);
  serializer_->SetBlendOrderAt(node_id, 0, 5);
  serializer_->SetGlobalBlendOrderEnabledAt(node_id, 0, true);
  serializer_->SetMorphWeights(node_id, {0.5f}, 0);
  serializer_->SetBones(node_id, {android_xr::schemas::Mat4f()});

  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 0);

  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 1);
}

TEST_F(SplitEngineIpcClientTest, DestroyRenderable_BatchesCommand) {
  serializer_->DestroyRenderable(50);

  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 0);

  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 1);
}

TEST_F(SplitEngineIpcClientTest, SetColliders_BatchesAddOrUpdate) {
  EntityId node_id = 60;
  serializer_->SetBoxCollider(node_id, android_xr::schemas::Box(), true);
  serializer_->SetMeshCollider(node_id, true);
  serializer_->SetSphereCollider(node_id,
                                 android_xr::schemas::SphereColliderT(), true);
  serializer_->SetCapsuleCollider(
      node_id, android_xr::schemas::CapsuleColliderT(), true);

  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 0);

  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 1);
}

TEST_F(SplitEngineIpcClientTest, ClearCollider_BatchesRemove) {
  serializer_->ClearCollider(60,
                             android_xr::schemas::ColliderType::BoxCollider);

  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 0);

  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 1);
}

TEST_F(SplitEngineIpcClientTest, TexturePipelineRenderers_BatchesCommands) {
  EntityId node_id = 70;
  serializer_->AddTexturePipelineRenderer(
      node_id,
      std::make_unique<android_xr::schemas::TexturePipelineRendererT>());
  serializer_->SetTexturePipelineRendererPassesEnabled(node_id, {true, false});
  serializer_->SetTexturePipelineRendererProjectionQuad(
      node_id, std::make_unique<android_xr::schemas::ProjectionQuadT>());

  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 0);

  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(),
            2);  // Add and Update are separate batches

  serializer_->RemoveTexturePipelineRenderer(node_id);
  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(),
            3);  // Remove is a third batch
}

TEST_F(SplitEngineIpcClientTest, NamedTextures_BatchesCommands) {
  serializer_->RegisterNamedTexture(1000, "tex1");
  serializer_->UnregisterNamedTexture(1000);

  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 0);

  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 2);
}

TEST_F(SplitEngineIpcClientTest, EnvironmentIbl_BatchesCommands) {
  serializer_->SetPreferredEnvironmentIblAsset(10, 1.0f,
                                               android_xr::schemas::Float3());
  serializer_->ClearPreferredEnvironmentIblAsset();
  serializer_->RemoveImageBasedLightingAsset(10);

  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 0);

  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 2);
}

TEST_F(SplitEngineIpcClientTest,
       SendAllBatches_WhenNoBatchesQueued_DoesNotBeginFrameUpdate) {
  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 0);
  EXPECT_EQ(*fake_transport_->GetActiveFrameUpdatesCount(), 0);
}

TEST_F(SplitEngineIpcClientTest, RemoveBufferMethods_BatchesCommands) {
  serializer_->RemoveMorphTargetBuffer(1);
  serializer_->RemoveVertexBuffer(2);
  serializer_->RemoveIndexBuffer(3);

  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 0);

  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(),
            2);  // RemoveMeshData and RemoveMorphTargetBuffers
}

TEST_F(SplitEngineIpcClientTest, RemoveMaterialAndAddInstance_BatchesCommands) {
  serializer_->RemoveMaterial(10);
  serializer_->AddMaterialInstance(20, 30);

  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 0);

  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 2);
}

TEST_F(SplitEngineIpcClientTest, SetMaterialParameter_BatchesCommands) {
  android_xr::schemas::MaterialParamValueUnion union_val;
  union_val.Set(android_xr::schemas::Bool(true));
  serializer_->SetMaterialParameter(10, "param1", std::move(union_val));
  serializer_->SetMaterialParameter(
      10, "param2", 20,
      std::make_unique<android_xr::schemas::TextureSamplerT>());

  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 0);

  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 1);
}

TEST_F(SplitEngineIpcClientTest, AddCompiledFilamentMaterial_BatchesCommand) {
  serializer_->AddCompiledFilamentMaterial(android_xr::schemas::MaterialT());

  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 0);

  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 1);
}

TEST_F(SplitEngineIpcClientTest, MaterialInstanceLifecycle_BatchesCommands) {
  serializer_->SetBuiltInMaterialParameters(
      android_xr::schemas::BuiltInMaterialInstanceParametersT());
  serializer_->DuplicateMaterialInstance(1, 2);
  serializer_->RemoveMaterialInstance(1);

  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 0);

  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 3);
}

TEST_F(SplitEngineIpcClientTest, StandaloneSetters_ExecuteContainsBranches) {
  serializer_->SetBones(101, {});
  serializer_->SetPriority(102, 1);
  serializer_->SetLayerMask(103, 1, 1);
  serializer_->SetEnabled(104, true);
  serializer_->SetParent(105, 1);
  serializer_->SetLocalTransform(106, android_xr::schemas::Mat4f());
  serializer_->SetLocalTransform(107, android_xr::schemas::Mat4());
  serializer_->SetGroups(108, {});
  serializer_->SetMeshCollider(109, true);
  serializer_->SetSphereCollider(110, android_xr::schemas::SphereColliderT(),
                                 true);
  serializer_->SetCapsuleCollider(111, android_xr::schemas::CapsuleColliderT(),
                                  true);
  serializer_->AddTexturePipelineRenderer(
      112, std::make_unique<android_xr::schemas::TexturePipelineRendererT>());
  serializer_->SetTexturePipelineRendererPassesEnabled(113, {});
  serializer_->SetTexturePipelineRendererProjectionQuad(
      114, std::make_unique<android_xr::schemas::ProjectionQuadT>());
  serializer_->SendAllBatches();
  EXPECT_GT(fake_transport_->GetSentMessages().size(), 0);
}

TEST_F(SplitEngineIpcClientTest,
       SendAllBatches_WhenBeginFrameUpdateFails_LogsError) {
  fake_transport_->SetBeginFrameUpdateStatus(absl::InternalError("fail"));
  serializer_->CreateNode(1);
  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 0);
}

TEST_F(SplitEngineIpcClientTest, SendAllBatches_WhenEndFails_LogsError) {
  fake_transport_->SetEndStatus(absl::InternalError("fail"));
  serializer_->CreateNode(1);
  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 1);
}

TEST_F(SplitEngineIpcClientTest, RemoveTexture_BatchesCommand) {
  serializer_->RemoveTexture(12345);

  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 0);

  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 1);
}

TEST_F(SplitEngineIpcClientTest, AddRenderable_BatchesCommand) {
  serializer_->AddRenderable(
      54321, 2, 5, std::make_unique<android_xr::schemas::MorphTargetDataT>(),
      std::make_unique<android_xr::schemas::RenderableFlagsT>());

  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 0);

  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 1);
}

TEST_F(SplitEngineIpcClientTest,
       UpdateMeshData_BatchesCommandAndExecutesReleaseCallbacks) {
  bool vb_callback_invoked = false;
  bool ib_callback_invoked = false;

  std::vector<VertexBufferUpdateInfo> vb_updates;
  VertexBufferUpdateInfo vb_info;
  vb_info.id = 1;
  VertexBlockUpdateInfo block_info;
  block_info.block_index = 0;
  block_info.offset = 0;
  std::array<uint8_t, 4> vb_data = {1, 2, 3, 4};
  block_info.buffer = BufferHandle(
      vb_data, [&](absl::Span<const uint8_t>) { vb_callback_invoked = true; });
  vb_info.block_updates.push_back(std::move(block_info));
  vb_updates.push_back(std::move(vb_info));

  std::vector<IndexBufferUpdateInfo> ib_updates;
  IndexBufferUpdateInfo ib_info;
  ib_info.id = 2;
  ib_info.offset = 0;
  std::array<uint8_t, 4> ib_data = {5, 6, 7, 8};
  ib_info.buffer = BufferHandle(
      ib_data, [&](absl::Span<const uint8_t>) { ib_callback_invoked = true; });
  ib_updates.push_back(std::move(ib_info));

  serializer_->UpdateMeshData(std::move(vb_updates), std::move(ib_updates));

  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 0);
  EXPECT_FALSE(vb_callback_invoked);
  EXPECT_FALSE(ib_callback_invoked);

  serializer_->SendAllBatches();
  EXPECT_EQ(fake_transport_->GetSentMessages().size(), 1);
  EXPECT_TRUE(vb_callback_invoked);
  EXPECT_TRUE(ib_callback_invoked);
}

}  // namespace
}  // namespace split_engine
