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

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "absl/functional/any_invocable.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "split_engine/client/batch.h"
#include "split_engine/client/batch_manager.h"
#include "split_engine/client/transport.h"
#include "split_engine/client/types.h"
#include "split_engine/schemas/split_engine_data_generated.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"
#include "split_engine/schemas/split_engine_ipc_size_calculator_generated.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_material_size_calculator_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"
#include "split_engine/schemas/split_engine_render_passes_generated.h"

namespace split_engine {

namespace {

static constexpr std::string_view kTag = "[SplitEngineIpcClient]: ";
constexpr size_t kScratchBufferCapacity = 2048;

template <typename T>
flatbuffers::Offset<android_xr::schemas::Command> CreateCommand(
    flatbuffers::FlatBufferBuilder& fbb,
    flatbuffers::Offset<T> command_offset) {
  return android_xr::schemas::CreateCommand(
      fbb, android_xr::schemas::CommandTypesTraits<T>::enum_value,
      command_offset.Union());
}

class StatusOrResponse {
 public:
  StatusOrResponse(absl::Status status) : value_(std::move(status)) {}
  StatusOrResponse(android_xr::schemas::ResponseTypesUnion response)
      : value_(std::move(response)) {}

  std::optional<absl::Status> Status() const {
    if (std::holds_alternative<absl::Status>(value_)) {
      return std::get<absl::Status>(value_);
    }
    return std::nullopt;
  }

  std::optional<android_xr::schemas::ResponseTypesUnion> Response() const {
    if (std::holds_alternative<android_xr::schemas::ResponseTypesUnion>(
            value_)) {
      return std::get<android_xr::schemas::ResponseTypesUnion>(value_);
    }
    return std::nullopt;
  }

 private:
  std::variant<absl::Status, android_xr::schemas::ResponseTypesUnion> value_;
};

StatusOrResponse UnpackResponseStatus(
    absl::StatusOr<android_xr::schemas::ResponseTypesUnion> response) {
  if (!response.ok()) {
    return response.status();
  }
  switch (response->type) {
    case android_xr::schemas::ResponseTypes::NONE:
      // An empty response is considered successful.
      return absl::OkStatus();
    case android_xr::schemas::ResponseTypes::ErrorResponse: {
      const android_xr::schemas::ErrorResponse* error_response =
          static_cast<const android_xr::schemas::ErrorResponse*>(
              response->value);
      switch (error_response->error_code()) {
        case android_xr::schemas::ErrorCode::UNKNOWN_ERROR:
          return absl::UnknownError(
              error_response->error_message()
                  ? error_response->error_message()->c_str()
                  : "Unknown error");
        case android_xr::schemas::ErrorCode::INVALID_ARGUMENT_ERROR:
          return absl::InvalidArgumentError(
              error_response->error_message()
                  ? error_response->error_message()->c_str()
                  : "Invalid argument");
        case android_xr::schemas::ErrorCode::FAILED_PRECONDITION_ERROR:
          return absl::FailedPreconditionError(
              error_response->error_message()
                  ? error_response->error_message()->c_str()
                  : "Failed precondition");
        case android_xr::schemas::ErrorCode::NOT_FOUND_ERROR:
          return absl::NotFoundError(
              error_response->error_message()
                  ? error_response->error_message()->c_str()
                  : "Not found");
        case android_xr::schemas::ErrorCode::ALREADY_EXISTS_ERROR:
          return absl::AlreadyExistsError(
              error_response->error_message()
                  ? error_response->error_message()->c_str()
                  : "Already exists");
        case android_xr::schemas::ErrorCode::OUT_OF_RANGE_ERROR:
          return absl::OutOfRangeError(
              error_response->error_message()
                  ? error_response->error_message()->c_str()
                  : "Out of range");
        case android_xr::schemas::ErrorCode::PERMISSION_DENIED_ERROR:
          return absl::PermissionDeniedError(
              error_response->error_message()
                  ? error_response->error_message()->c_str()
                  : "Permission denied");
        case android_xr::schemas::ErrorCode::INTERNAL_ERROR:
          return absl::InternalError(
              error_response->error_message()
                  ? error_response->error_message()->c_str()
                  : "Internal error");
        case android_xr::schemas::ErrorCode::UNIMPLEMENTED_ERROR:
          return absl::UnimplementedError(
              error_response->error_message()
                  ? error_response->error_message()->c_str()
                  : "Unimplemented");
        case android_xr::schemas::ErrorCode::ABORTED_ERROR:
          return absl::AbortedError(
              error_response->error_message()
                  ? error_response->error_message()->c_str()
                  : "Aborted");
        case android_xr::schemas::ErrorCode::UNAVAILABLE_ERROR:
          return absl::UnavailableError(
              error_response->error_message()
                  ? error_response->error_message()->c_str()
                  : "Unavailable");
        case android_xr::schemas::ErrorCode::RESOURCE_EXHAUSTED_ERROR:
          return absl::ResourceExhaustedError(
              error_response->error_message()
                  ? error_response->error_message()->c_str()
                  : "Resource exhausted");
        case android_xr::schemas::ErrorCode::DEADLINE_EXCEEDED_ERROR:
          return absl::DeadlineExceededError(
              error_response->error_message()
                  ? error_response->error_message()->c_str()
                  : "Deadline exceeded");
        case android_xr::schemas::ErrorCode::DATA_LOSS_ERROR:
          return absl::DataLossError(
              error_response->error_message()
                  ? error_response->error_message()->c_str()
                  : "Data loss");
        case android_xr::schemas::ErrorCode::UNAUTHENTICATED_ERROR:
          return absl::UnauthenticatedError(
              error_response->error_message()
                  ? error_response->error_message()->c_str()
                  : "Unauthenticated");
        case android_xr::schemas::ErrorCode::CANCELLED_ERROR:
          return absl::CancelledError(
              error_response->error_message()
                  ? error_response->error_message()->c_str()
                  : "Cancelled");
      }
    }
    default:
      return response.value();
  }
}

}  // namespace

using flatbuffers::Offset;

using android_xr::schemas::AddCustomMaterialRequest;
using android_xr::schemas::AddCustomMaterialRequestT;
using android_xr::schemas::AddImageBasedLightingAssets;
using android_xr::schemas::AddImageBasedLightingAssetsT;
using android_xr::schemas::AddMeshData;
using android_xr::schemas::AddMorphTargetBuffers;
using android_xr::schemas::AddOrUpdateColliderT;
using android_xr::schemas::AddRenderableT;
using android_xr::schemas::AddTexturePipelineRendererT;
using android_xr::schemas::AddTextures;
using android_xr::schemas::BonesT;
using android_xr::schemas::Bool;
using android_xr::schemas::BoundsInfoT;
using android_xr::schemas::Box;
using android_xr::schemas::BuiltInMaterialInstanceParametersT;
using android_xr::schemas::BuiltInMaterialRequest;
using android_xr::schemas::BuiltInMaterialRequestT;
using android_xr::schemas::CapsuleColliderT;
using android_xr::schemas::ColliderType;
using android_xr::schemas::Command;
using android_xr::schemas::CommandTypes;
using android_xr::schemas::DuplicateMaterialInstanceT;
using android_xr::schemas::Float3;
using android_xr::schemas::GeometryUpdateT;
using android_xr::schemas::LayerMaskT;
using android_xr::schemas::Mat4;
using android_xr::schemas::Mat4f;
using android_xr::schemas::MaterialParametersT;
using android_xr::schemas::MaterialParamInfoT;
using android_xr::schemas::MaterialParamValueUnion;
using android_xr::schemas::MaterialT;
using android_xr::schemas::MaterialTextureParameterT;
using android_xr::schemas::MeshColliderT;
using android_xr::schemas::MorphTargetDataT;
using android_xr::schemas::MorphWeightsT;
using android_xr::schemas::ParentT;
using android_xr::schemas::PrimitiveUpdateT;
using android_xr::schemas::ProjectionQuadT;
using android_xr::schemas::RegisterNamedTextureT;
using android_xr::schemas::RenderableFlagsT;
using android_xr::schemas::Request;
using android_xr::schemas::RequestTypes;
using android_xr::schemas::ResponseTypesUnion;
using android_xr::schemas::SphereColliderT;
using android_xr::schemas::TexturePipelineRendererT;
using android_xr::schemas::TextureSamplerT;
using android_xr::schemas::TransformT;
using android_xr::schemas::UInt16;
using android_xr::schemas::UInt32;
using android_xr::schemas::UInt64;
using android_xr::schemas::UInt8;
using android_xr::schemas::UpdateNodeT;
using android_xr::schemas::UpdateRenderableT;
using android_xr::schemas::UpdateTexturePipelineRendererT;

SplitEngineIpcClient::SplitEngineIpcClient(int32_t api_level,
                                           std::unique_ptr<Transport> transport)
    : api_level_(api_level),
      transport_(std::move(transport)),
      batch_manager_() {}

void SplitEngineIpcClient::AddRenderable(
    EntityId node_id, uint32_t primitive_count, uint32_t skinning_bone_count,
    std::unique_ptr<MorphTargetDataT> morph_target_data,
    std::unique_ptr<RenderableFlagsT> renderable_flags) {
  Batch<CommandTypes::AddRenderables>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::AddRenderables>({node_id});
  AddRenderableT add_renderable;
  add_renderable.entity = node_id;
  add_renderable.primitive_count = primitive_count;
  add_renderable.skinning_bone_count =
      std::make_unique<UInt32>(skinning_bone_count);
  add_renderable.morph_target_data = std::move(morph_target_data);
  add_renderable.renderable_flags = std::move(renderable_flags);

  batch.data.push_back(std::move(add_renderable));
}

void SplitEngineIpcClient::DestroyRenderable(EntityId node_id) {
  Batch<CommandTypes::RemoveRenderables>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::RemoveRenderables>(
          {node_id});
  batch.data.insert(node_id);
}

android_xr::schemas::UpdateRenderableT&
SplitEngineIpcClient::GetOrCreateUpdateRenderableCmdForEntity(
    EntityId node_id, const std::vector<ResourceId>& resource_dependencies) {
  Batch<CommandTypes::UpdateRenderables>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::UpdateRenderables>(
          {node_id}, resource_dependencies);
  auto it = batch.data.find(node_id);
  if (it == batch.data.end()) {
    it = batch.data.emplace(node_id, UpdateRenderableT()).first;
    it->second.entity = node_id;
  }
  return it->second;
}

void SplitEngineIpcClient::SetMaterialInstanceAt(
    EntityId node_id, size_t primitive_index, ResourceId material_instance_id) {
  android_xr::schemas::UpdateRenderableT& cmd =
      GetOrCreateUpdateRenderableCmdForEntity(node_id, {material_instance_id});

  if (cmd.primitives.size() <= primitive_index) {
    cmd.primitives.resize(primitive_index + 1);
  }
  if (!cmd.primitives[primitive_index]) {
    cmd.primitives[primitive_index] = std::make_unique<PrimitiveUpdateT>();
  }
  cmd.primitives[primitive_index]->material_instance_id =
      std::make_unique<UInt64>(material_instance_id);
}

int32_t SplitEngineIpcClient::GetApiLevel() const { return api_level_; }

void SplitEngineIpcClient::SetGeometryAt(EntityId node_id,
                                         size_t primitiveIndex, uint8_t type,
                                         ResourceId vertex_buffer_id,
                                         ResourceId index_buffer_id,
                                         uint32_t offset, uint32_t count) {
  std::vector<ResourceId> required_ids =
      (index_buffer_id != 0)
          ? std::vector<ResourceId>{vertex_buffer_id, index_buffer_id}
          : std::vector<ResourceId>{vertex_buffer_id};
  android_xr::schemas::UpdateRenderableT& cmd =
      GetOrCreateUpdateRenderableCmdForEntity(node_id, required_ids);
  if (cmd.primitives.size() <= primitiveIndex) {
    cmd.primitives.resize(primitiveIndex + 1);
  }
  if (!cmd.primitives[primitiveIndex]) {
    cmd.primitives[primitiveIndex] = std::make_unique<PrimitiveUpdateT>();
  }
  std::unique_ptr<GeometryUpdateT> geometry =
      std::make_unique<GeometryUpdateT>();
  geometry->vertex_buffer_id = vertex_buffer_id;
  geometry->index_buffer_id = index_buffer_id;
  geometry->offset = offset;
  geometry->count = count;
  geometry->primitive_type = type;

  cmd.primitives[primitiveIndex]->geometry = std::move(geometry);
}

void SplitEngineIpcClient::SetBones(EntityId node_id,
                                    std::vector<Mat4f> transforms) {
  android_xr::schemas::UpdateRenderableT& cmd =
      GetOrCreateUpdateRenderableCmdForEntity(node_id);
  cmd.bones = std::make_unique<BonesT>();
  cmd.bones->bones = std::move(transforms);
}

void SplitEngineIpcClient::SetAxisAlignedBoundingBox(EntityId node_id,
                                                     Box aabb) {
  android_xr::schemas::UpdateRenderableT& cmd =
      GetOrCreateUpdateRenderableCmdForEntity(node_id);
  cmd.bounds = std::make_unique<BoundsInfoT>();
  cmd.bounds->bounds = std::make_unique<Box>(aabb);
}

void SplitEngineIpcClient::SetPriority(EntityId node_id, uint8_t priority) {
  android_xr::schemas::UpdateRenderableT& cmd =
      GetOrCreateUpdateRenderableCmdForEntity(node_id);
  cmd.priority = std::make_unique<UInt8>(priority);
}

void SplitEngineIpcClient::SetLayerMask(EntityId node_id, uint8_t select,
                                        uint8_t values) {
  android_xr::schemas::UpdateRenderableT& cmd =
      GetOrCreateUpdateRenderableCmdForEntity(node_id);
  cmd.layer_mask = std::make_unique<LayerMaskT>();
  cmd.layer_mask->select = select;
  cmd.layer_mask->values = values;
}

void SplitEngineIpcClient::SetBlendOrderAt(EntityId node_id,
                                           size_t primitiveIndex,
                                           uint16_t order) {
  android_xr::schemas::UpdateRenderableT& cmd =
      GetOrCreateUpdateRenderableCmdForEntity(node_id);
  if (cmd.primitives.size() <= primitiveIndex) {
    cmd.primitives.resize(primitiveIndex + 1);
  }
  if (!cmd.primitives[primitiveIndex]) {
    cmd.primitives[primitiveIndex] = std::make_unique<PrimitiveUpdateT>();
  }
  cmd.primitives[primitiveIndex]->blend_order = std::make_unique<UInt16>(order);
}

void SplitEngineIpcClient::SetGlobalBlendOrderEnabledAt(EntityId node_id,
                                                        size_t primitiveIndex,
                                                        bool enabled) {
  android_xr::schemas::UpdateRenderableT& cmd =
      GetOrCreateUpdateRenderableCmdForEntity(node_id);
  if (cmd.primitives.size() <= primitiveIndex) {
    cmd.primitives.resize(primitiveIndex + 1);
  }
  if (!cmd.primitives[primitiveIndex]) {
    cmd.primitives[primitiveIndex] = std::make_unique<PrimitiveUpdateT>();
  }
  cmd.primitives[primitiveIndex]->global_blend_order_enabled =
      std::make_unique<Bool>(enabled);
}

void SplitEngineIpcClient::SetMorphWeights(EntityId node_id,
                                           std::vector<float> weights,
                                           size_t offset) {
  android_xr::schemas::UpdateRenderableT& cmd =
      GetOrCreateUpdateRenderableCmdForEntity(node_id);
  cmd.morph_weights = std::make_unique<MorphWeightsT>();
  cmd.morph_weights->weights = std::move(weights);
  cmd.morph_weights->offset = offset;
}

void SplitEngineIpcClient::AddTextures(AddTexturesInfo add_textures,
                                       absl::AnyInvocable<void()> on_done) {
  const size_t kBufferSize =
      GetSerializedSize(add_textures, api_level_) + kScratchBufferCapacity;
  const absl::StatusOr<MessageGroupId> group_id =
      transport_->BeginOneShot(kBufferSize);
  

  OwnedOrBorrowedFlatBufferBuilder builder =
      transport_->CreateBuilder(*group_id, kBufferSize);

  absl::Status status = transport_->AddMessage(
      *group_id, std::move(builder),
      [add_textures = std::move(add_textures),
       api_level = api_level_](flatbuffers::FlatBufferBuilder& builder) {
        return CreateCommand(builder, Pack(builder, add_textures, api_level));
      });
  if (!status.ok()) IMP_LOG(imp::FATAL) << "Failed to add message: " << status;

  status = transport_->End(*group_id);
  if (!status.ok()) IMP_LOG(imp::FATAL) << "Failed to end message group: " << status;

  transport_->Schedule([on_done = std::move(on_done)]() mutable {
    if (on_done) {
      on_done();
    }
    return absl::OkStatus();
  });
}

void SplitEngineIpcClient::RemoveTexture(ResourceId texture_id) {
  Batch<CommandTypes::RemoveTextures>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::RemoveTextures>();
  batch.data.push_back(texture_id);
}

void SplitEngineIpcClient::AddMeshData(AddMeshDataInfo mesh_data) {
  const size_t kBufferSize =
      GetSerializedSize(mesh_data) + kScratchBufferCapacity;
  const absl::StatusOr<MessageGroupId> group_id =
      transport_->BeginOneShot(kBufferSize);
  

  OwnedOrBorrowedFlatBufferBuilder builder =
      transport_->CreateBuilder(*group_id, kBufferSize);

  absl::Status status = transport_->AddMessage(
      *group_id, std::move(builder),
      [mesh_data =
           std::move(mesh_data)](flatbuffers::FlatBufferBuilder& builder) {
        return CreateCommand(builder, Pack(builder, mesh_data));
      });
  if (!status.ok()) IMP_LOG(imp::FATAL) << "Failed to add message: " << status;

  status = transport_->End(*group_id);
  if (!status.ok()) IMP_LOG(imp::FATAL) << "Failed to end message group: " << status;
}

void SplitEngineIpcClient::UpdateMeshData(
    std::vector<VertexBufferUpdateInfo> vertex_buffer_updates,
    std::vector<IndexBufferUpdateInfo> index_buffer_updates) {
  std::vector<ResourceId> resource_dependencies;
  resource_dependencies.reserve(vertex_buffer_updates.size() +
                                index_buffer_updates.size());
  for (const auto& vb : vertex_buffer_updates) {
    resource_dependencies.push_back(vb.id);
  }
  for (const auto& ib : index_buffer_updates) {
    resource_dependencies.push_back(ib.id);
  }
  Batch<CommandTypes::UpdateMeshData>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::UpdateMeshData>(
          {}, resource_dependencies);
  for (auto& vb_update : vertex_buffer_updates) {
    batch.data.vertex_buffers.push_back(std::move(vb_update));
  }
  for (auto& ib_update : index_buffer_updates) {
    batch.data.index_buffers.push_back(std::move(ib_update));
  }
}

void SplitEngineIpcClient::AddMorphTargetBuffers(
    AddMorphTargetBuffersInfo morph_target_buffers) {
  const size_t kBufferSize =
      GetSerializedSize(morph_target_buffers) + kScratchBufferCapacity;
  const absl::StatusOr<MessageGroupId> group_id =
      transport_->BeginOneShot(kBufferSize);
  

  OwnedOrBorrowedFlatBufferBuilder builder =
      transport_->CreateBuilder(*group_id, kBufferSize);

  absl::Status status = transport_->AddMessage(
      *group_id, std::move(builder),
      [morph_target_buffers = std::move(morph_target_buffers)](
          flatbuffers::FlatBufferBuilder& builder) {
        return CreateCommand(builder, Pack(builder, morph_target_buffers));
      });
  if (!status.ok()) IMP_LOG(imp::FATAL) << "Failed to add message: " << status;

  status = transport_->End(*group_id);
  if (!status.ok()) IMP_LOG(imp::FATAL) << "Failed to end message group: " << status;
}

void SplitEngineIpcClient::AddImageBasedLightingAsset(
    AddImageBasedLightingAssetsT add_assets) {
  const size_t kBufferSize = EstimateSize(add_assets) + kScratchBufferCapacity;
  const absl::StatusOr<MessageGroupId> group_id =
      transport_->BeginOneShot(kBufferSize);
  

  OwnedOrBorrowedFlatBufferBuilder builder =
      transport_->CreateBuilder(*group_id, kBufferSize);

  // Offload potentially expensive serialization (copying huge amount of data)
  absl::Status status = transport_->AddMessage(
      *group_id, std::move(builder),
      [add_assets =
           std::move(add_assets)](flatbuffers::FlatBufferBuilder& builder) {
        return CreateCommand(
            builder, AddImageBasedLightingAssets::Pack(builder, &add_assets));
      });
  if (!status.ok()) IMP_LOG(imp::FATAL) << "Failed to add message: " << status;

  status = transport_->End(*group_id);
  if (!status.ok()) IMP_LOG(imp::FATAL) << "Failed to end message group: " << status;
}

void SplitEngineIpcClient::RemoveImageBasedLightingAsset(
    ResourceId reflection_texture_id) {
  Batch<CommandTypes::RemoveImageBasedLightingAssets>& batch =
      batch_manager_
          .GetOrCreateBatch<CommandTypes::RemoveImageBasedLightingAssets>();
  batch.data.push_back(reflection_texture_id);
}

void SplitEngineIpcClient::SetPreferredEnvironmentIblAsset(
    ResourceId reflection_texture_id, float intensity, Float3 tint) {
  Batch<CommandTypes::SetPreferredEnvironmentIblAsset>& batch =
      batch_manager_
          .GetOrCreateBatch<CommandTypes::SetPreferredEnvironmentIblAsset>(
              {}, {reflection_texture_id});
  batch.data.image_based_lighting_asset_id = reflection_texture_id;
  batch.data.intensity = intensity;
  batch.data.tint = std::make_unique<Float3>(tint);
}

void SplitEngineIpcClient::ClearPreferredEnvironmentIblAsset() {
  // Setting the image based lighting asset id to 0 indicates to clear any
  // previously-set preferred environment IBL asset.
  Batch<CommandTypes::SetPreferredEnvironmentIblAsset>& batch =
      batch_manager_
          .GetOrCreateBatch<CommandTypes::SetPreferredEnvironmentIblAsset>();
  batch.data.image_based_lighting_asset_id = 0;
}

void SplitEngineIpcClient::RemoveMorphTargetBuffer(ResourceId buffer_id) {
  Batch<CommandTypes::RemoveMorphTargetBuffers>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::RemoveMorphTargetBuffers>();
  batch.data.push_back(buffer_id);
}

void SplitEngineIpcClient::RemoveVertexBuffer(ResourceId buffer_id) {
  Batch<CommandTypes::RemoveMeshData>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::RemoveMeshData>();
  batch.data.vertex_buffer_ids.push_back(buffer_id);
}

void SplitEngineIpcClient::RemoveIndexBuffer(ResourceId buffer_id) {
  Batch<CommandTypes::RemoveMeshData>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::RemoveMeshData>();
  batch.data.index_buffer_ids.push_back(buffer_id);
}

void SplitEngineIpcClient::RemoveMaterial(ResourceId material_id) {
  IMP_LOG(imp::INFO) << kTag << "remove material: " << material_id;
  Batch<CommandTypes::RemoveMaterials>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::RemoveMaterials>();
  batch.data.push_back(material_id);
}

void SplitEngineIpcClient::AddMaterialInstance(ResourceId material_id,
                                               ResourceId instance_id) {
  Batch<CommandTypes::AddMaterialInstances>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::AddMaterialInstances>(
          {}, {material_id, instance_id});
  batch.data.insert({instance_id, material_id});
}

void SplitEngineIpcClient::SetMaterialParameter(ResourceId material_id,
                                                std::string name,
                                                MaterialParamValueUnion value) {
  Batch<CommandTypes::SetMaterialParameters>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::SetMaterialParameters>(
          {}, {material_id});
  auto it = batch.data.find(material_id);
  if (it == batch.data.end()) {
    it = batch.data.emplace(material_id, MaterialParametersT()).first;
    it->second.material_id = material_id;
  }
  MaterialParametersT& material_params = it->second;

  std::unique_ptr<MaterialParamInfoT> param =
      std::make_unique<MaterialParamInfoT>();
  param->name = std::move(name);
  param->value = std::move(value);
  material_params.params.emplace_back(std::move(param));
}

void SplitEngineIpcClient::SetMaterialParameter(
    ResourceId material_id, std::string name, ResourceId texture_id,
    std::unique_ptr<TextureSamplerT> texture_sampler) {
  Batch<CommandTypes::SetMaterialParameters>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::SetMaterialParameters>(
          {}, {material_id});
  auto it = batch.data.find(material_id);
  if (it == batch.data.end()) {
    it = batch.data.emplace(material_id, MaterialParametersT()).first;
    it->second.material_id = material_id;
  }
  MaterialParametersT& material_params = it->second;

  std::unique_ptr<MaterialTextureParameterT> texture_param =
      std::make_unique<MaterialTextureParameterT>();
  texture_param->name = std::move(name);
  texture_param->texture_id = texture_id;
  texture_param->sampler = std::move(texture_sampler);
  material_params.texture_params.emplace_back(std::move(texture_param));
}

void SplitEngineIpcClient::AddCompiledFilamentMaterial(MaterialT material) {
  IMP_LOG(imp::INFO) << kTag << "add material: " << material.id;
  Batch<CommandTypes::AddMaterials>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::AddMaterials>(
          {}, {material.id});
  batch.data.emplace_back(std::move(material));
}

void SplitEngineIpcClient::RequestBuiltInMaterialInstance(
    BuiltInMaterialRequestT request,
    absl::AnyInvocable<void(absl::Status)> response_callback) {
  OwnedOrBorrowedFlatBufferBuilder builder = transport_->CreateRequestBuilder(
      EstimateSize(request) + kScratchBufferCapacity);
  Offset<Request> request_offset =
      CreateRequest(*builder, RequestTypes::BuiltInMaterialRequest,
                    BuiltInMaterialRequest::Pack(*builder, &request).Union());
  transport_->SendRequest(
      std::move(builder), request_offset,
      [response_callback = std::move(response_callback)](
          absl::StatusOr<ResponseTypesUnion> response) mutable {
        StatusOrResponse unpacked_response = UnpackResponseStatus(response);
        if (std::optional<absl::Status> status = unpacked_response.Status()) {
          response_callback(*status);
        }
      });
}

void SplitEngineIpcClient::AddCustomMaterial(
    AddCustomMaterialRequestT request,
    absl::AnyInvocable<void(absl::Status)> response_callback) {
  OwnedOrBorrowedFlatBufferBuilder builder = transport_->CreateRequestBuilder(
      EstimateSize(request) + kScratchBufferCapacity);
  Offset<Request> request_offset =
      CreateRequest(*builder, RequestTypes::AddCustomMaterialRequest,
                    AddCustomMaterialRequest::Pack(*builder, &request).Union());
  transport_->SendRequest(
      std::move(builder), request_offset,
      [response_callback = std::move(response_callback)](
          absl::StatusOr<ResponseTypesUnion> response) mutable {
        StatusOrResponse unpacked_response = UnpackResponseStatus(response);
        if (std::optional<absl::Status> status = unpacked_response.Status()) {
          response_callback(*status);
        }
      });
}

void SplitEngineIpcClient::SetBuiltInMaterialParameters(
    BuiltInMaterialInstanceParametersT parameters) {
  // Note: have to log this here because the material instance ID is lost
  // after the parameter values are serialized.
  IMP_LOG(imp::INFO) << kTag << "update material params for instance (built-in): "
            << parameters.material_instance_id;
  Batch<CommandTypes::SetBuiltInMaterialParameters>& batch =
      batch_manager_
          .GetOrCreateBatch<CommandTypes::SetBuiltInMaterialParameters>(
              {}, {parameters.material_instance_id});
  batch.data.push_back(std::move(parameters));
}

void SplitEngineIpcClient::DuplicateMaterialInstance(ResourceId instance_id,
                                                     ResourceId copy_id) {
  Batch<CommandTypes::DuplicateMaterialInstances>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::DuplicateMaterialInstances>(
          {}, {copy_id, instance_id});

  DuplicateMaterialInstanceT& cmd =
      batch.data.emplace_back(DuplicateMaterialInstanceT());
  cmd.original_id = instance_id;
  cmd.duplicate_id = copy_id;
}

void SplitEngineIpcClient::RemoveMaterialInstance(ResourceId instance_id) {
  Batch<CommandTypes::RemoveMaterialInstances>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::RemoveMaterialInstances>();
  batch.data.push_back(instance_id);
}

void SplitEngineIpcClient::CreateNode(EntityId node_id) {
  Batch<CommandTypes::AddNodes>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::AddNodes>({node_id});
  batch.data.insert(node_id);
}

void SplitEngineIpcClient::DestroyNode(
    EntityId node_id, const std::vector<EntityId>& dependencies) {
  Batch<CommandTypes::RemoveNodes>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::RemoveNodes>(dependencies);
  batch.data.insert(node_id);
}

UpdateNodeT& SplitEngineIpcClient::GetOrCreateUpdateNodeCmdForEntity(
    EntityId node_id, const std::vector<EntityId>& entity_dependencies,
    const std::vector<ResourceId>& resource_dependencies) {
  std::vector<EntityId> entity_dependencies_with_node_id;
  entity_dependencies_with_node_id.reserve(entity_dependencies.size() + 1);
  entity_dependencies_with_node_id.insert(
      entity_dependencies_with_node_id.end(), entity_dependencies.begin(),
      entity_dependencies.end());
  entity_dependencies_with_node_id.push_back(node_id);
  Batch<CommandTypes::UpdateNodes>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::UpdateNodes>(
          entity_dependencies_with_node_id, resource_dependencies);
  auto it = batch.data.find(node_id);
  if (it == batch.data.end()) {
    it = batch.data.emplace(node_id, UpdateNodeT()).first;
    it->second.entity = node_id;
  }
  return it->second;
}

void SplitEngineIpcClient::SetEnabled(EntityId node_id, bool enabled) {
  UpdateNodeT& cmd = GetOrCreateUpdateNodeCmdForEntity(node_id);
  cmd.enabled = std::make_unique<Bool>(enabled);
}

void SplitEngineIpcClient::SetName(EntityId node_id, std::string name) {
  UpdateNodeT& cmd = GetOrCreateUpdateNodeCmdForEntity(node_id);
  cmd.name = std::move(name);
}

void SplitEngineIpcClient::SetParent(EntityId node_id, EntityId parent) {
  UpdateNodeT& cmd = GetOrCreateUpdateNodeCmdForEntity(node_id, {parent});
  std::unique_ptr<ParentT> parent_data = std::make_unique<ParentT>();
  parent_data->entity = parent;
  cmd.parent = std::move(parent_data);
}

void SplitEngineIpcClient::SetLocalTransform(EntityId node_id,
                                             Mat4f transform) {
  UpdateNodeT& cmd = GetOrCreateUpdateNodeCmdForEntity(node_id);
  cmd.transform = std::make_unique<TransformT>();
  cmd.transform->data.Set(std::move(transform));
}

void SplitEngineIpcClient::SetLocalTransform(EntityId node_id, Mat4 transform) {
  UpdateNodeT& cmd = GetOrCreateUpdateNodeCmdForEntity(node_id);
  cmd.transform = std::make_unique<TransformT>();
  cmd.transform->data.Set(std::move(transform));
}

void SplitEngineIpcClient::SetGroups(EntityId node_id,
                                     std::vector<std::string> groups) {
  UpdateNodeT& cmd = GetOrCreateUpdateNodeCmdForEntity(node_id);
  cmd.groups.assign(groups.begin(), groups.end());
}

void SplitEngineIpcClient::AssignUserId(EntityId node_id, uint32_t user_id) {
  Batch<CommandTypes::AssignUserIdToNodes>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::AssignUserIdToNodes>(
          {node_id});
  batch.data[node_id] = user_id;
}

AddOrUpdateColliderT&
SplitEngineIpcClient::GetOrCreateAddOrUpdateColliderCmdForEntity(
    EntityId node_id) {
  Batch<CommandTypes::AddOrUpdateColliders>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::AddOrUpdateColliders>(
          {node_id});
  auto it = batch.data.find(node_id);
  if (it == batch.data.end()) {
    it = batch.data.emplace(node_id, AddOrUpdateColliderT()).first;
    it->second.entity = node_id;
  }
  return it->second;
}

void SplitEngineIpcClient::SetBoxCollider(EntityId node_id, Box box,
                                          bool enabled) {
  AddOrUpdateColliderT& cmd =
      GetOrCreateAddOrUpdateColliderCmdForEntity(node_id);
  cmd.collider_data.Set(box);
  cmd.enabled = std::make_unique<Bool>(enabled);
}

void SplitEngineIpcClient::SetMeshCollider(EntityId node_id, bool enabled) {
  AddOrUpdateColliderT& cmd =
      GetOrCreateAddOrUpdateColliderCmdForEntity(node_id);
  cmd.collider_data.Set(MeshColliderT());
  cmd.enabled = std::make_unique<Bool>(enabled);
}

void SplitEngineIpcClient::SetSphereCollider(EntityId node_id,
                                             SphereColliderT sphere,
                                             bool enabled) {
  AddOrUpdateColliderT& cmd =
      GetOrCreateAddOrUpdateColliderCmdForEntity(node_id);
  cmd.collider_data.Set(sphere);
  cmd.enabled = std::make_unique<Bool>(enabled);
}

void SplitEngineIpcClient::SetCapsuleCollider(EntityId node_id,
                                              CapsuleColliderT capsule,
                                              bool enabled) {
  AddOrUpdateColliderT& cmd =
      GetOrCreateAddOrUpdateColliderCmdForEntity(node_id);
  cmd.collider_data.Set(capsule);
  cmd.enabled = std::make_unique<Bool>(enabled);
}

void SplitEngineIpcClient::ClearCollider(EntityId node_id,
                                         ColliderType collider_type) {
  Batch<CommandTypes::RemoveColliders>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::RemoveColliders>({node_id});
  batch.data[node_id] = static_cast<ColliderType>(collider_type);
}

void SplitEngineIpcClient::AddTexturePipelineRenderer(
    EntityId node_id, std::unique_ptr<TexturePipelineRendererT> state) {
  Batch<CommandTypes::AddTexturePipelineRenderers>& batch =
      batch_manager_
          .GetOrCreateBatch<CommandTypes::AddTexturePipelineRenderers>(
              {node_id});
  auto it = batch.data.find(node_id);
  if (it == batch.data.end()) {
    it = batch.data.emplace(node_id, AddTexturePipelineRendererT()).first;
    it->second.entity = node_id;
  }
  it->second.state = std::move(state);
}

void SplitEngineIpcClient::RemoveTexturePipelineRenderer(EntityId node_id) {
  Batch<CommandTypes::RemoveTexturePipelineRenderers>& batch =
      batch_manager_
          .GetOrCreateBatch<CommandTypes::RemoveTexturePipelineRenderers>(
              {node_id});
  batch.data.push_back(node_id);
}

void SplitEngineIpcClient::SetTexturePipelineRendererPassesEnabled(
    EntityId node_id, std::vector<bool> enabled_passes) {
  Batch<CommandTypes::UpdateTexturePipelineRenderers>& batch =
      batch_manager_
          .GetOrCreateBatch<CommandTypes::UpdateTexturePipelineRenderers>(
              {node_id});
  auto it = batch.data.find(node_id);
  if (it == batch.data.end()) {
    it = batch.data.emplace(node_id, UpdateTexturePipelineRendererT()).first;
    it->second.entity = node_id;
  }
  it->second.enabled_passes = std::move(enabled_passes);
}

void SplitEngineIpcClient::SetTexturePipelineRendererProjectionQuad(
    EntityId node_id, std::unique_ptr<ProjectionQuadT> quad) {
  Batch<CommandTypes::UpdateTexturePipelineRenderers>& batch =
      batch_manager_
          .GetOrCreateBatch<CommandTypes::UpdateTexturePipelineRenderers>(
              {node_id});
  auto it = batch.data.find(node_id);
  if (it == batch.data.end()) {
    it = batch.data.emplace(node_id, UpdateTexturePipelineRendererT()).first;
    it->second.entity = node_id;
  }
  it->second.projection_quad = std::move(quad);
}

void SplitEngineIpcClient::RegisterNamedTexture(ResourceId texture_id,
                                                std::string name) {
  IMP_LOG(imp::INFO) << kTag << "RegisterNamedTexture - id: " << texture_id
             << ", name: " << name;
  Batch<CommandTypes::RegisterNamedTextures>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::RegisterNamedTextures>(
          {}, {texture_id});
  batch.data[texture_id] =
      RegisterNamedTextureT{.id = texture_id, .name = std::move(name)};
}

void SplitEngineIpcClient::UnregisterNamedTexture(ResourceId texture_id) {
  Batch<CommandTypes::UnregisterNamedTextures>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::UnregisterNamedTextures>(
          {}, {texture_id});
  batch.data.push_back(texture_id);
}

void SplitEngineIpcClient::SerializeBatch(MessageGroupId message_group_id,
                                          BatchBase& batch_base) {
  size_t serialized_size = batch_base.GetSerializedSize();
  if (serialized_size == 0) return;
  // Add extra space for the FlatBufferBuilder scratch buffer.
  OwnedOrBorrowedFlatBufferBuilder fbb = transport_->CreateBuilder(
      message_group_id, serialized_size + kScratchBufferCapacity);

  // Assumption is that serialization for frame updates happens fast and does
  // not require offloading to the background thread.
  const std::optional<flatbuffers::Offset<Command>> offset =
      batch_base.Serialize(*fbb);

  if (offset) {
    
  }
}

void SplitEngineIpcClient::SendAllBatches() {
  // Clear any message groups that were released by the transport.
  transport_->ClearReleasedMessageGroups();

  size_t total_size = 0;
  batch_manager_.ForEachBatchRun([&total_size](BatchBase& batch) {
    size_t size = batch.GetSerializedSize();
    if (size > 0) {
      total_size += size + kScratchBufferCapacity + 32;
    }
  });
  if (total_size == 0) return;

  absl::StatusOr<MessageGroupId> group_id =
      transport_->BeginFrameUpdate(total_size);
  if (!group_id.ok()) {
    IMP_LOG(imp::ERROR) << kTag << "Failed to begin frame update: "
               << group_id.status().ToString();
    return;
  }

  batch_manager_.ForEachBatchRunAndConsume(
      [this, group_id = *group_id](BatchBase& batch) {
        SerializeBatch(group_id, batch);
      });

  absl::Status transport_end_status = transport_->End(*group_id);
  if (!transport_end_status.ok()) {
    IMP_LOG(imp::ERROR) << kTag << "Failed to end frame update: "
               << transport_end_status.ToString();
  }
}

absl::StatusOr<int32_t> SplitEngineIpcClient::GetActiveFrameUpdatesCount()
    const {
  return transport_->GetActiveFrameUpdatesCount();
}

}  // namespace split_engine
