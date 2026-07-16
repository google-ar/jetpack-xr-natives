// Copyright 2024 Google LLC
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

#include "core/split_engine/split_engine_serializer_impl.h"

#include <sys/types.h>

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "absl/base/nullability.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "filament/filament/include/filament/MorphTargetBuffer.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/string.h"
#include "flatbuffers/vector.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/invocable.h"
#include "core/common/owned_ptr.h"
#include "core/common/type_helpers.h"
#include "core/config.h"
#include "core/geometry/shapes/box.h"
#include "core/geometry/shapes/capsule.h"
#include "core/geometry/shapes/sphere.h"
#include "core/lighting/image_based_lighting_types.h"
#include "core/loader/provider/details/loaded_model_builder.h"
#include "core/material_library/flatbuffer_utils.h"
#include "core/material_library/generic_material_spec.h"
#include "core/material_library/material_param_value.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/model/mesh/base_mesh_builder.h"
#include "core/render/base_texture_builder.h"
#include "core/split_engine/android/split_engine_android_bridge.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/image_based_lighting_helpers.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_custom_material.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_mesh_serializer.h"
#include "core/split_engine/split_engine_renderable_builder.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/split_engine/split_engine_serializer_data_types.h"
#include "core/split_engine/split_engine_serializer_transport.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"
#if IMP_PLATFORM(ANDROID)
#include "core/render/android/android_defines.h"
#include "core/render/android/platform_android_external_texture_surface.h"
#endif
#include "core/render/base_renderable_manager.h"
#if IMP_PLATFORM(ANDROID)
#include "core/split_engine/android/split_engine_platform_android_external_texture_surface.h"
#endif
#include "core/render_passes/texture_pipeline_renderer_state.proto.imp.h"
#include "core/split_engine/flatbuffer_size_calculator.h"
#include "core/split_engine/materials/split_engine_generic_material.h"
#include "core/split_engine/split_engine_mesh_builder.h"
#include "core/split_engine/split_engine_serializer_batch_manager.h"
#include "core/split_engine/split_engine_texture_builder.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"
#include "split_engine/schemas/split_engine_data_generated.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace imp::split_engine {

namespace {

static constexpr absl::string_view kTag = "[SplitEngineSerializer]: ";
static constexpr absl::string_view kIndent = "  ";

static constexpr Box kDefaultBox;

// This is used to ensure that we skip serializing the placeholder material when
// we are in local mode.
//
// This is needed for the legacy AddMaterial/RemoveMaterial commands.
// That being said, it still makes sense not to serialize the placeholder
// material anyways.
// TODO: (broken link) - Find a better way to check the placeholder material.
bool IsPlaceholderSplitEngineMaterial(const filament::Material* material) {
  return strcmp(material->getName(), "Split Engine Placeholder") == 0;
}

template <typename T>
[[nodiscard]]
flatbuffers::Offset<android_xr::schemas::Command> CreateCommand(
    flatbuffers::FlatBufferBuilder& fbb,
    flatbuffers::Offset<T> command_offset) {
  return android_xr::schemas::CreateCommand(
      fbb, android_xr::schemas::CommandTypesTraits<T>::enum_value,
      command_offset.Union());
}

template <typename T>
void LogMaterialParam(absl::string_view name,
                      android_xr::schemas::MaterialParamValue type,
                      const T& value) {
  IMP_LOG(imp::INFO) << kTag << kIndent << "material param: name: " << name
             << ", type: "
             << android_xr::schemas::EnumNameMaterialParamValue(type)
             << ", value: " << value;
}

template <typename T>
void LogMaterialParam(absl::string_view name,
                      android_xr::schemas::MaterialParamValue type,
                      const std::vector<T>& values) {
  std::stringstream ss;
  ss << "[";
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      ss << ", ";
    }
    ss << values[i];
  }
  ss << "]";
  IMP_LOG(imp::INFO) << kTag << kIndent << "material param: name: " << name
             << ", type: "
             << android_xr::schemas::EnumNameMaterialParamValue(type)
             << ", size " << values.size() << ", values: " << ss.str();
}

// Verify ColliderType enums match.
static_assert(DoEnumsMatch(SplitEngineSerializer::ColliderType::kBoxCollider,
                           android_xr::schemas::ColliderType::BoxCollider),
              "Enum mismatch");
static_assert(DoEnumsMatch(SplitEngineSerializer::ColliderType::kMeshCollider,
                           android_xr::schemas::ColliderType::MeshCollider),
              "Enum mismatch");
static_assert(DoEnumsMatch(SplitEngineSerializer::ColliderType::kSphereCollider,
                           android_xr::schemas::ColliderType::SphereCollider),
              "Enum mismatch");
static_assert(
    DoEnumsMatch(SplitEngineSerializer::ColliderType::kCapsuleCollider,
                 android_xr::schemas::ColliderType::CapsuleCollider),
    "Enum mismatch");
static_assert(android_xr::schemas::ColliderType::MAX ==
                  android_xr::schemas::ColliderType::CapsuleCollider,
              "New fields added but assert not updated");

// Verify BuiltInMaterialParameters enums match.
static_assert(
    DoEnumsMatch(BuiltInMaterialParameters::NONE,
                 android_xr::schemas::BuiltInMaterialParameters::NONE),
    "Enum mismatch");
static_assert(DoEnumsMatch(BuiltInMaterialParameters::GenericMaterialParameters,
                           android_xr::schemas::BuiltInMaterialParameters::
                               GenericMaterialParameters),
              "Enum mismatch");
static_assert(
    DoEnumsMatch(BuiltInMaterialParameters::BuiltInMaterial5cf26af8Parameters,
                 android_xr::schemas::BuiltInMaterialParameters::
                     BuiltInMaterial5cf26af8Parameters),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(BuiltInMaterialParameters::BuiltInMaterialE3ca0ab9Parameters,
                 android_xr::schemas::BuiltInMaterialParameters::
                     BuiltInMaterialE3ca0ab9Parameters),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(BuiltInMaterialParameters::BuiltInMaterialD1750064Parameters,
                 android_xr::schemas::BuiltInMaterialParameters::
                     BuiltInMaterialD1750064Parameters),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(BuiltInMaterialParameters::BuiltInMaterialEb117dd9Parameters,
                 android_xr::schemas::BuiltInMaterialParameters::
                     BuiltInMaterialEb117dd9Parameters),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(BuiltInMaterialParameters::BuiltInMaterial1b616c8aParameters,
                 android_xr::schemas::BuiltInMaterialParameters::
                     BuiltInMaterial1b616c8aParameters),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(BuiltInMaterialParameters::BuiltInMaterial0d0cb9aaParameters,
                 android_xr::schemas::BuiltInMaterialParameters::
                     BuiltInMaterial0d0cb9aaParameters),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(
        BuiltInMaterialParameters::BuiltInMaterialTextureExternalParameters,
        android_xr::schemas::BuiltInMaterialParameters::
            BuiltInMaterialTextureExternalParameters),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(BuiltInMaterialParameters::BuiltInMaterialbd7fe08cParameters,
                 android_xr::schemas::BuiltInMaterialParameters::
                     BuiltInMaterialbd7fe08cParameters),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(BuiltInMaterialParameters::BuiltInMaterialGsplatParameters,
                 android_xr::schemas::BuiltInMaterialParameters::
                     BuiltInMaterialGsplatParameters),
    "Enum mismatch");

static_assert(DoEnumsMatch(BuiltInMaterialParameters::MIN,
                           android_xr::schemas::BuiltInMaterialParameters::MIN),
              "Enum mismatch");
static_assert(DoEnumsMatch(BuiltInMaterialParameters::MAX,
                           android_xr::schemas::BuiltInMaterialParameters::MAX),
              "Enum mismatch");

static_assert(android_xr::schemas::BuiltInMaterialParameters::MAX ==
                  android_xr::schemas::BuiltInMaterialParameters::
                      BuiltInMaterialGsplatParameters,
              "New fields added but assert not updated");

}  // namespace

// TODO: (broken link) - Add more using statements (esp. android_xr::schemas) to
// shorten code and make it more readable.
using android_xr::schemas::CommandTypes;
using filament::Box;
using filament::IndexBuffer;
using filament::VertexBuffer;

SplitEngineSerializerImpl::SplitEngineSerializerImpl(
    BaseView& view, int32_t api_level,
    imp::OwnedPtr<SplitEngineSerializerTransport> transport,
    size_t bridge_buffer_size_bytes)
    : Updater(view),
      view_(view),
      api_level_(api_level),
      transport_(std::move(transport)),
      bridge_buffer_size_bytes_(bridge_buffer_size_bytes),
      batch_manager_() {
  view_.GetRenderableManager().SetSpy(*this);
}

void SplitEngineSerializerImpl::SetSpy(BaseRenderableManager& spy) {}

filament::RenderableManager::Instance SplitEngineSerializerImpl::GetInstance(
    utils::Entity entity) const {
  return 0;
}

bool SplitEngineSerializerImpl::HasComponent(utils::Entity entity) const {
  return false;
}

void SplitEngineSerializerImpl::Destroy(utils::Entity entity) {
  SerializerDataTypes::Batch<CommandTypes::RemoveRenderables>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::RemoveRenderables>(
          {entity});
  batch.data.insert(entity);
}

size_t SplitEngineSerializerImpl::GetPrimitiveCount(
    filament::RenderableManager::Instance instance) const {
  return 0;
}
const Box& SplitEngineSerializerImpl::GetAxisAlignedBoundingBox(
    filament::RenderableManager::Instance instance) const {
  return kDefaultBox;
}

utils::Entity SplitEngineSerializerImpl::GetEntity(
    const filament::RenderableManager::Instance& instance) const {
  return view_.GetSharedEngine()->getRenderableManager().getEntity(instance);
}

void SplitEngineSerializerImpl::SetMaterialInstanceAt(
    filament::RenderableManager::Instance instance, size_t primitiveIndex,
    const filament::MaterialInstance* material_instance) {
  utils::Entity entity = GetEntity(instance);
  const ResourceId material_instance_id = GetId(material_instance);
  SerializerDataTypes::Batch<CommandTypes::UpdateRenderables>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::UpdateRenderables>(
          {entity}, {material_instance_id});
  batch.data[entity].primitives[primitiveIndex].material_instance_id =
      material_instance_id;
}
void SplitEngineSerializerImpl::SetGeometryAt(
    filament::RenderableManager::Instance instance, size_t primitiveIndex,
    filament::backend::PrimitiveType type, VertexBuffer* vertices,
    IndexBuffer* indices, size_t offset, size_t count) {
  const ResourceId vertex_buffer_id = GetId(vertices);
  const ResourceId index_buffer_id = GetId(indices);
  SerializerDataTypes::GeometryUpdateInfo geometry_update_info;
  geometry_update_info.vertex_buffer_id = vertex_buffer_id;
  geometry_update_info.index_buffer_id = index_buffer_id;
  geometry_update_info.offset = offset;
  geometry_update_info.count = count;
  geometry_update_info.primitive_type = static_cast<uint8_t>(type);

  utils::Entity entity = GetEntity(instance);

  SerializerDataTypes::Batch<CommandTypes::UpdateRenderables>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::UpdateRenderables>(
          {entity}, {vertex_buffer_id, index_buffer_id});
  batch.data[entity].primitives[primitiveIndex].geometry = geometry_update_info;
}
void SplitEngineSerializerImpl::SetBonesInternal(
    filament::RenderableManager::Instance instance,
    filament::RenderableManager::Bone const* transforms, size_t boneCount,
    size_t offset) {
  IMP_LOG(imp::FATAL) << kTag
             << "setBones(entity, filament::RenderableManager::Bone*) is not "
                "supported";
}

void SplitEngineSerializerImpl::SetBonesInternal(
    filament::RenderableManager::Instance instance, mat4f const* transforms,
    size_t boneCount, size_t offset) {
  if (offset != 0) {
    IMP_LOG(imp::FATAL) << kTag << "Nonzero offset is not supported.";
  }

  utils::Entity entity = GetEntity(instance);
  SerializerDataTypes::Batch<CommandTypes::UpdateRenderables>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::UpdateRenderables>(
          {entity});
  imp::BorrowedPtr<SplitEngineSerializerTransport::MessageBuilder> fbb =
      BorrowFlatBufferBuilder(batch);
  batch.data[entity].bones = android_xr::schemas::CreateBones(
      **fbb, (*fbb)->CreateVectorOfNativeStructs<android_xr::schemas::Mat4f>(
                 transforms, boneCount, Pack));
}

void SplitEngineSerializerImpl::SetAxisAlignedBoundingBox(
    filament::RenderableManager::Instance instance, const Box& aabb) {
  utils::Entity entity = GetEntity(instance);
  SerializerDataTypes::Batch<CommandTypes::UpdateRenderables>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::UpdateRenderables>(
          {entity});
  batch.data[entity].bounds = aabb;
}

void SplitEngineSerializerImpl::SetPriority(
    filament::RenderableManager::Instance instance, uint8_t priority) {
  utils::Entity entity = GetEntity(instance);
  SerializerDataTypes::Batch<CommandTypes::UpdateRenderables>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::UpdateRenderables>(
          {entity});
  batch.data[entity].priority = priority;
}

void SplitEngineSerializerImpl::SetChannel(
    filament::RenderableManager::Instance instance, uint8_t channel) {
  IMP_LOG(imp::WARNING) << kTag << "SetChannel not implemented.";
}

uint8_t SplitEngineSerializerImpl::GetLayerMask(
    filament::RenderableManager::Instance instance) const {
  return 0;
}

void SplitEngineSerializerImpl::SetLayerMask(
    filament::RenderableManager::Instance instance, uint8_t select,
    uint8_t values) {
  utils::Entity entity = GetEntity(instance);
  SerializerDataTypes::Batch<CommandTypes::UpdateRenderables>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::UpdateRenderables>(
          {entity});
  batch.data[entity].layer_mask =
      SerializerDataTypes::LayerMask{select, values};
}

void SplitEngineSerializerImpl::SetBlendOrderAt(
    filament::RenderableManager::Instance instance, size_t primitiveIndex,
    uint16_t order) {
  utils::Entity entity = GetEntity(instance);
  SerializerDataTypes::Batch<CommandTypes::UpdateRenderables>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::UpdateRenderables>(
          {entity});
  batch.data[entity].primitives[primitiveIndex].blend_order = order;
}

void SplitEngineSerializerImpl::SetGlobalBlendOrderEnabledAt(
    filament::RenderableManager::Instance instance, size_t primitiveIndex,
    bool enabled) {
  utils::Entity entity = GetEntity(instance);
  SerializerDataTypes::Batch<CommandTypes::UpdateRenderables>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::UpdateRenderables>(
          {entity});
  batch.data[entity].primitives[primitiveIndex].global_blend_order_enabled =
      enabled;
}

bool SplitEngineSerializerImpl::IsShadowCaster(
    filament::RenderableManager::Instance instance) const {
  return false;
}

void SplitEngineSerializerImpl::SetCastShadows(
    filament::RenderableManager::Instance instance, bool enable) {
  IMP_LOG(imp::WARNING) << kTag << "SetCastShadows not implemented.";
}

bool SplitEngineSerializerImpl::IsShadowReceiver(
    filament::RenderableManager::Instance instance) const {
  return false;
}

void SplitEngineSerializerImpl::SetReceiveShadows(
    filament::RenderableManager::Instance instance, bool enable) {
  IMP_LOG(imp::WARNING) << kTag << "SetReceiveShadows not implemented.";
}

bool SplitEngineSerializerImpl::GetFogEnabled(
    filament::RenderableManager::Instance instance) const {
  return false;
}

void SplitEngineSerializerImpl::SetFogEnabled(
    filament::RenderableManager::Instance instance, bool enable) {
  IMP_LOG(imp::WARNING) << kTag << "SetFogEnabled not implemented.";
}

size_t SplitEngineSerializerImpl::GetMorphTargetCount(
    filament::RenderableManager::Instance instance) const {
  return 0;
}

void SplitEngineSerializerImpl::SetMorphWeights(
    filament::RenderableManager::Instance instance, float const* weights,
    size_t count, size_t offset) {
  utils::Entity entity = GetEntity(instance);
  SerializerDataTypes::Batch<CommandTypes::UpdateRenderables>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::UpdateRenderables>(
          {entity});
  imp::BorrowedPtr<SplitEngineSerializerTransport::MessageBuilder> fbb =
      BorrowFlatBufferBuilder(batch);

  batch.data[entity].morph_weights = android_xr::schemas::CreateMorphWeights(
      **fbb, (*fbb)->CreateVector(weights, count), offset);
}

bool SplitEngineSerializerImpl::IsCullingEnabled(
    filament::RenderableManager::Instance instance) const {
  return false;
}

std::unique_ptr<BaseRenderableManager::Builder>
SplitEngineSerializerImpl::NewBuilder(size_t count) {
  return std::make_unique<SplitEngineRenderableBuilder>(
      [this](utils::Entity entity,
             SerializerDataTypes::AddRenderableInfo add_info,
             SerializerDataTypes::UpdateRenderableInfo update_info) {
        SerializerDataTypes::Batch<CommandTypes::AddRenderables>& add_batch =
            batch_manager_.GetOrCreateBatch<CommandTypes::AddRenderables>(
                {entity});
        add_batch.data[entity] = std::move(add_info);

        SerializerDataTypes::Batch<
            CommandTypes::UpdateRenderables>& update_batch =
            batch_manager_.GetOrCreateBatch<CommandTypes::UpdateRenderables>(
                {entity});
        update_batch.data[entity] = std::move(update_info);
      },
      count);
}

imp::OwnedPtr<SplitEngineSerializerTransport::MessageBuilder>
SplitEngineSerializerImpl::CreateFlatBufferBuilder(size_t size_bytes) {
  // Message groups are lazily began the first time anyone attempts to build a
  // message (create a FlatBufferBuilder) in a frame, and ended in Update() if
  // any messages were sent that frame.
  // There may be additional logic in the future to have multiple message groups
  // per frame, but for now it's always exactly one per frame.
  if (!frame_update_group_id_.has_value()) {
    const absl::StatusOr<MessageGroupId> group_id =
        transport_->BeginFrameUpdate(bridge_buffer_size_bytes_);
    
    frame_update_group_id_ = *group_id;
  }

  return transport_->CreateBuilder(*frame_update_group_id_, size_bytes);
}

imp::OwnedPtr<SplitEngineSerializerTransport::MessageBuilder>
SplitEngineSerializerImpl::CreateFlatBufferBuilder() {
  constexpr size_t kInitialSize = 1024;
  return CreateFlatBufferBuilder(kInitialSize);
}

void SplitEngineSerializerImpl::SerializeTexture(
    std::unique_ptr<const SplitEngineTextureSerializer>
        split_engine_texture_serializer,
    imp::Invocable<void()> on_done) {
  const size_t kNumTextures = 1;
  std::vector<size_t> image_buffer_sizes =
      split_engine_texture_serializer->GetTextureBufferSizes();
  const size_t kBufferSize = FlatbufferSizeCalculator()
                                 .AddTextureAndDependentData(image_buffer_sizes)
                                 .AddReferenceVector(kNumTextures)
                                 .AddAddTextureRequest(kNumTextures)
                                 .AddRequest()
                                 .Finish()
                                 .AddScratchSpace()
                                 .ComputeSize();
  const absl::StatusOr<MessageGroupId> group_id =
      transport_->BeginOneShot(kBufferSize);
  

  // MessageBuilder has to be shared between different tasks.
  imp::OwnedPtr<SplitEngineSerializerTransport::MessageBuilder> builder =
      transport_->CreateBuilder(*group_id, kBufferSize);

  // Offload potentially expensive serialization (copying huge amount of data)
  absl::Status status = transport_->AddMessage(
      *group_id, std::move(builder),
      [split_engine_texture_serializer =
           std::move(split_engine_texture_serializer)](
          flatbuffers::FlatBufferBuilder& builder) {
        flatbuffers::Offset<android_xr::schemas::Texture> offset =
            split_engine_texture_serializer->SerializeTexture(builder);

        VectorOffset<android_xr::schemas::Texture> texture_vector;
        texture_vector.push_back(offset);
        return CreateCommand(
            builder, android_xr::schemas::CreateAddTextures(
                         builder, builder.CreateVector(texture_vector)));
      });
  if (!status.ok()) IMP_LOG(imp::FATAL) << "Failed to add message: " << status;

  status = transport_->End(*group_id);
  if (!status.ok()) IMP_LOG(imp::FATAL) << "Failed to end message group: " << status;

  transport_->Schedule([on_done = std::move(on_done)]() {
    on_done();
    return absl::OkStatus();
  });
}

void SplitEngineSerializerImpl::RemoveTexture(filament::Texture& texture) {
  const ResourceId texture_id = GetId(&texture);
  SerializerDataTypes::Batch<CommandTypes::RemoveTextures>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::RemoveTextures>();
  batch.data.push_back(texture_id);

  // It is safe to remove the id now, because the Texture serializer uses
  // stored texture_id instead of calling SplitEngineSerializer::GetId
  RemoveId(texture_id);
}

void SplitEngineSerializerImpl::SerializeMesh(
    std::unique_ptr<const SplitEngineMeshSerializer>
        split_engine_mesh_serializer) {
  SerializeMeshIndicesAndVertices(*split_engine_mesh_serializer);
  // SerializeMeshMorphTargets is the last call in this routine, and it shall
  // take ownership of the unique_ptr
  SerializeMeshMorphTargets(std::move(split_engine_mesh_serializer));
}

void SplitEngineSerializerImpl::SerializeMeshIndicesAndVertices(
    const SplitEngineMeshSerializer& split_engine_mesh_serializer) {
  FlatbufferSizeCalculator mesh_calculator;
  split_engine_mesh_serializer.ContributeVertexBufferSizes(mesh_calculator);
  split_engine_mesh_serializer.ContributeIndexBufferSizes(mesh_calculator);

  const size_t kAddMeshBufferSize = mesh_calculator.AddAddMeshData()
                                        .AddRequest()
                                        .Finish()
                                        .AddScratchSpace()
                                        .ComputeSize();

  const absl::StatusOr<MessageGroupId> group_id =
      transport_->BeginOneShot(kAddMeshBufferSize);
  

  imp::OwnedPtr<SplitEngineSerializerTransport::MessageBuilder> mesh_builder =
      transport_->CreateBuilder(*group_id, kAddMeshBufferSize);

  // Offload potentially expensive serialization (copying huge amount of data)
  auto status = transport_->AddMessage(
      *group_id, std::move(mesh_builder),
      [&split_engine_mesh_serializer,
       group_id = *group_id](flatbuffers::FlatBufferBuilder& builder) {
        SplitEngineMeshSerializer::VertexBufferVector vertex_buffer_offsets =
            split_engine_mesh_serializer.SerializeVertexBuffers(builder);

        SplitEngineMeshSerializer::IndexBufferVector index_buffer_offsets =
            split_engine_mesh_serializer.SerializeIndexBuffers(builder);

        return CreateCommand(
            builder, android_xr::schemas::CreateAddMeshData(
                         builder, vertex_buffer_offsets, index_buffer_offsets));
      });
  if (!status.ok()) IMP_LOG(imp::FATAL) << "Failed to add message: " << status;

  status = transport_->End(*group_id);
  if (!status.ok()) IMP_LOG(imp::FATAL) << "Failed to end message group: " << status;
}

void SplitEngineSerializerImpl::SerializeMeshMorphTargets(
    std::unique_ptr<const SplitEngineMeshSerializer>
        split_engine_mesh_serializer) {
  FlatbufferSizeCalculator morph_target_calculator;
  split_engine_mesh_serializer->ContributeMorphTargetBufferSizes(
      morph_target_calculator);
  if (morph_target_calculator.ComputeSize() == 0) {
    // This mesh has no morph targets, so there's nothing to send.
    //
    // We transfer the ownership of the unique_ptr to the lambda, so that
    // the destructor of `split_engine_mesh_serializer` is called after the
    // serialization is completed.
    transport_->Schedule(
        [keep_alive = std::move(split_engine_mesh_serializer)]() {
          return absl::OkStatus();
        });
    return;
  }

  const size_t kAddMorphTargetBufferSize =
      morph_target_calculator.AddAddMorphTargetBuffers()
          .AddRequest()
          .Finish()
          .AddScratchSpace()
          .ComputeSize();

  const absl::StatusOr<MessageGroupId> group_id =
      transport_->BeginOneShot(kAddMorphTargetBufferSize);
  

  imp::OwnedPtr<SplitEngineSerializerTransport::MessageBuilder>
      morph_target_buffer_builder =
          transport_->CreateBuilder(*group_id, kAddMorphTargetBufferSize);

  // Offload potentially expensive serialization (copying huge amount of data)
  auto status = transport_->AddMessage(
      *group_id, std::move(morph_target_buffer_builder),
      [group_id = *group_id,

       split_engine_mesh_serializer = std::move(split_engine_mesh_serializer)](
          flatbuffers::FlatBufferBuilder& builder) {
        SplitEngineMeshSerializer::MorphTargetBufferVector
            morph_buffer_offsets =
                split_engine_mesh_serializer->SerializeMorphTargetBuffers(
                    builder);

        return CreateCommand(builder,
                             android_xr::schemas::CreateAddMorphTargetBuffers(
                                 builder, morph_buffer_offsets));
      });
  if (!status.ok()) IMP_LOG(imp::FATAL) << "Failed to add message: " << status;

  status = transport_->End(*group_id);
  if (!status.ok()) IMP_LOG(imp::FATAL) << "Failed to end message group: " << status;
}

size_t SplitEngineSerializerImpl::EstimateImageBasedLightingAssetBufferSize(
    const SphericalHarmonics* /*absl_nullable*/  spherical_harmonics,
    const ImageBasedLightingAssetCubemapImages& cubemap_images) {
  FlatbufferSizeCalculator calculator;

  for (const CubemapLevelImageContents& ibl_cubemap_image :
       cubemap_images.ibl_cubemap_images) {
    const size_t imageBufferSize =
        ibl_cubemap_image.stitched_face_image->GetSize();
    calculator.AddCubemapLevelImageContentsAndDependentData(imageBufferSize);
  }
  if (spherical_harmonics) {
    calculator.AddFloat3Vector(spherical_harmonics->coefficients.size())
        .AddSphericalHarmonics();
  }

  if (cubemap_images.skybox_cubemap_images.has_value()) {
    const size_t imageBufferSize = cubemap_images.skybox_cubemap_images.value()
                                       .stitched_face_image->GetSize();
    calculator.AddCubemapLevelImageContentsAndDependentData(imageBufferSize);
  }
  calculator.AddReferenceVector(cubemap_images.ibl_cubemap_images.size());

  return calculator.AddImageBasedLightingAsset()
      .AddImageBasedLightingAsset()
      .AddReferenceVector(1)
      .AddAddImageBasedLightingAssets()
      .AddRequest()
      .Finish()
      .AddScratchSpace()
      .ComputeSize();
}

void SplitEngineSerializerImpl::SerializeImageBasedLightingAsset(
    filament::Texture& reflection_texture,
    std::unique_ptr<SphericalHarmonics> /*absl_nullable*/  spherical_harmonics,
    ImageBasedLightingAssetCubemapImages cubemap_images) {
  const size_t kBufferSize = EstimateImageBasedLightingAssetBufferSize(
      spherical_harmonics.get(), cubemap_images);
  const ResourceId texture_id = GetId(&reflection_texture);

  const absl::StatusOr<MessageGroupId> group_id =
      transport_->BeginOneShot(kBufferSize);
  

  imp::OwnedPtr<SplitEngineSerializerTransport::MessageBuilder> builder =
      transport_->CreateBuilder(*group_id, kBufferSize);

  // Offload potentially expensive serialization (copying huge amount of data)
  auto status = transport_->AddMessage(
      *group_id, std::move(builder),
      [texture_id, spherical_harmonics = std::move(spherical_harmonics),
       cubemap_images =
           std::move(cubemap_images)](flatbuffers::FlatBufferBuilder& builder) {
        flatbuffers::Offset<android_xr::schemas::ImageBasedLightingAsset>
            asset = PackImageBasedLightingAsset(
                builder, texture_id, spherical_harmonics.get(), cubemap_images);
        return CreateCommand(
            builder, android_xr::schemas::CreateAddImageBasedLightingAssets(
                         builder, builder.CreateVector({asset})));
      });
  if (!status.ok()) IMP_LOG(imp::FATAL) << "Failed to add message: " << status;

  status = transport_->End(*group_id);
  if (!status.ok()) IMP_LOG(imp::FATAL) << "Failed to end message group: " << status;
}

void SplitEngineSerializerImpl::RemoveImageBasedLightingAsset(
    filament::Texture& reflection_texture) {
  const ResourceId ibl_id = GetId(&reflection_texture);
  SerializerDataTypes::Batch<CommandTypes::RemoveImageBasedLightingAssets>&
      batch =
          batch_manager_
              .GetOrCreateBatch<CommandTypes::RemoveImageBasedLightingAssets>();
  batch.data.push_back(ibl_id);
  RemoveId(ibl_id);
}

void SplitEngineSerializerImpl::SetPreferredEnvironmentIblAsset(
    filament::Texture& reflection_texture, float intensity,
    const float3& tint) {
  const ResourceId ibl_id = GetId(&reflection_texture);
  SerializerDataTypes::Batch<CommandTypes::SetPreferredEnvironmentIblAsset>&
      batch =
          batch_manager_
              .GetOrCreateBatch<CommandTypes::SetPreferredEnvironmentIblAsset>(
                  {}, {ibl_id});
  batch.data = SerializerDataTypes::EnvironmentLightParams{
      .image_based_lighting_asset_id = ibl_id,
      .intensity = intensity,
      .tint = tint};
}

void SplitEngineSerializerImpl::ClearPreferredEnvironmentIblAsset() {
  // Setting the image based lighting asset id to 0 indicates to clear any
  // previously-set preferred environment IBL asset.
  SerializerDataTypes::Batch<
      CommandTypes::SetPreferredEnvironmentIblAsset>& batch =
      batch_manager_
          .GetOrCreateBatch<CommandTypes::SetPreferredEnvironmentIblAsset>();
  batch.data = SerializerDataTypes::EnvironmentLightParams{
      .image_based_lighting_asset_id = 0, .intensity = 0, .tint = {0, 0, 0}};
}

void SplitEngineSerializerImpl::RemoveMorphTargetBuffer(
    filament::MorphTargetBuffer* morph_target_buffer) {
  const ResourceId buffer_id = GetId(morph_target_buffer);
  SerializerDataTypes::Batch<CommandTypes::RemoveMorphTargetBuffers>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::RemoveMorphTargetBuffers>();
  batch.data.push_back(buffer_id);

  // It is safe to remove the id now, because the MorphTargetBuffer serializer
  // uses stored morph_target_buffer_id instead of calling
  // SplitEngineSerializer::GetId
  RemoveId(buffer_id);
}

void SplitEngineSerializerImpl::RemoveVertexBuffer(
    VertexBuffer* vertex_buffer) {
  const ResourceId buffer_id = GetId(vertex_buffer);
  SerializerDataTypes::Batch<CommandTypes::RemoveMeshData>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::RemoveMeshData>();
  batch.data.vertex_buffers.push_back(buffer_id);

  // It is safe to remove the id now, because the VertexBuffer serializer uses
  // stored vertex_buffer_id instead of calling SplitEngineSerializer::GetId
  RemoveId(buffer_id);
}

void SplitEngineSerializerImpl::RemoveIndexBuffer(IndexBuffer* index_buffer) {
  const ResourceId buffer_id = GetId(index_buffer);
  SerializerDataTypes::Batch<CommandTypes::RemoveMeshData>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::RemoveMeshData>();
  batch.data.index_buffers.push_back(buffer_id);

  // It is safe to remove the id now, because the IndexBuffer serializer uses
  // stored index_buffer_id instead of calling SplitEngineSerializer::GetId
  RemoveId(buffer_id);
}

imp::BorrowedPtr<SplitEngineSerializerTransport::MessageBuilder>
SplitEngineSerializerImpl::BorrowFlatBufferBuilder(
    SerializerDataTypes::CommandBatchBase& batch) {
  // Return existing or create new FlatBufferBuilder.
  auto it = fbb_.find(&batch);
  if (it != fbb_.end()) {
    return it->second.Borrow();
  }

  return fbb_.emplace(&batch, CreateFlatBufferBuilder()).first->second.Borrow();
}

imp::OwnedPtr<SplitEngineSerializerTransport::MessageBuilder>
SplitEngineSerializerImpl::ReleaseFlatBufferBuilder(
    SerializerDataTypes::CommandBatchBase& batch) {
  auto it = fbb_.find(&batch);
  if (it == fbb_.end()) {
    // Some commands does not create flatbuffer builder at the time of batch
    // creation. In this case we just return an empty builder.
    return CreateFlatBufferBuilder();
  }
  imp::OwnedPtr<SplitEngineSerializerTransport::MessageBuilder> fbb =
      std::move(it->second);
  fbb_.erase(it);
  return fbb;
}

int32_t SplitEngineSerializerImpl::GetApiLevel() const { return api_level_; }

SplitEngineAndroidBridge& SplitEngineSerializerImpl::GetBridge() {
  absl::StatusOr<
      std::reference_wrapper<imp::split_engine::SplitEngineAndroidBridge>>
      bridge = transport_->GetBridge();
  
  return *bridge;
}

bool SplitEngineSerializerImpl::ReadyForNextFrame() const {
  const absl::StatusOr<int32_t> in_flight_frame_count =
      transport_->GetActiveFrameUpdatesCount();
  return in_flight_frame_count.ok() &&
         *in_flight_frame_count < kMaxInFlightFrames;
}

void SplitEngineSerializerImpl::AddMaterial(
    const filament::Material* material, const BufferAccess& data,
    const MaterialPreCompileOptions& material_pre_compile_options) {
  if (IsPlaceholderSplitEngineMaterial(material) ||
      !view_.AreSplitEngineMaterialsInLocalMode()) {
    return;
  }

  const ResourceId material_id = GetId(material);
  IMP_LOG(imp::INFO) << kTag << "add material: " << material_id;
  SerializerDataTypes::Batch<CommandTypes::AddMaterials>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::AddMaterials>(
          {}, {material_id});
  imp::BorrowedPtr<SplitEngineSerializerTransport::MessageBuilder> fbb =
      BorrowFlatBufferBuilder(batch);

  std::vector<
      flatbuffers::Offset<android_xr::schemas::MaterialPrecompileConstant>>
      constants;
  constants.reserve(material_pre_compile_options.constants.size());
  IMP_LOG(imp::INFO) << kTag << kIndent << "material precompile constant size: "
             << material_pre_compile_options.constants.size();

  for (const auto& constant : material_pre_compile_options.constants) {
    switch (constant.value.index()) {
      case MaterialPreCompileConstant::kValue_IntValue:
        constants.push_back(
            android_xr::schemas::CreateMaterialPrecompileConstant(
                **fbb, (*fbb)->CreateString(constant.name),
                android_xr::schemas::MaterialPrecompileConstantValue::Int,
                (*fbb)->CreateStruct(Pack(*constant.int_value())).Union()));
        IMP_LOG(imp::INFO) << kTag << kIndent
                   << "material precompile constant: " << constant.name
                   << " int value: " << *constant.int_value();
        break;
      case MaterialPreCompileConstant::kValue_FloatValue:
        constants.push_back(
            android_xr::schemas::CreateMaterialPrecompileConstant(
                **fbb, (*fbb)->CreateString(constant.name),
                android_xr::schemas::MaterialPrecompileConstantValue::Float,
                (*fbb)->CreateStruct(Pack(*constant.float_value())).Union()));
        IMP_LOG(imp::INFO) << kTag << kIndent
                   << "material precompile constant: " << constant.name
                   << " float value: " << *constant.float_value();
        break;
      case MaterialPreCompileConstant::kValue_BoolValue:
        constants.push_back(
            android_xr::schemas::CreateMaterialPrecompileConstant(
                **fbb, (*fbb)->CreateString(constant.name),
                android_xr::schemas::MaterialPrecompileConstantValue::Bool,
                (*fbb)->CreateStruct(Pack(*constant.bool_value())).Union()));
        IMP_LOG(imp::INFO) << kTag << kIndent
                   << "material precompile constant: " << constant.name
                   << " boolean value: " << *constant.bool_value();
        break;
      case MaterialPreCompileConstant::kValue_Unknown:
        IMP_LOG(imp::FATAL) << kTag << kIndent << "Unknown material precompile constant";
        break;
    }
  }

  flatbuffers::Offset<android_xr::schemas::MaterialPrecompileOptions> options =
      android_xr::schemas::CreateMaterialPrecompileOptions(
          **fbb, (*fbb)->CreateVector(constants));

  batch.data.push_back(android_xr::schemas::CreateMaterial(
      **fbb, material_id, (*fbb)->CreateVector(data.Data(), data.Size()),
      options));
}

void SplitEngineSerializerImpl::RemoveMaterial(
    const filament::Material* material) {
  const ResourceId material_id = GetId(material);
  IMP_LOG(imp::INFO) << kTag << "remove material: " << material_id;
  SerializerDataTypes::Batch<CommandTypes::RemoveMaterials>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::RemoveMaterials>();
  batch.data.push_back(material_id);
  RemoveId(material_id);
}

void SplitEngineSerializerImpl::AddMaterialInstance(
    const filament::Material* material,
    const filament::MaterialInstance* instance) {
  ResourceId material_id = GetId(material);
  ResourceId instance_id = GetId(instance);
  SerializerDataTypes::Batch<CommandTypes::AddMaterialInstances>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::AddMaterialInstances>(
          {}, {material_id, instance_id});
  batch.data.insert({instance_id, material_id});
}

flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> AddMaterialParam(
    flatbuffers::FlatBufferBuilder& fbb, absl::string_view name,
    const MaterialParamValue& value) {
  struct Visitor {
    flatbuffers::FlatBufferBuilder& fbb;
    absl::string_view name;
    flatbuffers::Offset<flatbuffers::String> fb_name;

    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const float value) {
      LogMaterialParam(name, android_xr::schemas::MaterialParamValue::Float,
                       value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Float,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const float2& value) {
      LogMaterialParam(name, android_xr::schemas::MaterialParamValue::Float2,
                       value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Float2,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const float3& value) {
      LogMaterialParam(name, android_xr::schemas::MaterialParamValue::Float3,
                       value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Float3,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const float4& value) {
      LogMaterialParam(name, android_xr::schemas::MaterialParamValue::Float4,
                       value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Float4,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const int value) {
      LogMaterialParam(name, android_xr::schemas::MaterialParamValue::Int,
                       value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Int,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const int2& value) {
      LogMaterialParam(name, android_xr::schemas::MaterialParamValue::Int2,
                       value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Int2,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const int3& value) {
      LogMaterialParam(name, android_xr::schemas::MaterialParamValue::Int3,
                       value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Int3,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const int4& value) {
      LogMaterialParam(name, android_xr::schemas::MaterialParamValue::Int4,
                       value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Int4,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const uint value) {
      LogMaterialParam(name, android_xr::schemas::MaterialParamValue::Uint,
                       value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Uint,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const uint2& value) {
      LogMaterialParam(name, android_xr::schemas::MaterialParamValue::Uint2,
                       value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Uint2,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const uint3& value) {
      LogMaterialParam(name, android_xr::schemas::MaterialParamValue::Uint3,
                       value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Uint3,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const uint4& value) {
      LogMaterialParam(name, android_xr::schemas::MaterialParamValue::Uint4,
                       value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Uint4,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const bool value) {
      LogMaterialParam(name, android_xr::schemas::MaterialParamValue::Bool,
                       value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Bool,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const bool2& value) {
      LogMaterialParam(name, android_xr::schemas::MaterialParamValue::Bool2,
                       value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Bool2,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const bool3& value) {
      LogMaterialParam(name, android_xr::schemas::MaterialParamValue::Bool3,
                       value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Bool3,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const bool4& value) {
      LogMaterialParam(name, android_xr::schemas::MaterialParamValue::Bool4,
                       value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Bool4,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const loader::details::LoadedModelBuilder::MaterialTextureId&
            raw_value) {
      IMP_LOG(imp::FATAL) << kTag
                 << "SetMaterialParam(TextureId) should never be called. It "
                    "should be converted into a Texture* instead.";
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name,
          android_xr::schemas::MaterialParamValue::MaterialTextureId,
          fbb.CreateStruct(
                 android_xr::schemas::MaterialTextureId(uint16_t{raw_value}))
              .Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const mat3f& value) {
      LogMaterialParam(name, android_xr::schemas::MaterialParamValue::Mat3f,
                       value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Mat3f,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const mat4f& value) {
      LogMaterialParam(name, android_xr::schemas::MaterialParamValue::Mat4f,
                       value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Mat4f,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const std::vector<float>& value) {
      LogMaterialParam(
          name, android_xr::schemas::MaterialParamValue::FloatVector, value);
      std::vector<android_xr::schemas::Float> packed_values;
      packed_values.reserve(value.size());
      for (const auto& v : value) {
        packed_values.push_back(Pack(v));
      }
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::FloatVector,
          android_xr::schemas::CreateFloatVector(
              fbb, fbb.CreateVectorOfStructs(packed_values))
              .Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const std::vector<float2>& value) {
      LogMaterialParam(
          name, android_xr::schemas::MaterialParamValue::Float2Vector, value);
      std::vector<android_xr::schemas::Float2> packed_values;
      packed_values.reserve(value.size());
      for (const auto& v : value) {
        packed_values.push_back(Pack(v));
      }
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Float2Vector,
          android_xr::schemas::CreateFloat2Vector(
              fbb, fbb.CreateVectorOfStructs(packed_values))
              .Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const std::vector<float3>& value) {
      LogMaterialParam(
          name, android_xr::schemas::MaterialParamValue::Float3Vector, value);
      std::vector<android_xr::schemas::Float3> packed_values;
      packed_values.reserve(value.size());
      for (const auto& v : value) {
        packed_values.push_back(Pack(v));
      }
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Float3Vector,
          android_xr::schemas::CreateFloat3Vector(
              fbb, fbb.CreateVectorOfStructs(packed_values))
              .Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const std::vector<float4>& value) {
      LogMaterialParam(
          name, android_xr::schemas::MaterialParamValue::Float4Vector, value);
      std::vector<android_xr::schemas::Float4> packed_values;
      packed_values.reserve(value.size());
      for (const auto& v : value) {
        packed_values.push_back(Pack(v));
      }
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Float4Vector,
          android_xr::schemas::CreateFloat4Vector(
              fbb, fbb.CreateVectorOfStructs(packed_values))
              .Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const std::vector<int>& value) {
      LogMaterialParam(name, android_xr::schemas::MaterialParamValue::IntVector,
                       value);
      std::vector<android_xr::schemas::Int> packed_values;
      packed_values.reserve(value.size());
      for (const auto& v : value) {
        packed_values.push_back(Pack(v));
      }
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::IntVector,
          android_xr::schemas::CreateIntVector(
              fbb, fbb.CreateVectorOfStructs(packed_values))
              .Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const std::vector<int2>& value) {
      LogMaterialParam(
          name, android_xr::schemas::MaterialParamValue::Int2Vector, value);
      std::vector<android_xr::schemas::Int2> packed_values;
      packed_values.reserve(value.size());
      for (const auto& v : value) {
        packed_values.push_back(Pack(v));
      }
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Int2Vector,
          android_xr::schemas::CreateInt2Vector(
              fbb, fbb.CreateVectorOfStructs(packed_values))
              .Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const std::vector<int3>& value) {
      LogMaterialParam(
          name, android_xr::schemas::MaterialParamValue::Int3Vector, value);
      std::vector<android_xr::schemas::Int3> packed_values;
      packed_values.reserve(value.size());
      for (const auto& v : value) {
        packed_values.push_back(Pack(v));
      }
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Int3Vector,
          android_xr::schemas::CreateInt3Vector(
              fbb, fbb.CreateVectorOfStructs(packed_values))
              .Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const std::vector<int4>& value) {
      LogMaterialParam(
          name, android_xr::schemas::MaterialParamValue::Int4Vector, value);
      std::vector<android_xr::schemas::Int4> packed_values;
      packed_values.reserve(value.size());
      for (const auto& v : value) {
        packed_values.push_back(Pack(v));
      }
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Int4Vector,
          android_xr::schemas::CreateInt4Vector(
              fbb, fbb.CreateVectorOfStructs(packed_values))
              .Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const std::vector<uint>& value) {
      LogMaterialParam(
          name, android_xr::schemas::MaterialParamValue::UintVector, value);
      std::vector<android_xr::schemas::Uint> packed_values;
      packed_values.reserve(value.size());
      for (const auto& v : value) {
        packed_values.push_back(Pack(v));
      }
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::UintVector,
          android_xr::schemas::CreateUintVector(
              fbb, fbb.CreateVectorOfStructs(packed_values))
              .Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const std::vector<uint2>& value) {
      LogMaterialParam(
          name, android_xr::schemas::MaterialParamValue::Uint2Vector, value);
      std::vector<android_xr::schemas::Uint2> packed_values;
      packed_values.reserve(value.size());
      for (const auto& v : value) {
        packed_values.push_back(Pack(v));
      }
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Uint2Vector,
          android_xr::schemas::CreateUint2Vector(
              fbb, fbb.CreateVectorOfStructs(packed_values))
              .Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const std::vector<uint3>& value) {
      LogMaterialParam(
          name, android_xr::schemas::MaterialParamValue::Uint3Vector, value);
      std::vector<android_xr::schemas::Uint3> packed_values;
      packed_values.reserve(value.size());
      for (const auto& v : value) {
        packed_values.push_back(Pack(v));
      }
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Uint3Vector,
          android_xr::schemas::CreateUint3Vector(
              fbb, fbb.CreateVectorOfStructs(packed_values))
              .Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const std::vector<uint4>& value) {
      LogMaterialParam(
          name, android_xr::schemas::MaterialParamValue::Uint4Vector, value);
      std::vector<android_xr::schemas::Uint4> packed_values;
      packed_values.reserve(value.size());
      for (const auto& v : value) {
        packed_values.push_back(Pack(v));
      }
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Uint4Vector,
          android_xr::schemas::CreateUint4Vector(
              fbb, fbb.CreateVectorOfStructs(packed_values))
              .Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const std::vector<bool>& value) {
      LogMaterialParam(
          name, android_xr::schemas::MaterialParamValue::BoolVector, value);
      std::vector<android_xr::schemas::Bool> packed_values;
      packed_values.reserve(value.size());
      for (const auto& v : value) {
        packed_values.push_back(Pack(v));
      }
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::BoolVector,
          android_xr::schemas::CreateBoolVector(
              fbb, fbb.CreateVectorOfStructs(packed_values))
              .Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const std::vector<bool2>& value) {
      LogMaterialParam(
          name, android_xr::schemas::MaterialParamValue::Bool2Vector, value);
      std::vector<android_xr::schemas::Bool2> packed_values;
      packed_values.reserve(value.size());
      for (const auto& v : value) {
        packed_values.push_back(Pack(v));
      }
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Bool2Vector,
          android_xr::schemas::CreateBool2Vector(
              fbb, fbb.CreateVectorOfStructs(packed_values))
              .Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const std::vector<bool3>& value) {
      LogMaterialParam(
          name, android_xr::schemas::MaterialParamValue::Bool3Vector, value);
      std::vector<android_xr::schemas::Bool3> packed_values;
      packed_values.reserve(value.size());
      for (const auto& v : value) {
        packed_values.push_back(Pack(v));
      }
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Bool3Vector,
          android_xr::schemas::CreateBool3Vector(
              fbb, fbb.CreateVectorOfStructs(packed_values))
              .Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const std::vector<bool4>& value) {
      LogMaterialParam(
          name, android_xr::schemas::MaterialParamValue::Bool4Vector, value);
      std::vector<android_xr::schemas::Bool4> packed_values;
      packed_values.reserve(value.size());
      for (const auto& v : value) {
        packed_values.push_back(Pack(v));
      }
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Bool4Vector,
          android_xr::schemas::CreateBool4Vector(
              fbb, fbb.CreateVectorOfStructs(packed_values))
              .Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const std::vector<mat3f>& value) {
      LogMaterialParam(
          name, android_xr::schemas::MaterialParamValue::Mat3fVector, value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Mat3fVector,
          android_xr::schemas::CreateMat3fVector(
              fbb, fbb.CreateVectorOfNativeStructs<android_xr::schemas::Mat3f>(
                       value.data(), value.size(), Pack))
              .Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const std::vector<mat4f>& value) {
      LogMaterialParam(
          name, android_xr::schemas::MaterialParamValue::Mat4fVector, value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Mat4fVector,
          android_xr::schemas::CreateMat4fVector(
              fbb, fbb.CreateVectorOfNativeStructs<android_xr::schemas::Mat4f>(
                       value.data(), value.size(), Pack))
              .Union());
    }
  };

  return std::visit(
      Visitor{fbb, name, fbb.CreateString(name.data(), name.size())}, value);
}

void SplitEngineSerializerImpl::SetMaterialParameter(
    const filament::MaterialInstance* material, absl::string_view name,
    const MaterialParamValue& value) {
  const ResourceId material_id = GetId(material);
  SerializerDataTypes::Batch<CommandTypes::SetMaterialParameters>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::SetMaterialParameters>(
          {}, {material_id});
  imp::BorrowedPtr<SplitEngineSerializerTransport::MessageBuilder> fbb =
      BorrowFlatBufferBuilder(batch);

  batch.data[material_id].params.push_back(
      AddMaterialParam(**fbb, name, value));
}

void SplitEngineSerializerImpl::SetMaterialParameter(
    const filament::MaterialInstance* material, absl::string_view name,
    const filament::Texture* texture, const filament::TextureSampler& sampler) {
  const ResourceId material_id = GetId(material);
  SerializerDataTypes::Batch<CommandTypes::SetMaterialParameters>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::SetMaterialParameters>(
          {}, {material_id});
  imp::BorrowedPtr<SplitEngineSerializerTransport::MessageBuilder> fbb =
      BorrowFlatBufferBuilder(batch);

  flatbuffers::Offset<android_xr::schemas::TextureSampler> sampler_offset =
      CreateTextureSampler<SplitEngineTextureSamplerCreator>(**fbb, sampler);
  batch.data[material_id].texture_params.push_back(
      android_xr::schemas::CreateMaterialTextureParameter(
          **fbb, (*fbb)->CreateString(std::string(name)), GetId(texture),
          sampler_offset));
}

Future<GenericMaterialPtr> SplitEngineSerializerImpl::CreateGenericMaterial(
    const GenericMaterialSpec& spec) {
  return SplitEngineGenericMaterial::Create(view_, spec)
      .Then([](std::unique_ptr<SplitEngineGenericMaterial> material) {
        return static_cast<GenericMaterialPtr>(std::move(material));
      });
}

MaterialPtr SplitEngineSerializerImpl::CreateCustomMaterial(
    MaterialPtr material) {
  return std::make_unique<split_engine::SplitEngineCustomMaterial>(
      view_, std::move(material));
}

Future<absl::Status> SplitEngineSerializerImpl::RequestCustomFilamentMaterial(
    absl::string_view material_source, filament::Material* filament_material,
    const MaterialPreCompileOptions& precompile_options) {
  return SplitEngineCustomMaterial::RequestCustomFilamentMaterial(
      view_, material_source, filament_material, precompile_options);
}

void SplitEngineSerializerImpl::SetBuiltInMaterialParameters(
    const filament::MaterialInstance* material, BuiltInMaterialParameters type,
    SerializeBuiltInMaterialParametersFunc serialize_func) {
  // Note: this cast is safe because the BuiltInMaterialParameters enum is
  // a copy of the SplitEngineSerializer::BuiltInMaterialParameters enum, which
  // is also static_asserted to be equivalent.
  android_xr::schemas::BuiltInMaterialParameters schema_type =
      static_cast<android_xr::schemas::BuiltInMaterialParameters>(type);

  const ResourceId material_id = GetId(material);
  // Note: have to log this here because the material instance ID is lost after
  // the parameter values are serialized.
  IMP_LOG(imp::INFO) << kTag << "update material params for instance (built-in): "
            << material_id;
  SerializerDataTypes::Batch<CommandTypes::SetBuiltInMaterialParameters>&
      batch = batch_manager_
                  .GetOrCreateBatch<CommandTypes::SetBuiltInMaterialParameters>(
                      {}, {material_id});
  imp::BorrowedPtr<SplitEngineSerializerTransport::MessageBuilder> fbb =
      BorrowFlatBufferBuilder(batch);

  batch.data.push_back(
      android_xr::schemas::CreateBuiltInMaterialInstanceParameters(
          **fbb, material_id, schema_type, serialize_func(**fbb)));
}

void SplitEngineSerializerImpl::DuplicateMaterialInstance(
    const filament::MaterialInstance* instance,
    const filament::MaterialInstance* copy) {
  const ResourceId instance_id = GetId(instance);
  const ResourceId copy_id = GetId(copy);
  SerializerDataTypes::Batch<CommandTypes::DuplicateMaterialInstances>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::DuplicateMaterialInstances>(
          {}, {copy_id, instance_id});
  batch.data.push_back({copy_id, instance_id});
}

void SplitEngineSerializerImpl::RemoveMaterialInstance(
    const filament::MaterialInstance* instance) {
  const ResourceId instance_id = GetId(instance);
  SerializerDataTypes::Batch<CommandTypes::RemoveMaterialInstances>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::RemoveMaterialInstances>();
  batch.data.push_back(instance_id);
  RemoveId(instance_id);
}

void SplitEngineSerializerImpl::CreateNode(utils::Entity entity) {
  SerializerDataTypes::Batch<CommandTypes::AddNodes>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::AddNodes>({entity});
  batch.data.insert(entity);
}

void SplitEngineSerializerImpl::DestroyNode(
    utils::Entity entity, const std::vector<utils::Entity>& dependencies) {
  // The `dependencies` parameter specifies entities whose pending operations
  // must complete before this `RemoveNodes` command for `entity` can be
  // executed. For example, when Child Entity was a child of Parent Entity:
  // (1) RemoveNodes(StandaloneEntity)
  // (2) UpdateRenderables(ChildEntity)
  // (3) RemoveNodes(ParentEntity)
  // (1) and (3) shouldn't be batched together.

  SerializerDataTypes::Batch<CommandTypes::RemoveNodes>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::RemoveNodes>(dependencies);
  batch.data.insert(entity);
}

void SplitEngineSerializerImpl::SetEnabled(utils::Entity entity, bool enabled) {
  SerializerDataTypes::Batch<CommandTypes::UpdateNodes>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::UpdateNodes>({entity});
  batch.data[entity].enabled = enabled;
}

void SplitEngineSerializerImpl::SetName(utils::Entity entity,
                                        absl::string_view name) {
  SerializerDataTypes::Batch<CommandTypes::UpdateNodes>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::UpdateNodes>({entity});
  batch.data[entity].name = std::string(name);
}

void SplitEngineSerializerImpl::SetParent(utils::Entity entity,
                                          utils::Entity parent) {
  SerializerDataTypes::Batch<CommandTypes::UpdateNodes>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::UpdateNodes>(
          {entity, parent});
  batch.data[entity].parent = parent;
}

void SplitEngineSerializerImpl::SetLocalTransform(utils::Entity entity,
                                                  const mat4f& transform) {
  SerializerDataTypes::Batch<CommandTypes::UpdateNodes>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::UpdateNodes>({entity});
  batch.data[entity].transform = transform;
}
void SplitEngineSerializerImpl::SetLocalTransform(utils::Entity entity,
                                                  const mat4& transform) {
  SerializerDataTypes::Batch<CommandTypes::UpdateNodes>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::UpdateNodes>({entity});
  batch.data[entity].transform = transform;
}

void SplitEngineSerializerImpl::SetGroups(
    utils::Entity entity, absl::Span<const absl::string_view> groups) {
  SerializerDataTypes::Batch<CommandTypes::UpdateNodes>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::UpdateNodes>({entity});
  batch.data[entity].groups.emplace().assign(groups.begin(), groups.end());
}

void SplitEngineSerializerImpl::AssignUserId(utils::Entity entity,
                                             uint32_t user_id) {
  SerializerDataTypes::Batch<CommandTypes::AssignUserIdToNodes>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::AssignUserIdToNodes>(
          {entity});
  batch.data[entity] = user_id;
}

std::unique_ptr<BaseTextureBuilder>
SplitEngineSerializerImpl::CreateTextureBuilder() {
  return std::make_unique<SplitEngineTextureBuilder>(*this);
}

std::unique_ptr<BaseMeshBuilder>
SplitEngineSerializerImpl::CreateMeshBuilder() {
  return std::make_unique<SplitEngineMeshBuilder>(*this,
                                                  *view_.GetSharedEngine());
}

#if IMP_PLATFORM(ANDROID)
Future<std::unique_ptr<PlatformAndroidExternalTextureSurface>>
SplitEngineSerializerImpl::CreateAndroidExternalTextureSurface(
    ContentSecurityLevel security_level,
    absl::Span<const SurfaceViewType> view_types) {
  return SplitEnginePlatformAndroidExternalTextureSurface::Create(
      view_, security_level, view_types);
}
#endif

void SplitEngineSerializerImpl::SetBoxCollider(utils::Entity entity,
                                               const Box& box, bool enabled) {
  SerializerDataTypes::Batch<CommandTypes::AddOrUpdateColliders>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::AddOrUpdateColliders>(
          {entity});
  batch.data[entity] =
      SerializerDataTypes::AddOrUpdateColliderInfo{box, enabled};
}

void SplitEngineSerializerImpl::SetMeshCollider(utils::Entity entity,
                                                bool enabled) {
  SerializerDataTypes::Batch<CommandTypes::AddOrUpdateColliders>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::AddOrUpdateColliders>(
          {entity});
  batch.data[entity] = SerializerDataTypes::AddOrUpdateColliderInfo{
      SerializerDataTypes::MeshCollider(), enabled};
}

void SplitEngineSerializerImpl::SetSphereCollider(utils::Entity entity,
                                                  const Sphere& sphere,
                                                  bool enabled) {
  SerializerDataTypes::Batch<CommandTypes::AddOrUpdateColliders>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::AddOrUpdateColliders>(
          {entity});
  batch.data[entity] =
      SerializerDataTypes::AddOrUpdateColliderInfo{sphere, enabled};
}

void SplitEngineSerializerImpl::SetCapsuleCollider(utils::Entity entity,
                                                   const Capsule& capsule,
                                                   bool enabled) {
  SerializerDataTypes::Batch<CommandTypes::AddOrUpdateColliders>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::AddOrUpdateColliders>(
          {entity});
  batch.data[entity] =
      SerializerDataTypes::AddOrUpdateColliderInfo{capsule, enabled};
}

void SplitEngineSerializerImpl::ClearCollider(
    utils::Entity entity,
    split_engine::SplitEngineSerializer::ColliderType collider_type) {
  SerializerDataTypes::Batch<CommandTypes::RemoveColliders>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::RemoveColliders>({entity});
  batch.data[entity] =
      static_cast<android_xr::schemas::ColliderType>(collider_type);
}

void SplitEngineSerializerImpl::AddTexturePipelineRenderer(
    utils::Entity entity, const TexturePipelineRendererState& state) {
  SerializerDataTypes::Batch<CommandTypes::AddTexturePipelineRenderers>& batch =
      batch_manager_
          .GetOrCreateBatch<CommandTypes::AddTexturePipelineRenderers>(
              {entity});
  batch.data[entity].state = state;
}

void SplitEngineSerializerImpl::RemoveTexturePipelineRenderer(
    utils::Entity entity) {
  SerializerDataTypes::Batch<CommandTypes::RemoveTexturePipelineRenderers>&
      batch =
          batch_manager_
              .GetOrCreateBatch<CommandTypes::RemoveTexturePipelineRenderers>(
                  {entity});
  batch.data.push_back(entity);
}

void SplitEngineSerializerImpl::SetTexturePipelineRendererPassesEnabled(
    utils::Entity entity, const std::vector<bool>& enabled_passes) {
  SerializerDataTypes::Batch<CommandTypes::UpdateTexturePipelineRenderers>&
      batch =
          batch_manager_
              .GetOrCreateBatch<CommandTypes::UpdateTexturePipelineRenderers>(
                  {entity});
  auto& info = batch.data[entity];
  info.enabled_passes = enabled_passes;
}

void SplitEngineSerializerImpl::SetTexturePipelineRendererProjectionQuad(
    utils::Entity entity,
    const std::optional<TexturePipelineRendererProjectionQuad>& quad) {
  SerializerDataTypes::Batch<CommandTypes::UpdateTexturePipelineRenderers>&
      batch =
          batch_manager_
              .GetOrCreateBatch<CommandTypes::UpdateTexturePipelineRenderers>(
                  {entity});
  batch.data[entity].projection_quad = quad;
}

void SplitEngineSerializerImpl::RegisterNamedTexture(
    const filament::Texture& texture, absl::string_view name) {
  uint64_t id = GetId(&texture);
  IMP_LOG(imp::INFO) << kTag << "RegisterNamedTexture - id: " << id
             << ", name: " << name;
  // Named texture registration doesn't depend on entities, but might depend on
  // texture creation if we tracked it. For now, we just batch it.
  SerializerDataTypes::Batch<CommandTypes::RegisterNamedTextures>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::RegisterNamedTextures>(
          {}, {id});
  batch.data[id] =
      SerializerDataTypes::RegisterNamedTextureInfo{std::string(name)};
}

void SplitEngineSerializerImpl::UnregisterNamedTexture(
    const filament::Texture& texture) {
  uint64_t id = GetId(&texture);
  SerializerDataTypes::Batch<CommandTypes::UnregisterNamedTextures>& batch =
      batch_manager_.GetOrCreateBatch<CommandTypes::UnregisterNamedTextures>(
          {}, {id});
  batch.data.push_back(id);
}

void SplitEngineSerializerImpl::SendMessage(
    SerializerDataTypes::CommandBatchBase* batch_base) {
  imp::OwnedPtr<SplitEngineSerializerTransport::MessageBuilder> fbb =
      ReleaseFlatBufferBuilder(*batch_base);

  // Assumption is that serialization for frame updates happens fast and does
  // not require offloading to the background thread.
  const std::optional<flatbuffers::Offset<android_xr::schemas::Command>>
      offset = batch_base->Serialize(**fbb);

  if (offset) {
    if (absl::Status status = transport_->AddMessage(*frame_update_group_id_, std::move(fbb),
                                    *offset); !status.ok()) { IMP_LOG(imp::FATAL) << "Transport error: " << status; }
  }
}

void SplitEngineSerializerImpl::SendAllBatches() {
  batch_manager_.ForEachBatchRunAndConsume(
      [this](SerializerDataTypes::CommandBatchBase* batch) {
        SendMessage(batch);
      });

  // If `SendMessage` was never called in the code above, it means that
  // `BeginMessageGroup` was never called, and `EndMessageGroup`
  // should not be called too.
  if (!frame_update_group_id_.has_value()) return;

  if (absl::Status status = transport_->End(*frame_update_group_id_); !status.ok()) { IMP_LOG(imp::FATAL) << "Transport error: " << status; }
  frame_update_group_id_ = std::nullopt;

  // Clean up
  fbb_.clear();
}

void SplitEngineSerializerImpl::Update(const FrameTime& frame_time) {
  // Check if any message groups are eligible for release.
  transport_->ClearReleasedMessageGroups();

  // Send all pending Command batches.
  SendAllBatches();
}

}  // namespace imp::split_engine
