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

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "absl/algorithm/container.h"
#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "filament/filament/include/filament/MorphTargetBuffer.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "filament/libs/utils/include/utils/FixedCapacityVector.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/string.h"
#include "flatbuffers/vector.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/config.h"
#include "core/geometry/shapes/box.h"
#include "core/geometry/shapes/capsule.h"
#include "core/geometry/shapes/sphere.h"
#include "core/lighting/image_based_lighting_types.h"
#include "core/loader/provider/details/loaded_model_builder.h"
#include "core/material_library/flatbuffer_utils.h"
#include "core/material_library/generic_material_spec.h"
#include "core/material_library/material_param_value.h"
#include "core/math/flatbuffer_support.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/model/mesh/base_mesh_builder.h"
#include "core/ncsb/node_handle.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/image_based_lighting_helpers.h"
#include "core/split_engine/split_engine_mesh_serializer.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/split_engine/split_engine_texture_serializer.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"
#if IMP_PLATFORM(ANDROID)
#include "core/render/android/android_defines.h"
#include "core/render/android/platform_android_external_texture_surface.h"
#endif
#include "core/render/base_renderable_manager.h"
#include "core/render/base_texture_builder.h"
#if IMP_PLATFORM(ANDROID)
#include "core/split_engine/android/split_engine_platform_android_external_texture_surface.h"
#endif
#include "core/split_engine/flatbuffer_size_calculator.h"
#include "core/split_engine/materials/split_engine_generic_material.h"
#include "core/split_engine/split_engine_bridge_sender.h"
#include "core/split_engine/split_engine_mesh_builder.h"
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
// This is needed because of an issue where GenericMaterial uses remote
// materials even when in local mode, in which case it uses the placeholder
// material to represent the material on the app side. If that material gets
// serialized across split engine, there is a race condition that can cause the
// placeholder material to be used to actually render.
//
// That being said, it still makes sense not to serialize the placeholder
// material anyways.
//
// TODO: We should fix GenericMaterial to not use remote
// materials in local mode.
#if IMP_USE_LOCAL_SPLIT_ENGINE_MATERIALS
bool IsPlaceholderSplitEngineMaterial(const filament::Material* material) {
  return strcmp(material->getName(), "Split Engine Placeholder") == 0;
}
#endif

template <typename T>
void CreateCommand(flatbuffers::FlatBufferBuilder& fbb,
                   flatbuffers::Offset<T> command_offset) {
  flatbuffers::Offset<android_xr::schemas::Command> command =
      android_xr::schemas::CreateCommand(
          fbb, android_xr::schemas::CommandTypesTraits<T>::enum_value,
          command_offset.Union());
  fbb.Finish(command);
}

template <typename T>
void LogMaterialParam(absl::string_view name, const T& value) {
  IMP_LOG(imp::INFO) << kTag << kIndent << "material param: name: " << name
             << " value: " << value;
}

constexpr android_xr::schemas::ColliderType GetColliderType(
    SplitEngineSerializer::ColliderType collider_type) {
  switch (collider_type) {
    case SplitEngineSerializer::ColliderType::kBoxCollider:
      return android_xr::schemas::ColliderType::BoxCollider;
    case SplitEngineSerializer::ColliderType::kMeshCollider:
      return android_xr::schemas::ColliderType::MeshCollider;
    case SplitEngineSerializer::ColliderType::kSphereCollider:
      return android_xr::schemas::ColliderType::SphereCollider;
    case SplitEngineSerializer::ColliderType::kCapsuleCollider:
      return android_xr::schemas::ColliderType::CapsuleCollider;
  }
}

constexpr bool DoColliderEnumsMatch(
    SplitEngineSerializer::ColliderType serizlier_enum,
    android_xr::schemas::ColliderType renderer_enum) {
  return GetColliderType(serizlier_enum) == renderer_enum;
}

static_assert(
    DoColliderEnumsMatch(SplitEngineSerializer::ColliderType::kBoxCollider,
                         android_xr::schemas::ColliderType::BoxCollider) &&
        DoColliderEnumsMatch(SplitEngineSerializer::ColliderType::kMeshCollider,
                             android_xr::schemas::ColliderType::MeshCollider) &&
        DoColliderEnumsMatch(
            SplitEngineSerializer::ColliderType::kSphereCollider,
            android_xr::schemas::ColliderType::SphereCollider) &&
        DoColliderEnumsMatch(
            SplitEngineSerializer::ColliderType::kCapsuleCollider,
            android_xr::schemas::ColliderType::CapsuleCollider),
    "Collider enums don't match");

}  // namespace

// TODO Add more using statements (esp. android_xr::schemas) to
// shorten code and make it more readable.
using android_xr::schemas::ColliderData;
using filament::Box;
using filament::IndexBuffer;
using filament::VertexBuffer;

class SplitEngineSerializerImpl::RenderableBuilder
    : public BaseRenderableManager::Builder {
 public:
  RenderableBuilder(SplitEngineSerializerImpl& serializer,
                    size_t count) noexcept
      : serializer_(serializer) {
    add_renderable_info_.primitive_count = count;
  }

  RenderableBuilder& Geometry(size_t index,
                              filament::backend::PrimitiveType type,
                              filament::VertexBuffer* vertices,
                              filament::IndexBuffer* indices, size_t offset,
                              size_t minIndex, size_t maxIndex,
                              size_t count) noexcept override {
    return Geometry(index, type, vertices, indices, offset, count);
  }
  RenderableBuilder& Geometry(size_t index,
                              filament::backend::PrimitiveType type,
                              filament::VertexBuffer* vertices,
                              filament::IndexBuffer* indices, size_t offset,
                              size_t count) noexcept override {
    GeometryUpdateInfo geometry_update_info;
    geometry_update_info.vertex_buffer_id = GetId(vertices);
    geometry_update_info.index_buffer_id = GetId(indices);
    geometry_update_info.offset = offset;
    geometry_update_info.count = count;
    geometry_update_info.primitive_type = static_cast<uint8_t>(type);
    update_renderable_info_.primitives[index].geometry = geometry_update_info;

    return *this;
  }
  RenderableBuilder& Geometry(
      size_t index, filament::backend::PrimitiveType type,
      filament::VertexBuffer* vertices,
      filament::IndexBuffer* indices) noexcept override {
    return Geometry(index, type, vertices, indices, 0,
                    indices->getIndexCount());
  }

  RenderableBuilder& Material(
      size_t index,
      const filament::MaterialInstance* material_instance) noexcept override {
    update_renderable_info_.primitives[index].material_instance_id =
        GetId(material_instance);
    return *this;
  }

  RenderableBuilder& BoundingBox(
      const Box& axisAlignedBoundingBox) noexcept override {
    update_renderable_info_.bounds = axisAlignedBoundingBox;
    return *this;
  }
  RenderableBuilder& LayerMask(uint8_t select,
                               uint8_t values) noexcept override {
    update_renderable_info_.layer_mask =
        SplitEngineSerializerImpl::LayerMask{select, values};
    return *this;
  }
  RenderableBuilder& Priority(uint8_t priority) noexcept override {
    update_renderable_info_.priority = priority;
    return *this;
  }
  RenderableBuilder& Channel(uint8_t channel) noexcept override {
    IMP_LOG(imp::WARNING) << kTag << "Builder::Channel is not supported.";
    return *this;
  }
  RenderableBuilder& Culling(bool enable) noexcept override {
    if (!add_renderable_info_.renderable_flags) {
      add_renderable_info_.renderable_flags = RenderableFlags();
    }
    add_renderable_info_.renderable_flags->culling_enabled = enable;
    return *this;
  }
  RenderableBuilder& CastShadows(bool enable) noexcept override {
    IMP_LOG(imp::WARNING) << kTag << "Builder::CastShadows is not supported.";
    return *this;
  }
  RenderableBuilder& ReceiveShadows(bool enable) noexcept override {
    IMP_LOG(imp::WARNING) << kTag << "Builder::ReceiveShadows is not supported.";
    return *this;
  }
  RenderableBuilder& ScreenSpaceContactShadows(bool enable) noexcept override {
    IMP_LOG(imp::WARNING) << kTag
                 << "Builder::ScreenSpaceContactShadows is not supported.";
    return *this;
  }
  RenderableBuilder& Skinning(filament::SkinningBuffer* skinningBuffer,
                              size_t count, size_t offset) noexcept override {
    IMP_LOG(imp::FATAL) << "skinning(filament::SkinningBuffer*, size_t, size_t) is not "
                  "supported.";
    return *this;
  }
  RenderableBuilder& Skinning(size_t boneCount,
                              mat4f const* transforms) noexcept override {
    IMP_LOG(imp::FATAL) << "skinning(size_t, mat4f*) is not supported.";

    return *this;
  }
  RenderableBuilder& Skinning(
      size_t boneCount,
      filament::RenderableManager::Bone const* bones) noexcept override {
    IMP_LOG(imp::FATAL) << "skinning(size_t, filament::RenderableManager::Bone*) is not "
                  "supported.";
    return *this;
  }
  RenderableBuilder& Skinning(size_t boneCount) noexcept override {
    add_renderable_info_.skinning_bone_count = boneCount;
    return *this;
  }
  RenderableBuilder& BoneIndicesAndWeights(
      size_t primitiveIndex, float2 const* indicesAndWeights, size_t count,
      size_t bonesPerVertex) noexcept override {
    // No-op. This is called when the skinning info is more advanced (has more
    // than 4 bone weights per vertex). It is not supported in split engine.
    IMP_LOG(imp::FATAL) << kTag
               << "Each vertex can only be influenced by a maximum of 4 bones.";
    return *this;
  }
  RenderableBuilder& BoneIndicesAndWeights(
      size_t primitiveIndex,
      utils::FixedCapacityVector<utils::FixedCapacityVector<float2>>
          indicesAndWeightsVector) noexcept override {
    IMP_LOG(imp::FATAL) << "boneIndicesAndWeights(size_t, "
                  "utils::FixedCapacityVector<float2>) is not supported.";

    return *this;
  }

  RenderableBuilder& Morphing(
      filament::MorphTargetBuffer* morphTargetBuffer) noexcept override {
    if (!add_renderable_info_.morph_target_data) {
      add_renderable_info_.morph_target_data = MorphTargetData();
    }
    add_renderable_info_.morph_target_data->morph_target_buffer_id =
        GetId(morphTargetBuffer);
    return *this;
  }
  RenderableBuilder& Morphing(uint8_t level, size_t primitiveIndex,
                              size_t offset, size_t count) noexcept override {
    if (!add_renderable_info_.morph_target_data) {
      add_renderable_info_.morph_target_data = MorphTargetData();
    }
    add_renderable_info_.morph_target_data->morph_target_info.resize(
        add_renderable_info_.primitive_count);

    MorphTargetInfo& morph_target_info =
        add_renderable_info_.morph_target_data
            ->morph_target_info[primitiveIndex];
    morph_target_info.morph_target_buffer_offset = offset;
    morph_target_info.morph_target_buffer_count = count;

    return *this;
  }
  RenderableBuilder& BlendOrder(size_t primitiveIndex,
                                uint16_t order) noexcept override {
    update_renderable_info_.primitives[primitiveIndex].blend_order = order;
    return *this;
  }
  RenderableBuilder& GlobalBlendOrderEnabled(size_t primitiveIndex,
                                             bool enabled) noexcept override {
    update_renderable_info_.primitives[primitiveIndex]
        .global_blend_order_enabled = enabled;
    return *this;
  }
  RenderableBuilder& Instances(size_t instanceCount) noexcept override {
    IMP_LOG(imp::WARNING) << kTag << "Builder::Instances is not supported.";
    return *this;
  }
  RenderableBuilder& Instances(
      size_t instanceCount,
      filament::InstanceBuffer* instanceBuffer) noexcept override {
    IMP_LOG(imp::WARNING) << kTag << "Builder::Instances is not supported.";
    return *this;
  }
  filament::RenderableManager::Builder::Result Build(
      filament::Engine& engine, utils::Entity entity) override {
    serializer_.add_renderables_[entity] = std::move(add_renderable_info_);
    serializer_.renderable_updates_[entity] =
        std::move(update_renderable_info_);

    return filament::RenderableManager::Builder::Result::Success;
  }

 protected:
  RenderableBuilder& LightChannelInternal(unsigned int channel,
                                          bool enable) noexcept override {
    return *this;
  }
  RenderableBuilder& EnableSkinningBuffersInternal(
      bool enabled) noexcept override {
    IMP_LOG(imp::WARNING) << kTag << "EnableSkinningBuffers is not supported.";
    return *this;
  }
  RenderableBuilder& FogInternal(bool enabled) noexcept override {
    return *this;
  }

 private:
  SplitEngineSerializerImpl& serializer_;

  // Stores the information needed to later serialize a command to add a new
  // renderable. When Build is called, this is moved to the
  // SplitEngineSerializerImpl.
  AddRenderableInfo add_renderable_info_;

  // The Builder also includes methods that update the renderable with state
  // serialized through the Update command, not just the Add command. This
  // info is stored here and moved to the SplitEngineSerializerImpl when Build
  // is called.
  UpdateRenderableInfo update_renderable_info_;
};

SplitEngineSerializerImpl::SplitEngineSerializerImpl(
    BaseView& view, std::unique_ptr<SplitEngineBridgeSender> bridge_sender,
    std::unique_ptr<SplitEngineBridgeSender> one_shot_bridge_sender,
    size_t bridge_buffer_size_bytes)
    : Updater(view),
      view_(view),
      bridge_sender_(std::move(bridge_sender)),
      one_shot_bridge_sender_(std::move(one_shot_bridge_sender)),
      bridge_buffer_size_bytes_(bridge_buffer_size_bytes) {
  view_.GetRenderableManager().SetSpy(*this);
}

void SplitEngineSerializerImpl::SetSpy(BaseRenderableManager& spy) {}

filament::RenderableManager::Instance SplitEngineSerializerImpl::GetInstance(
    utils::Entity e) const {
  return 0;
}

bool SplitEngineSerializerImpl::HasComponent(utils::Entity e) const {
  return false;
}

void SplitEngineSerializerImpl::Destroy(utils::Entity e) {
  remove_renderables_.insert(e);
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
  renderable_updates_[entity].primitives[primitiveIndex].material_instance_id =
      GetId(material_instance);
}
void SplitEngineSerializerImpl::SetGeometryAt(
    filament::RenderableManager::Instance instance, size_t primitiveIndex,
    filament::backend::PrimitiveType type, VertexBuffer* vertices,
    IndexBuffer* indices, size_t offset, size_t count) {
  GeometryUpdateInfo geometry_update_info;
  geometry_update_info.vertex_buffer_id = GetId(vertices);
  geometry_update_info.index_buffer_id = GetId(indices);
  geometry_update_info.offset = offset;
  geometry_update_info.count = count;
  geometry_update_info.primitive_type = static_cast<uint8_t>(type);

  utils::Entity entity = GetEntity(instance);
  renderable_updates_[entity].primitives[primitiveIndex].geometry =
      geometry_update_info;
}
void SplitEngineSerializerImpl::SetBonesInternal(
    filament::RenderableManager::Instance instance,
    filament::RenderableManager::Bone const* transforms, size_t boneCount,
    size_t offset) {
  IMP_LOG(imp::FATAL) << "setBones(entity, filament::RenderableManager::Bone*) is not "
                "supported";
}

void SplitEngineSerializerImpl::SetBonesInternal(
    filament::RenderableManager::Instance instance, mat4f const* transforms,
    size_t boneCount, size_t offset) {
  if (offset != 0) {
    IMP_LOG(imp::FATAL) << "Nonzero offset is not supported.";
  }

  flatbuffers::FlatBufferBuilder* fbb = GetFlatBufferBuilderFor(
      android_xr::schemas::CommandTypes::UpdateRenderables);
  utils::Entity entity = GetEntity(instance);
  renderable_updates_[entity].bones = android_xr::schemas::CreateBones(
      *fbb, fbb->CreateVectorOfNativeStructs<android_xr::schemas::Mat4f>(
                transforms, boneCount, Pack));
}
void SplitEngineSerializerImpl::SetAxisAlignedBoundingBox(
    filament::RenderableManager::Instance instance, const Box& aabb) {
  utils::Entity entity = GetEntity(instance);
  renderable_updates_[entity].bounds = aabb;
}
void SplitEngineSerializerImpl::SetPriority(
    filament::RenderableManager::Instance instance, uint8_t priority) {
  utils::Entity entity = GetEntity(instance);
  renderable_updates_[entity].priority = priority;
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
  renderable_updates_[entity].layer_mask = LayerMask{select, values};
}
void SplitEngineSerializerImpl::SetBlendOrderAt(
    filament::RenderableManager::Instance instance, size_t primitiveIndex,
    uint16_t order) {
  utils::Entity entity = GetEntity(instance);
  renderable_updates_[entity].primitives[primitiveIndex].blend_order = order;
}
void SplitEngineSerializerImpl::SetGlobalBlendOrderEnabledAt(
    filament::RenderableManager::Instance instance, size_t primitiveIndex,
    bool enabled) {
  utils::Entity entity = GetEntity(instance);
  renderable_updates_[entity]
      .primitives[primitiveIndex]
      .global_blend_order_enabled = enabled;
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
  flatbuffers::FlatBufferBuilder* fbb = GetFlatBufferBuilderFor(
      android_xr::schemas::CommandTypes::UpdateRenderables);
  utils::Entity entity = GetEntity(instance);
  renderable_updates_[entity].morph_weights =
      android_xr::schemas::CreateMorphWeights(
          *fbb, fbb->CreateVector(weights, count), offset);
}

std::unique_ptr<BaseRenderableManager::Builder>
SplitEngineSerializerImpl::NewBuilder(size_t count) {
  return std::make_unique<RenderableBuilder>(*this, count);
}

// Copied from loaded_model_builder.cc
flatbuffers::Offset<android_xr::schemas::BoundsInfo> CreateBoundsInfo(
    flatbuffers::FlatBufferBuilder& fbb, const Box& bounds) {
  const auto center = bounds.center;
  const auto half_extent = bounds.halfExtent;
  android_xr::schemas::Box box(Pack(center), Pack(half_extent));

  return android_xr::schemas::CreateBoundsInfo(fbb, &box);
}

SplitEngineSerializerImpl::FlatBufferBuilderPtr
SplitEngineSerializerImpl::CreateFlatBufferBuilder(size_t size_bytes) {
  // Message groups are lazily began the first time anyone attempts to build a
  // message (create a FlatBufferBuilder) in a frame, and ended in Update() if
  // any messages were sent that frame.
  // There may be additional logic in the future to have multiple message groups
  // per frame, but for now it's always exactly one per frame.
  if (!bridge_sender_->IsMessageGroupActive()) {
    bridge_sender_->BeginMessageGroup(bridge_buffer_size_bytes_);
  }
  return bridge_sender_->CreateFlatBufferBuilder(size_bytes);
}

SplitEngineSerializerImpl::FlatBufferBuilderPtr
SplitEngineSerializerImpl::CreateFlatBufferBuilder() {
  constexpr size_t kInitialSize = 1024;
  return CreateFlatBufferBuilder(kInitialSize);
}

void SplitEngineSerializerImpl::AddTexture(
    filament::Texture& texture,
    SplitEngineTextureSerializer& split_engine_texture_serializer) {
  const size_t kNumTextures = 1;
  std::vector<size_t> image_buffer_sizes =
      split_engine_texture_serializer.GetTextureBufferSizes();
  const size_t kBufferSize = FlatbufferSizeCalculator()
                                 .AddTextureAndDependentData(image_buffer_sizes)
                                 .AddReferenceVector(kNumTextures)
                                 .AddAddTextureRequest(kNumTextures)
                                 .AddRequest()
                                 .Finish()
                                 .AddScratchSpace()
                                 .ComputeSize();
  IMP_LOG(imp::INFO) << kTag << "texture: " << GetId(&texture);
  one_shot_bridge_sender_->BeginMessageGroup(kBufferSize);
  std::unique_ptr<flatbuffers::FlatBufferBuilder> builder =
      one_shot_bridge_sender_->CreateFlatBufferBuilder(kBufferSize);

  flatbuffers::Offset<android_xr::schemas::Texture> offset =
      split_engine_texture_serializer.SerializeTexture(texture, *builder);

  std::vector<flatbuffers::Offset<android_xr::schemas::Texture>> texture_vector;
  texture_vector.push_back(offset);
  CreateCommand(*builder, android_xr::schemas::CreateAddTextures(
                              *builder, builder->CreateVector(texture_vector)));

  one_shot_bridge_sender_->SendMessage(*builder);
  one_shot_bridge_sender_->EndMessageGroup();
}

void SplitEngineSerializerImpl::RemoveTexture(filament::Texture& texture) {
  textures_to_remove_.push_back(GetId(&texture));
}

void SplitEngineSerializerImpl::SerializeMesh(
    SplitEngineMeshSerializer& split_engine_mesh_serializer) {
  SerializeMeshIndicesAndVertices(split_engine_mesh_serializer);
  SerializeMeshMorphTargets(split_engine_mesh_serializer);
}

void SplitEngineSerializerImpl::SerializeMeshIndicesAndVertices(
    SplitEngineMeshSerializer& split_engine_mesh_serializer) {
  FlatbufferSizeCalculator mesh_calculator;
  split_engine_mesh_serializer.ContributeVertexBufferSizes(mesh_calculator);
  split_engine_mesh_serializer.ContributeIndexBufferSizes(mesh_calculator);

  const size_t kAddMeshBufferSize = mesh_calculator.AddAddMeshData()
                                        .AddRequest()
                                        .Finish()
                                        .AddScratchSpace()
                                        .ComputeSize();

  one_shot_bridge_sender_->BeginMessageGroup(kAddMeshBufferSize);
  std::unique_ptr<flatbuffers::FlatBufferBuilder> mesh_builder =
      one_shot_bridge_sender_->CreateFlatBufferBuilder(kAddMeshBufferSize);
  SplitEngineMeshSerializer::VertexBufferVector vertex_buffer_offsets =
      split_engine_mesh_serializer.SerializeVertexBuffers(*mesh_builder);
  SplitEngineMeshSerializer::IndexBufferVector index_buffer_offsets =
      split_engine_mesh_serializer.SerializeIndexBuffers(*mesh_builder);

  CreateCommand(*mesh_builder, android_xr::schemas::CreateAddMeshData(
                                   *mesh_builder, vertex_buffer_offsets,
                                   index_buffer_offsets));
  one_shot_bridge_sender_->SendMessage(*mesh_builder);
  one_shot_bridge_sender_->EndMessageGroup();
}

void SplitEngineSerializerImpl::SerializeMeshMorphTargets(
    SplitEngineMeshSerializer& split_engine_mesh_serializer) {
  FlatbufferSizeCalculator morph_target_calculator;
  split_engine_mesh_serializer.ContributeMorphTargetBufferSizes(
      morph_target_calculator);
  if (morph_target_calculator.ComputeSize() == 0) {
    // This mesh has no morph targets, so there's nothing to send.
    return;
  }

  const size_t kAddMorphTargetBufferSize =
      morph_target_calculator.AddAddMorphTargetBuffers()
          .AddRequest()
          .Finish()
          .AddScratchSpace()
          .ComputeSize();

  one_shot_bridge_sender_->BeginMessageGroup(kAddMorphTargetBufferSize);
  std::unique_ptr<flatbuffers::FlatBufferBuilder> morph_target_buffer_builder =
      one_shot_bridge_sender_->CreateFlatBufferBuilder(
          kAddMorphTargetBufferSize);
  SplitEngineMeshSerializer::MorphTargetBufferVector morph_buffer_offsets =
      split_engine_mesh_serializer.SerializeMorphTargetBuffers(
          *morph_target_buffer_builder);
  CreateCommand(*morph_target_buffer_builder,
                android_xr::schemas::CreateAddMorphTargetBuffers(
                    *morph_target_buffer_builder, morph_buffer_offsets));
  one_shot_bridge_sender_->SendMessage(*morph_target_buffer_builder);
  one_shot_bridge_sender_->EndMessageGroup();
}

size_t SplitEngineSerializerImpl::EstimateImageBasedLightingAssetBufferSize(
    const SphericalHarmonics& spherical_harmonics,
    const ImageBasedLightingAssetCubemapImages& cubemap_images) {
  FlatbufferSizeCalculator calculator;

  for (const CubemapLevelImageContents& ibl_cubemap_image :
       cubemap_images.ibl_cubemap_images) {
    const size_t imageBufferSize =
        ibl_cubemap_image.stitched_face_image->GetSize();
    calculator.AddCubemapLevelImageContentsAndDependentData(imageBufferSize);
  }
  calculator.AddFloat3Vector(spherical_harmonics.coefficients.size())
      .AddSphericalHarmonics();

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
    const SphericalHarmonics& spherical_harmonics,
    const ImageBasedLightingAssetCubemapImages& cubemap_images) {
  const size_t kBufferSize = EstimateImageBasedLightingAssetBufferSize(
      spherical_harmonics, cubemap_images);

  one_shot_bridge_sender_->BeginMessageGroup(kBufferSize);
  std::unique_ptr<flatbuffers::FlatBufferBuilder> builder =
      one_shot_bridge_sender_->CreateFlatBufferBuilder(kBufferSize);

  flatbuffers::Offset<android_xr::schemas::ImageBasedLightingAsset> asset =
      PackImageBasedLightingAsset(builder.get(), GetId(&reflection_texture),
                                  spherical_harmonics, cubemap_images);

  CreateCommand(*builder,
                android_xr::schemas::CreateAddImageBasedLightingAssets(
                    *builder, builder->CreateVector({asset})));

  one_shot_bridge_sender_->SendMessage(*builder);
  one_shot_bridge_sender_->EndMessageGroup();
}

void SplitEngineSerializerImpl::RemoveImageBasedLightingAsset(
    filament::Texture& reflection_texture) {
  image_based_lighting_assets_to_remove_.push_back(GetId(&reflection_texture));
}

void SplitEngineSerializerImpl::SetPreferredEnvironmentIblAsset(
    filament::Texture& reflection_texture, float intensity,
    const float3& tint) {
  preferred_environment_ibl_asset_id_ = EnvironmentLightParams{
      .image_based_lighting_asset_id = GetId(&reflection_texture),
      .intensity = intensity,
      .tint = tint};
}

void SplitEngineSerializerImpl::ClearPreferredEnvironmentIblAsset() {
  // Setting the image based lighting asset id to 0 indicates to clear any
  // previously-set preferred environment IBL asset.
  preferred_environment_ibl_asset_id_ = EnvironmentLightParams{
      .image_based_lighting_asset_id = 0, .intensity = 0, .tint = {0, 0, 0}};
}

void SplitEngineSerializerImpl::RemoveMorphTargetBuffer(
    filament::MorphTargetBuffer* morph_target_buffer) {
  morph_target_buffers_to_remove_.push_back(GetId(morph_target_buffer));
}

void SplitEngineSerializerImpl::RemoveVertexBuffer(
    filament::VertexBuffer* vertex_buffer) {
  vertex_buffers_to_remove_.push_back(GetId(vertex_buffer));
}

void SplitEngineSerializerImpl::RemoveIndexBuffer(
    filament::IndexBuffer* index_buffer) {
  index_buffers_to_remove_.push_back(GetId(index_buffer));
}

flatbuffers::FlatBufferBuilder*
SplitEngineSerializerImpl::GetFlatBufferBuilderFor(
    android_xr::schemas::CommandTypes command_type) {
  switch (command_type) {
    case android_xr::schemas::CommandTypes::AddMeshData:
      // Because mesh data can be significantly larger than other
      // types of data, they should be added via a one-use buffer.
      IMP_LOG(imp::FATAL) << "Command for FlatBufferBuilder for AddMeshData unsupported "
                 << "via shared allocator.";
      return nullptr;
    case android_xr::schemas::CommandTypes::AddMorphTargetBuffers:
      // Because morph target data can be significantly larger than other
      // types of data, they should be added via a one-use buffer.
      IMP_LOG(imp::FATAL) << "Command for FlatBufferBuilder for AddMorphTargetBuffers "
                    "unsupported via shared allocator.";
      return nullptr;
    case android_xr::schemas::CommandTypes::AddMaterials:
      if (!materials_builder_) {
        materials_builder_ = CreateFlatBufferBuilder();
      }
      return materials_builder_.get();
    case android_xr::schemas::CommandTypes::UpdateRenderables:
      if (!renderable_updates_builder_) {
        renderable_updates_builder_ = CreateFlatBufferBuilder();
      }
      return renderable_updates_builder_.get();
    case android_xr::schemas::CommandTypes::AddTextures:
      // Because textures can be significantly larger than other
      // types of data, they should be added via
      // OpenFlatBufferBuilderForAddTexture, which uses a one-use buffer.
      IMP_LOG(imp::FATAL) << "Command for FlatBufferBuilder for AddTexture unsupported "
                 << "via shared allocator.";
      return nullptr;
    case android_xr::schemas::CommandTypes::AddImageBasedLightingAssets:
      // Because IBL asset data can be significantly larger than other
      // types of data, they should be added via a one-use buffer.
      IMP_LOG(imp::FATAL)
          << "Command for FlatBufferBuilder for AddImageBasedLightingAssets "
             "unsupported via shared allocator.";
      return nullptr;
    default:
      return nullptr;
  }
}

void SplitEngineSerializerImpl::AddMaterial(const filament::Material* material,
                                            const BufferAccess& data) {
  flatbuffers::FlatBufferBuilder* fbb =
      GetFlatBufferBuilderFor(android_xr::schemas::CommandTypes::AddMaterials);
#if IMP_USE_LOCAL_SPLIT_ENGINE_MATERIALS
  if (!IsPlaceholderSplitEngineMaterial(material)) {
    materials_to_add_.push_back(android_xr::schemas::CreateMaterial(
        *fbb, GetId(material), fbb->CreateVector(data.Data(), data.Size())));
  }
#else
  materials_to_add_.push_back(
      android_xr::schemas::CreateMaterial(*fbb, GetId(material)));
#endif
}

void SplitEngineSerializerImpl::RemoveMaterial(
    const filament::Material* material) {
  materials_to_remove_.push_back(GetId(material));
}

void SplitEngineSerializerImpl::AddMaterialInstance(
    const filament::Material* material,
    const filament::MaterialInstance* instance) {
#if IMP_USE_LOCAL_SPLIT_ENGINE_MATERIALS
  if (IsPlaceholderSplitEngineMaterial(material)) return;

  if (material_instances_to_duplicate_.find(GetId(instance)) !=
      material_instances_to_duplicate_.end()) {
    // Do nothing - this will be handled as part of the duplicates command.
    return;
  }
  material_instances_to_add_[GetId(instance)] = GetId(material);
#endif
}

SplitEngineSerializerImpl::FlatBufferBuilderPtr&
SplitEngineSerializerImpl::GetMaterialUpdateBuilder(
    MaterialUpdateBuilderType type) {
  // If current builder is null or for a different type of update, create a new
  // builder and return it.
  if (material_update_commands_.empty() ||
      current_material_update_builder_type_ != type) {
    // First commit the current builder, which creates a Command.
    CommitCurrentMaterialUpdateBuilder();
    // Set the new builder type and create a new builder for subsequent data.
    current_material_update_builder_type_ = type;
    return material_update_commands_.emplace_back(CreateFlatBufferBuilder());
  }
  // Return the current builder as it is of a matching type.
  return material_update_commands_.back();
}

void SplitEngineSerializerImpl::CommitCurrentMaterialUpdateBuilder() {
  if (material_update_commands_.empty()) return;

  FlatBufferBuilderPtr& fbb = material_update_commands_.back();
  switch (current_material_update_builder_type_) {
    case MaterialUpdateBuilderType::kMaterialParameters:
      CommitMaterialParameters(fbb);
      break;
    case MaterialUpdateBuilderType::kBuiltInMaterialParameters:
      CommitBuiltInMaterialParameters(fbb);
      break;
    case MaterialUpdateBuilderType::kMaterialDuplicates:
      CommitMaterialDuplicates(fbb);
      break;
  }
}

void SplitEngineSerializerImpl::CommitMaterialParameters(
    FlatBufferBuilderPtr& fbb) {
  if (material_params_.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "material parameters: updating "
             << material_params_.size() << " materials";
  std::vector<flatbuffers::Offset<android_xr::schemas::MaterialParameters>>
      parameters(material_params_.size());
  absl::c_transform(
      material_params_, parameters.data(), [&fbb](const auto& entry) {
        return android_xr::schemas::CreateMaterialParameters(
            *fbb, entry.first,
            fbb->CreateVector(entry.second.params.data(),
                              entry.second.params.size()),
            fbb->CreateVector(entry.second.texture_params.data(),
                              entry.second.texture_params.size()));
      });
  CreateCommand(*fbb, android_xr::schemas::CreateSetMaterialParameters(
                          *fbb, fbb->CreateVector(parameters)));
  material_params_.clear();
}

void SplitEngineSerializerImpl::CommitBuiltInMaterialParameters(
    FlatBufferBuilderPtr& fbb) {
  if (built_in_material_parameters_.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "built-in material parameters: updating "
             << built_in_material_parameters_.size() << " materials";
  CreateCommand(*fbb,
                android_xr::schemas::CreateSetBuiltInMaterialParameters(
                    *fbb, fbb->CreateVector(built_in_material_parameters_)));
  built_in_material_parameters_.clear();
}

void SplitEngineSerializerImpl::CommitMaterialDuplicates(
    FlatBufferBuilderPtr& fbb) {
  if (material_instances_to_duplicate_.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "material duplicates: ";
  std::vector<
      flatbuffers::Offset<android_xr::schemas::DuplicateMaterialInstance>>
      duplicates(material_instances_to_duplicate_.size());
  absl::c_transform(
      material_instances_to_duplicate_, duplicates.data(),
      [&fbb](const auto& entry) {
        IMP_LOG(imp::INFO) << kTag << kIndent << entry.first << " -> " << entry.second;
        return android_xr::schemas::CreateDuplicateMaterialInstance(
            *fbb, entry.first, entry.second);
      });
  CreateCommand(*fbb, android_xr::schemas::CreateDuplicateMaterialInstances(
                          *fbb, fbb->CreateVector(duplicates)));
  material_instances_to_duplicate_.clear();
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
      LogMaterialParam(name, value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Float,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const float2& value) {
      LogMaterialParam(name, value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Float2,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const float3& value) {
      LogMaterialParam(name, value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Float3,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const float4& value) {
      LogMaterialParam(name, value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Float4,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const int value) {
      LogMaterialParam(name, value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Int,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const int2& value) {
      LogMaterialParam(name, value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Int2,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const int3& value) {
      LogMaterialParam(name, value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Int3,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const int4& value) {
      LogMaterialParam(name, value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Int4,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const bool value) {
      LogMaterialParam(name, value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Bool,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const bool2& value) {
      LogMaterialParam(name, value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Bool2,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const bool3& value) {
      LogMaterialParam(name, value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Bool3,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const bool4& value) {
      LogMaterialParam(name, value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Bool4,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const loader::details::LoadedModelBuilder::MaterialTextureId&
            raw_value) {
      IMP_LOG(imp::FATAL)
          << "SetMaterialParam(TextureId) should never be called. It should "
             "be converted into a Texture* instead.";
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name,
          android_xr::schemas::MaterialParamValue::MaterialTextureId,
          fbb.CreateStruct(
                 android_xr::schemas::MaterialTextureId(uint16_t{raw_value}))
              .Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const mat3f& value) {
      LogMaterialParam(name, value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Mat3f,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const std::vector<mat3f>& value) {
      IMP_LOG(imp::INFO) << kTag << kIndent << "material param: name: " << name
                 << " value: <vector of mat3f>";
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Mat3fArray,
          android_xr::schemas::CreateMat3fArray(
              fbb, fbb.CreateVectorOfNativeStructs<android_xr::schemas::Mat3f>(
                       value.data(), value.size(), Pack))
              .Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const mat4f& value) {
      LogMaterialParam(name, value);
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Mat4f,
          fbb.CreateStruct(Pack(value)).Union());
    }
    flatbuffers::Offset<android_xr::schemas::MaterialParamInfo> operator()(
        const std::vector<mat4f>& value) {
      IMP_LOG(imp::INFO) << kTag << kIndent << "material param: name: " << name
                 << " value: <vector of mat4f>";
      return android_xr::schemas::CreateMaterialParamInfo(
          fbb, fb_name, android_xr::schemas::MaterialParamValue::Mat4fArray,
          android_xr::schemas::CreateMat4fArray(
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
  FlatBufferBuilderPtr& fbb =
      GetMaterialUpdateBuilder(MaterialUpdateBuilderType::kMaterialParameters);
  MaterialParameters& parameters = material_params_[GetId(material)];
  parameters.params.push_back(AddMaterialParam(*fbb, name, value));
}

void SplitEngineSerializerImpl::SetMaterialParameter(
    const filament::MaterialInstance* material, absl::string_view name,
    const filament::Texture* texture, const filament::TextureSampler& sampler) {
  FlatBufferBuilderPtr& fbb =
      GetMaterialUpdateBuilder(MaterialUpdateBuilderType::kMaterialParameters);
  MaterialParameters& parameters = material_params_[GetId(material)];
  flatbuffers::Offset<android_xr::schemas::TextureSampler> sampler_offset =
      CreateTextureSampler<SplitEngineTextureSamplerCreator>(*fbb, sampler);
  parameters.texture_params.push_back(
      android_xr::schemas::CreateMaterialTextureParameter(
          *fbb, fbb->CreateString(std::string(name)), GetId(texture),
          sampler_offset));
}

Future<GenericMaterialPtr> SplitEngineSerializerImpl::CreateGenericMaterial(
    const GenericMaterialSpec& spec) {
  return SplitEngineGenericMaterial::Create(view_, spec)
      .Then([](std::unique_ptr<SplitEngineGenericMaterial> material) {
        return static_cast<GenericMaterialPtr>(std::move(material));
      });
}

void SplitEngineSerializerImpl::SetBuiltInMaterialParameters(
    const filament::MaterialInstance* material,
    android_xr::schemas::BuiltInMaterialParameters type,
    SerializeBuiltInMaterialParametersFunc serialize_func) {
  FlatBufferBuilderPtr& fbb = GetMaterialUpdateBuilder(
      MaterialUpdateBuilderType::kBuiltInMaterialParameters);
  built_in_material_parameters_.push_back(
      android_xr::schemas::CreateBuiltInMaterialInstanceParameters(
          *fbb, GetId(material), type, serialize_func(*fbb)));
}

void SplitEngineSerializerImpl::DuplicateMaterialInstance(
    const filament::MaterialInstance* instance,
    const filament::MaterialInstance* copy) {
  // Prior to duplicating, all existing material parameter updates need to be
  // serialized to a Command. This maintains ordering between duplicating and
  // setting parameters. Duplicates don't need the builder return value but
  // calling GetMaterialUpdateBuilder() will commit the current builder if
  // needed.
  GetMaterialUpdateBuilder(MaterialUpdateBuilderType::kMaterialDuplicates);
  material_instances_to_duplicate_[GetId(copy)] = GetId(instance);
}

void SplitEngineSerializerImpl::RemoveMaterialInstance(
    const filament::MaterialInstance* instance) {
  material_instances_to_remove_.push_back(GetId(instance));
}

void SplitEngineSerializerImpl::CreateNode(utils::Entity entity) {
  add_nodes_.insert(entity);
}

void SplitEngineSerializerImpl::DestroyNode(utils::Entity entity) {
  remove_nodes_.insert(entity);

  // Discard any pending updates related to this node as they are no longer
  // needed. Sending them shouldn't be harmful, but it is unnecessary.
  node_updates_.erase(entity);
  add_renderables_.erase(entity);
  user_id_assignments_.erase(entity);
  renderable_updates_.erase(entity);
  remove_renderables_.erase(entity);
  collider_add_or_updates_.erase(entity);
  collider_removals_.erase(entity);
}

void SplitEngineSerializerImpl::SetEnabled(utils::Entity entity, bool enabled) {
  node_updates_[entity].enabled = enabled;
}

void SplitEngineSerializerImpl::SetName(utils::Entity entity,
                                        absl::string_view name) {
  node_updates_[entity].name = name;
}

void SplitEngineSerializerImpl::SetParent(utils::Entity entity,
                                          utils::Entity parent) {
  node_updates_[entity].parent = parent;
}
void SplitEngineSerializerImpl::SetLocalTransform(utils::Entity entity,
                                                  const mat4f& transform) {
  node_updates_[entity].transform = transform;
}
void SplitEngineSerializerImpl::SetLocalTransform(utils::Entity entity,
                                                  const mat4& transform) {
  node_updates_[entity].transform = transform;
}

void SplitEngineSerializerImpl::AssignUserId(utils::Entity entity,
                                             uint32_t user_id) {
  user_id_assignments_[entity] = user_id;
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
std::unique_ptr<PlatformAndroidExternalTextureSurface>
SplitEngineSerializerImpl::CreateAndroidExternalTextureSurface(
    ContentSecurityLevel security_level,
    absl::Span<const SurfaceViewType> view_types) {
  return std::make_unique<SplitEnginePlatformAndroidExternalTextureSurface>(
      view_, security_level, view_types);
}
#endif

void SplitEngineSerializerImpl::SetBoxCollider(utils::Entity entity,
                                               const Box& box, bool enabled) {
  collider_add_or_updates_[entity] = AddOrUpdateColliderInfo{box, enabled};
}

void SplitEngineSerializerImpl::SetMeshCollider(utils::Entity entity,
                                                bool enabled) {
  collider_add_or_updates_[entity] =
      AddOrUpdateColliderInfo{MeshCollider(), enabled};
}

void SplitEngineSerializerImpl::SetSphereCollider(utils::Entity entity,
                                                  const Sphere& sphere,
                                                  bool enabled) {
  collider_add_or_updates_[entity] = AddOrUpdateColliderInfo{sphere, enabled};
}

void SplitEngineSerializerImpl::SetCapsuleCollider(utils::Entity entity,
                                                   const Capsule& capsule,
                                                   bool enabled) {
  collider_add_or_updates_[entity] = AddOrUpdateColliderInfo{capsule, enabled};
}

void SplitEngineSerializerImpl::ClearCollider(
    utils::Entity entity,
    split_engine::SplitEngineSerializer::ColliderType collider_type) {
  // Remove the data from the collider_add_or_updates as the add/update data is
  // no longer valid.
  collider_add_or_updates_.erase(entity);
  collider_removals_[entity] = GetColliderType(collider_type);
}

void SplitEngineSerializerImpl::SerializeAddOrUpdateColliders(
    std::vector<FlatBufferBuilderPtr>& command_queue) {
  if (collider_add_or_updates_.empty()) return;

  command_queue.push_back(CreateFlatBufferBuilder());
  flatbuffers::FlatBufferBuilder& fbb = *command_queue.back();
  std::vector<flatbuffers::Offset<android_xr::schemas::AddOrUpdateCollider>>
      collider_add_or_updates(collider_add_or_updates_.size());
  absl::c_transform(
      collider_add_or_updates_, collider_add_or_updates.data(),
      [&fbb](const auto& entry) {
        const AddOrUpdateColliderInfo& collider_update = entry.second;

        struct Visitor {
          flatbuffers::FlatBufferBuilder& fbb;
          const uint32_t entity_id;
          const android_xr::schemas::Bool* enabled;
          flatbuffers::Offset<android_xr::schemas::AddOrUpdateCollider>
          operator()(const Box& value) {
            const android_xr::schemas::Float3 center(
                value.center.x, value.center.y, value.center.z);
            const android_xr::schemas::Float3 half_extent(
                value.halfExtent.x, value.halfExtent.y, value.halfExtent.z);
            return android_xr::schemas::CreateAddOrUpdateCollider(
                fbb, entity_id, ColliderData::BoxCollider,
                android_xr::schemas::CreateBoxCollider(fbb, &center,
                                                       &half_extent)
                    .Union(),
                enabled);
          }
          flatbuffers::Offset<android_xr::schemas::AddOrUpdateCollider>
          operator()(const MeshCollider& value) {
            return android_xr::schemas::CreateAddOrUpdateCollider(
                fbb, entity_id, ColliderData::MeshCollider,
                android_xr::schemas::CreateMeshCollider(fbb).Union(), enabled);
          }
          flatbuffers::Offset<android_xr::schemas::AddOrUpdateCollider>
          operator()(const Sphere& value) {
            const android_xr::schemas::Float3 center(
                value.center.x, value.center.y, value.center.z);
            const android_xr::schemas::Float radius(value.radius);
            return android_xr::schemas::CreateAddOrUpdateCollider(
                fbb, entity_id, ColliderData::SphereCollider,
                android_xr::schemas::CreateSphereCollider(fbb, &center, &radius)
                    .Union(),
                enabled);
          }
          flatbuffers::Offset<android_xr::schemas::AddOrUpdateCollider>
          operator()(const Capsule& value) {
            const android_xr::schemas::Float3 center(
                value.center.x, value.center.y, value.center.z);
            const android_xr::schemas::Float height(value.height);
            const android_xr::schemas::Float radius(value.radius);
            return android_xr::schemas::CreateAddOrUpdateCollider(
                fbb, entity_id, ColliderData::CapsuleCollider,
                android_xr::schemas::CreateCapsuleCollider(fbb, &center,
                                                           &height, &radius)
                    .Union(),
                enabled);
          }
        };
        return std::visit(Visitor{fbb, entry.first.getId(),
                                  collider_update.enabled.has_value()
                                      ? &collider_update.enabled.value()
                                      : nullptr},
                          collider_update.collider);
      });
  CreateCommand(fbb, android_xr::schemas::CreateAddOrUpdateColliders(
                         fbb, fbb.CreateVector(collider_add_or_updates)));
  collider_add_or_updates_.clear();
}

void SplitEngineSerializerImpl::SerializeRemoveColliders(
    std::vector<FlatBufferBuilderPtr>& command_queue) {
  if (collider_removals_.empty()) return;

  command_queue.push_back(CreateFlatBufferBuilder());
  flatbuffers::FlatBufferBuilder& fbb = *command_queue.back();
  std::vector<flatbuffers::Offset<android_xr::schemas::RemoveCollider>>
      collider_removals(collider_removals_.size());
  absl::c_transform(collider_removals_, collider_removals.data(),
                    [&fbb](const auto& entry) {
                      return android_xr::schemas::CreateRemoveCollider(
                          fbb, entry.first.getId(), entry.second);
                    });

  CreateCommand(fbb, android_xr::schemas::CreateRemoveColliders(
                         fbb, fbb.CreateVector(collider_removals)));
  collider_removals_.clear();
}

void SplitEngineSerializerImpl::SerializeRemoveMorphTargetBuffers(
    std::vector<FlatBufferBuilderPtr>& command_queue) {
  if (morph_target_buffers_to_remove_.empty()) return;

  command_queue.push_back(CreateFlatBufferBuilder());
  flatbuffers::FlatBufferBuilder& fbb = *command_queue.back();
  CreateCommand(
      fbb, android_xr::schemas::CreateRemoveMorphTargetBuffers(
               fbb, fbb.CreateVector(morph_target_buffers_to_remove_.data(),
                                     morph_target_buffers_to_remove_.size())));
  morph_target_buffers_to_remove_.clear();
}

void SplitEngineSerializerImpl::SerializeRemoveMeshData(
    std::vector<FlatBufferBuilderPtr>& command_queue) {
  if (vertex_buffers_to_remove_.empty() && index_buffers_to_remove_.empty()) {
    return;
  }
  command_queue.push_back(CreateFlatBufferBuilder());
  flatbuffers::FlatBufferBuilder& fbb = *command_queue.back();
  CreateCommand(fbb, android_xr::schemas::CreateRemoveMeshData(
                         fbb,
                         fbb.CreateVector(vertex_buffers_to_remove_.data(),
                                          vertex_buffers_to_remove_.size()),
                         fbb.CreateVector(index_buffers_to_remove_.data(),
                                          index_buffers_to_remove_.size())));
  vertex_buffers_to_remove_.clear();
  index_buffers_to_remove_.clear();
}

void SplitEngineSerializerImpl::SerializeRemoveImageBasedLightingAssets(
    std::vector<FlatBufferBuilderPtr>& command_queue) {
  if (image_based_lighting_assets_to_remove_.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "remove IBLs: count: "
             << image_based_lighting_assets_to_remove_.size();
  command_queue.push_back(CreateFlatBufferBuilder());
  flatbuffers::FlatBufferBuilder& fbb = *command_queue.back();

  CreateCommand(
      fbb, android_xr::schemas::CreateRemoveImageBasedLightingAssets(
               fbb, fbb.CreateVector(image_based_lighting_assets_to_remove_)));
  image_based_lighting_assets_to_remove_.clear();
}

void SplitEngineSerializerImpl::SerializeSetPreferredEnvironmentIblAsset(
    std::vector<FlatBufferBuilderPtr>& command_queue) {
  if (!preferred_environment_ibl_asset_id_.has_value()) return;

  IMP_LOG(imp::INFO)
      << kTag << "set preferred IBL: "
      << preferred_environment_ibl_asset_id_->image_based_lighting_asset_id;
  command_queue.push_back(CreateFlatBufferBuilder());
  flatbuffers::FlatBufferBuilder& fbb = *command_queue.back();
  const EnvironmentLightParams& params = *preferred_environment_ibl_asset_id_;
  const android_xr::schemas::Float3 tint = Pack(params.tint);
  CreateCommand(fbb, android_xr::schemas::CreateSetPreferredEnvironmentIblAsset(
                         fbb, params.image_based_lighting_asset_id,
                         params.intensity, &tint));
  preferred_environment_ibl_asset_id_ = std::nullopt;
}

void SplitEngineSerializerImpl::SerializeAddMaterials(
    std::vector<FlatBufferBuilderPtr>& command_queue) {
  if (!materials_builder_) return;

  IMP_LOG(imp::INFO) << kTag << "materials: " << materials_to_add_.size()
             << " new materials";
  CreateCommand(*materials_builder_,
                android_xr::schemas::CreateAddMaterials(
                    *materials_builder_,
                    materials_builder_->CreateVector(materials_to_add_)));
  command_queue.push_back(std::move(materials_builder_));
  materials_to_add_.clear();
}

void SplitEngineSerializerImpl::SerializeRemoveMaterials(
    std::vector<FlatBufferBuilderPtr>& command_queue) {
  if (materials_to_remove_.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "destroy materials: ";
  for (auto id : materials_to_remove_) {
    IMP_LOG(imp::INFO) << kTag << kIndent << id;
  }
  command_queue.push_back(CreateFlatBufferBuilder());
  flatbuffers::FlatBufferBuilder& fbb = *command_queue.back();
  CreateCommand(fbb, android_xr::schemas::CreateRemoveMaterials(
                         fbb, fbb.CreateVector(materials_to_remove_.data(),
                                               materials_to_remove_.size())));
  materials_to_remove_.clear();
}

void SplitEngineSerializerImpl::SerializeAddMaterialInstances(
    std::vector<FlatBufferBuilderPtr>& command_queue) {
  if (material_instances_to_add_.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "material instances: ";
  command_queue.push_back(CreateFlatBufferBuilder());

  flatbuffers::FlatBufferBuilder& fbb = *command_queue.back();
  std::vector<flatbuffers::Offset<android_xr::schemas::MaterialInstance>>
      instances(material_instances_to_add_.size());
  absl::c_transform(
      material_instances_to_add_, instances.data(), [&fbb](const auto& entry) {
        IMP_LOG(imp::INFO) << kTag << kIndent << entry.first << " -> " << entry.second;
        return android_xr::schemas::CreateMaterialInstance(fbb, entry.first,
                                                           entry.second);
      });
  CreateCommand(fbb,
                android_xr::schemas::CreateAddMaterialInstances(
                    fbb, fbb.CreateVector(instances.data(), instances.size())));
  material_instances_to_add_.clear();
}

void SplitEngineSerializerImpl::SerializeRemoveMaterialInstances(
    std::vector<FlatBufferBuilderPtr>& command_queue) {
  if (material_instances_to_remove_.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "destroy material instances: ";
  for (auto id : material_instances_to_remove_) {
    IMP_LOG(imp::INFO) << kTag << kIndent << id;
  }
  command_queue.push_back(CreateFlatBufferBuilder());
  flatbuffers::FlatBufferBuilder& fbb = *command_queue.back();
  CreateCommand(
      fbb, android_xr::schemas::CreateRemoveMaterialInstances(
               fbb, fbb.CreateVector(material_instances_to_remove_.data(),
                                     material_instances_to_remove_.size())));
  material_instances_to_remove_.clear();
}

void SplitEngineSerializerImpl::SerializeRemoveTextures(
    std::vector<FlatBufferBuilderPtr>& command_queue) {
  if (textures_to_remove_.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "destroy textures: ";
  for (auto id : textures_to_remove_) {
    IMP_LOG(imp::INFO) << kTag << kIndent << id;
  }
  command_queue.push_back(CreateFlatBufferBuilder());
  flatbuffers::FlatBufferBuilder& fbb = *command_queue.back();
  CreateCommand(fbb, android_xr::schemas::CreateRemoveTextures(
                         fbb, fbb.CreateVector(textures_to_remove_.data(),
                                               textures_to_remove_.size())));
  textures_to_remove_.clear();
}

void SplitEngineSerializerImpl::SerializeAddNodes(
    std::vector<FlatBufferBuilderPtr>& command_queue) {
  if (add_nodes_.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "adding nodes:";
  command_queue.push_back(CreateFlatBufferBuilder());
  flatbuffers::FlatBufferBuilder& fbb = *command_queue.back();
  std::vector<flatbuffers::Offset<android_xr::schemas::AddNode>>
      add_nodes_offset(add_nodes_.size());
  absl::c_transform(
      add_nodes_, add_nodes_offset.data(), [&fbb](const utils::Entity& entry) {
        return android_xr::schemas::CreateAddNode(fbb, entry.getId());
      });
  CreateCommand(fbb, android_xr::schemas::CreateAddNodes(
                         fbb, fbb.CreateVector(add_nodes_offset.data(),
                                               add_nodes_offset.size())));
  add_nodes_.clear();
}

void SplitEngineSerializerImpl::SerializeRemoveNodes(
    std::vector<FlatBufferBuilderPtr>& command_queue) {
  if (remove_nodes_.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "removing nodes:";
  for (const utils::Entity& entity : remove_nodes_) {
    IMP_LOG(imp::INFO) << kTag << kIndent << entity.getId();
  }
  command_queue.push_back(CreateFlatBufferBuilder());
  flatbuffers::FlatBufferBuilder& fbb = *command_queue.back();
  std::vector<flatbuffers::Offset<android_xr::schemas::RemoveNode>>
      remove_nodes_offset(remove_nodes_.size());
  absl::c_transform(remove_nodes_, remove_nodes_offset.data(),
                    [&fbb](const utils::Entity& entry) {
                      return android_xr::schemas::CreateRemoveNode(
                          fbb, entry.getId());
                    });
  CreateCommand(fbb, android_xr::schemas::CreateRemoveNodes(
                         fbb, fbb.CreateVector(remove_nodes_offset.data(),
                                               remove_nodes_offset.size())));
  remove_nodes_.clear();
}

void SplitEngineSerializerImpl::SerializeAssignUserIdToNodes(
    std::vector<FlatBufferBuilderPtr>& command_queue) {
  if (user_id_assignments_.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "assigning user ids:";
  command_queue.push_back(CreateFlatBufferBuilder());
  flatbuffers::FlatBufferBuilder& fbb = *command_queue.back();
  std::vector<flatbuffers::Offset<android_xr::schemas::AssignUserIdToNode>>
      user_id_assignments_offset(user_id_assignments_.size());
  absl::c_transform(user_id_assignments_, user_id_assignments_offset.data(),
                    [&fbb](const auto& entry) {
                      return android_xr::schemas::CreateAssignUserIdToNode(
                          fbb, entry.first.getId(), entry.second);
                    });
  CreateCommand(fbb,
                android_xr::schemas::CreateAssignUserIdToNodes(
                    fbb, fbb.CreateVector(user_id_assignments_offset.data(),
                                          user_id_assignments_offset.size())));
  user_id_assignments_.clear();
}

void SplitEngineSerializerImpl::SerializeUpdateNodes(
    std::vector<FlatBufferBuilderPtr>& command_queue) {
  if (node_updates_.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "updating nodes:";
  command_queue.push_back(CreateFlatBufferBuilder());
  flatbuffers::FlatBufferBuilder& fbb = *command_queue.back();
  std::vector<flatbuffers::Offset<android_xr::schemas::UpdateNode>>
      node_updates(node_updates_.size());
  absl::c_transform(
      node_updates_, node_updates.data(), [&fbb](const auto& entry) {
        const UpdateNodeInfo& update = entry.second;

        // Process the name of the node.
        flatbuffers::Offset<flatbuffers::String> name;
        if (update.name.has_value()) {
          name = fbb.CreateString(*update.name);
        }

        flatbuffers::Offset<android_xr::schemas::Transform> transform;
        if (update.transform.has_value()) {
          struct Visitor {
            flatbuffers::FlatBufferBuilder& fbb;
            flatbuffers::Offset<android_xr::schemas::Transform> operator()(
                const mat4f& value) {
              return android_xr::schemas::CreateTransform(
                  fbb, android_xr::schemas::TransformData::Mat4f,
                  fbb.CreateStruct(flatbuffers::Pack(value)).Union());
            }
            flatbuffers::Offset<android_xr::schemas::Transform> operator()(
                const mat4& value) {
              return android_xr::schemas::CreateTransform(
                  fbb, android_xr::schemas::TransformData::Mat4,
                  fbb.CreateStruct(flatbuffers::Pack(value)).Union());
            }
          };
          transform = std::visit(Visitor{fbb}, *update.transform);
        }

        flatbuffers::Offset<android_xr::schemas::Parent> parent;
        if (update.parent.has_value()) {
          parent =
              android_xr::schemas::CreateParent(fbb, update.parent->getId());
        }

        NodeHandle node = NodeHandle(entry.first);
        IMP_LOG(imp::INFO) << kTag << kIndent << ToString(node);
        return android_xr::schemas::CreateUpdateNode(
            fbb, entry.first.getId(), name, PointerFromOptional(update.enabled),
            transform, parent);
      });
  CreateCommand(fbb, android_xr::schemas::CreateUpdateNodes(
                         fbb, fbb.CreateVector(node_updates.data(),
                                               node_updates.size())));
  node_updates_.clear();
}

void SplitEngineSerializerImpl::SerializeAddRenderables(
    std::vector<FlatBufferBuilderPtr>& command_queue) {
  if (add_renderables_.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "adding renderables:";
  command_queue.push_back(CreateFlatBufferBuilder());
  flatbuffers::FlatBufferBuilder& fbb = *command_queue.back();
  std::vector<flatbuffers::Offset<android_xr::schemas::AddRenderable>>
      add_renderables_offset(add_renderables_.size());
  absl::c_transform(
      add_renderables_, add_renderables_offset.data(),
      [&fbb](const auto& entry) {
        const AddRenderableInfo& add = entry.second;

        flatbuffers::Offset<android_xr::schemas::MorphTargetData>
            morph_target_data;
        if (add.morph_target_data.has_value()) {
          VectorOffset<android_xr::schemas::MorphTargetInfo> morph_target_info(
              add.morph_target_data->morph_target_info.size());
          absl::c_transform(add.morph_target_data->morph_target_info,
                            morph_target_info.data(),
                            [&fbb](const MorphTargetInfo& morph_target) {
                              return android_xr::schemas::CreateMorphTargetInfo(
                                  fbb, morph_target.morph_target_buffer_offset,
                                  morph_target.morph_target_buffer_count);
                            });
          IMP_LOG(imp::INFO) << kTag << kIndent << "morph target size: "
                     << add.morph_target_data->morph_target_info.size();
          morph_target_data = android_xr::schemas::CreateMorphTargetData(
              fbb,
              PointerFromOptional(
                  add.morph_target_data->morph_target_buffer_id),
              fbb.CreateVector(morph_target_info));
        }
        flatbuffers::Offset<android_xr::schemas::RenderableFlags>
            renderable_flags;
        if (add.renderable_flags.has_value()) {
          renderable_flags = android_xr::schemas::CreateRenderableFlags(
              fbb, PointerFromOptional(add.renderable_flags->culling_enabled));
          IMP_LOG(imp::INFO) << kTag << kIndent << "culling enabled: "
                     << add.renderable_flags->culling_enabled->value();
        }

        if (add.skinning_bone_count.has_value()) {
          IMP_LOG(imp::INFO) << kTag << kIndent << "skinning bone count: "
                     << add.skinning_bone_count->value();
        }

        return android_xr::schemas::CreateAddRenderable(
            fbb, entry.first.getId(), add.primitive_count,
            PointerFromOptional(add.skinning_bone_count), morph_target_data,
            renderable_flags);
      });
  CreateCommand(fbb, android_xr::schemas::CreateAddRenderables(
                         fbb, fbb.CreateVector(add_renderables_offset.data(),
                                               add_renderables_offset.size())));
  add_renderables_.clear();
}

void SplitEngineSerializerImpl::SerializeRemoveRenderables(
    std::vector<FlatBufferBuilderPtr>& command_queue) {
  if (remove_renderables_.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "removing renderables: ";
  command_queue.push_back(CreateFlatBufferBuilder());
  flatbuffers::FlatBufferBuilder& fbb = *command_queue.back();
  std::vector<flatbuffers::Offset<android_xr::schemas::RemoveRenderable>>
      remove_renderables_offset(remove_renderables_.size());
  absl::c_transform(remove_renderables_, remove_renderables_offset.data(),
                    [&fbb](const utils::Entity& entry) {
                      return android_xr::schemas::CreateRemoveRenderable(
                          fbb, entry.getId());
                    });
  CreateCommand(fbb,
                android_xr::schemas::CreateRemoveRenderables(
                    fbb, fbb.CreateVector(remove_renderables_offset.data(),
                                          remove_renderables_offset.size())));
  remove_renderables_.clear();
}

void SplitEngineSerializerImpl::SerializeUpdateRenderables(
    std::vector<FlatBufferBuilderPtr>& command_queue) {
  if (renderable_updates_.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "update renderables:";

  flatbuffers::FlatBufferBuilder& fbb = *GetFlatBufferBuilderFor(
      android_xr::schemas::CommandTypes::UpdateRenderables);
  std::vector<flatbuffers::Offset<android_xr::schemas::UpdateRenderable>>
      renderable_updates_offset(renderable_updates_.size());
  absl::c_transform(
      renderable_updates_, renderable_updates_offset.data(),
      [&fbb](const auto& entry) {
        const UpdateRenderableInfo& update = entry.second;

        VectorOffset<android_xr::schemas::PrimitiveUpdate> primitives(
            update.primitives.size());
        absl::c_transform(
            update.primitives, primitives.data(), [&fbb](const auto& entry) {
              IMP_LOG(imp::INFO) << kTag << kIndent << "primitive: " << entry.first;
              const PrimitiveUpdateInfo& primitive = entry.second;

              flatbuffers::Offset<android_xr::schemas::GeometryUpdate> geometry;
              if (primitive.geometry.has_value()) {
                IMP_LOG(imp::INFO) << kTag << kIndent << kIndent << "geometry: "
                           << primitive.geometry->vertex_buffer_id << " "
                           << primitive.geometry->index_buffer_id;
                geometry = android_xr::schemas::CreateGeometryUpdate(
                    fbb, primitive.geometry->vertex_buffer_id,
                    primitive.geometry->index_buffer_id,
                    primitive.geometry->offset, primitive.geometry->count,
                    primitive.geometry->primitive_type);
              }

              if (primitive.material_instance_id) {
                IMP_LOG(imp::INFO) << kTag << kIndent << kIndent << "material: "
                           << primitive.material_instance_id->value();
              }

              if (primitive.blend_order) {
                IMP_LOG(imp::INFO) << kTag << kIndent << kIndent
                           << "blend_order: " << primitive.blend_order->value();
              }

              if (primitive.global_blend_order_enabled) {
                IMP_LOG(imp::INFO) << kTag << kIndent << kIndent
                           << "global_blend_order_enabled: "
                           << primitive.global_blend_order_enabled->value();
              }

              IMP_LOG(imp::INFO) << kTag << kIndent << kIndent
                         << "done creating primitive update info";

              return android_xr::schemas::CreatePrimitiveUpdate(
                  fbb, entry.first, geometry,
                  PointerFromOptional(primitive.material_instance_id),
                  PointerFromOptional(primitive.blend_order),
                  PointerFromOptional(primitive.global_blend_order_enabled));
            });

        flatbuffers::Offset<android_xr::schemas::BoundsInfo> bounds_info;
        if (update.bounds) {
          IMP_LOG(imp::INFO) << kTag << kIndent << "bounds: " << update.bounds->center
                     << ", " << update.bounds->halfExtent;
          bounds_info = CreateBoundsInfo(fbb, *update.bounds);
        }

        flatbuffers::Offset<android_xr::schemas::LayerMask> layer_mask;
        if (update.layer_mask) {
          IMP_LOG(imp::INFO) << kTag << kIndent
                     << "layer_mask: " << update.layer_mask->select << ", "
                     << update.layer_mask->values;
          layer_mask = android_xr::schemas::CreateLayerMask(
              fbb, update.layer_mask->select, update.layer_mask->values);
        }

        if (update.priority) {
          IMP_LOG(imp::INFO) << kTag << kIndent
                     << "priority: " << update.priority->value();
        }

        IMP_LOG(imp::INFO) << kTag << kIndent << "renderable for entity "
                   << entry.first.getId();

        return android_xr::schemas::CreateUpdateRenderable(
            fbb, entry.first.getId(), fbb.CreateVector(primitives), bounds_info,
            layer_mask, update.bones, update.morph_weights,
            PointerFromOptional(update.priority));
      });
  CreateCommand(fbb,
                android_xr::schemas::CreateUpdateRenderables(
                    fbb, fbb.CreateVector(renderable_updates_offset.data(),
                                          renderable_updates_offset.size())));
  command_queue.push_back(std::move(renderable_updates_builder_));
  renderable_updates_.clear();
}

void SplitEngineSerializerImpl::SendCommandQueue(
    std::vector<FlatBufferBuilderPtr>& command_queue) {
  if (!command_queue.empty()) {
    SendMessageGroup(command_queue);
  } else {
    assert(materials_to_add_.empty());
    assert(materials_to_remove_.empty());
    assert(material_instances_to_add_.empty());
    assert(material_instances_to_remove_.empty());
    assert(material_params_.empty());
    assert(built_in_material_parameters_.empty());
    assert(material_update_commands_.empty());
    assert(textures_to_remove_.empty());
    assert(add_nodes_.empty());
    assert(node_updates_.empty());
    assert(remove_nodes_.empty());
    assert(user_id_assignments_.empty());
    assert(add_renderables_.empty());
    assert(renderable_updates_.empty());
    assert(remove_renderables_.empty());
    assert(image_based_lighting_assets_to_remove_.empty());
    assert(collider_removals_.empty());
    assert(collider_add_or_updates_.empty());
    assert(morph_target_buffers_to_remove_.empty());
    assert(vertex_buffers_to_remove_.empty());
    assert(index_buffers_to_remove_.empty());
  }
}

void SplitEngineSerializerImpl::SendMessageGroup(
    std::vector<FlatBufferBuilderPtr>& messages) {
  assert(!messages.empty());
  for (auto& message : messages) {
    bridge_sender_->SendMessage(*message);
  }
  bridge_sender_->EndMessageGroup();
}

// Serialization methods must be called in correct order to ensure that the
// renderer side can handle the commands.
// TODO: Automatically order commands based on their dependencies,
//                    as already done for material updates.
// TODO: Add unit tests for all these methods.
void SplitEngineSerializerImpl::Update(const FrameTime& frame_time) {
  // Note that FlatBufferBuilders that get pushed onto this queue not only get
  // queued for sending but also get queued for destruction right after the
  // send, because this (local/stack) vector takes ownership of the unique_ptrs.
  std::vector<FlatBufferBuilderPtr> command_queue;

  // Start by removing everything that was destroyed or removed this frame.
  // The order is determined by the order in which the corresponding types of
  // objects reference each other. i.e. Renderables reference Material
  // Instances, which reference Materials, which reference Textures, so they
  // should be removed in that order

  // Remove renderables. This is only written for renderables that are removed
  // without the node being destroyed, since destroying the node implies the
  // renderable is destroyed.
  SerializeRemoveRenderables(command_queue);

  // Remove colliders. This is only written for colliders that are removed
  // without the node being destroyed, since destroying the node implies the
  // colliders are destroyed.
  SerializeRemoveColliders(command_queue);

  // Remove all nodes destroyed this frame.
  SerializeRemoveNodes(command_queue);

  // Removes assets.
  SerializeRemoveMaterialInstances(command_queue);
  SerializeRemoveMaterials(command_queue);
  SerializeRemoveTextures(command_queue);
  SerializeRemoveMorphTargetBuffers(command_queue);
  SerializeRemoveMeshData(command_queue);
  SerializeRemoveImageBasedLightingAssets(command_queue);

  // Now start adding & updating things from this frame. This also must be done
  // in the correct order, i.e. a node must be added before a renderable can be
  // attached to it.

  SerializeSetPreferredEnvironmentIblAsset(command_queue);

  // Add materials before material instances reference them.
  SerializeAddMaterials(command_queue);
  SerializeAddMaterialInstances(command_queue);

  // If there are any parameters that have yet to be serialized in commands, do
  // that now.
  CommitCurrentMaterialUpdateBuilder();

  // Add all material update commands to the queue. These commands were already
  // created to maintain ordering.
  for (FlatBufferBuilderPtr& command_builder : material_update_commands_) {
    command_queue.push_back(std::move(command_builder));
  }
  material_update_commands_.clear();

  SerializeAddNodes(command_queue);

  // Assign user ids before updating nodes so node updates can use the user ids.
  SerializeAssignUserIdToNodes(command_queue);
  SerializeUpdateNodes(command_queue);

  SerializeAddRenderables(command_queue);
  SerializeUpdateRenderables(command_queue);

  SerializeAddOrUpdateColliders(command_queue);

  SendCommandQueue(command_queue);
}

}  // namespace imp::split_engine
