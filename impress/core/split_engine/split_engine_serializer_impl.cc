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

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/base/nullability.h"
#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "filament/filament/include/filament/MorphTargetBuffer.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "filament/filament/include/filament/TransformManager.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "filament/libs/utils/include/utils/FixedCapacityVector.h"
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
#include "core/math/flatbuffer_support.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/model/mesh/base_mesh_builder.h"
#include "core/ncsb/node_handle.h"
#include "core/split_engine/android/split_engine_android_bridge.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/image_based_lighting_helpers.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_custom_material.h"
#include "core/split_engine/shared/split_engine_defines.h"
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
// TODO: (broken link) - We should fix GenericMaterial to not use remote
// materials in local mode.
// TODO: (broken link) - Find a better way to check the placeholder material.
bool IsPlaceholderSplitEngineMaterial(const filament::Material* material) {
  return strcmp(material->getName(), "Split Engine Placeholder") == 0;
}

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
static_assert(
    DoEnumsMatch(
        BuiltInMaterialParameters::BuiltInMaterialGsplatBackgroundParameters,
        android_xr::schemas::BuiltInMaterialParameters::
            BuiltInMaterialGsplatBackgroundParameters),
    "Enum mismatch");

static_assert(DoEnumsMatch(BuiltInMaterialParameters::MIN,
                           android_xr::schemas::BuiltInMaterialParameters::MIN),
              "Enum mismatch");
static_assert(DoEnumsMatch(BuiltInMaterialParameters::MAX,
                           android_xr::schemas::BuiltInMaterialParameters::MAX),
              "Enum mismatch");

static_assert(android_xr::schemas::BuiltInMaterialParameters::MAX ==
                  android_xr::schemas::BuiltInMaterialParameters::
                      BuiltInMaterialGsplatBackgroundParameters,
              "New fields added but assert not updated");

}  // namespace

// TODO: (broken link) - Add more using statements (esp. android_xr::schemas) to
// shorten code and make it more readable.
using android_xr::schemas::ColliderData;
using android_xr::schemas::CommandTypes;
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
                              VertexBuffer* vertices, IndexBuffer* indices,
                              size_t offset, size_t minIndex, size_t maxIndex,
                              size_t count) noexcept override {
    return Geometry(index, type, vertices, indices, offset, count);
  }
  RenderableBuilder& Geometry(size_t index,
                              filament::backend::PrimitiveType type,
                              VertexBuffer* vertices, IndexBuffer* indices,
                              size_t offset, size_t count) noexcept override {
    GeometryUpdateInfo geometry_update_info;
    geometry_update_info.vertex_buffer_id = GetId(vertices);
    geometry_update_info.index_buffer_id = GetId(indices);
    geometry_update_info.offset = offset;
    geometry_update_info.count = count;
    geometry_update_info.primitive_type = static_cast<uint8_t>(type);
    update_renderable_info_.primitives[index].geometry = geometry_update_info;

    return *this;
  }
  RenderableBuilder& Geometry(size_t index,
                              filament::backend::PrimitiveType type,
                              VertexBuffer* vertices,
                              IndexBuffer* indices) noexcept override {
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
    Batch<CommandTypes::AddRenderables>& add_batch =
        serializer_.GetOrCreateBatch<CommandTypes::AddRenderables>({entity});
    add_batch.data[entity] = std::move(add_renderable_info_);

    Batch<CommandTypes::UpdateRenderables>& update_batch =
        serializer_.GetOrCreateBatch<CommandTypes::UpdateRenderables>({entity});
    update_batch.data[entity] = std::move(update_renderable_info_);

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
    BaseView& view, int32_t api_level,
    std::unique_ptr<SplitEngineAndroidBridge> bridge,
    std::unique_ptr<SplitEngineBridgeSender> bridge_sender,
    size_t bridge_buffer_size_bytes)
    : Updater(view),
      view_(view),
      api_level_(api_level),
      bridge_(std::move(bridge)),
      bridge_sender_(std::move(bridge_sender)),
      bridge_buffer_size_bytes_(bridge_buffer_size_bytes) {
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
  Batch<CommandTypes::RemoveRenderables>& batch =
      GetOrCreateBatch<CommandTypes::RemoveRenderables>({entity});
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
  Batch<CommandTypes::UpdateRenderables>& batch =
      GetOrCreateBatch<CommandTypes::UpdateRenderables>({entity},
                                                        {material_instance_id});
  batch.data[entity].primitives[primitiveIndex].material_instance_id =
      material_instance_id;
}
void SplitEngineSerializerImpl::SetGeometryAt(
    filament::RenderableManager::Instance instance, size_t primitiveIndex,
    filament::backend::PrimitiveType type, VertexBuffer* vertices,
    IndexBuffer* indices, size_t offset, size_t count) {
  const ResourceId vertex_buffer_id = GetId(vertices);
  const ResourceId index_buffer_id = GetId(indices);
  GeometryUpdateInfo geometry_update_info;
  geometry_update_info.vertex_buffer_id = vertex_buffer_id;
  geometry_update_info.index_buffer_id = index_buffer_id;
  geometry_update_info.offset = offset;
  geometry_update_info.count = count;
  geometry_update_info.primitive_type = static_cast<uint8_t>(type);

  utils::Entity entity = GetEntity(instance);

  Batch<CommandTypes::UpdateRenderables>& batch =
      GetOrCreateBatch<CommandTypes::UpdateRenderables>(
          {entity}, {vertex_buffer_id, index_buffer_id});
  batch.data[entity].primitives[primitiveIndex].geometry = geometry_update_info;
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

  utils::Entity entity = GetEntity(instance);
  Batch<CommandTypes::UpdateRenderables>& batch =
      GetOrCreateBatch<CommandTypes::UpdateRenderables>({entity});
  imp::BorrowedPtr<flatbuffers::FlatBufferBuilder> fbb =
      BorrowFlatBufferBuilder(batch);
  batch.data[entity].bones = android_xr::schemas::CreateBones(
      *fbb, fbb->CreateVectorOfNativeStructs<android_xr::schemas::Mat4f>(
                transforms, boneCount, Pack));
}

void SplitEngineSerializerImpl::SetAxisAlignedBoundingBox(
    filament::RenderableManager::Instance instance, const Box& aabb) {
  utils::Entity entity = GetEntity(instance);
  Batch<CommandTypes::UpdateRenderables>& batch =
      GetOrCreateBatch<CommandTypes::UpdateRenderables>({entity});
  batch.data[entity].bounds = aabb;
}

void SplitEngineSerializerImpl::SetPriority(
    filament::RenderableManager::Instance instance, uint8_t priority) {
  utils::Entity entity = GetEntity(instance);
  Batch<CommandTypes::UpdateRenderables>& batch =
      GetOrCreateBatch<CommandTypes::UpdateRenderables>({entity});
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
  Batch<CommandTypes::UpdateRenderables>& batch =
      GetOrCreateBatch<CommandTypes::UpdateRenderables>({entity});
  batch.data[entity].layer_mask = LayerMask{select, values};
}

void SplitEngineSerializerImpl::SetBlendOrderAt(
    filament::RenderableManager::Instance instance, size_t primitiveIndex,
    uint16_t order) {
  utils::Entity entity = GetEntity(instance);
  Batch<CommandTypes::UpdateRenderables>& batch =
      GetOrCreateBatch<CommandTypes::UpdateRenderables>({entity});
  batch.data[entity].primitives[primitiveIndex].blend_order = order;
}

void SplitEngineSerializerImpl::SetGlobalBlendOrderEnabledAt(
    filament::RenderableManager::Instance instance, size_t primitiveIndex,
    bool enabled) {
  utils::Entity entity = GetEntity(instance);
  Batch<CommandTypes::UpdateRenderables>& batch =
      GetOrCreateBatch<CommandTypes::UpdateRenderables>({entity});
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
  Batch<CommandTypes::UpdateRenderables>& batch =
      GetOrCreateBatch<CommandTypes::UpdateRenderables>({entity});
  imp::BorrowedPtr<flatbuffers::FlatBufferBuilder> fbb =
      BorrowFlatBufferBuilder(batch);

  batch.data[entity].morph_weights = android_xr::schemas::CreateMorphWeights(
      *fbb, fbb->CreateVector(weights, count), offset);
}

std::unique_ptr<BaseRenderableManager::Builder>
SplitEngineSerializerImpl::NewBuilder(size_t count) {
  return std::make_unique<RenderableBuilder>(*this, count);
}

// Copied from loaded_model_builder.cc
flatbuffers::Offset<android_xr::schemas::BoundsInfo> CreateBoundsInfo(
    flatbuffers::FlatBufferBuilder& fbb, const Box& bounds) {
  const float3 center = bounds.center;
  const float3 half_extent = bounds.halfExtent;
  android_xr::schemas::Box box(Pack(center), Pack(half_extent));

  return android_xr::schemas::CreateBoundsInfo(fbb, &box);
}

imp::OwnedPtr<flatbuffers::FlatBufferBuilder>
SplitEngineSerializerImpl::CreateFlatBufferBuilder(size_t size_bytes) {
  // Message groups are lazily began the first time anyone attempts to build a
  // message (create a FlatBufferBuilder) in a frame, and ended in Update() if
  // any messages were sent that frame.
  // There may be additional logic in the future to have multiple message groups
  // per frame, but for now it's always exactly one per frame.
  if (!frame_update_group_id_.has_value()) {
    const absl::StatusOr<MessageGroupId> group_id =
        bridge_sender_->BeginMessageGroup(
            bridge_buffer_size_bytes_,
            SplitEngineBridgeSender::MessageType::kFrameUpdate);
    
    frame_update_group_id_ = *group_id;
  }
  return bridge_sender_->CreateFlatBufferBuilder(*frame_update_group_id_,
                                                 size_bytes);
}

imp::OwnedPtr<flatbuffers::FlatBufferBuilder>
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
      bridge_sender_->BeginMessageGroup(
          kBufferSize, SplitEngineBridgeSender::MessageType::kOneShot);
  
  // Builder has to be shared between different tasks.
  imp::OwnedPtr<flatbuffers::FlatBufferBuilder> builder =
      bridge_sender_->CreateFlatBufferBuilder(*group_id, kBufferSize);

  // Offload potentially expensive serialization (copying huge amount of data)
  bridge_sender_->Schedule([split_engine_texture_serializer =
                                std::move(split_engine_texture_serializer),
                            builder = builder.Borrow()]() {
    flatbuffers::Offset<android_xr::schemas::Texture> offset =
        split_engine_texture_serializer->SerializeTexture(*builder);

    VectorOffset<android_xr::schemas::Texture> texture_vector;
    texture_vector.push_back(offset);
    CreateCommand(*builder,
                  android_xr::schemas::CreateAddTextures(
                      *builder, builder->CreateVector(texture_vector)));
    return absl::OkStatus();
  });

  bridge_sender_->SendMessage(*group_id, std::move(builder));
  bridge_sender_->EndMessageGroup(*group_id);

  bridge_sender_->Schedule([on_done = std::move(on_done)]() {
    on_done();
    return absl::OkStatus();
  });
}

void SplitEngineSerializerImpl::RemoveTexture(filament::Texture& texture) {
  const ResourceId texture_id = GetId(&texture);
  Batch<CommandTypes::RemoveTextures>& batch =
      GetOrCreateEndOfFrameBatch<CommandTypes::RemoveTextures>(
          RemoveResourceChannel::kTexture);
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
      bridge_sender_->BeginMessageGroup(
          kAddMeshBufferSize, SplitEngineBridgeSender::MessageType::kOneShot);
  

  imp::OwnedPtr<flatbuffers::FlatBufferBuilder> mesh_builder =
      bridge_sender_->CreateFlatBufferBuilder(*group_id, kAddMeshBufferSize);

  bridge_sender_->Schedule(
      [mesh_builder = mesh_builder.Borrow(), &split_engine_mesh_serializer]() {
        SplitEngineMeshSerializer::VertexBufferVector vertex_buffer_offsets =
            split_engine_mesh_serializer.SerializeVertexBuffers(*mesh_builder);

        SplitEngineMeshSerializer::IndexBufferVector index_buffer_offsets =
            split_engine_mesh_serializer.SerializeIndexBuffers(*mesh_builder);

        CreateCommand(*mesh_builder, android_xr::schemas::CreateAddMeshData(
                                         *mesh_builder, vertex_buffer_offsets,
                                         index_buffer_offsets));
        return absl::OkStatus();
      });

  bridge_sender_->SendMessage(*group_id, std::move(mesh_builder));
  bridge_sender_->EndMessageGroup(*group_id);
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
    bridge_sender_->Schedule(
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
      bridge_sender_->BeginMessageGroup(
          kAddMorphTargetBufferSize,
          SplitEngineBridgeSender::MessageType::kOneShot);
  

  imp::OwnedPtr<flatbuffers::FlatBufferBuilder> morph_target_buffer_builder =
      bridge_sender_->CreateFlatBufferBuilder(*group_id,
                                              kAddMorphTargetBufferSize);

  bridge_sender_->Schedule([morph_target_buffer_builder =
                                morph_target_buffer_builder.Borrow(),
                            split_engine_mesh_serializer =
                                std::move(split_engine_mesh_serializer)]() {
    SplitEngineMeshSerializer::MorphTargetBufferVector morph_buffer_offsets =
        split_engine_mesh_serializer->SerializeMorphTargetBuffers(
            *morph_target_buffer_builder);
    CreateCommand(*morph_target_buffer_builder,
                  android_xr::schemas::CreateAddMorphTargetBuffers(
                      *morph_target_buffer_builder, morph_buffer_offsets));

    return absl::OkStatus();
  });

  bridge_sender_->SendMessage(*group_id,
                                       std::move(morph_target_buffer_builder));
  bridge_sender_->EndMessageGroup(*group_id);
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
    SphericalHarmonics spherical_harmonics,
    ImageBasedLightingAssetCubemapImages cubemap_images) {
  const size_t kBufferSize = EstimateImageBasedLightingAssetBufferSize(
      spherical_harmonics, cubemap_images);
  const ResourceId texture_id = GetId(&reflection_texture);

  const absl::StatusOr<MessageGroupId> group_id =
      bridge_sender_->BeginMessageGroup(
          kBufferSize, SplitEngineBridgeSender::MessageType::kOneShot);
  

  imp::OwnedPtr<flatbuffers::FlatBufferBuilder> builder =
      bridge_sender_->CreateFlatBufferBuilder(*group_id, kBufferSize);
  bridge_sender_->Schedule([builder = builder.Borrow(), texture_id,
                            spherical_harmonics =
                                std::move(spherical_harmonics),
                            cubemap_images = std::move(cubemap_images)]() {
    flatbuffers::Offset<android_xr::schemas::ImageBasedLightingAsset> asset =
        PackImageBasedLightingAsset(*builder, texture_id, spherical_harmonics,
                                    cubemap_images);

    CreateCommand(*builder,
                  android_xr::schemas::CreateAddImageBasedLightingAssets(
                      *builder, builder->CreateVector({asset})));

    return absl::OkStatus();
  });

  bridge_sender_->SendMessage(*group_id, std::move(builder));
  bridge_sender_->EndMessageGroup(*group_id);
}

void SplitEngineSerializerImpl::RemoveImageBasedLightingAsset(
    filament::Texture& reflection_texture) {
  const ResourceId ibl_id = GetId(&reflection_texture);
  Batch<CommandTypes::RemoveImageBasedLightingAssets>& batch =
      GetOrCreateEndOfFrameBatch<CommandTypes::RemoveImageBasedLightingAssets>(
          RemoveResourceChannel::kTexture);
  batch.data.push_back(ibl_id);
  RemoveId(ibl_id);
}

void SplitEngineSerializerImpl::SetPreferredEnvironmentIblAsset(
    filament::Texture& reflection_texture, float intensity,
    const float3& tint) {
  const ResourceId ibl_id = GetId(&reflection_texture);
  Batch<CommandTypes::SetPreferredEnvironmentIblAsset>& batch =
      GetOrCreateBatch<CommandTypes::SetPreferredEnvironmentIblAsset>({},
                                                                      {ibl_id});
  batch.data = EnvironmentLightParams{.image_based_lighting_asset_id = ibl_id,
                                      .intensity = intensity,
                                      .tint = tint};
}

void SplitEngineSerializerImpl::ClearPreferredEnvironmentIblAsset() {
  // Setting the image based lighting asset id to 0 indicates to clear any
  // previously-set preferred environment IBL asset.
  Batch<CommandTypes::SetPreferredEnvironmentIblAsset>& batch =
      GetOrCreateBatch<CommandTypes::SetPreferredEnvironmentIblAsset>();
  batch.data = EnvironmentLightParams{
      .image_based_lighting_asset_id = 0, .intensity = 0, .tint = {0, 0, 0}};
}

void SplitEngineSerializerImpl::RemoveMorphTargetBuffer(
    filament::MorphTargetBuffer* morph_target_buffer) {
  const ResourceId buffer_id = GetId(morph_target_buffer);
  Batch<CommandTypes::RemoveMorphTargetBuffers>& batch =
      GetOrCreateEndOfFrameBatch<CommandTypes::RemoveMorphTargetBuffers>(
          RemoveResourceChannel::kMesh);
  batch.data.push_back(buffer_id);

  // It is safe to remove the id now, because the MorphTargetBuffer serializer
  // uses stored morph_target_buffer_id instead of calling
  // SplitEngineSerializer::GetId
  RemoveId(buffer_id);
}

void SplitEngineSerializerImpl::RemoveVertexBuffer(
    VertexBuffer* vertex_buffer) {
  const ResourceId buffer_id = GetId(vertex_buffer);
  Batch<CommandTypes::RemoveMeshData>& batch =
      GetOrCreateEndOfFrameBatch<CommandTypes::RemoveMeshData>(
          RemoveResourceChannel::kMesh);
  batch.data.vertex_buffers.push_back(buffer_id);

  // It is safe to remove the id now, because the VertexBuffer serializer uses
  // stored vertex_buffer_id instead of calling SplitEngineSerializer::GetId
  RemoveId(buffer_id);
}

void SplitEngineSerializerImpl::RemoveIndexBuffer(IndexBuffer* index_buffer) {
  const ResourceId buffer_id = GetId(index_buffer);
  Batch<CommandTypes::RemoveMeshData>& batch =
      GetOrCreateEndOfFrameBatch<CommandTypes::RemoveMeshData>(
          RemoveResourceChannel::kMesh);
  batch.data.index_buffers.push_back(buffer_id);

  // It is safe to remove the id now, because the IndexBuffer serializer uses
  // stored index_buffer_id instead of calling SplitEngineSerializer::GetId
  RemoveId(buffer_id);
}

imp::BorrowedPtr<flatbuffers::FlatBufferBuilder>
SplitEngineSerializerImpl::BorrowFlatBufferBuilder(CommandBatchBase& batch) {
  // Return existing or create new FlatBufferBuilder.
  auto it = fbb_.find(&batch);
  if (it != fbb_.end()) {
    return it->second.Borrow();
  }

  return fbb_.emplace(&batch, CreateFlatBufferBuilder()).first->second.Borrow();
}

imp::OwnedPtr<flatbuffers::FlatBufferBuilder>
SplitEngineSerializerImpl::ReleaseFlatBufferBuilder(CommandBatchBase& batch) {
  auto it = fbb_.find(&batch);
  if (it == fbb_.end()) {
    // Some commands does not create flatbuffer builder at the time of batch
    // creation. In this case we just return an empty builder.
    return CreateFlatBufferBuilder();
  }
  imp::OwnedPtr<flatbuffers::FlatBufferBuilder> fbb = std::move(it->second);
  fbb_.erase(it);
  return fbb;
}

int32_t SplitEngineSerializerImpl::GetApiLevel() const { return api_level_; }

SplitEngineAndroidBridge& SplitEngineSerializerImpl::GetBridge() {
  return *bridge_;
}

bool SplitEngineSerializerImpl::ReadyForNextFrame() const {
  const absl::StatusOr<size_t> in_flight_frame_count =
      bridge_sender_->GetActiveMessageGroupCount();
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
  Batch<CommandTypes::AddMaterials>& batch =
      GetOrCreateBatch<CommandTypes::AddMaterials>({}, {material_id});
  imp::BorrowedPtr<flatbuffers::FlatBufferBuilder> fbb =
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
                *fbb, fbb->CreateString(constant.name),
                android_xr::schemas::MaterialPrecompileConstantValue::Int,
                fbb->CreateStruct(Pack(*constant.int_value())).Union()));
        IMP_LOG(imp::INFO) << kTag << kIndent
                   << "material precompile constant: " << constant.name
                   << " int value: " << *constant.int_value();
        break;
      case MaterialPreCompileConstant::kValue_FloatValue:
        constants.push_back(
            android_xr::schemas::CreateMaterialPrecompileConstant(
                *fbb, fbb->CreateString(constant.name),
                android_xr::schemas::MaterialPrecompileConstantValue::Float,
                fbb->CreateStruct(Pack(*constant.float_value())).Union()));
        IMP_LOG(imp::INFO) << kTag << kIndent
                   << "material precompile constant: " << constant.name
                   << " float value: " << *constant.float_value();
        break;
      case MaterialPreCompileConstant::kValue_BoolValue:
        constants.push_back(
            android_xr::schemas::CreateMaterialPrecompileConstant(
                *fbb, fbb->CreateString(constant.name),
                android_xr::schemas::MaterialPrecompileConstantValue::Bool,
                fbb->CreateStruct(Pack(*constant.bool_value())).Union()));
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
          *fbb, fbb->CreateVector(constants));

  batch.data.push_back(android_xr::schemas::CreateMaterial(
      *fbb, material_id, fbb->CreateVector(data.Data(), data.Size()), options));
}

void SplitEngineSerializerImpl::RemoveMaterial(
    const filament::Material* material) {
  if (IsPlaceholderSplitEngineMaterial(material) ||
      !view_.AreSplitEngineMaterialsInLocalMode()) {
    return;
  }
  const ResourceId material_id = GetId(material);
  IMP_LOG(imp::INFO) << kTag << "remove material: " << material_id;
  Batch<CommandTypes::RemoveMaterials>& batch =
      GetOrCreateEndOfFrameBatch<CommandTypes::RemoveMaterials>(
          RemoveResourceChannel::kMaterial);
  batch.data.push_back(material_id);
  RemoveId(material_id);
}

void SplitEngineSerializerImpl::AddMaterialInstance(
    const filament::Material* material,
    const filament::MaterialInstance* instance) {
  if (IsPlaceholderSplitEngineMaterial(material)) {
    return;
  }
  AddMaterialInstance(GetId(material), GetId(instance));
}

void SplitEngineSerializerImpl::AddMaterialInstance(uint64_t material_id,
                                                    uint64_t instance_id) {
  IMP_LOG(imp::INFO) << kTag << "add material instance: " << instance_id
            << " (using material id: " << material_id << ")";
  Batch<CommandTypes::AddMaterialInstances>& batch =
      GetOrCreateBatch<CommandTypes::AddMaterialInstances>(
          {}, {material_id, instance_id});
  batch.data.insert({instance_id, material_id});
}

template <>
void SplitEngineSerializerImpl::Batch<CommandTypes::SetMaterialParameters>::
    Serialize(flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return;

  VectorOffset<android_xr::schemas::MaterialParameters> params(data.size());
  absl::c_transform(data, params.data(), [&fbb](const auto& entry) {
    IMP_LOG(imp::INFO) << kTag << "update material params for instance: " << entry.first;
    return android_xr::schemas::CreateMaterialParameters(
        fbb, entry.first,
        fbb.CreateVector(entry.second.params.data(),
                         entry.second.params.size()),
        fbb.CreateVector(entry.second.texture_params.data(),
                         entry.second.texture_params.size()));
  });
  CreateCommand(fbb, android_xr::schemas::CreateSetMaterialParameters(
                         fbb, fbb.CreateVector(params)));
}

template <>
void SplitEngineSerializerImpl::
    Batch<CommandTypes::SetBuiltInMaterialParameters>::Serialize(
        flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return;

  CreateCommand(fbb, android_xr::schemas::CreateSetBuiltInMaterialParameters(
                         fbb, fbb.CreateVector(data)));
}

template <>
void SplitEngineSerializerImpl::
    Batch<CommandTypes::DuplicateMaterialInstances>::Serialize(
        flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return;

  std::vector<
      flatbuffers::Offset<android_xr::schemas::DuplicateMaterialInstance>>
      duplicates(data.size());
  absl::c_transform(data, duplicates.data(), [&fbb](const auto& entry) {
    IMP_LOG(imp::INFO) << kTag << "duplicate material instance: " << entry.second
               << " -> " << entry.first;
    return android_xr::schemas::CreateDuplicateMaterialInstance(
        fbb, entry.first, entry.second);
  });
  CreateCommand(fbb, android_xr::schemas::CreateDuplicateMaterialInstances(
                         fbb, fbb.CreateVector(duplicates)));
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
  Batch<CommandTypes::SetMaterialParameters>& batch =
      GetOrCreateBatch<CommandTypes::SetMaterialParameters>({}, {material_id});
  imp::BorrowedPtr<flatbuffers::FlatBufferBuilder> fbb =
      BorrowFlatBufferBuilder(batch);

  batch.data[material_id].params.push_back(AddMaterialParam(*fbb, name, value));
}

void SplitEngineSerializerImpl::SetMaterialParameter(
    const filament::MaterialInstance* material, absl::string_view name,
    const filament::Texture* texture, const filament::TextureSampler& sampler) {
  const ResourceId material_id = GetId(material);
  Batch<CommandTypes::SetMaterialParameters>& batch =
      GetOrCreateBatch<CommandTypes::SetMaterialParameters>({}, {material_id});
  imp::BorrowedPtr<flatbuffers::FlatBufferBuilder> fbb =
      BorrowFlatBufferBuilder(batch);

  flatbuffers::Offset<android_xr::schemas::TextureSampler> sampler_offset =
      CreateTextureSampler<SplitEngineTextureSamplerCreator>(*fbb, sampler);
  batch.data[material_id].texture_params.push_back(
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

MaterialPtr SplitEngineSerializerImpl::CreateCustomMaterial(
    MaterialPtr material) {
  return std::make_unique<split_engine::SplitEngineCustomMaterial>(
      *this, std::move(material));
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
  Batch<CommandTypes::SetBuiltInMaterialParameters>& batch =
      GetOrCreateBatch<CommandTypes::SetBuiltInMaterialParameters>(
          {}, {material_id});
  imp::BorrowedPtr<flatbuffers::FlatBufferBuilder> fbb =
      BorrowFlatBufferBuilder(batch);

  batch.data.push_back(
      android_xr::schemas::CreateBuiltInMaterialInstanceParameters(
          *fbb, material_id, schema_type, serialize_func(*fbb)));
}

void SplitEngineSerializerImpl::DuplicateMaterialInstance(
    const filament::MaterialInstance* instance,
    const filament::MaterialInstance* copy) {
  const ResourceId instance_id = GetId(instance);
  const ResourceId copy_id = GetId(copy);
  Batch<CommandTypes::DuplicateMaterialInstances>& batch =
      GetOrCreateBatch<CommandTypes::DuplicateMaterialInstances>(
          {}, {copy_id, instance_id});
  batch.data.insert({copy_id, instance_id});
}

void SplitEngineSerializerImpl::RemoveMaterialInstance(
    const filament::MaterialInstance* instance) {
  const ResourceId instance_id = GetId(instance);
  Batch<CommandTypes::RemoveMaterialInstances>& batch =
      GetOrCreateEndOfFrameBatch<CommandTypes::RemoveMaterialInstances>(
          RemoveResourceChannel::kMaterialInstance);
  batch.data.push_back(instance_id);
  RemoveId(instance_id);
}

void SplitEngineSerializerImpl::CreateNode(utils::Entity entity) {
  Batch<CommandTypes::AddNodes>& batch =
      GetOrCreateBatch<CommandTypes::AddNodes>({entity});
  batch.data.insert(entity);
}

void FillVectorWithChildren(filament::TransformManager& tm,
                            utils::Entity entity,
                            std::vector<utils::Entity>& out_entities) {
  filament::TransformManager::Instance ti = tm.getInstance(entity);

  if (!ti) {
    return;
  }

  std::size_t child_count = tm.getChildCount(ti);

  if (child_count == 0) {
    return;
  }

  out_entities.reserve(out_entities.size() + child_count);

  std::vector<utils::Entity> children(child_count);
  tm.getChildren(ti, children.data(), children.size());

  for (utils::Entity child : children) {
    FillVectorWithChildren(tm, child, out_entities);
  }

  out_entities.insert(out_entities.end(), children.begin(), children.end());
}

void SplitEngineSerializerImpl::DestroyNode(utils::Entity entity) {
  // When we get a RemoveNodes command, we need to add the given Entity's
  // children as its dependencies as well. For example, when Child Entity was a
  // child of Parent Entity:
  // (1) RemoveNodes(StandaloneEntity)
  // (2) UpdateRenderables(ChildEntity)
  // (3) RemoveNodes(ParentEntity)
  // (1) and (3) shouldn't be batched together.
  filament::TransformManager& tm =
      view_.GetSharedEngine()->getTransformManager();
  std::vector<utils::Entity> dependencies;
  FillVectorWithChildren(tm, entity, dependencies);
  dependencies.push_back(entity);

  Batch<CommandTypes::RemoveNodes>& batch =
      GetOrCreateBatch<CommandTypes::RemoveNodes>(dependencies);
  batch.data.insert(entity);
}

void SplitEngineSerializerImpl::SetEnabled(utils::Entity entity, bool enabled) {
  Batch<CommandTypes::UpdateNodes>& batch =
      GetOrCreateBatch<CommandTypes::UpdateNodes>({entity});
  batch.data[entity].enabled = enabled;
}

void SplitEngineSerializerImpl::SetName(utils::Entity entity,
                                        absl::string_view name) {
  Batch<CommandTypes::UpdateNodes>& batch =
      GetOrCreateBatch<CommandTypes::UpdateNodes>({entity});
  batch.data[entity].name = std::string(name);
}

void SplitEngineSerializerImpl::SetParent(utils::Entity entity,
                                          utils::Entity parent) {
  Batch<CommandTypes::UpdateNodes>& batch =
      GetOrCreateBatch<CommandTypes::UpdateNodes>({entity, parent});
  batch.data[entity].parent = parent;
}

void SplitEngineSerializerImpl::SetLocalTransform(utils::Entity entity,
                                                  const mat4f& transform) {
  Batch<CommandTypes::UpdateNodes>& batch =
      GetOrCreateBatch<CommandTypes::UpdateNodes>({entity});
  batch.data[entity].transform = transform;
}
void SplitEngineSerializerImpl::SetLocalTransform(utils::Entity entity,
                                                  const mat4& transform) {
  Batch<CommandTypes::UpdateNodes>& batch =
      GetOrCreateBatch<CommandTypes::UpdateNodes>({entity});
  batch.data[entity].transform = transform;
}

void SplitEngineSerializerImpl::AssignUserId(utils::Entity entity,
                                             uint32_t user_id) {
  Batch<CommandTypes::AssignUserIdToNodes>& batch =
      GetOrCreateBatch<CommandTypes::AssignUserIdToNodes>({entity});
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
  Batch<CommandTypes::AddOrUpdateColliders>& batch =
      GetOrCreateBatch<CommandTypes::AddOrUpdateColliders>({entity});
  batch.data[entity] = AddOrUpdateColliderInfo{box, enabled};
}

void SplitEngineSerializerImpl::SetMeshCollider(utils::Entity entity,
                                                bool enabled) {
  Batch<CommandTypes::AddOrUpdateColliders>& batch =
      GetOrCreateBatch<CommandTypes::AddOrUpdateColliders>({entity});
  batch.data[entity] = AddOrUpdateColliderInfo{MeshCollider(), enabled};
}

void SplitEngineSerializerImpl::SetSphereCollider(utils::Entity entity,
                                                  const Sphere& sphere,
                                                  bool enabled) {
  Batch<CommandTypes::AddOrUpdateColliders>& batch =
      GetOrCreateBatch<CommandTypes::AddOrUpdateColliders>({entity});
  batch.data[entity] = AddOrUpdateColliderInfo{sphere, enabled};
}

void SplitEngineSerializerImpl::SetCapsuleCollider(utils::Entity entity,
                                                   const Capsule& capsule,
                                                   bool enabled) {
  Batch<CommandTypes::AddOrUpdateColliders>& batch =
      GetOrCreateBatch<CommandTypes::AddOrUpdateColliders>({entity});
  batch.data[entity] = AddOrUpdateColliderInfo{capsule, enabled};
}

void SplitEngineSerializerImpl::ClearCollider(
    utils::Entity entity,
    split_engine::SplitEngineSerializer::ColliderType collider_type) {
  Batch<CommandTypes::RemoveColliders>& batch =
      GetOrCreateBatch<CommandTypes::RemoveColliders>({entity});
  batch.data[entity] =
      static_cast<android_xr::schemas::ColliderType>(collider_type);
}

template <>
void SplitEngineSerializerImpl::Batch<CommandTypes::AddOrUpdateColliders>::
    Serialize(flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return;

  VectorOffset<android_xr::schemas::AddOrUpdateCollider> add_or_updates(
      data.size());
  absl::c_transform(data, add_or_updates.data(), [&fbb](const auto& entry) {
    const AddOrUpdateColliderInfo& collider_update = entry.second;

    struct Visitor {
      flatbuffers::FlatBufferBuilder& fbb;
      const uint32_t entity_id;
      const android_xr::schemas::Bool* enabled;
      flatbuffers::Offset<android_xr::schemas::AddOrUpdateCollider> operator()(
          const Box& value) {
        const android_xr::schemas::Float3 center(value.center.x, value.center.y,
                                                 value.center.z);
        const android_xr::schemas::Float3 half_extent(
            value.halfExtent.x, value.halfExtent.y, value.halfExtent.z);
        return android_xr::schemas::CreateAddOrUpdateCollider(
            fbb, entity_id, ColliderData::BoxCollider,
            android_xr::schemas::CreateBoxCollider(fbb, &center, &half_extent)
                .Union(),
            enabled);
      }
      flatbuffers::Offset<android_xr::schemas::AddOrUpdateCollider> operator()(
          const MeshCollider& value) {
        return android_xr::schemas::CreateAddOrUpdateCollider(
            fbb, entity_id, ColliderData::MeshCollider,
            android_xr::schemas::CreateMeshCollider(fbb).Union(), enabled);
      }
      flatbuffers::Offset<android_xr::schemas::AddOrUpdateCollider> operator()(
          const Sphere& value) {
        const android_xr::schemas::Float3 center(value.center.x, value.center.y,
                                                 value.center.z);
        const android_xr::schemas::Float radius(value.radius);
        return android_xr::schemas::CreateAddOrUpdateCollider(
            fbb, entity_id, ColliderData::SphereCollider,
            android_xr::schemas::CreateSphereCollider(fbb, &center, &radius)
                .Union(),
            enabled);
      }
      flatbuffers::Offset<android_xr::schemas::AddOrUpdateCollider> operator()(
          const Capsule& value) {
        const android_xr::schemas::Float3 center(value.center.x, value.center.y,
                                                 value.center.z);
        const android_xr::schemas::Float height(value.height);
        const android_xr::schemas::Float radius(value.radius);
        return android_xr::schemas::CreateAddOrUpdateCollider(
            fbb, entity_id, ColliderData::CapsuleCollider,
            android_xr::schemas::CreateCapsuleCollider(fbb, &center, &height,
                                                       &radius)
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
                         fbb, fbb.CreateVector(add_or_updates)));
}

template <>
void SplitEngineSerializerImpl::Batch<CommandTypes::RemoveColliders>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return;

  VectorOffset<android_xr::schemas::RemoveCollider> removals(data.size());
  absl::c_transform(data, removals.data(), [&fbb](const auto& entry) {
    return android_xr::schemas::CreateRemoveCollider(fbb, entry.first.getId(),
                                                     entry.second);
  });

  CreateCommand(fbb, android_xr::schemas::CreateRemoveColliders(
                         fbb, fbb.CreateVector(removals)));
}

template <>
void SplitEngineSerializerImpl::Batch<CommandTypes::RemoveMorphTargetBuffers>::
    Serialize(flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return;

  CreateCommand(fbb, android_xr::schemas::CreateRemoveMorphTargetBuffers(
                         fbb, fbb.CreateVector(data.data(), data.size())));
}

template <>
void SplitEngineSerializerImpl::Batch<CommandTypes::RemoveMeshData>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.vertex_buffers.empty() && data.index_buffers.empty()) return;

  CreateCommand(fbb, android_xr::schemas::CreateRemoveMeshData(
                         fbb,
                         fbb.CreateVector(data.vertex_buffers.data(),
                                          data.vertex_buffers.size()),
                         fbb.CreateVector(data.index_buffers.data(),
                                          data.index_buffers.size())));
}

template <>
void SplitEngineSerializerImpl::
    Batch<CommandTypes::RemoveImageBasedLightingAssets>::Serialize(
        flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "remove IBLs: count: " << data.size();

  CreateCommand(fbb, android_xr::schemas::CreateRemoveImageBasedLightingAssets(
                         fbb, fbb.CreateVector(data)));
}

template <>
void SplitEngineSerializerImpl::
    Batch<CommandTypes::SetPreferredEnvironmentIblAsset>::Serialize(
        flatbuffers::FlatBufferBuilder& fbb) {
  if (!data.has_value()) return;

  IMP_LOG(imp::INFO) << kTag
             << "set preferred IBL: " << data->image_based_lighting_asset_id;
  const android_xr::schemas::Float3 tint = Pack(data->tint);
  CreateCommand(fbb, android_xr::schemas::CreateSetPreferredEnvironmentIblAsset(
                         fbb, data->image_based_lighting_asset_id,
                         data->intensity, &tint));
}

template <>
void SplitEngineSerializerImpl::Batch<CommandTypes::AddMaterials>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  IMP_LOG(imp::INFO) << kTag << "materials: " << data.size() << " new materials";
  CreateCommand(fbb, android_xr::schemas::CreateAddMaterials(
                         fbb, fbb.CreateVector(data)));
}

template <>
void SplitEngineSerializerImpl::Batch<CommandTypes::RemoveMaterials>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "destroy materials: ";
  for (auto id : data) {
    IMP_LOG(imp::INFO) << kTag << kIndent << id;
  }
  CreateCommand(fbb, android_xr::schemas::CreateRemoveMaterials(
                         fbb, fbb.CreateVector(data.data(), data.size())));
}

template <>
void SplitEngineSerializerImpl::Batch<CommandTypes::AddMaterialInstances>::
    Serialize(flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "material instances: ";
  VectorOffset<android_xr::schemas::MaterialInstance> instances(data.size());
  absl::c_transform(data, instances.data(), [&fbb](const auto& entry) {
    IMP_LOG(imp::INFO) << kTag << kIndent << entry.first << " -> " << entry.second;
    return android_xr::schemas::CreateMaterialInstance(fbb, entry.first,
                                                       entry.second);
  });
  CreateCommand(fbb,
                android_xr::schemas::CreateAddMaterialInstances(
                    fbb, fbb.CreateVector(instances.data(), instances.size())));
}

template <>
void SplitEngineSerializerImpl::Batch<CommandTypes::RemoveMaterialInstances>::
    Serialize(flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "destroy material instances: ";
  for (auto id : data) {
    IMP_LOG(imp::INFO) << kTag << kIndent << id;
  }
  CreateCommand(fbb, android_xr::schemas::CreateRemoveMaterialInstances(
                         fbb, fbb.CreateVector(data.data(), data.size())));
}

template <>
void SplitEngineSerializerImpl::Batch<CommandTypes::RemoveTextures>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "destroy textures: ";
  for (const auto id : data) {
    IMP_LOG(imp::INFO) << kTag << kIndent << id;
  }

  CreateCommand(fbb, android_xr::schemas::CreateRemoveTextures(
                         fbb, fbb.CreateVector(data.data(), data.size())));
}

template <>
void SplitEngineSerializerImpl::Batch<CommandTypes::AddNodes>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "adding nodes:";
  VectorOffset<android_xr::schemas::AddNode> offset(data.size());
  absl::c_transform(data, offset.data(), [&fbb](const utils::Entity& entry) {
    IMP_LOG(imp::INFO) << kTag << kIndent << entry.getId();
    return android_xr::schemas::CreateAddNode(fbb, entry.getId());
  });
  CreateCommand(fbb, android_xr::schemas::CreateAddNodes(
                         fbb, fbb.CreateVector(offset.data(), offset.size())));
}

template <>
void SplitEngineSerializerImpl::Batch<CommandTypes::RemoveNodes>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "removing nodes:";
  VectorOffset<android_xr::schemas::RemoveNode> offset(data.size());
  absl::c_transform(data, offset.data(), [&fbb](const utils::Entity& entry) {
    IMP_LOG(imp::INFO) << kTag << kIndent << entry.getId();
    return android_xr::schemas::CreateRemoveNode(fbb, entry.getId());
  });
  CreateCommand(fbb, android_xr::schemas::CreateRemoveNodes(
                         fbb, fbb.CreateVector(offset.data(), offset.size())));
}

template <>
void SplitEngineSerializerImpl::Batch<CommandTypes::AssignUserIdToNodes>::
    Serialize(flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "assigning user ids:";
  VectorOffset<android_xr::schemas::AssignUserIdToNode> offset(data.size());
  absl::c_transform(data, offset.data(), [&fbb](const auto& entry) {
    IMP_LOG(imp::INFO) << kTag << kIndent << entry.first.getId() << " -> "
               << entry.second;
    return android_xr::schemas::CreateAssignUserIdToNode(
        fbb, entry.first.getId(), entry.second);
  });
  CreateCommand(fbb, android_xr::schemas::CreateAssignUserIdToNodes(
                         fbb, fbb.CreateVector(offset.data(), offset.size())));
}

template <>
void SplitEngineSerializerImpl::Batch<CommandTypes::UpdateNodes>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return;

  IMP_LOG(imp::INFO) << kTag << "updating nodes:";
  VectorOffset<android_xr::schemas::UpdateNode> node_updates(data.size());
  absl::c_transform(data, node_updates.data(), [&fbb](const auto& entry) {
    const UpdateNodeInfo& update = entry.second;
    IMP_LOG(imp::INFO) << kTag << kIndent << entry.first.getId();
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
      parent = android_xr::schemas::CreateParent(fbb, update.parent->getId());
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
}

template <>
void SplitEngineSerializerImpl::Batch<CommandTypes::AddRenderables>::Serialize(
    flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return;

  VectorOffset<android_xr::schemas::AddRenderable> offset(data.size());
  absl::c_transform(data, offset.data(), [&fbb](const auto& entry) {
    const AddRenderableInfo& add = entry.second;
    IMP_LOG(imp::INFO) << kTag << "add renderable " << entry.first.getId() << ":";

    flatbuffers::Offset<android_xr::schemas::MorphTargetData> morph_target_data;
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
          PointerFromOptional(add.morph_target_data->morph_target_buffer_id),
          fbb.CreateVector(morph_target_info));
    }
    flatbuffers::Offset<android_xr::schemas::RenderableFlags> renderable_flags;
    if (add.renderable_flags.has_value()) {
      renderable_flags = android_xr::schemas::CreateRenderableFlags(
          fbb, PointerFromOptional(add.renderable_flags->culling_enabled));
      IMP_LOG(imp::INFO) << kTag << kIndent << "culling enabled: "
                 << add.renderable_flags->culling_enabled->value();
    }

    if (add.skinning_bone_count.has_value()) {
      IMP_LOG(imp::INFO) << kTag << kIndent
                 << "skinning bone count: " << add.skinning_bone_count->value();
    }

    return android_xr::schemas::CreateAddRenderable(
        fbb, entry.first.getId(), add.primitive_count,
        PointerFromOptional(add.skinning_bone_count), morph_target_data,
        renderable_flags);
  });
  CreateCommand(fbb, android_xr::schemas::CreateAddRenderables(
                         fbb, fbb.CreateVector(offset.data(), offset.size())));
}

template <>
void SplitEngineSerializerImpl::Batch<CommandTypes::RemoveRenderables>::
    Serialize(flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return;

  VectorOffset<android_xr::schemas::RemoveRenderable> offset(data.size());
  absl::c_transform(data, offset.data(), [&fbb](const utils::Entity& entry) {
    IMP_LOG(imp::INFO) << kTag << "remove renderable: " << entry.getId();
    return android_xr::schemas::CreateRemoveRenderable(fbb, entry.getId());
  });
  CreateCommand(fbb, android_xr::schemas::CreateRemoveRenderables(
                         fbb, fbb.CreateVector(offset.data(), offset.size())));
}

template <>
void SplitEngineSerializerImpl::Batch<CommandTypes::UpdateRenderables>::
    Serialize(flatbuffers::FlatBufferBuilder& fbb) {
  if (data.empty()) return;

  VectorOffset<android_xr::schemas::UpdateRenderable> offset(data.size());
  absl::c_transform(data, offset.data(), [&fbb](const auto& entry) {
    const UpdateRenderableInfo& update = entry.second;
    IMP_LOG(imp::INFO) << kTag << "update renderable: " << entry.first.getId() << ":";

    VectorOffset<android_xr::schemas::PrimitiveUpdate> primitives(
        update.primitives.size());
    absl::c_transform(
        update.primitives, primitives.data(), [&fbb](const auto& entry) {
          IMP_LOG(imp::INFO) << kTag << kIndent << "primitive: " << entry.first;
          const PrimitiveUpdateInfo& primitive = entry.second;

          flatbuffers::Offset<android_xr::schemas::GeometryUpdate> geometry;
          if (primitive.geometry.has_value()) {
            IMP_LOG(imp::INFO) << kTag << kIndent << kIndent
                       << "geometry: " << primitive.geometry->vertex_buffer_id
                       << " " << primitive.geometry->index_buffer_id;
            geometry = android_xr::schemas::CreateGeometryUpdate(
                fbb, primitive.geometry->vertex_buffer_id,
                primitive.geometry->index_buffer_id, primitive.geometry->offset,
                primitive.geometry->count, primitive.geometry->primitive_type);
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
                 << "priority: " << static_cast<int>(update.priority->value());
    }

    return android_xr::schemas::CreateUpdateRenderable(
        fbb, entry.first.getId(), fbb.CreateVector(primitives), bounds_info,
        layer_mask, update.bones, update.morph_weights,
        PointerFromOptional(update.priority));
  });
  CreateCommand(fbb, android_xr::schemas::CreateUpdateRenderables(
                         fbb, fbb.CreateVector(offset.data(), offset.size())));
}

void SplitEngineSerializerImpl::StoreAffectedDependenciesBatchIdx(
    const std::vector<utils::Entity>& entity_dependencies,
    const std::vector<ResourceId>& resource_dependencies, int batch_idx) {
  // Store for each entity and resource that the CommandBatch identified by
  // batch_idx contains the last command that affected this entity or resource.
  // The next command that affects the same entity or resource will have to be
  // executed in a later batch.
  for (const utils::Entity& entity : entity_dependencies) {
    last_batch_idx_affecting_entity_[entity] = batch_idx;
  }
  for (const auto& resource_id : resource_dependencies) {
    last_batch_idx_affecting_resource_[resource_id] = batch_idx;
  }
}

SplitEngineSerializerImpl::CommandBatchBase* /*absl_nullable*/ 
SplitEngineSerializerImpl::FindBatch(
    android_xr::schemas::CommandTypes command,
    const std::vector<utils::Entity>& entity_dependencies,
    const std::vector<ResourceId>& resource_dependencies) {
  // For each dependency, find the last batch that affected it. If no prior
  // batch affected this dependency, we can add this command to the first batch
  // we find that has the same type.
  int last_batch_idx = -1;
  for (const auto& dependency : entity_dependencies) {
    auto it = last_batch_idx_affecting_entity_.find(dependency);
    if (it != last_batch_idx_affecting_entity_.end()) {
      last_batch_idx = std::max(last_batch_idx, it->second);
    }
  }
  for (const auto& dependency : resource_dependencies) {
    auto it = last_batch_idx_affecting_resource_.find(dependency);
    if (it != last_batch_idx_affecting_resource_.end()) {
      last_batch_idx = std::max(last_batch_idx, it->second);
    }
  }

  // Check if we already have a batch of this command type that does not disrupt
  // the dependency chain.
  // NOTE: Since batch pointers are inserted in the order they are created, we
  // could use binary search on the index to find the first batch whose index is
  // greater than the last batch index.
  auto it = batches_.find(command);
  if (it != batches_.end()) {
    for (CommandBatchBase* batch : it->second) {
      // NOTE: An argument could be made that we should use greater instead of
      // greater or equal here. The difference is that if we allow for equal,
      // and the last batch is of the same type, we just append the current
      // command to the batch. This keeps correct order if the data is stored in
      // a format that by itself keeps order, like a vector, but order may be
      // broken if the data structure does not keep order, like a map. In this
      // case, removing "equal" would create a new batch even if the last batch
      // is of the same type and enforce order independent of the data type.
      // We could also add a boolean to each command type that indicates if the
      // data is stored in order or not and allow for the last batch to be used
      // or not based on this.
      if (batch->index >= last_batch_idx) {
        StoreAffectedDependenciesBatchIdx(entity_dependencies,
                                          resource_dependencies, batch->index);
        return batch;
      }
    }
  }

  return nullptr;
}

SplitEngineSerializerImpl::CommandBatchBase* /*absl_nonnull*/ 
SplitEngineSerializerImpl::AddBatch(
    std::unique_ptr<SplitEngineSerializerImpl::CommandBatchBase> batch,
    const std::vector<utils::Entity>& entity_dependencies,
    const std::vector<ResourceId>& resource_dependencies) {
  batch_queue_.push(std::move(batch));
  CommandBatchBase* batch_ptr = batch_queue_.back().get();
  batches_[batch_ptr->type].push_back(batch_ptr);
  StoreAffectedDependenciesBatchIdx(entity_dependencies, resource_dependencies,
                                    batch_ptr->index);
  return batch_ptr;
}

template <CommandTypes CommandT>
SplitEngineSerializerImpl::Batch<CommandT>&
SplitEngineSerializerImpl::GetOrCreateBatch(
    const std::vector<utils::Entity>& entity_dependencies,
    const std::vector<ResourceId>& resource_dependencies) {
  CommandBatchBase* batch =
      FindBatch(CommandT, entity_dependencies, resource_dependencies);

  if (batch == nullptr) {
    // The position of the batch in the queue is its index. This is required to
    // keep dependent batches in order.
    batch = AddBatch(std::make_unique<Batch<CommandT>>(batch_queue_.size()),
                     entity_dependencies, resource_dependencies);
  }

  return *static_cast<Batch<CommandT>*>(batch);
}

template <CommandTypes CommandT>
SplitEngineSerializerImpl::Batch<CommandT>&
SplitEngineSerializerImpl::GetOrCreateEndOfFrameBatch(
    RemoveResourceChannel channel) {
  size_t channel_index = static_cast<size_t>(channel);
  if (!end_of_frame_batches_[channel_index]) {
    end_of_frame_batches_[channel_index] = std::make_unique<Batch<CommandT>>(0);
  }
  return *static_cast<Batch<CommandT>*>(
      end_of_frame_batches_[channel_index].get());
}

void SplitEngineSerializerImpl::SendMessage(CommandBatchBase* batch_base) {
  imp::OwnedPtr<flatbuffers::FlatBufferBuilder> fbb =
      ReleaseFlatBufferBuilder(*batch_base);

  // Assumption is that serialization for frame updates happens fast and does
  // not require offloading to the background thread.
  batch_base->Serialize(*fbb);

  
      bridge_sender_->SendMessage(*frame_update_group_id_, std::move(fbb));
}

void SplitEngineSerializerImpl::SendAllBatches() {
  while (!batch_queue_.empty()) {
    CommandBatchBase* batch = batch_queue_.front().get();
    SendMessage(batch);
    batch_queue_.pop();
  }

  for (size_t i = 0; i < kRemoveResourceChannelCount; ++i) {
    if (!end_of_frame_batches_[i]) continue;

    CommandBatchBase* batch = end_of_frame_batches_[i].get();
    SendMessage(batch);
    end_of_frame_batches_[i] = nullptr;
  }

  // If `SendMessage` was never called in the code above, it means that
  // `BeginMessageGroup` was never called, and `EndMessageGroup`
  // should not be called too.
  if (!frame_update_group_id_.has_value()) return;

  bridge_sender_->EndMessageGroup(*frame_update_group_id_);
  frame_update_group_id_ = std::nullopt;

  // Clean up
  last_batch_idx_affecting_entity_.clear();
  last_batch_idx_affecting_resource_.clear();
  batches_.clear();
  fbb_.clear();
}

void SplitEngineSerializerImpl::Update(const FrameTime& frame_time) {
  // Check if any message groups are eligible for release.
  bridge_sender_->ClearReleasedMessageGroups();

  // Send all pending Command batches.
  SendAllBatches();
}

}  // namespace imp::split_engine
