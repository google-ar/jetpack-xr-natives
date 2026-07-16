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

#include "core/split_engine/split_engine_renderable_builder.h"

#include <cstddef>
#include <cstdint>
#include <utility>

#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Box.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "filament/filament/include/filament/MorphTargetBuffer.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "filament/libs/utils/include/utils/FixedCapacityVector.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/split_engine/split_engine_serializer_data_types.h"

namespace imp::split_engine {

namespace {
static constexpr absl::string_view kTag = "[SplitEngineRenderableBuilder]: ";
}  // namespace

SplitEngineRenderableBuilder::SplitEngineRenderableBuilder(
    SerializeCallback callback, size_t count) noexcept
    : serialize_callback_(std::move(callback)) {
  add_renderable_info_.primitive_count = count;
}

SplitEngineRenderableBuilder& SplitEngineRenderableBuilder::Geometry(
    size_t index, filament::backend::PrimitiveType type,
    filament::VertexBuffer* vertices, filament::IndexBuffer* indices,
    size_t offset, size_t minIndex, size_t maxIndex, size_t count) noexcept {
  return Geometry(index, type, vertices, indices, offset, count);
}

SplitEngineRenderableBuilder& SplitEngineRenderableBuilder::Geometry(
    size_t index, filament::backend::PrimitiveType type,
    filament::VertexBuffer* vertices, filament::IndexBuffer* indices,
    size_t offset, size_t count) noexcept {
  SerializerDataTypes::GeometryUpdateInfo geometry_update_info;
  geometry_update_info.vertex_buffer_id =
      SplitEngineSerializer::GetId(vertices);
  geometry_update_info.index_buffer_id = SplitEngineSerializer::GetId(indices);
  geometry_update_info.offset = offset;
  geometry_update_info.count = count;
  geometry_update_info.primitive_type = static_cast<uint8_t>(type);
  update_renderable_info_.primitives[index].geometry = geometry_update_info;

  return *this;
}

SplitEngineRenderableBuilder& SplitEngineRenderableBuilder::Geometry(
    size_t index, filament::backend::PrimitiveType type,
    filament::VertexBuffer* vertices, filament::IndexBuffer* indices) noexcept {
  return Geometry(index, type, vertices, indices, 0, indices->getIndexCount());
}

SplitEngineRenderableBuilder& SplitEngineRenderableBuilder::Material(
    size_t index,
    const filament::MaterialInstance* material_instance) noexcept {
  update_renderable_info_.primitives[index].material_instance_id =
      SplitEngineSerializer::GetId(material_instance);
  return *this;
}

SplitEngineRenderableBuilder& SplitEngineRenderableBuilder::BoundingBox(
    const filament::Box& axisAlignedBoundingBox) noexcept {
  update_renderable_info_.bounds = axisAlignedBoundingBox;
  return *this;
}

SplitEngineRenderableBuilder& SplitEngineRenderableBuilder::LayerMask(
    uint8_t select, uint8_t values) noexcept {
  update_renderable_info_.layer_mask =
      SerializerDataTypes::LayerMask{select, values};
  return *this;
}

SplitEngineRenderableBuilder& SplitEngineRenderableBuilder::Priority(
    uint8_t priority) noexcept {
  update_renderable_info_.priority = priority;
  return *this;
}

SplitEngineRenderableBuilder& SplitEngineRenderableBuilder::Channel(
    uint8_t channel) noexcept {
  IMP_LOG(imp::WARNING) << kTag << "Builder::Channel is not supported.";
  return *this;
}

SplitEngineRenderableBuilder& SplitEngineRenderableBuilder::Culling(
    bool enable) noexcept {
  if (!add_renderable_info_.renderable_flags) {
    add_renderable_info_.renderable_flags =
        SerializerDataTypes::RenderableFlags();
  }
  add_renderable_info_.renderable_flags->culling_enabled = enable;
  return *this;
}

SplitEngineRenderableBuilder& SplitEngineRenderableBuilder::CastShadows(
    bool enable) noexcept {
  IMP_LOG(imp::WARNING) << kTag << "Builder::CastShadows is not supported.";
  return *this;
}

SplitEngineRenderableBuilder& SplitEngineRenderableBuilder::ReceiveShadows(
    bool enable) noexcept {
  IMP_LOG(imp::WARNING) << kTag << "Builder::ReceiveShadows is not supported.";
  return *this;
}

SplitEngineRenderableBuilder&
SplitEngineRenderableBuilder::ScreenSpaceContactShadows(bool enable) noexcept {
  IMP_LOG(imp::WARNING) << kTag
               << "Builder::ScreenSpaceContactShadows is not supported.";
  return *this;
}

SplitEngineRenderableBuilder& SplitEngineRenderableBuilder::Skinning(
    filament::SkinningBuffer* skinningBuffer, size_t count,
    size_t offset) noexcept {
  IMP_LOG(imp::FATAL) << kTag
             << "skinning(filament::SkinningBuffer*, size_t, size_t) is not "
                "supported.";
  return *this;
}

SplitEngineRenderableBuilder& SplitEngineRenderableBuilder::Skinning(
    size_t boneCount, mat4f const* transforms) noexcept {
  IMP_LOG(imp::FATAL) << kTag << "skinning(size_t, mat4f*) is not supported.";
  return *this;
}

SplitEngineRenderableBuilder& SplitEngineRenderableBuilder::Skinning(
    size_t boneCount, filament::RenderableManager::Bone const* bones) noexcept {
  IMP_LOG(imp::FATAL) << kTag
             << "skinning(size_t, filament::RenderableManager::Bone*) is not "
                "supported.";
  return *this;
}

SplitEngineRenderableBuilder& SplitEngineRenderableBuilder::Skinning(
    size_t boneCount) noexcept {
  add_renderable_info_.skinning_bone_count = boneCount;
  return *this;
}

SplitEngineRenderableBuilder&
SplitEngineRenderableBuilder::BoneIndicesAndWeights(
    size_t primitiveIndex, float2 const* indicesAndWeights, size_t count,
    size_t bonesPerVertex) noexcept {
  // No-op. This is called when the skinning info is more advanced (has more
  // than 4 bone weights per vertex). It is not supported in split engine.
  IMP_LOG(imp::FATAL) << kTag
             << "Each vertex can only be influenced by a maximum of 4 bones.";
  return *this;
}

SplitEngineRenderableBuilder&
SplitEngineRenderableBuilder::BoneIndicesAndWeights(
    size_t primitiveIndex,
    utils::FixedCapacityVector<utils::FixedCapacityVector<float2>>
        indicesAndWeightsVector) noexcept {
  IMP_LOG(imp::FATAL) << kTag
             << "boneIndicesAndWeights(size_t, "
                "utils::FixedCapacityVector<float2>) is not supported.";
  return *this;
}

SplitEngineRenderableBuilder& SplitEngineRenderableBuilder::Morphing(
    filament::MorphTargetBuffer* morphTargetBuffer) noexcept {
  if (!add_renderable_info_.morph_target_data) {
    add_renderable_info_.morph_target_data =
        SerializerDataTypes::MorphTargetData();
  }
  add_renderable_info_.morph_target_data->morph_target_buffer_id =
      SplitEngineSerializer::GetId(morphTargetBuffer);
  return *this;
}

SplitEngineRenderableBuilder& SplitEngineRenderableBuilder::Morphing(
    uint8_t level, size_t primitiveIndex, size_t offset,
    size_t count) noexcept {
  if (!add_renderable_info_.morph_target_data) {
    add_renderable_info_.morph_target_data =
        SerializerDataTypes::MorphTargetData();
  }
  add_renderable_info_.morph_target_data->morph_target_info.resize(
      add_renderable_info_.primitive_count);

  SerializerDataTypes::MorphTargetInfo& morph_target_info =
      add_renderable_info_.morph_target_data->morph_target_info[primitiveIndex];
  morph_target_info.morph_target_buffer_offset = offset;
  morph_target_info.morph_target_buffer_count = count;

  return *this;
}

SplitEngineRenderableBuilder& SplitEngineRenderableBuilder::BlendOrder(
    size_t primitiveIndex, uint16_t order) noexcept {
  update_renderable_info_.primitives[primitiveIndex].blend_order = order;
  return *this;
}

SplitEngineRenderableBuilder&
SplitEngineRenderableBuilder::GlobalBlendOrderEnabled(size_t primitiveIndex,
                                                      bool enabled) noexcept {
  update_renderable_info_.primitives[primitiveIndex]
      .global_blend_order_enabled = enabled;
  return *this;
}

SplitEngineRenderableBuilder& SplitEngineRenderableBuilder::Instances(
    size_t instanceCount) noexcept {
  IMP_LOG(imp::WARNING) << kTag << "Builder::Instances is not supported.";
  return *this;
}

SplitEngineRenderableBuilder& SplitEngineRenderableBuilder::Instances(
    size_t instanceCount, filament::InstanceBuffer* instanceBuffer) noexcept {
  IMP_LOG(imp::WARNING) << kTag << "Builder::Instances is not supported.";
  return *this;
}

filament::RenderableManager::Builder::Result
SplitEngineRenderableBuilder::Build(filament::Engine& engine,
                                    utils::Entity entity) {
  serialize_callback_(entity, std::move(add_renderable_info_),
                      std::move(update_renderable_info_));
  return filament::RenderableManager::Builder::Result::Success;
}

SplitEngineRenderableBuilder&
SplitEngineRenderableBuilder::LightChannelInternal(unsigned int channel,
                                                   bool enable) noexcept {
  return *this;
}

SplitEngineRenderableBuilder&
SplitEngineRenderableBuilder::EnableSkinningBuffersInternal(
    bool enabled) noexcept {
  IMP_LOG(imp::WARNING) << kTag << "EnableSkinningBuffers is not supported.";
  return *this;
}

SplitEngineRenderableBuilder& SplitEngineRenderableBuilder::FogInternal(
    bool enabled) noexcept {
  return *this;
}

}  // namespace imp::split_engine
