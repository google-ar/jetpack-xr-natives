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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_RENDERABLE_BUILDER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_RENDERABLE_BUILDER_H_

#include <cstddef>
#include <cstdint>

#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Box.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "filament/filament/include/filament/MorphTargetBuffer.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "filament/libs/utils/include/utils/FixedCapacityVector.h"
#include "core/common/invocable.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/render/base_renderable_manager.h"
#include "core/split_engine/split_engine_serializer_data_types.h"

namespace imp::split_engine {

class SplitEngineRenderableBuilder : public BaseRenderableManager::Builder {
 public:
  using SerializeCallback =
      imp::Invocable<void(utils::Entity, SerializerDataTypes::AddRenderableInfo,
                          SerializerDataTypes::UpdateRenderableInfo)>;

  SplitEngineRenderableBuilder(SerializeCallback callback,
                               size_t count) noexcept;

  SplitEngineRenderableBuilder& Geometry(size_t index,
                                         filament::backend::PrimitiveType type,
                                         filament::VertexBuffer* vertices,
                                         filament::IndexBuffer* indices,
                                         size_t offset, size_t minIndex,
                                         size_t maxIndex,
                                         size_t count) noexcept override;
  SplitEngineRenderableBuilder& Geometry(size_t index,
                                         filament::backend::PrimitiveType type,
                                         filament::VertexBuffer* vertices,
                                         filament::IndexBuffer* indices,
                                         size_t offset,
                                         size_t count) noexcept override;
  SplitEngineRenderableBuilder& Geometry(
      size_t index, filament::backend::PrimitiveType type,
      filament::VertexBuffer* vertices,
      filament::IndexBuffer* indices) noexcept override;

  SplitEngineRenderableBuilder& Material(
      size_t index,
      const filament::MaterialInstance* material_instance) noexcept override;

  SplitEngineRenderableBuilder& BoundingBox(
      const filament::Box& axisAlignedBoundingBox) noexcept override;
  SplitEngineRenderableBuilder& LayerMask(uint8_t select,
                                          uint8_t values) noexcept override;
  SplitEngineRenderableBuilder& Priority(uint8_t priority) noexcept override;
  SplitEngineRenderableBuilder& Channel(uint8_t channel) noexcept override;
  SplitEngineRenderableBuilder& Culling(bool enable) noexcept override;
  SplitEngineRenderableBuilder& CastShadows(bool enable) noexcept override;
  SplitEngineRenderableBuilder& ReceiveShadows(bool enable) noexcept override;
  SplitEngineRenderableBuilder& ScreenSpaceContactShadows(
      bool enable) noexcept override;
  SplitEngineRenderableBuilder& Skinning(
      filament::SkinningBuffer* skinningBuffer, size_t count,
      size_t offset) noexcept override;
  SplitEngineRenderableBuilder& Skinning(
      size_t boneCount, mat4f const* transforms) noexcept override;
  SplitEngineRenderableBuilder& Skinning(
      size_t boneCount,
      filament::RenderableManager::Bone const* bones) noexcept override;
  SplitEngineRenderableBuilder& Skinning(size_t boneCount) noexcept override;
  SplitEngineRenderableBuilder& BoneIndicesAndWeights(
      size_t primitiveIndex, float2 const* indicesAndWeights, size_t count,
      size_t bonesPerVertex) noexcept override;
  SplitEngineRenderableBuilder& BoneIndicesAndWeights(
      size_t primitiveIndex,
      utils::FixedCapacityVector<utils::FixedCapacityVector<float2>>
          indicesAndWeightsVector) noexcept override;

  SplitEngineRenderableBuilder& Morphing(
      filament::MorphTargetBuffer* morphTargetBuffer) noexcept override;
  SplitEngineRenderableBuilder& Morphing(uint8_t level, size_t primitiveIndex,
                                         size_t offset,
                                         size_t count) noexcept override;
  SplitEngineRenderableBuilder& BlendOrder(size_t primitiveIndex,
                                           uint16_t order) noexcept override;
  SplitEngineRenderableBuilder& GlobalBlendOrderEnabled(
      size_t primitiveIndex, bool enabled) noexcept override;
  SplitEngineRenderableBuilder& Instances(
      size_t instanceCount) noexcept override;
  SplitEngineRenderableBuilder& Instances(
      size_t instanceCount,
      filament::InstanceBuffer* instanceBuffer) noexcept override;
  filament::RenderableManager::Builder::Result Build(
      filament::Engine& engine, utils::Entity entity) override;

 protected:
  SplitEngineRenderableBuilder& LightChannelInternal(
      unsigned int channel, bool enable) noexcept override;
  SplitEngineRenderableBuilder& EnableSkinningBuffersInternal(
      bool enabled) noexcept override;
  SplitEngineRenderableBuilder& FogInternal(bool enabled) noexcept override;

 private:
  SerializeCallback serialize_callback_;

  // Stores the information needed to later serialize a command to add a new
  // renderable. When Build is called, this is moved to the
  // SplitEngineSerializer.
  SerializerDataTypes::AddRenderableInfo add_renderable_info_;

  // The Builder also includes methods that update the renderable with state
  // serialized through the Update command, not just the Add command. This
  // info is stored here and moved to the SplitEngineSerializer when Build
  // is called.
  SerializerDataTypes::UpdateRenderableInfo update_renderable_info_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_RENDERABLE_BUILDER_H_
