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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_BASE_RENDERABLE_MANAGER_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_BASE_RENDERABLE_MANAGER_H_

#include <cstddef>
#include <cstdint>
#include <memory>

#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Box.h"
#include "filament/filament/include/filament/Material.h"
#include "filament/filament/include/filament/MorphTargetBuffer.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "filament/libs/utils/include/utils/FixedCapacityVector.h"
#include "core/math/mat.h"
#include "core/math/vec.h"

namespace imp {

class Material;

// Defines an interface for wrapping filament::RenderableManager.
// This allows SplitEngineSerializerImpl to act as a spy on all RM operations.
// TODO: this API is messy, would be nice if there was only one way
// to do this, i.e. only use the builder pattern and skip creation of the
// primitives if the entity already has renderables.
struct BaseRenderableManager {
  virtual ~BaseRenderableManager() = default;

  virtual void SetSpy(BaseRenderableManager& spy) = 0;
  virtual filament::RenderableManager::Instance GetInstance(
      utils::Entity e) const = 0;
  virtual bool HasComponent(utils::Entity e) const = 0;
  virtual void Destroy(utils::Entity e) = 0;
  virtual size_t GetPrimitiveCount(
      filament::RenderableManager::Instance instance) const = 0;
  virtual const filament::Box& GetAxisAlignedBoundingBox(
      filament::RenderableManager::Instance instance) const = 0;
  virtual void SetMaterialInstanceAt(
      filament::RenderableManager::Instance instance, size_t primitiveIndex,
      const filament::MaterialInstance* material_instance) = 0;
  virtual void ClearMaterialInstanceAt(
      filament::RenderableManager::Instance instance,
      size_t primitiveIndex) = 0;
  virtual void SetGeometryAt(filament::RenderableManager::Instance instance,
                             size_t primitiveIndex,
                             filament::backend::PrimitiveType type,
                             filament::VertexBuffer* vertices,
                             filament::IndexBuffer* indices, size_t offset,
                             size_t count) = 0;
  void SetBones(filament::RenderableManager::Instance instance,
                filament::RenderableManager::Bone const* transforms,
                size_t boneCount = 1, size_t offset = 0) {
    SetBonesInternal(instance, transforms, boneCount, offset);
  }
  void SetBones(filament::RenderableManager::Instance instance,
                mat4f const* transforms, size_t boneCount = 1,
                size_t offset = 0) {
    SetBonesInternal(instance, transforms, boneCount, offset);
  }
  virtual void SetAxisAlignedBoundingBox(
      filament::RenderableManager::Instance instance,
      const filament::Box& aabb) = 0;
  virtual void SetPriority(filament::RenderableManager::Instance instance,
                           uint8_t priority) = 0;
  virtual void SetChannel(filament::RenderableManager::Instance instance,
                          uint8_t channel) = 0;
  virtual uint8_t GetLayerMask(
      filament::RenderableManager::Instance instance) const = 0;
  virtual void SetLayerMask(filament::RenderableManager::Instance instance,
                            uint8_t select, uint8_t values) = 0;
  virtual void SetBlendOrderAt(filament::RenderableManager::Instance instance,
                               size_t primitiveIndex, uint16_t order) = 0;
  virtual void SetGlobalBlendOrderEnabledAt(
      filament::RenderableManager::Instance instance, size_t primitiveIndex,
      bool enabled) = 0;
  virtual bool IsShadowCaster(
      filament::RenderableManager::Instance instance) const = 0;
  virtual void SetCastShadows(filament::RenderableManager::Instance instance,
                              bool enable) = 0;
  virtual bool IsShadowReceiver(
      filament::RenderableManager::Instance instance) const = 0;
  virtual void SetReceiveShadows(filament::RenderableManager::Instance instance,
                                 bool enable) = 0;
  virtual bool GetFogEnabled(
      filament::RenderableManager::Instance instance) const = 0;
  virtual void SetFogEnabled(filament::RenderableManager::Instance instance,
                             bool enable) = 0;
  virtual size_t GetMorphTargetCount(
      filament::RenderableManager::Instance instance) const = 0;
  virtual void SetMorphWeights(filament::RenderableManager::Instance instance,
                               float const* weights, size_t count,
                               size_t offset = 0) = 0;
  virtual bool IsCullingEnabled(
      filament::RenderableManager::Instance instance) const = 0;

  /**
   * Adds renderable components to entities using a builder pattern.
   */
  struct Builder {
    virtual ~Builder() = default;
    virtual Builder& Geometry(size_t index,
                              filament::backend::PrimitiveType type,
                              filament::VertexBuffer* vertices,
                              filament::IndexBuffer* indices, size_t offset,
                              size_t minIndex, size_t maxIndex,
                              size_t count) noexcept = 0;
    virtual Builder& Geometry(size_t index,
                              filament::backend::PrimitiveType type,
                              filament::VertexBuffer* vertices,
                              filament::IndexBuffer* indices, size_t offset,
                              size_t count) noexcept = 0;
    virtual Builder& Geometry(size_t index,
                              filament::backend::PrimitiveType type,
                              filament::VertexBuffer* vertices,
                              filament::IndexBuffer* indices) noexcept = 0;

    virtual Builder& Material(
        size_t index,
        const filament::MaterialInstance* material_instance) noexcept = 0;

    virtual Builder& BoundingBox(
        const filament::Box& axisAlignedBoundingBox) noexcept = 0;
    virtual Builder& LayerMask(uint8_t select, uint8_t values) noexcept = 0;
    virtual Builder& Priority(uint8_t priority) noexcept = 0;
    virtual Builder& Channel(uint8_t channel) noexcept = 0;
    virtual Builder& Culling(bool enable) noexcept = 0;
    Builder& LightChannel(unsigned int channel, bool enable = true) noexcept {
      return LightChannelInternal(channel, enable);
    }
    virtual Builder& CastShadows(bool enable) noexcept = 0;
    virtual Builder& ReceiveShadows(bool enable) noexcept = 0;
    virtual Builder& ScreenSpaceContactShadows(bool enable) noexcept = 0;
    Builder& EnableSkinningBuffers(bool enabled = true) noexcept {
      return EnableSkinningBuffersInternal(enabled);
    }
    Builder& Fog(bool enabled = true) noexcept { return FogInternal(enabled); }
    virtual Builder& Skinning(filament::SkinningBuffer* skinningBuffer,
                              size_t count, size_t offset) noexcept = 0;
    virtual Builder& Skinning(size_t boneCount,
                              mat4f const* transforms) noexcept = 0;
    virtual Builder& Skinning(
        size_t boneCount,
        filament::RenderableManager::Bone const* bones) noexcept = 0;
    virtual Builder& Skinning(size_t boneCount) noexcept = 0;
    virtual Builder& BoneIndicesAndWeights(size_t primitiveIndex,
                                           float2 const* indicesAndWeights,
                                           size_t count,
                                           size_t bonesPerVertex) noexcept = 0;
    virtual Builder& BoneIndicesAndWeights(
        size_t primitiveIndex,
        utils::FixedCapacityVector<utils::FixedCapacityVector<float2>>
            indicesAndWeightsVector) noexcept = 0;
    virtual Builder& Morphing(
        filament::MorphTargetBuffer* morphTargetBuffer) noexcept = 0;
    virtual Builder& Morphing(uint8_t level, size_t primitiveIndex,
                              size_t offset, size_t count) noexcept = 0;
    virtual Builder& BlendOrder(size_t primitiveIndex,
                                uint16_t order) noexcept = 0;
    virtual Builder& GlobalBlendOrderEnabled(size_t primitiveIndex,
                                             bool enabled) noexcept = 0;
    virtual Builder& Instances(size_t instanceCount) noexcept = 0;
    virtual Builder& Instances(
        size_t instanceCount,
        filament::InstanceBuffer* instanceBuffer) noexcept = 0;
    virtual filament::RenderableManager::Builder::Result Build(
        filament::Engine& engine, utils::Entity entity) = 0;

   protected:
    virtual Builder& LightChannelInternal(unsigned int channel,
                                          bool enable) noexcept = 0;
    virtual Builder& EnableSkinningBuffersInternal(bool enabled) noexcept = 0;
    virtual Builder& FogInternal(bool enabled) noexcept = 0;
  };

  virtual std::unique_ptr<Builder> NewBuilder(size_t count) = 0;

 protected:
  virtual void SetBonesInternal(
      filament::RenderableManager::Instance instance,
      filament::RenderableManager::Bone const* transforms, size_t boneCount,
      size_t offset) = 0;
  virtual void SetBonesInternal(filament::RenderableManager::Instance instance,
                                mat4f const* transforms, size_t boneCount,
                                size_t offset) = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_BASE_RENDERABLE_MANAGER_H_
