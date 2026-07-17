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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_SPLIT_ENGINE_API_RENDERABLE_MANAGER_WRAPPER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_SPLIT_ENGINE_API_RENDERABLE_MANAGER_WRAPPER_H_

#include <cstddef>
#include <cstdint>
#include <memory>

#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Box.h"
#include "filament/filament/include/filament/MorphTargetBuffer.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "filament/libs/utils/include/utils/FixedCapacityVector.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/render/base_renderable_manager.h"
#include "core/view/base_view.h"

namespace imp {

// Wrapper for filament::RenderableManager that both passes through to the
// filament version and allows SplitEngine to spy on all these API calls.
class RenderableManagerWrapper : public BaseRenderableManager {
 public:
  using Instance = filament::RenderableManager::Instance;

  explicit RenderableManagerWrapper(BaseView& view);

  void SetSpy(BaseRenderableManager& spy) override;

  filament::RenderableManager::Instance GetInstance(
      utils::Entity e) const override;
  bool HasComponent(utils::Entity e) const override;
  void Destroy(utils::Entity e) override;
  size_t GetPrimitiveCount(
      filament::RenderableManager::Instance instance) const override;
  const filament::Box& GetAxisAlignedBoundingBox(
      filament::RenderableManager::Instance instance) const override;
  void SetMaterialInstanceAt(
      filament::RenderableManager::Instance instance, size_t primitiveIndex,
      const filament::MaterialInstance* material_instance) override;
  void ClearMaterialInstanceAt(filament::RenderableManager::Instance instance,
                               size_t primitiveIndex) override;
  void SetGeometryAt(filament::RenderableManager::Instance instance,
                     size_t primitiveIndex,
                     filament::backend::PrimitiveType type,
                     filament::VertexBuffer* vertices,
                     filament::IndexBuffer* indices, size_t offset,
                     size_t count) override;
  void SetAxisAlignedBoundingBox(filament::RenderableManager::Instance instance,
                                 const filament::Box& aabb) override;
  void SetPriority(filament::RenderableManager::Instance instance,
                   uint8_t priority) override;
  void SetChannel(filament::RenderableManager::Instance instance,
                  uint8_t channel) override;
  uint8_t GetLayerMask(
      filament::RenderableManager::Instance instance) const override;
  void SetLayerMask(filament::RenderableManager::Instance instance,
                    uint8_t select, uint8_t values) override;
  void SetBlendOrderAt(filament::RenderableManager::Instance instance,
                       size_t primitiveIndex, uint16_t order) override;
  void SetGlobalBlendOrderEnabledAt(
      filament::RenderableManager::Instance instance, size_t primitiveIndex,
      bool enabled) override;
  bool IsShadowCaster(
      filament::RenderableManager::Instance instance) const override;
  void SetCastShadows(filament::RenderableManager::Instance instance,
                      bool enable) override;
  bool IsShadowReceiver(
      filament::RenderableManager::Instance instance) const override;
  void SetReceiveShadows(filament::RenderableManager::Instance instance,
                         bool enable) override;
  bool GetFogEnabled(
      filament::RenderableManager::Instance instance) const override;
  void SetFogEnabled(filament::RenderableManager::Instance instance,
                     bool enable) override;
  size_t GetMorphTargetCount(
      filament::RenderableManager::Instance instance) const override;
  void SetMorphWeights(filament::RenderableManager::Instance instance,
                       float const* weights, size_t count,
                       size_t offset) override;
  bool IsCullingEnabled(
      filament::RenderableManager::Instance instance) const override;

  // Adds renderable components to entities using a builder pattern.
  class Builder : public BaseRenderableManager::Builder {
   public:
    explicit Builder(std::unique_ptr<BaseRenderableManager::Builder> spy,
                     size_t count) noexcept;

    /*! \cond PRIVATE */
    Builder(Builder const& rhs) = delete;
    Builder(Builder&& rhs) noexcept;
    ~Builder() noexcept;
    Builder& operator=(Builder& rhs) = delete;
    // Builder& operator=(Builder&& rhs) noexcept;
    Builder& operator=(Builder&& rhs);
    /*! \endcond */

    RenderableManagerWrapper::Builder& Geometry(
        size_t index, filament::backend::PrimitiveType type,
        filament::VertexBuffer* vertices, filament::IndexBuffer* indices,
        size_t offset, size_t minIndex, size_t maxIndex,
        size_t count) noexcept override;
    RenderableManagerWrapper::Builder& Geometry(
        size_t index, filament::backend::PrimitiveType type,
        filament::VertexBuffer* vertices, filament::IndexBuffer* indices,
        size_t offset, size_t count) noexcept override;
    RenderableManagerWrapper::Builder& Geometry(
        size_t index, filament::backend::PrimitiveType type,
        filament::VertexBuffer* vertices,
        filament::IndexBuffer* indices) noexcept override;

    RenderableManagerWrapper::Builder& Material(
        size_t index,
        const filament::MaterialInstance* material_instance) noexcept;

    RenderableManagerWrapper::Builder& BoundingBox(
        const filament::Box& axisAlignedBoundingBox) noexcept override;
    RenderableManagerWrapper::Builder& LayerMask(
        uint8_t select, uint8_t values) noexcept override;
    RenderableManagerWrapper::Builder& Priority(
        uint8_t priority) noexcept override;
    RenderableManagerWrapper::Builder& Channel(
        uint8_t channel) noexcept override;
    RenderableManagerWrapper::Builder& Culling(bool enable) noexcept override;
    RenderableManagerWrapper::Builder& CastShadows(
        bool enable) noexcept override;
    RenderableManagerWrapper::Builder& ReceiveShadows(
        bool enable) noexcept override;
    RenderableManagerWrapper::Builder& ScreenSpaceContactShadows(
        bool enable) noexcept override;
    RenderableManagerWrapper::Builder& Skinning(
        filament::SkinningBuffer* skinningBuffer, size_t count,
        size_t offset) noexcept override;
    RenderableManagerWrapper::Builder& Skinning(
        size_t boneCount, mat4f const* transforms) noexcept override;
    RenderableManagerWrapper::Builder& Skinning(
        size_t boneCount,
        filament::RenderableManager::Bone const* bones) noexcept override;
    RenderableManagerWrapper::Builder& Skinning(
        size_t boneCount) noexcept override;
    RenderableManagerWrapper::Builder& BoneIndicesAndWeights(
        size_t primitiveIndex, float2 const* indicesAndWeights, size_t count,
        size_t bonesPerVertex) noexcept override;
    RenderableManagerWrapper::Builder& BoneIndicesAndWeights(
        size_t primitiveIndex,
        utils::FixedCapacityVector<utils::FixedCapacityVector<float2>>
            indicesAndWeightsVector) noexcept override;
    RenderableManagerWrapper::Builder& Morphing(
        filament::MorphTargetBuffer* morphTargetBuffer) noexcept override;
    RenderableManagerWrapper::Builder& Morphing(uint8_t level,
                                                size_t primitiveIndex,
                                                size_t offset,
                                                size_t count) noexcept override;
    RenderableManagerWrapper::Builder& BlendOrder(
        size_t primitiveIndex, uint16_t order) noexcept override;
    RenderableManagerWrapper::Builder& GlobalBlendOrderEnabled(
        size_t primitiveIndex, bool enabled) noexcept override;
    RenderableManagerWrapper::Builder& Instances(
        size_t instanceCount) noexcept override;
    RenderableManagerWrapper::Builder& Instances(
        size_t instanceCount,
        filament::InstanceBuffer* instanceBuffer) noexcept override;
    filament::RenderableManager::Builder::Result Build(
        filament::Engine& engine, utils::Entity entity) override;

   protected:
    RenderableManagerWrapper::Builder& LightChannelInternal(
        unsigned int channel, bool enable) noexcept override;
    RenderableManagerWrapper::Builder& EnableSkinningBuffersInternal(
        bool enabled) noexcept override;
    RenderableManagerWrapper::Builder& FogInternal(
        bool enabled) noexcept override;

   private:
    std::unique_ptr<BaseRenderableManager::Builder> spy_;
    // We forward everything to a real builder.
    filament::RenderableManager::Builder real_builder_;
  };

  std::unique_ptr<BaseRenderableManager::Builder> NewBuilder(
      size_t count) override;

 protected:
  void SetBonesInternal(filament::RenderableManager::Instance instance,
                        filament::RenderableManager::Bone const* transforms,
                        size_t boneCount, size_t offset) override;
  void SetBonesInternal(filament::RenderableManager::Instance instance,
                        mat4f const* transforms, size_t boneCount,
                        size_t offset) override;

 private:
  filament::RenderableManager& GetRenderableManager() const;
  BaseView& view_;
  BaseRenderableManager* spy_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_SPLIT_ENGINE_API_RENDERABLE_MANAGER_WRAPPER_H_
