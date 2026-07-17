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

#include "core/view/framework/render/renderable_manager_wrapper.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>

#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Box.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "filament/libs/utils/include/utils/FixedCapacityVector.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/render/base_renderable_manager.h"
#include "core/view/base_view.h"

namespace imp {

using filament::Box;
using filament::IndexBuffer;
using filament::InstanceBuffer;
using filament::SkinningBuffer;
using filament::VertexBuffer;
using filament::backend::PrimitiveType;

RenderableManagerWrapper::RenderableManagerWrapper(BaseView& view)
    : view_(view), spy_(nullptr) {}

void RenderableManagerWrapper::SetSpy(BaseRenderableManager& spy) {
  spy_ = &spy;
}

filament::RenderableManager& RenderableManagerWrapper::GetRenderableManager()
    const {
  return BaseView::GetSharedEngine()->getRenderableManager();
}

filament::RenderableManager::Instance RenderableManagerWrapper::GetInstance(
    utils::Entity e) const {
  return GetRenderableManager().getInstance(e);
}

bool RenderableManagerWrapper::HasComponent(utils::Entity e) const {
  return GetRenderableManager().hasComponent(e);
}

void RenderableManagerWrapper::Destroy(utils::Entity e) {
  if (spy_) spy_->Destroy(e);
  GetRenderableManager().destroy(e);
}
size_t RenderableManagerWrapper::GetPrimitiveCount(
    filament::RenderableManager::Instance instance) const {
  return GetRenderableManager().getPrimitiveCount(instance);
}

const filament::Box& RenderableManagerWrapper::GetAxisAlignedBoundingBox(
    filament::RenderableManager::Instance instance) const {
  return GetRenderableManager().getAxisAlignedBoundingBox(instance);
}

void RenderableManagerWrapper::SetMaterialInstanceAt(
    filament::RenderableManager::Instance instance, size_t primitiveIndex,
    const filament::MaterialInstance* material_instance) {
  if (spy_)
    spy_->SetMaterialInstanceAt(instance, primitiveIndex, material_instance);
  GetRenderableManager().setMaterialInstanceAt(instance, primitiveIndex,
                                               material_instance);
}
void RenderableManagerWrapper::ClearMaterialInstanceAt(
    filament::RenderableManager::Instance instance, size_t primitiveIndex) {
  if (spy_) spy_->ClearMaterialInstanceAt(instance, primitiveIndex);
  GetRenderableManager().clearMaterialInstanceAt(instance, primitiveIndex);
}
void RenderableManagerWrapper::SetGeometryAt(
    filament::RenderableManager::Instance instance, size_t primitiveIndex,
    filament::backend::PrimitiveType type, VertexBuffer* vertices,
    IndexBuffer* indices, size_t offset, size_t count) {
  if (spy_)
    spy_->SetGeometryAt(instance, primitiveIndex, type, vertices, indices,
                        offset, count);
  GetRenderableManager().setGeometryAt(instance, primitiveIndex, type, vertices,
                                       indices, offset, count);
}

void RenderableManagerWrapper::SetBonesInternal(
    filament::RenderableManager::Instance instance,
    filament::RenderableManager::Bone const* transforms, size_t boneCount,
    size_t offset) {
  if (spy_) {
    spy_->SetBones(instance, transforms, boneCount, offset);
    // TODO: calling setBones() every frame results in a crash if
    // we only call isolated pre/post render instead of RenderNextFrame.
    return;
  }
  GetRenderableManager().setBones(instance, transforms, boneCount, offset);
}

void RenderableManagerWrapper::SetBonesInternal(
    filament::RenderableManager::Instance instance, mat4f const* transforms,
    size_t boneCount, size_t offset) {
  if (spy_) {
    // TODO: calling setBones() every frame results in a crash if
    // we only call isolated pre/post render instead of RenderNextFrame.
    spy_->SetBones(instance, transforms, boneCount, offset);
    return;
  }
  GetRenderableManager().setBones(instance, transforms, boneCount, offset);
}

void RenderableManagerWrapper::SetAxisAlignedBoundingBox(
    filament::RenderableManager::Instance instance, const Box& aabb) {
  if (spy_) spy_->SetAxisAlignedBoundingBox(instance, aabb);
  GetRenderableManager().setAxisAlignedBoundingBox(instance, aabb);
}
void RenderableManagerWrapper::SetPriority(
    filament::RenderableManager::Instance instance, uint8_t priority) {
  if (spy_) spy_->SetPriority(instance, priority);
  GetRenderableManager().setPriority(instance, priority);
}
void RenderableManagerWrapper::SetChannel(
    filament::RenderableManager::Instance instance, uint8_t channel) {
  if (spy_) spy_->SetChannel(instance, channel);
  GetRenderableManager().setChannel(instance, channel);
}
uint8_t RenderableManagerWrapper::GetLayerMask(
    filament::RenderableManager::Instance instance) const {
  return GetRenderableManager().getLayerMask(instance);
}
void RenderableManagerWrapper::SetLayerMask(
    filament::RenderableManager::Instance instance, uint8_t select,
    uint8_t values) {
  if (spy_) spy_->SetLayerMask(instance, select, values);
  GetRenderableManager().setLayerMask(instance, select, values);
}
void RenderableManagerWrapper::SetBlendOrderAt(
    filament::RenderableManager::Instance instance, size_t primitiveIndex,
    uint16_t order) {
  if (spy_) spy_->SetBlendOrderAt(instance, primitiveIndex, order);
  GetRenderableManager().setBlendOrderAt(instance, primitiveIndex, order);
}
void RenderableManagerWrapper::SetGlobalBlendOrderEnabledAt(
    filament::RenderableManager::Instance instance, size_t primitiveIndex,
    bool enabled) {
  if (spy_)
    spy_->SetGlobalBlendOrderEnabledAt(instance, primitiveIndex, enabled);
  GetRenderableManager().setGlobalBlendOrderEnabledAt(instance, primitiveIndex,
                                                      enabled);
}
bool RenderableManagerWrapper::IsShadowCaster(
    filament::RenderableManager::Instance instance) const {
  return GetRenderableManager().isShadowCaster(instance);
}
void RenderableManagerWrapper::SetCastShadows(
    filament::RenderableManager::Instance instance, bool enable) {
  if (spy_) spy_->SetCastShadows(instance, enable);
  GetRenderableManager().setCastShadows(instance, enable);
}
bool RenderableManagerWrapper::IsShadowReceiver(
    filament::RenderableManager::Instance instance) const {
  return GetRenderableManager().isShadowReceiver(instance);
}
void RenderableManagerWrapper::SetReceiveShadows(
    filament::RenderableManager::Instance instance, bool enable) {
  if (spy_) spy_->SetReceiveShadows(instance, enable);
  GetRenderableManager().setReceiveShadows(instance, enable);
}
bool RenderableManagerWrapper::GetFogEnabled(
    filament::RenderableManager::Instance instance) const {
  return GetRenderableManager().getFogEnabled(instance);
}
void RenderableManagerWrapper::SetFogEnabled(
    filament::RenderableManager::Instance instance, bool enable) {
  if (spy_) spy_->SetFogEnabled(instance, enable);
  GetRenderableManager().setFogEnabled(instance, enable);
}

size_t RenderableManagerWrapper::GetMorphTargetCount(
    filament::RenderableManager::Instance instance) const {
  return GetRenderableManager().getMorphTargetCount(instance);
}

void RenderableManagerWrapper::SetMorphWeights(
    filament::RenderableManager::Instance instance, float const* weights,
    size_t count, size_t offset) {
  if (spy_) {
    spy_->SetMorphWeights(instance, weights, count, offset);
    // Calling setMorphWeights() every frame results in a crash if we only call
    // isolated pre/post render instead of RenderNextFrame. Also, it doesn't
    // make sense to ask filament to do work that we don't actually need it to
    // do in the split engine case, so return early.
    return;
  }
  GetRenderableManager().setMorphWeights(instance, weights, count, offset);
}

bool RenderableManagerWrapper::IsCullingEnabled(
    filament::RenderableManager::Instance instance) const {
  return GetRenderableManager().isCullingEnabled(instance);
}

std::unique_ptr<BaseRenderableManager::Builder>
RenderableManagerWrapper::NewBuilder(size_t count) {
  return std::make_unique<RenderableManagerWrapper::Builder>(
      spy_ ? spy_->NewBuilder(count) : nullptr, count);
}

RenderableManagerWrapper::Builder::Builder(
    std::unique_ptr<BaseRenderableManager::Builder> spy, size_t count) noexcept
    : spy_(std::move(spy)), real_builder_(count) {}
RenderableManagerWrapper::Builder::~Builder() noexcept = default;
RenderableManagerWrapper::Builder::Builder(
    RenderableManagerWrapper::Builder&& rhs) noexcept
    : spy_(std::move(rhs.spy_)), real_builder_(std::move(rhs.real_builder_)) {}

RenderableManagerWrapper::Builder& RenderableManagerWrapper::Builder::operator=(
    RenderableManagerWrapper::Builder&& rhs) {
  spy_ = std::move(rhs.spy_);
  real_builder_ = std::move(rhs.real_builder_);
  return *this;
}

RenderableManagerWrapper::Builder& RenderableManagerWrapper::Builder::Geometry(
    size_t index, PrimitiveType type, VertexBuffer* vertices,
    IndexBuffer* indices, size_t offset, size_t minIndex, size_t maxIndex,
    size_t count) noexcept {
  if (spy_)
    spy_->Geometry(index, type, vertices, indices, offset, minIndex, maxIndex,
                   count);
  real_builder_.geometry(index, type, vertices, indices, offset, minIndex,
                         maxIndex, count);
  return *this;
}

RenderableManagerWrapper::Builder& RenderableManagerWrapper::Builder::Geometry(
    size_t index, PrimitiveType type, VertexBuffer* vertices,
    IndexBuffer* indices) noexcept {
  return Geometry(index, type, vertices, indices, 0, 0,
                  vertices->getVertexCount() - 1, indices->getIndexCount());
}

RenderableManagerWrapper::Builder& RenderableManagerWrapper::Builder::Geometry(
    size_t index, PrimitiveType type, VertexBuffer* vertices,
    IndexBuffer* indices, size_t offset, size_t count) noexcept {
  return Geometry(index, type, vertices, indices, offset, 0,
                  vertices->getVertexCount() - 1, count);
}

RenderableManagerWrapper::Builder& RenderableManagerWrapper::Builder::Material(
    size_t index,
    const filament::MaterialInstance* material_instance) noexcept {
  if (spy_) spy_->Material(index, material_instance);
  real_builder_.material(index, material_instance);
  return *this;
}

RenderableManagerWrapper::Builder&
RenderableManagerWrapper::Builder::BoundingBox(
    const Box& axisAlignedBoundingBox) noexcept {
  if (spy_) spy_->BoundingBox(axisAlignedBoundingBox);
  real_builder_.boundingBox(axisAlignedBoundingBox);
  return *this;
}

RenderableManagerWrapper::Builder& RenderableManagerWrapper::Builder::LayerMask(
    uint8_t select, uint8_t values) noexcept {
  if (spy_) spy_->LayerMask(select, values);
  real_builder_.layerMask(select, values);
  return *this;
}

RenderableManagerWrapper::Builder& RenderableManagerWrapper::Builder::Priority(
    uint8_t priority) noexcept {
  if (spy_) spy_->Priority(priority);
  real_builder_.priority(priority);
  return *this;
}

RenderableManagerWrapper::Builder& RenderableManagerWrapper::Builder::Channel(
    uint8_t channel) noexcept {
  if (spy_) spy_->Channel(channel);
  real_builder_.channel(channel);
  return *this;
}

RenderableManagerWrapper::Builder& RenderableManagerWrapper::Builder::Culling(
    bool enable) noexcept {
  if (spy_) spy_->Culling(enable);
  real_builder_.culling(enable);
  return *this;
}

RenderableManagerWrapper::Builder&
RenderableManagerWrapper::Builder::LightChannelInternal(unsigned int channel,
                                                        bool enable) noexcept {
  if (spy_) spy_->LightChannel(channel, enable);
  real_builder_.lightChannel(channel, enable);
  return *this;
}

RenderableManagerWrapper::Builder&
RenderableManagerWrapper::Builder::CastShadows(bool enable) noexcept {
  if (spy_) spy_->CastShadows(enable);
  real_builder_.castShadows(enable);
  return *this;
}

RenderableManagerWrapper::Builder&
RenderableManagerWrapper::Builder::ReceiveShadows(bool enable) noexcept {
  if (spy_) spy_->ReceiveShadows(enable);
  real_builder_.receiveShadows(enable);
  return *this;
}

RenderableManagerWrapper::Builder&
RenderableManagerWrapper::Builder::ScreenSpaceContactShadows(
    bool enable) noexcept {
  if (spy_) spy_->ScreenSpaceContactShadows(enable);
  real_builder_.screenSpaceContactShadows(enable);
  return *this;
}

RenderableManagerWrapper::Builder& RenderableManagerWrapper::Builder::Skinning(
    size_t boneCount) noexcept {
  if (spy_) spy_->Skinning(boneCount);
  real_builder_.skinning(boneCount);
  return *this;
}

RenderableManagerWrapper::Builder& RenderableManagerWrapper::Builder::Skinning(
    size_t boneCount, filament::RenderableManager::Bone const* bones) noexcept {
  if (spy_) spy_->Skinning(boneCount, bones);
  real_builder_.skinning(boneCount, bones);
  return *this;
}

RenderableManagerWrapper::Builder& RenderableManagerWrapper::Builder::Skinning(
    size_t boneCount, mat4f const* transforms) noexcept {
  if (spy_) spy_->Skinning(boneCount, transforms);
  real_builder_.skinning(boneCount, transforms);
  return *this;
}

RenderableManagerWrapper::Builder& RenderableManagerWrapper::Builder::Skinning(
    SkinningBuffer* skinningBuffer, size_t count, size_t offset) noexcept {
  if (spy_) spy_->Skinning(skinningBuffer, count, offset);
  real_builder_.skinning(skinningBuffer, count, offset);
  return *this;
}

RenderableManagerWrapper::Builder&
RenderableManagerWrapper::Builder::EnableSkinningBuffersInternal(
    bool enabled) noexcept {
  if (spy_) spy_->EnableSkinningBuffers(enabled);
  real_builder_.enableSkinningBuffers(enabled);
  return *this;
}

RenderableManagerWrapper::Builder&
RenderableManagerWrapper::Builder::BoneIndicesAndWeights(
    size_t primitiveIndex, float2 const* indicesAndWeights, size_t count,
    size_t bonesPerVertex) noexcept {
  if (spy_)
    spy_->BoneIndicesAndWeights(primitiveIndex, indicesAndWeights, count,
                                bonesPerVertex);
  real_builder_.boneIndicesAndWeights(primitiveIndex, indicesAndWeights, count,
                                      bonesPerVertex);
  return *this;
}

RenderableManagerWrapper::Builder&
RenderableManagerWrapper::Builder::BoneIndicesAndWeights(
    size_t primitiveIndex,
    utils::FixedCapacityVector<utils::FixedCapacityVector<float2>>
        indicesAndWeightsVector) noexcept {
  if (spy_)
    spy_->BoneIndicesAndWeights(primitiveIndex, indicesAndWeightsVector);
  real_builder_.boneIndicesAndWeights(primitiveIndex, indicesAndWeightsVector);
  return *this;
}

RenderableManagerWrapper::Builder&
RenderableManagerWrapper::Builder::FogInternal(bool enabled) noexcept {
  if (spy_) spy_->Fog(enabled);
  real_builder_.fog(enabled);
  return *this;
}

RenderableManagerWrapper::Builder& RenderableManagerWrapper::Builder::Morphing(
    filament::MorphTargetBuffer* morphTargetBuffer) noexcept {
  if (spy_) {
    spy_->Morphing(morphTargetBuffer);
  }

  real_builder_.morphing(morphTargetBuffer);
  return *this;
}

RenderableManagerWrapper::Builder& RenderableManagerWrapper::Builder::Morphing(
    uint8_t level, size_t primitiveIndex, size_t offset,
    size_t count) noexcept {
  if (spy_) {
    spy_->Morphing(level, primitiveIndex, offset, count);
  }

  // TODO the count param is no longer needed by the Filament
  // builder, so it should also be removed from BaseRenderableManager::Builder.
  real_builder_.morphing(level, primitiveIndex, offset);
  return *this;
}

RenderableManagerWrapper::Builder&
RenderableManagerWrapper::Builder::BlendOrder(size_t index,
                                              uint16_t blendOrder) noexcept {
  if (spy_) spy_->BlendOrder(index, blendOrder);
  real_builder_.blendOrder(index, blendOrder);
  return *this;
}

RenderableManagerWrapper::Builder&
RenderableManagerWrapper::Builder::GlobalBlendOrderEnabled(
    size_t index, bool enabled) noexcept {
  if (spy_) spy_->GlobalBlendOrderEnabled(index, enabled);
  real_builder_.globalBlendOrderEnabled(index, enabled);
  return *this;
}

RenderableManagerWrapper::Builder& RenderableManagerWrapper::Builder::Instances(
    size_t instanceCount) noexcept {
  if (spy_) spy_->Instances(instanceCount);
  real_builder_.instances(instanceCount);
  return *this;
}

RenderableManagerWrapper::Builder& RenderableManagerWrapper::Builder::Instances(
    size_t instanceCount, InstanceBuffer* instanceBuffer) noexcept {
  if (spy_) spy_->Instances(instanceCount, instanceBuffer);
  real_builder_.instances(instanceCount, instanceBuffer);
  return *this;
}

filament::RenderableManager::Builder::Result
RenderableManagerWrapper::Builder::Build(filament::Engine& engine,
                                         utils::Entity entity) {
  if (spy_) spy_->Build(engine, entity);
  return real_builder_.build(engine, entity);
}

}  // namespace imp
