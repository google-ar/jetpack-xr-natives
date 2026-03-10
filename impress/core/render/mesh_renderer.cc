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

#include "core/render/mesh_renderer.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/InstanceBuffer.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/common/small_source_location.h"
#include "core/geometry/shapes/box.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/model/mesh/mesh.h"
#include "core/model/model_data.h"
#include "core/ncsb/node.h"
#include "core/render/base_renderable_manager.h"
#include "core/render/render_order_constants.h"
#include "core/view/base_view.h"

namespace imp {
namespace {

using model::ModelData;
using RenderFlags = ModelData::RenderFlags;

}  // namespace

void MeshRenderer::Cleanup() {
  GetRenderableManager().Destroy(GetEntity());
  if (instance_buffer_) {
    BaseView::GetSharedEngine()->destroy(instance_buffer_);
    instance_buffer_ = nullptr;
  }
}

void MeshRenderer::Setup(size_t primitive_count) {
  Setup({
      .primitive_count = primitive_count,
      .culling_mode = FrustumCullingMode::kEnabled,
      .num_instances = 1,
      .instancing_mode = InstancingMode::kGpuIndices,
      .num_bones = 0,
  });
}

void MeshRenderer::Setup(FrustumCullingMode culling_mode,
                         size_t primitive_count) {
  Setup({
      .primitive_count = primitive_count,
      .culling_mode = culling_mode,
      .num_instances = 1,
      .instancing_mode = InstancingMode::kGpuIndices,
      .num_bones = 0,
  });
}

void MeshRenderer::Setup(const SetupOptions& options) {
  BuildRenderables(options);

  // Do not display until the component is "awake"
  GetRenderableManager().SetLayerMask(GetInstance(), 0xff, 0);
}

size_t MeshRenderer::GetInstanceCount() const { return num_instances_; }

absl::Status MeshRenderer::UpdateInstanceTransformsInRange(
    absl::Span<const imp::mat4f> new_instance_transforms,
    size_t first_instance_index) {
  if (GetInstanceCount() <= 1) {
    return absl::UnavailableError("Instancing not enabled on this renderable");
  }
  if (instance_transforms_.empty()) {
    return absl::UnavailableError(
        "Instance transforms are not enabled, explicit transforms cannot be "
        "set. Use getInstanceIndex() in your material instead");
  }
  if (first_instance_index + new_instance_transforms.size() >
      GetInstanceCount()) {
    return absl::OutOfRangeError(
        absl::StrCat("Renderable has ", GetInstanceCount(),
                     " instances but first_instance_index + "
                     "new_instance_transforms.size() is ",
                     first_instance_index + new_instance_transforms.size()));
  }
  std::copy(new_instance_transforms.begin(), new_instance_transforms.end(),
            instance_transforms_.begin() + first_instance_index);

  instance_buffer_->setLocalTransforms(new_instance_transforms.data(),
                                       new_instance_transforms.size(),
                                       first_instance_index);

  UpdateRenderableAabb();
  return absl::OkStatus();
}

absl::Status MeshRenderer::UpdateBoneTransformsInRange(
    absl::Span<const imp::mat4f> new_bones, uint8_t first_bone_index) {
  if (GetBoneCount() == 0) {
    return absl::UnavailableError("Boneless renderable!");
  }
  if (first_bone_index + new_bones.size() > GetBoneCount()) {
    return absl::OutOfRangeError(absl::StrCat(
        "Renderable has too few bones! Renderable has bones with size ",
        GetBoneCount(), " but first_bone_index + new_bones.size() is ",
        first_bone_index + new_bones.size()));
  }
  std::copy(new_bones.begin(), new_bones.end(),
            bones_.begin() + first_bone_index);

  GetRenderableManager().SetBones(GetInstance(), bones_.data(), GetBoneCount());

  UpdateRenderableAabb();
  return absl::OkStatus();
}

uint8_t MeshRenderer::GetBoneCount() const { return bones_.size(); }

bool MeshRenderer::IsOwnedOrBorrowedPtrType(
    const HeldPtrType& held_ptr_type) const {
  return held_ptr_type == HeldPtrType::kBorrowedPointer ||
         held_ptr_type == HeldPtrType::kOwnedPointer;
}

void MeshRenderer::BuildRenderables(const SetupOptions& options) {
  size_t primitive_count = options.primitive_count;
  FrustumCullingMode culling_mode = options.culling_mode;
  size_t num_instances = options.num_instances;
  InstancingMode instancing_mode = options.instancing_mode;
  uint8_t num_bones = options.num_bones;

  std::unique_ptr<BaseRenderableManager::Builder> builder =
      GetRenderableManager().NewBuilder(primitive_count);

  // This class owns this enum, so use an exhaustive switch ((broken link)).
  switch (culling_mode) {
    case FrustumCullingMode::kEnabled:
      builder->Culling(true);
      break;
    case FrustumCullingMode::kDisabled:
      builder->Culling(false);
      break;
  }

  culling_mode_ = culling_mode;

  if (num_instances > 1) {
    num_instances_ = num_instances;
    switch (instancing_mode) {
      case InstancingMode::kCpuTransforms: {
        filament::InstanceBuffer::Builder buffer_builder(num_instances);
        instance_transforms_.resize(num_instances, kIdentityMat4f);
        buffer_builder.localTransforms(instance_transforms_.data());
        instance_buffer_ = buffer_builder.build(*BaseView::GetSharedEngine());
        builder->Instances(num_instances, instance_buffer_);
        break;
      }
      case InstancingMode::kGpuIndices:
        builder->Instances(num_instances);
        break;
    }
  }

  if (num_bones > 0) {
    bones_.resize(num_bones);

    std::fill(bones_.begin(), bones_.end(), kIdentityMat4f);

    builder->EnableSkinningBuffers(false);
    // The Filament `Skinning` function takes a size_t for the number of bones,
    // but the docstring states
    // - "@param boneCount ... the number of bone transforms (up to 255)"
    // so making the public API a uint8_t enforces this limit at compile time.
    builder->Skinning(static_cast<size_t>(num_bones), bones_.data());
  }

  builder->Priority(priority_);
  builder->Build(*BaseView::GetSharedEngine(), GetNode()->GetEntity());

  primitives_.resize(primitive_count);
  layer_mask_ = GetRenderableManager().GetLayerMask(GetInstance());
}

void MeshRenderer::OnActiveStatusChanged(bool is_active) {
  GetRenderableManager().SetLayerMask(GetInstance(), 0xff,
                                      is_active ? layer_mask_ : 0);
}

size_t MeshRenderer::GetPrimitiveCount() const {
  return GetRenderableManager().GetPrimitiveCount(GetInstance());
}

void MeshRenderer::SetMaterial(Material* material, size_t primitive_index) {
  if (!IsWithinCount(primitive_index)) {
    return;
  }
  GetRenderableManager().SetMaterialInstanceAt(
      GetInstance(), primitive_index, material->GetFilamentMaterialInstance());
  primitives_[primitive_index].owned_or_borrowed_material =
      OwnedOrBorrowedPtr<Material>();
  primitives_[primitive_index].raw_material = material;
  primitives_[primitive_index].held_material_type = HeldPtrType::kRawPointer;
}

void MeshRenderer::SetMaterial(OwnedMaterialPtr material,
                               size_t primitive_index) {
  if (!IsWithinCount(primitive_index)) {
    return;
  }
  GetRenderableManager().SetMaterialInstanceAt(
      GetInstance(), primitive_index, material->GetFilamentMaterialInstance());
  primitives_[primitive_index].owned_or_borrowed_material =
      OwnedOrBorrowedPtr<Material>(std::move(material));
  primitives_[primitive_index].raw_material = nullptr;
  primitives_[primitive_index].held_material_type = HeldPtrType::kOwnedPointer;
}

void MeshRenderer::SetMaterial(BorrowedMaterialPtr material,
                               size_t primitive_index) {
  if (!IsWithinCount(primitive_index)) {
    return;
  }
  GetRenderableManager().SetMaterialInstanceAt(
      GetInstance(), primitive_index, material->GetFilamentMaterialInstance());
  primitives_[primitive_index].owned_or_borrowed_material =
      OwnedOrBorrowedPtr<Material>(material);
  primitives_[primitive_index].raw_material = nullptr;
  primitives_[primitive_index].held_material_type =
      HeldPtrType::kBorrowedPointer;
}

Material* MeshRenderer::GetMaterial(size_t primitive_index) const {
  if (IsWithinCount(primitive_index) && primitive_index < primitives_.size()) {
    if (IsOwnedOrBorrowedPtrType(
            primitives_[primitive_index].held_material_type)) {
      return primitives_[primitive_index]
          .owned_or_borrowed_material.operator->();
    } else {
      return primitives_[primitive_index].raw_material;
    }
  }
  return nullptr;
}

BorrowedMaterialPtr MeshRenderer::BorrowMaterial(
    size_t primitive_index, SmallSourceLocation loc) const {
  if (IsWithinCount(primitive_index) && primitive_index < primitives_.size()) {
    const PrimitiveData& primitive = primitives_[primitive_index];
    if (IsOwnedOrBorrowedPtrType(primitive.held_material_type)) {
      return primitive.owned_or_borrowed_material.Borrow(loc);
    } else if (primitive.held_material_type != HeldPtrType::kNone) {
      IMP_LOG(imp::WARNING) << "Cannot BorrowMaterial when the material was set from a "
                      "MaterialPtr or Material*";
    }
  }

  return {};
}

void MeshRenderer::SetMesh(Mesh* mesh, size_t primitive_index) {
  if (!IsWithinCount(primitive_index)) {
    return;
  }

  SetRenderableGeometry(*mesh, primitive_index);
  primitives_[primitive_index].mesh_index_count = mesh->GetIndexRenderCount();
  primitives_[primitive_index].mesh_index_offset = mesh->GetIndexRenderOffset();
  primitives_[primitive_index].held_mesh_type = HeldPtrType::kRawPointer;
  primitives_[primitive_index].owned_or_borrowed_mesh =
      OwnedOrBorrowedPtr<Mesh>();
  primitives_[primitive_index].raw_mesh = mesh;
  UpdateRenderableAabb();
}

void MeshRenderer::SetMesh(BorrowedMeshPtr mesh, size_t primitive_index) {
  if (!IsWithinCount(primitive_index)) {
    return;
  }

  SetRenderableGeometry(*mesh, primitive_index);
  primitives_[primitive_index].mesh_index_count = mesh->GetIndexRenderCount();
  primitives_[primitive_index].mesh_index_offset = mesh->GetIndexRenderOffset();
  primitives_[primitive_index].held_mesh_type = HeldPtrType::kBorrowedPointer;
  primitives_[primitive_index].owned_or_borrowed_mesh =
      OwnedOrBorrowedPtr<Mesh>(mesh);
  primitives_[primitive_index].held_mesh_type = HeldPtrType::kBorrowedPointer;
  primitives_[primitive_index].raw_mesh = nullptr;
  UpdateRenderableAabb();
}

void MeshRenderer::SetMesh(OwnedMeshPtr mesh, size_t primitive_index) {
  if (!IsWithinCount(primitive_index)) {
    return;
  }

  SetRenderableGeometry(*mesh, primitive_index);
  primitives_[primitive_index].mesh_index_count = mesh->GetIndexRenderCount();
  primitives_[primitive_index].mesh_index_offset = mesh->GetIndexRenderOffset();
  primitives_[primitive_index].held_mesh_type = HeldPtrType::kOwnedPointer;
  primitives_[primitive_index].owned_or_borrowed_mesh =
      OwnedOrBorrowedPtr<Mesh>(std::move(mesh));
  primitives_[primitive_index].held_mesh_type = HeldPtrType::kOwnedPointer;
  primitives_[primitive_index].raw_mesh = nullptr;
  UpdateRenderableAabb();
}

void MeshRenderer::SetRenderableGeometry(Mesh& mesh, size_t primitive_index) {
  filament::RenderableManager::Instance instance = GetInstance();
  GetRenderableManager().SetGeometryAt(
      instance, primitive_index, mesh.GetPrimitiveType(),
      mesh.GetVertexBuffer(), mesh.GetIndexBuffer(),
      mesh.GetIndexRenderOffset(), mesh.GetIndexRenderCount());
}

Mesh* MeshRenderer::GetMesh(size_t primitive_index) const {
  if (IsWithinCount(primitive_index) && primitive_index < primitives_.size()) {
    if (IsOwnedOrBorrowedPtrType(primitives_[primitive_index].held_mesh_type)) {
      return primitives_[primitive_index].owned_or_borrowed_mesh.operator->();
    } else {
      return primitives_[primitive_index].raw_mesh;
    }
  }
  return nullptr;
}

BorrowedMeshPtr MeshRenderer::BorrowMesh(size_t primitive_index,
                                         SmallSourceLocation loc) const {
  if (IsWithinCount(primitive_index) && primitive_index < primitives_.size()) {
    const PrimitiveData& primitive = primitives_[primitive_index];
    if (IsOwnedOrBorrowedPtrType(primitive.held_mesh_type)) {
      return primitive.owned_or_borrowed_mesh.Borrow(loc);
    } else if (primitive.held_mesh_type != HeldPtrType::kNone) {
      IMP_LOG(imp::WARNING) << "Cannot BorrowMesh when the mesh was set from a "
                      "MeshPtr or Mesh*";
    }
  }

  return {};
}

void MeshRenderer::SetShadowCastingMode(ShadowMode shadow_mode) {
  // This class owns this enum, so use an exhaustive switch ((broken link)).
  switch (shadow_mode) {
    case ShadowMode::kHardShadows:
      GetRenderableManager().SetCastShadows(GetInstance(), true);
      break;
    case ShadowMode::kNone:
      GetRenderableManager().SetCastShadows(GetInstance(), false);
      break;
  }
}

MeshRenderer::ShadowMode MeshRenderer::GetShadowCastingMode() const {
  bool is_shadow_caster = GetRenderableManager().IsShadowCaster(GetInstance());
  return is_shadow_caster ? ShadowMode::kHardShadows : ShadowMode::kNone;
}

void MeshRenderer::SetShadowReceivingMode(ShadowMode shadow_mode) {
  // This class owns this enum, so use an exhaustive switch ((broken link)).
  switch (shadow_mode) {
    case ShadowMode::kHardShadows:
      GetRenderableManager().SetReceiveShadows(GetInstance(), true);
      break;
    case ShadowMode::kNone:
      GetRenderableManager().SetReceiveShadows(GetInstance(), false);
      break;
  }
}

MeshRenderer::ShadowMode MeshRenderer::GetShadowReceivingMode() const {
  bool is_shadow_receiver =
      GetRenderableManager().IsShadowReceiver(GetInstance());
  return is_shadow_receiver ? ShadowMode::kHardShadows : ShadowMode::kNone;
}

void MeshRenderer::SetPriority(uint8_t priority) {
  // Filament also clamps the priority between 0 and 7, but filament has no
  // getter for the priority so we need to track it ourselves.
  priority_ = std::clamp(priority, kMinPriority, kMaxPriority);
  GetRenderableManager().SetPriority(GetInstance(), priority_);
}

uint8_t MeshRenderer::GetPriority() const { return priority_; }

void MeshRenderer::SetChannel(uint8_t channel) {
  // Filament also clamps the channel between 0 and 3, but filament has no
  // getter for the channel so we need to track it ourselves.
  channel_ = std::clamp(channel, kMinChannel, kMaxChannel);
  GetRenderableManager().SetChannel(GetInstance(), channel_);
}

void MeshRenderer::SetFogEnabled(bool enable) {
  GetRenderableManager().SetFogEnabled(GetInstance(), enable);
}

uint8_t MeshRenderer::GetChannel() const { return channel_; }

void MeshRenderer::SetBlendOrder(uint16_t blend_order, BlendOrderMode mode,
                                 size_t primitive) {
  filament::RenderableManager::Instance instance = GetInstance();
  // Filament also clamps the order to 15 bits.
  blend_order = std::clamp(blend_order, kMinBlendOrder, kMaxBlendOrder);
  GetRenderableManager().SetBlendOrderAt(instance, primitive, blend_order);
  GetRenderableManager().SetGlobalBlendOrderEnabledAt(
      instance, primitive, mode == BlendOrderMode::kLocal ? false : true);
}

// TODO: Use filament::RenderableManager::getCulling() here when
// feature added.
MeshRenderer::FrustumCullingMode MeshRenderer::GetFrustumCullingMode() const {
  return culling_mode_;
}

BaseRenderableManager& MeshRenderer::GetRenderableManager() const {
  return GetView().GetRenderableManager();
}

filament::RenderableManager::Instance MeshRenderer::GetInstance() const {
  return GetRenderableManager().GetInstance(GetEntity());
}

bool MeshRenderer::IsWithinCount(size_t primitive_index) const {
  size_t primitive_count = GetPrimitiveCount();
  if (primitive_index >= primitive_count) {
    IMP_LOG(imp::FATAL) << "Primitive index " << primitive_index
               << " out of bounds: " << primitive_count;
    return false;
  }
  return true;
}

void MeshRenderer::UpdateRenderableAabb() {
  size_t primitive_count = GetPrimitiveCount();
  Box aabb;
  bool found_primitive = false;

  for (size_t i = 0; i < primitive_count; i++) {
    if (IsOwnedOrBorrowedPtrType(primitives_[i].held_mesh_type)) {
      if (primitives_[i].owned_or_borrowed_mesh.operator->()) {
        if (!found_primitive) {
          aabb = primitives_[i].owned_or_borrowed_mesh->GetAabb();
          found_primitive = true;
        } else {
          aabb =
              aabb.unionSelf(primitives_[i].owned_or_borrowed_mesh->GetAabb());
        }
      }
    } else {
      if (primitives_[i].raw_mesh) {
        if (!found_primitive) {
          aabb = primitives_[i].raw_mesh->GetAabb();
          found_primitive = true;
        } else {
          aabb = aabb.unionSelf(primitives_[i].raw_mesh->GetAabb());
        }
      }
    }
  }

  if (GetBoneCount() > 0) {
    Box combined_aabb = aabb;
    for (const imp::mat4f& bone_transformation : bones_) {
      combined_aabb = combined_aabb.unionSelf(Box::transform(
          bone_transformation.upperLeft(), bone_transformation[3].xyz, aabb));
    }
    aabb = combined_aabb;
  }

  if (!instance_transforms_.empty()) {
    Box bones_aabb = aabb;
    const imp::mat4f& first_transform = instance_transforms_.front();
    Box combined_aabb = Box::transform(first_transform.upperLeft(),
                                       first_transform[3].xyz, bones_aabb);
    for (int i = 1; i < instance_transforms_.size(); ++i) {
      const imp::mat4f& instance_transformation = instance_transforms_[i];
      combined_aabb = combined_aabb.unionSelf(
          Box::transform(instance_transformation.upperLeft(),
                         instance_transformation[3].xyz, bones_aabb));
    }
    aabb = combined_aabb;
  }

  GetRenderableManager().SetAxisAlignedBoundingBox(GetInstance(), aabb);
}

const Box& MeshRenderer::GetRenderableAabb() const {
  return GetRenderableManager().GetAxisAlignedBoundingBox(GetInstance());
}

void MeshRenderer::ApplyAllMeshPropertyChanges() {
  for (size_t i = 0; i < primitives_.size(); i++) {
    if (!primitives_[i].raw_mesh &&
        !primitives_[i].owned_or_borrowed_mesh.operator->()) {
      continue;
    }

    Mesh& mesh = IsOwnedOrBorrowedPtrType(primitives_[i].held_mesh_type)
                     ? *primitives_[i].owned_or_borrowed_mesh
                     : *primitives_[i].raw_mesh;

    if (primitives_[i].mesh_index_offset != mesh.GetIndexRenderOffset() ||
        primitives_[i].mesh_index_count != mesh.GetIndexRenderCount()) {
      SetRenderableGeometry(mesh, i);
      primitives_[i].mesh_index_count = mesh.GetIndexRenderCount();
      primitives_[i].mesh_index_offset = mesh.GetIndexRenderOffset();
    }
  }
  UpdateRenderableAabb();
}

}  // namespace imp
