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

#include "core/view/framework/render/render_component.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/types/variant.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "core/async/future.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/geometry/shapes/box.h"
#include "core/math/almost_equal.h"
#include "core/model/mesh/mesh.h"
#include "core/model/model_data.h"
#include "core/ncsb/node.h"
#include "core/render/base_renderable_manager.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/render/material.h"
#include "core/view/framework/render/material_definition.proto.imp.h"
#include "core/view/framework/render/mesh_factory.h"
#include "core/view/framework/render/render_component_state.proto.imp.h"

namespace imp {
namespace {

using model::ModelData;
using RenderFlags = ModelData::RenderFlags;

static constexpr uint8_t kMinPriority = 0;
static constexpr uint8_t kMaxPriority = 7;
static constexpr uint8_t kMinChannel = 0;
static constexpr uint8_t kMaxChannel = 3;
static constexpr uint16_t kMinBlendOrder = 0;
static constexpr uint16_t kMaxBlendOrder = 0x7FFF;
static constexpr char kDebugNamePrefix[] = "rc_";

// TODO: Combine Proto frustum culling mode and C++ frustum
// culling mode.
RenderComponent::FrustumCullingMode ConvertFrustrumMode(
    RenderComponentState::FrustrumCullingMode mode) {
  switch (mode) {
    case RenderComponentState::FrustrumCullingMode::DEFAULT_ENABLED:
      return RenderComponent::FrustumCullingMode::kEnabled;
    case RenderComponentState::FrustrumCullingMode::DISABLED:
      return RenderComponent::FrustumCullingMode::kDisabled;
    default:
      return RenderComponent::FrustumCullingMode::kEnabled;
  }
}

// TODO: Combine Proto shadow mode and C++ shadow mode.
RenderComponent::ShadowMode ConvertShadowMode(
    RenderComponentState::ShadowMode mode) {
  switch (mode) {
    case RenderComponentState::ShadowMode::DEFAULT_HARD_SHADOWS:
      return RenderComponent::ShadowMode::kHardShadows;
    case RenderComponentState::ShadowMode::NONE:
      return RenderComponent::ShadowMode::kNone;
    default:
      return RenderComponent::ShadowMode::kHardShadows;
  }
}

CreateBoxSettings MapSettings(const RenderComponentState::BoxMesh& mesh) {
  CreateBoxSettings settings;
  if (mesh.size.has_value()) settings.size = *mesh.size;
  settings.flip_uv = mesh.flip_uv;
  settings.name = std::string(kDebugNamePrefix).append(std::string("box"));
  settings.color = mesh.color;
  return settings;
}

CreateSphereSettings MapSettings(const RenderComponentState::SphereMesh& mesh) {
  CreateSphereSettings settings;
  if (mesh.radius.has_value()) settings.radius = *mesh.radius;
  if (mesh.resolution.has_value()) settings.resolution = *mesh.resolution;
  settings.flip_uv = mesh.flip_uv;
  settings.name = std::string(kDebugNamePrefix).append(std::string("sphere"));
  settings.color = mesh.color;
  return settings;
}

CreateCylinderSettings MapSettings(
    const RenderComponentState::CylinderMesh& mesh) {
  CreateCylinderSettings settings;
  if (mesh.radius.has_value()) settings.radius = *mesh.radius;
  if (mesh.height.has_value()) settings.height = *mesh.height;
  if (mesh.resolution.has_value()) settings.resolution = *mesh.resolution;
  settings.flip_uv = mesh.flip_uv;
  settings.name = std::string(kDebugNamePrefix).append(std::string("cylinder"));
  settings.color = mesh.color;
  return settings;
}

CreateCapsuleSettings MapSettings(
    const RenderComponentState::CapsuleMesh& mesh) {
  CreateCapsuleSettings settings;
  if (mesh.radius.has_value()) settings.radius = *mesh.radius;
  if (mesh.height.has_value()) settings.height = *mesh.height;
  if (mesh.resolution.has_value()) settings.resolution = *mesh.resolution;
  settings.flip_uv = mesh.flip_uv;
  settings.name = std::string(kDebugNamePrefix).append(std::string("capsule"));
  settings.color = mesh.color;
  return settings;
}

CreateConeSettings MapSettings(const RenderComponentState::ConeMesh& mesh) {
  CreateConeSettings settings;
  if (mesh.radius.has_value()) settings.radius = *mesh.radius;
  if (mesh.height.has_value()) settings.height = *mesh.height;
  if (mesh.resolution.has_value()) settings.resolution = *mesh.resolution;
  settings.flip_uv = mesh.flip_uv;
  settings.name = std::string(kDebugNamePrefix).append(std::string("cone"));
  settings.color = mesh.color;
  return settings;
}

CreateQuadSettings MapSettings(const RenderComponentState::QuadMesh& mesh) {
  CreateQuadSettings settings;
  if (mesh.size.has_value()) {
    settings.size = *mesh.size;
  }
  settings.z = mesh.z;
  settings.flip_uv = mesh.flip_uv;
  if (mesh.radius.has_value()) {
    settings.radius = *mesh.radius;
  }
  if (mesh.corner_radius.has_value()) {
    settings.corner_radius = *mesh.corner_radius;
  }
  if (mesh.corner_resolution.has_value()) {
    settings.corner_resolution = *mesh.corner_resolution;
  }
  if (mesh.resolution.has_value() && *mesh.resolution > 1) {
    settings.resolution = *mesh.resolution;
  } else if (mesh.radius.has_value() || mesh.corner_radius.has_value()) {
    settings.resolution = kDefaultQuadResolution;
  }
  settings.name = std::string(kDebugNamePrefix).append(std::string("quad"));
  settings.color = mesh.color;
  return settings;
}

// Generates a mesh from the settings of a primitive in RenderComponentState.
OwnedMeshPtr CreateMeshForPrimitive(
    BaseView& view, const RenderComponentState::Primitive& primitive) {
  return absl::visit(
      [&view](const auto& mesh) -> OwnedMeshPtr {
        using ParamT = std::decay_t<decltype(mesh)>;
        if constexpr (std::is_same_v<ParamT, RenderComponentState::BoxMesh>) {
          return view.GetMeshFactory().CreateBox(MapSettings(mesh));
        } else if constexpr (std::is_same_v<ParamT,
                                            RenderComponentState::SphereMesh>) {
          return view.GetMeshFactory().CreateSphere(MapSettings(mesh));
        } else if constexpr (std::is_same_v<
                                 ParamT, RenderComponentState::CylinderMesh>) {
          return view.GetMeshFactory().CreateCylinder(MapSettings(mesh));
        } else if constexpr (std::is_same_v<
                                 ParamT, RenderComponentState::CapsuleMesh>) {
          return view.GetMeshFactory().CreateCapsule(MapSettings(mesh));
        } else if constexpr (std::is_same_v<ParamT,
                                            RenderComponentState::ConeMesh>) {
          return view.GetMeshFactory().CreateCone(MapSettings(mesh));
        } else if constexpr (std::is_same_v<ParamT,
                                            RenderComponentState::QuadMesh>) {
          CreateQuadSettings settings = MapSettings(mesh);
          if ((!settings.radius.has_value() ||
               AlmostEqual(*settings.radius, 0.0f)) &&
              (!settings.corner_radius.has_value() ||
               AlmostEqual(*settings.corner_radius, 0.0f)) &&
              settings.resolution <= 2) {
            return view.GetMeshFactory().CreateQuad(settings);
          } else {
            std::optional<float> radius = std::nullopt;
            if (settings.radius > 0) {
              radius = settings.radius;
            }
            if (settings.corner_radius > 0 &&
                (settings.corner_radius > settings.size.x / 2 ||
                 settings.corner_radius > settings.size.y / 2)) {
              IMP_LOG(imp::WARNING) << "Invalid corner radius "
                           << *settings.corner_radius << " is > size / 2.";
              return {};
            }
            return view.GetMeshFactory().CreatePanel(settings);
          }
        } else {
          return {};
        }
      },
      primitive.mesh);
}

}  // namespace

void RenderComponent::Cleanup() { GetRenderableManager().Destroy(GetEntity()); }

Future<absl::Status> RenderComponent::SetupWithState() {
  Future<absl::Status> result = OnIsfStateChanged();
  // Do not display until the component is "awake"
  GetRenderableManager().SetLayerMask(GetInstance(), 0xff, 0);
  return result;
}

void RenderComponent::Setup(size_t primitive_count) {
  Setup(FrustumCullingMode::kEnabled, primitive_count);
}

void RenderComponent::Setup(FrustumCullingMode culling_mode,
                            size_t primitive_count) {
  BuildRenderables(culling_mode, primitive_count);

  // Do not display until the component is "awake"
  GetRenderableManager().SetLayerMask(GetInstance(), 0xff, 0);
}

bool RenderComponent::IsOwnedOrBorrowedPtrType(
    const HeldPtrType& held_ptr_type) const {
  return held_ptr_type == HeldPtrType::kBorrowedPointer ||
         held_ptr_type == HeldPtrType::kOwnedPointer;
}

void RenderComponent::BuildRenderables(FrustumCullingMode culling_mode,
                                       size_t primitive_count) {
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

  builder->Priority(priority_);
  builder->Build(*BaseView::GetSharedEngine(), GetNode()->GetEntity());

  primitives_.resize(primitive_count);
  layer_mask_ = GetRenderableManager().GetLayerMask(GetInstance());
}

Future<absl::Status> RenderComponent::OnIsfStateChanged() {
  Future<absl::Status> result(absl::OkStatus());

  const size_t kNumPrimitives = state_.primitives.size();

  // Build using frustum from Isf and the correct number of primitives.
  // If there are no primitives from Isf, default to 1 primitive.s
  BuildRenderables(ConvertFrustrumMode(state_.frustrum_culling_mode),
                   kNumPrimitives != 0 ? kNumPrimitives : 1);

  // Assign shadow modes from Isf.
  SetShadowCastingMode(ConvertShadowMode(state_.shadow_casting_mode));
  SetShadowReceivingMode(ConvertShadowMode(state_.shadow_receiving_mode));

  // Assign priority from Isf.
  if (state_.priority.has_value()) {
    SetPriority(*state_.priority);
  }

  // Assign channel from Isf.
  if (state_.channel.has_value()) {
    SetChannel(*state_.channel);
  }

  for (size_t i = 0; i < kNumPrimitives; i++) {
    const RenderComponentState::Primitive& primitive = state_.primitives.at(i);

    // Assign mesh from Isf if there is one.
    OwnedMeshPtr mesh = CreateMeshForPrimitive(GetView(), primitive);
    if (mesh) {
      SetMesh(std::move(mesh), i);
    }

    // Assign material from Isf if there is one.
    if (primitive.material.has_value()) {
      result = result.Combine(GetView()
                                  .GetMaterialFactory()
                                  .LoadMaterial(*primitive.material)
                                  .Then([this, i](OwnedMaterialPtr material) {
                                    SetMaterial(std::move(material), i);
                                  }));
    }
  }

  return result;
}

void RenderComponent::OnActiveStatusChanged(bool is_active) {
  GetRenderableManager().SetLayerMask(GetInstance(), 0xff,
                                      is_active ? layer_mask_ : 0);
}

size_t RenderComponent::GetPrimitiveCount() const {
  return GetRenderableManager().GetPrimitiveCount(GetInstance());
}

void RenderComponent::SetMaterial(MaterialPtr material,
                                  size_t primitive_index) {
  if (!IsWithinCount(primitive_index)) {
    return;
  }
  GetRenderableManager().SetMaterialInstanceAt(
      GetInstance(), primitive_index, material->GetFilamentMaterialInstance());
  primitives_[primitive_index].owned_or_borrowed_material =
      OwnedOrBorrowedPtr<Material>();
  primitives_[primitive_index].material.Set(std::move(material));
  primitives_[primitive_index].held_material_type = HeldPtrType::kUniquePtr;
}

void RenderComponent::SetMaterial(Material* material, size_t primitive_index) {
  if (!IsWithinCount(primitive_index)) {
    return;
  }
  GetRenderableManager().SetMaterialInstanceAt(
      GetInstance(), primitive_index, material->GetFilamentMaterialInstance());
  primitives_[primitive_index].owned_or_borrowed_material =
      OwnedOrBorrowedPtr<Material>();
  primitives_[primitive_index].material.Set(material);
  primitives_[primitive_index].held_material_type = HeldPtrType::kRawPointer;
}

void RenderComponent::SetMaterial(OwnedMaterialPtr material,
                                  size_t primitive_index) {
  if (!IsWithinCount(primitive_index)) {
    return;
  }
  GetRenderableManager().SetMaterialInstanceAt(
      GetInstance(), primitive_index, material->GetFilamentMaterialInstance());
  primitives_[primitive_index].owned_or_borrowed_material =
      OwnedOrBorrowedPtr<Material>(std::move(material));
  primitives_[primitive_index].material.Reset();
  primitives_[primitive_index].held_material_type = HeldPtrType::kOwnedPointer;
}

void RenderComponent::SetMaterial(BorrowedMaterialPtr material,
                                  size_t primitive_index) {
  if (!IsWithinCount(primitive_index)) {
    return;
  }
  GetRenderableManager().SetMaterialInstanceAt(
      GetInstance(), primitive_index, material->GetFilamentMaterialInstance());
  primitives_[primitive_index].owned_or_borrowed_material =
      OwnedOrBorrowedPtr<Material>(material);
  primitives_[primitive_index].material.Reset();
  primitives_[primitive_index].held_material_type =
      HeldPtrType::kBorrowedPointer;
}

Future<absl::Status> RenderComponent::SetMaterial(MaterialDefinition material,
                                                  size_t primitive_index) {
  if (!IsWithinCount(primitive_index)) {
    return Future<absl::Status>(absl::InvalidArgumentError(
        absl::StrFormat("Invalid index: %d", primitive_index)));
  }
  state_.primitives[primitive_index].material = material;
  return OnIsfStateChanged();
}

Future<absl::Status> RenderComponent::SetMaterialParameter(
    MaterialDefinition::Parameter parameter, size_t primitive_index) {
  if (!IsWithinCount(primitive_index)) {
    return Future<absl::Status>(absl::InvalidArgumentError(
        absl::StrFormat("Invalid index: %d", primitive_index)));
  }
  MaterialDefinition material_definition;
  std::optional<MaterialDefinition> existing_definition =
      state_.primitives[primitive_index].material;
  if (existing_definition.has_value()) {
    material_definition = *existing_definition;
  }
  bool name_matches_existing = false;
  for (auto& existing : material_definition.parameters) {
    if (existing.name == parameter.name) {
      existing.val = parameter.val;
      name_matches_existing = true;
      break;
    }
  }
  if (!name_matches_existing) {
    material_definition.parameters.push_back(parameter);
  }
  state_.primitives[primitive_index].material = material_definition;

  Material* material = GetMaterial(primitive_index);
  if (!material) {
    return Future<absl::Status>(absl::InvalidArgumentError(
        absl::StrFormat("No material at index %d", primitive_index)));
  }
  return MaterialFactory::SetMaterialParameters(GetView(), material,
                                                material_definition.parameters)
      .Then([this]() { return OnIsfStateChanged(); });
}

Material* RenderComponent::GetMaterial(size_t primitive_index) const {
  if (IsWithinCount(primitive_index) && primitive_index < primitives_.size()) {
    if (IsOwnedOrBorrowedPtrType(
            primitives_[primitive_index].held_material_type)) {
      return primitives_[primitive_index]
          .owned_or_borrowed_material.operator->();
    } else {
      return primitives_[primitive_index].material.Get();
    }
  }
  return nullptr;
}

void RenderComponent::SetMesh(Mesh* mesh, size_t primitive_index) {
  if (!IsWithinCount(primitive_index)) {
    return;
  }

  SetRenderableGeometry(*mesh, primitive_index);
  primitives_[primitive_index].mesh_index_count = mesh->GetIndexRenderCount();
  primitives_[primitive_index].mesh_index_offset = mesh->GetIndexRenderOffset();
  primitives_[primitive_index].held_mesh_type = HeldPtrType::kRawPointer;
  primitives_[primitive_index].owned_or_borrowed_mesh =
      OwnedOrBorrowedPtr<Mesh>();
  primitives_[primitive_index].mesh.Set(mesh);
  UpdateRenderableAabb();
}

void RenderComponent::SetMesh(MeshPtr mesh, size_t primitive_index) {
  if (!IsWithinCount(primitive_index)) {
    return;
  }

  SetRenderableGeometry(*mesh, primitive_index);
  primitives_[primitive_index].mesh_index_count = mesh->GetIndexRenderCount();
  primitives_[primitive_index].mesh_index_offset = mesh->GetIndexRenderOffset();
  primitives_[primitive_index].held_mesh_type = HeldPtrType::kUniquePtr;
  primitives_[primitive_index].owned_or_borrowed_mesh =
      OwnedOrBorrowedPtr<Mesh>();
  primitives_[primitive_index].mesh.Set(std::move(mesh));
  UpdateRenderableAabb();
}

void RenderComponent::SetMesh(BorrowedMeshPtr mesh, size_t primitive_index) {
  if (!IsWithinCount(primitive_index)) {
    return;
  }

  SetRenderableGeometry(*mesh, primitive_index);
  primitives_[primitive_index].mesh_index_count = mesh->GetIndexRenderCount();
  primitives_[primitive_index].mesh_index_offset = mesh->GetIndexRenderOffset();
  primitives_[primitive_index].held_mesh_type = HeldPtrType::kBorrowedPointer;
  primitives_[primitive_index].owned_or_borrowed_mesh =
      OwnedOrBorrowedPtr<Mesh>(mesh);
  primitives_[primitive_index].mesh.Reset();
  UpdateRenderableAabb();
}

void RenderComponent::SetMesh(OwnedMeshPtr mesh, size_t primitive_index) {
  if (!IsWithinCount(primitive_index)) {
    return;
  }

  SetRenderableGeometry(*mesh, primitive_index);
  primitives_[primitive_index].mesh_index_count = mesh->GetIndexRenderCount();
  primitives_[primitive_index].mesh_index_offset = mesh->GetIndexRenderOffset();
  primitives_[primitive_index].held_mesh_type = HeldPtrType::kOwnedPointer;
  primitives_[primitive_index].owned_or_borrowed_mesh =
      OwnedOrBorrowedPtr<Mesh>(std::move(mesh));
  primitives_[primitive_index].mesh.Reset();
  UpdateRenderableAabb();
}

void RenderComponent::SetRenderableGeometry(Mesh& mesh,
                                            size_t primitive_index) {
  filament::RenderableManager::Instance instance = GetInstance();
  GetRenderableManager().SetGeometryAt(
      instance, primitive_index, mesh.GetPrimitiveType(),
      mesh.GetVertexBuffer(), mesh.GetIndexBuffer(),
      mesh.GetIndexRenderOffset(), mesh.GetIndexRenderCount());
}

Mesh* RenderComponent::GetMesh(size_t primitive_index) const {
  if (IsWithinCount(primitive_index) && primitive_index < primitives_.size()) {
    if (IsOwnedOrBorrowedPtrType(primitives_[primitive_index].held_mesh_type)) {
      return primitives_[primitive_index].owned_or_borrowed_mesh.operator->();
    } else {
      return primitives_[primitive_index].mesh.Get();
    }
  }
  return nullptr;
}

void RenderComponent::SetShadowCastingMode(ShadowMode shadow_mode) {
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

RenderComponent::ShadowMode RenderComponent::GetShadowCastingMode() const {
  bool is_shadow_caster = GetRenderableManager().IsShadowCaster(GetInstance());
  return is_shadow_caster ? ShadowMode::kHardShadows : ShadowMode::kNone;
}

void RenderComponent::SetShadowReceivingMode(ShadowMode shadow_mode) {
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

RenderComponent::ShadowMode RenderComponent::GetShadowReceivingMode() const {
  bool is_shadow_receiver =
      GetRenderableManager().IsShadowReceiver(GetInstance());
  return is_shadow_receiver ? ShadowMode::kHardShadows : ShadowMode::kNone;
}

void RenderComponent::SetPriority(uint8_t priority) {
  // Filament also clamps the priority between 0 and 7, but filament has no
  // getter for the priority so we need to track it ourselves.
  priority_ = std::clamp(priority, kMinPriority, kMaxPriority);
  GetRenderableManager().SetPriority(GetInstance(), priority_);
}

uint8_t RenderComponent::GetPriority() const { return priority_; }

void RenderComponent::SetChannel(uint8_t channel) {
  // Filament also clamps the channel between 0 and 3, but filament has no
  // getter for the channel so we need to track it ourselves.
  channel_ = std::clamp(channel, kMinChannel, kMaxChannel);
  GetRenderableManager().SetChannel(GetInstance(), channel_);
}

void RenderComponent::SetFogEnabled(bool enable) {
  GetRenderableManager().SetFogEnabled(GetInstance(), enable);
}

uint8_t RenderComponent::GetChannel() const { return channel_; }

void RenderComponent::SetBlendOrder(uint16_t blend_order, BlendOrderMode mode,
                                    size_t primitive) {
  filament::RenderableManager::Instance instance = GetInstance();
  // Filament also clamps the order to 15 bits.
  blend_order = std::clamp(blend_order, kMinBlendOrder, kMaxBlendOrder);
  GetRenderableManager().SetBlendOrderAt(instance, primitive, blend_order);
  GetRenderableManager().SetGlobalBlendOrderEnabledAt(
      instance, primitive, mode == BlendOrderMode::kLocal ? false : true);
}

BaseRenderableManager& RenderComponent::GetRenderableManager() const {
  return GetView().GetRenderableManager();
}

filament::RenderableManager::Instance RenderComponent::GetInstance() const {
  return GetRenderableManager().GetInstance(GetEntity());
}

bool RenderComponent::IsWithinCount(size_t primitive_index) const {
  size_t primitive_count = GetPrimitiveCount();
  if (primitive_index >= primitive_count) {
    IMP_LOG(imp::FATAL) << "Primitive index " << primitive_index
               << " out of bounds: " << primitive_count;
    return false;
  }
  return true;
}

void RenderComponent::UpdateRenderableAabb() {
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
      if (primitives_[i].mesh) {
        if (!found_primitive) {
          aabb = primitives_[i].mesh->GetAabb();
          found_primitive = true;
        } else {
          aabb = aabb.unionSelf(primitives_[i].mesh->GetAabb());
        }
      }
    }
  }

  GetRenderableManager().SetAxisAlignedBoundingBox(GetInstance(), aabb);
}

const Box& RenderComponent::GetRenderableAabb() const {
  return GetRenderableManager().GetAxisAlignedBoundingBox(GetInstance());
}

void RenderComponent::ApplyAllMeshPropertyChanges() {
  for (size_t i = 0; i < primitives_.size(); i++) {
    if (!primitives_[i].mesh &&
        !primitives_[i].owned_or_borrowed_mesh.operator->()) {
      continue;
    }
    if (primitives_[i].mesh_index_offset !=
            primitives_[i].mesh->GetIndexRenderOffset() ||
        primitives_[i].mesh_index_count !=
            primitives_[i].mesh->GetIndexRenderCount()) {
      Mesh& mesh = IsOwnedOrBorrowedPtrType(primitives_[i].held_mesh_type)
                       ? *primitives_[i].owned_or_borrowed_mesh
                       : *primitives_[i].mesh;
      SetRenderableGeometry(mesh, i);
      primitives_[i].mesh_index_count = mesh.GetIndexRenderCount();
      primitives_[i].mesh_index_offset = mesh.GetIndexRenderOffset();
    }
  }
  UpdateRenderableAabb();
}

}  // namespace imp
