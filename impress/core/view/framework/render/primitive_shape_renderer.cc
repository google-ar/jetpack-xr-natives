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

#include "core/view/framework/render/primitive_shape_renderer.h"

#include <cstdint>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/types/variant.h"
#include "core/async/future.h"
#include "core/materials/material.h"
#include "core/math/almost_equal.h"
#include "core/model/mesh/mesh.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/render/base_renderable_manager.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/render/material_definition.proto.imp.h"
#include "core/view/framework/render/mesh_factory.h"
#include "core/view/framework/render/mesh_renderer.h"
#include "core/view/framework/render/primitive_shape_renderer_state.proto.imp.h"
#include "core/view/framework/render/primitive_shape_type.h"

namespace imp {
namespace {

static constexpr char kDebugNamePrefix[] = "rc_";

// TODO: Combine Proto shadow mode and C++ shadow mode.
MeshRenderer::ShadowMode ConvertShadowCastingMode(
    PrimitiveShapeRendererState::ShadowMode mode) {
  switch (mode) {
    case PrimitiveShapeRendererState::ShadowMode::HARD_SHADOWS:
      return MeshRenderer::ShadowMode::kHardShadows;
    case PrimitiveShapeRendererState::ShadowMode::NONE:
      return MeshRenderer::ShadowMode::kNone;
    default:
      return MeshRenderer::ShadowMode::kNone;
  }
}

PrimitiveShapeRendererState::ShadowMode ConvertShadowCastingMode(
    MeshRenderer::ShadowMode mode) {
  switch (mode) {
    case MeshRenderer::ShadowMode::kHardShadows:
      return PrimitiveShapeRendererState::ShadowMode::HARD_SHADOWS;
    case MeshRenderer::ShadowMode::kNone:
      return PrimitiveShapeRendererState::ShadowMode::NONE;
    default:
      return PrimitiveShapeRendererState::ShadowMode::NONE;
  }
}

MeshRenderer::ShadowMode ConvertShadowReceivingMode(
    PrimitiveShapeRendererState::ShadowMode mode) {
  switch (mode) {
    case PrimitiveShapeRendererState::ShadowMode::HARD_SHADOWS:
      return MeshRenderer::ShadowMode::kHardShadows;
    case PrimitiveShapeRendererState::ShadowMode::NONE:
      return MeshRenderer::ShadowMode::kNone;
    default:
      return MeshRenderer::ShadowMode::kHardShadows;
  }
}

PrimitiveShapeRendererState::ShadowMode ConvertShadowReceivingMode(
    MeshRenderer::ShadowMode mode) {
  switch (mode) {
    case MeshRenderer::ShadowMode::kHardShadows:
      return PrimitiveShapeRendererState::ShadowMode::HARD_SHADOWS;
    case MeshRenderer::ShadowMode::kNone:
      return PrimitiveShapeRendererState::ShadowMode::NONE;
    default:
      return PrimitiveShapeRendererState::ShadowMode::HARD_SHADOWS;
  }
}

CreateBoxSettings MapSettings(
    const PrimitiveShapeRendererState::BoxMesh& mesh) {
  CreateBoxSettings settings;
  if (mesh.size.has_value()) settings.size = *mesh.size;
  settings.flip_uv = mesh.flip_uv;
  settings.name = std::string(kDebugNamePrefix).append(std::string("box"));
  settings.color = mesh.color;
  return settings;
}

CreateSphereSettings MapSettings(
    const PrimitiveShapeRendererState::SphereMesh& mesh) {
  CreateSphereSettings settings;
  if (mesh.radius.has_value()) settings.radius = *mesh.radius;
  if (mesh.resolution.has_value()) settings.resolution = *mesh.resolution;
  settings.flip_uv = mesh.flip_uv;
  settings.flip_face_direction = mesh.flip_face_direction;
  settings.name = std::string(kDebugNamePrefix).append(std::string("sphere"));
  settings.color = mesh.color;
  return settings;
}

CreateCylinderSettings MapSettings(
    const PrimitiveShapeRendererState::CylinderMesh& mesh) {
  CreateCylinderSettings settings;
  if (mesh.radius.has_value()) settings.radius = *mesh.radius;
  if (mesh.height.has_value()) settings.height = *mesh.height;
  if (mesh.resolution.has_value()) settings.resolution = *mesh.resolution;
  settings.name = std::string(kDebugNamePrefix).append(std::string("cylinder"));
  settings.flip_uv = mesh.flip_uv;
  settings.color = mesh.color;
  return settings;
}

CreateCapsuleSettings MapSettings(
    const PrimitiveShapeRendererState::CapsuleMesh& mesh) {
  CreateCapsuleSettings settings;
  if (mesh.radius.has_value()) settings.radius = *mesh.radius;
  if (mesh.height.has_value()) settings.height = *mesh.height;
  if (mesh.resolution.has_value()) settings.resolution = *mesh.resolution;
  settings.flip_uv = mesh.flip_uv;
  settings.name = std::string(kDebugNamePrefix).append(std::string("capsule"));
  settings.color = mesh.color;
  return settings;
}

CreateConeSettings MapSettings(
    const PrimitiveShapeRendererState::ConeMesh& mesh) {
  CreateConeSettings settings;
  if (mesh.radius.has_value()) settings.radius = *mesh.radius;
  if (mesh.height.has_value()) settings.height = *mesh.height;
  if (mesh.resolution.has_value()) settings.resolution = *mesh.resolution;
  settings.name = std::string(kDebugNamePrefix).append(std::string("cone"));
  settings.flip_uv = mesh.flip_uv;
  settings.color = mesh.color;
  return settings;
}

CreateQuadSettings MapSettings(
    const PrimitiveShapeRendererState::QuadMesh& mesh) {
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

PrimitiveShapeType GetShapeTypeForPrimitive(
    const PrimitiveShapeRendererState::Primitive& primitive) {
  return absl::visit(
      [](const auto& mesh) -> PrimitiveShapeType {
        using ParamT = std::decay_t<decltype(mesh)>;
        if constexpr (std::is_same_v<ParamT,
                                     PrimitiveShapeRendererState::BoxMesh>) {
          return PrimitiveShapeType::kBox;
        } else if constexpr (std::is_same_v<
                                 ParamT,
                                 PrimitiveShapeRendererState::SphereMesh>) {
          return PrimitiveShapeType::kSphere;
        } else if constexpr (std::is_same_v<
                                 ParamT,
                                 PrimitiveShapeRendererState::CapsuleMesh>) {
          return PrimitiveShapeType::kCapsule;
        } else if constexpr (std::is_same_v<
                                 ParamT,
                                 PrimitiveShapeRendererState::QuadMesh>) {
          return PrimitiveShapeType::kQuad;
        } else if constexpr (std::is_same_v<
                                 ParamT,
                                 PrimitiveShapeRendererState::CylinderMesh>) {
          return PrimitiveShapeType::kCylinder;
        } else if constexpr (std::is_same_v<
                                 ParamT,
                                 PrimitiveShapeRendererState::ConeMesh>) {
          return PrimitiveShapeType::kCone;
        } else {
          return PrimitiveShapeType::kPanel;
        }
      },
      primitive.mesh);
}

// Generates a mesh from the settings of a primitive in
// PrimitiveShapeRendererState.
OwnedMeshPtr CreateMeshForPrimitive(
    BaseView& view, const PrimitiveShapeRendererState::Primitive& primitive) {
  return absl::visit(
      [&view](const auto& mesh) -> OwnedMeshPtr {
        using ParamT = std::decay_t<decltype(mesh)>;
        if constexpr (std::is_same_v<ParamT,
                                     PrimitiveShapeRendererState::BoxMesh>) {
          return view.GetMeshFactory().CreateBox(MapSettings(mesh));
        } else if constexpr (std::is_same_v<
                                 ParamT,
                                 PrimitiveShapeRendererState::SphereMesh>) {
          return view.GetMeshFactory().CreateSphere(MapSettings(mesh));
        } else if constexpr (std::is_same_v<
                                 ParamT,
                                 PrimitiveShapeRendererState::CylinderMesh>) {
          return view.GetMeshFactory().CreateCylinder(MapSettings(mesh));
        } else if constexpr (std::is_same_v<
                                 ParamT,
                                 PrimitiveShapeRendererState::CapsuleMesh>) {
          return view.GetMeshFactory().CreateCapsule(MapSettings(mesh));
        } else if constexpr (std::is_same_v<
                                 ParamT,
                                 PrimitiveShapeRendererState::ConeMesh>) {
          return view.GetMeshFactory().CreateCone(MapSettings(mesh));
        } else if constexpr (std::is_same_v<
                                 ParamT,
                                 PrimitiveShapeRendererState::QuadMesh>) {
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

Future<absl::Status> PrimitiveShapeRenderer::Setup() {
  return SetupWithState();
}

Future<absl::Status> PrimitiveShapeRenderer::SetupWithState() {
  Future<absl::Status> result = OnIsfStateChanged();
  return result;
}

void PrimitiveShapeRenderer::Cleanup() {
  mesh_renderer_ = ComponentHandle<MeshRenderer>();
  GetNode()->RemoveComponent<MeshRenderer>();
}

Future<absl::Status> PrimitiveShapeRenderer::OnIsfStateChanged() {
  Future<absl::Status> result(absl::OkStatus());

  MeshRenderer::FrustumCullingMode culling_mode =
      static_cast<MeshRenderer::FrustumCullingMode>(
          state_.frustrum_culling_mode);
  mesh_renderer_ = GetNode()->AddComponent<MeshRenderer>(culling_mode, 1);

  // Assign shadow modes from Isf.
  mesh_renderer_->SetShadowCastingMode(
      ConvertShadowCastingMode(state_.shadow_casting_mode));
  mesh_renderer_->SetShadowReceivingMode(
      ConvertShadowReceivingMode(state_.shadow_receiving_mode));

  // Assign priority from Isf.
  if (state_.priority.has_value()) {
    SetPriority(*state_.priority);
  }

  // Assign channel from Isf.
  if (state_.channel.has_value()) {
    SetChannel(*state_.channel);
  }

  const PrimitiveShapeRendererState::Primitive& primitive = state_.primitive;

  // Assign mesh from Isf if there is one.
  OwnedMeshPtr mesh = CreateMeshForPrimitive(GetView(), primitive);
  if (mesh) {
    mesh_renderer_->SetMesh(std::move(mesh));
  } else {
    return Future<absl::Status>(
        absl::UnavailableError("No mesh found on primitive."));
  }

  // Store the primitive shape type.
  shape_type_ = GetShapeTypeForPrimitive(primitive);

  // Assign material from Isf if there is one.
  if (primitive.material.has_value()) {
    result = result.Combine(GetView()
                                .GetMaterialFactory()
                                .LoadMaterial(*primitive.material)
                                .Then([this](OwnedMaterialPtr material) {
                                  return SetMaterial(std::move(material));
                                }));
  }

  return result;
}

void PrimitiveShapeRenderer::OnActiveStatusChanged(bool is_active) {
  if (mesh_renderer_ && mesh_renderer_.IsValid()) {
    mesh_renderer_->OnActiveStatusChanged(is_active);
  }
}

Future<absl::Status> PrimitiveShapeRenderer::SetMaterial(MaterialPtr material) {
  mesh_renderer_->SetMaterial(std::move(material));
  // Note: material info will not be saved to state.
  return Future<absl::Status>(absl::OkStatus());
}

Future<absl::Status> PrimitiveShapeRenderer::SetMaterial(Material* material) {
  mesh_renderer_->SetMaterial(std::move(material));
  // Note: material info will not be saved to state.
  return Future<absl::Status>(absl::OkStatus());
}

Future<absl::Status> PrimitiveShapeRenderer::SetMaterial(
    OwnedMaterialPtr material) {
  mesh_renderer_->SetMaterial(std::move(material));
  // Note: material info will not be saved to state.
  return Future<absl::Status>(absl::OkStatus());
}

Future<absl::Status> PrimitiveShapeRenderer::SetMaterial(
    BorrowedMaterialPtr material) {
  mesh_renderer_->SetMaterial(std::move(material));
  // Note: material info will not be saved to state.
  return Future<absl::Status>(absl::OkStatus());
}

Future<absl::Status> PrimitiveShapeRenderer::SetMaterial(
    MaterialDefinition material) {
  state_.primitive.material = material;
  return OnIsfStateChanged();
}

Future<absl::Status> PrimitiveShapeRenderer::SetMaterialParameter(
    MaterialDefinition::Parameter parameter) {
  MaterialDefinition material_definition;
  std::optional<MaterialDefinition> existing_definition =
      state_.primitive.material;
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
  state_.primitive.material = material_definition;

  return MaterialFactory::SetMaterialParameters(GetView(), GetMaterial(),
                                                material_definition.parameters)
      .Then([this]() { return OnIsfStateChanged(); });
}

Material* PrimitiveShapeRenderer::GetMaterial() const {
  return mesh_renderer_->GetMaterial();
}

absl::StatusOr<Mesh*> PrimitiveShapeRenderer::GetMesh() const {
  if (mesh_renderer_->GetMesh() == nullptr) {
    return absl::Status(
        absl::InvalidArgumentError(absl::StrFormat("No mesh found.")));
  }
  return mesh_renderer_->GetMesh();
}

PrimitiveShapeType PrimitiveShapeRenderer::GetShapeType() {
  return shape_type_;
}

void PrimitiveShapeRenderer::SetShadowCastingMode(
    MeshRenderer::ShadowMode shadow_mode) {
  state_.shadow_casting_mode = ConvertShadowCastingMode(shadow_mode);
  mesh_renderer_->SetShadowCastingMode(shadow_mode);
}

MeshRenderer::ShadowMode PrimitiveShapeRenderer::GetShadowCastingMode() const {
  return mesh_renderer_->GetShadowCastingMode();
}

void PrimitiveShapeRenderer::SetShadowReceivingMode(
    MeshRenderer::ShadowMode shadow_mode) {
  state_.shadow_receiving_mode = ConvertShadowReceivingMode(shadow_mode);
  mesh_renderer_->SetShadowReceivingMode(shadow_mode);
}

MeshRenderer::ShadowMode PrimitiveShapeRenderer::GetShadowReceivingMode()
    const {
  return mesh_renderer_->GetShadowReceivingMode();
}

void PrimitiveShapeRenderer::SetPriority(uint8_t priority) {
  state_.priority = priority;
  mesh_renderer_->SetPriority(priority);
}

uint8_t PrimitiveShapeRenderer::GetPriority() const {
  return mesh_renderer_->GetPriority();
}

void PrimitiveShapeRenderer::SetChannel(uint8_t channel) {
  state_.channel = channel;
  mesh_renderer_->SetChannel(channel);
}

uint8_t PrimitiveShapeRenderer::GetChannel() const {
  return mesh_renderer_->GetChannel();
}

void PrimitiveShapeRenderer::SetFogEnabled(bool enable) {
  mesh_renderer_->SetFogEnabled(enable);
}

absl::Status PrimitiveShapeRenderer::SetBlendOrder(
    uint16_t blend_order, MeshRenderer::BlendOrderMode mode) {
  mesh_renderer_->SetBlendOrder(blend_order, mode);
  return absl::OkStatus();
}

}  // namespace imp
