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

#include "core/effects/shadow/contact_shadow_receiver.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/common/platform_helpers.h"
#include "core/effects/shadow/contact_shadow_projector.h"
#include "core/math/mat.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/render/texture_factory.h"
#include "core/render/texture_registry.h"
#include "core/view/framework/render/material.h"
#include "core/view/framework/render/mesh_renderer.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// Retrieve the world to clip matrix of the projector's camera.
void ContactShadowReceiver::UpdateProjectorClipToWorld() {
  ContactShadowProjector::System& projector_system =
      GetView()
          .GetComponentManager()
          .GetComponentSystem<ContactShadowProjector>();
  absl::StatusOr<mat4f> pv_matrix_status =
      projector_system.GetClipFromWorld(state_.target_group);
  if (pv_matrix_status.ok()) {
    GetNode()->GetComponent<MeshRenderer>()->GetMaterial()->SetParameter(
        "projection", pv_matrix_status.value());
  }
}

void ContactShadowReceiver::Update(const FrameTime& frame_time) {
  UpdateProjectorClipToWorld();
}

absl::Status ContactShadowReceiver::Setup() {
  if (state_.target_group.empty()) {
    return absl::Status(absl::FailedPreconditionError(
        "Unable to create contact shadow receiver without a target group."));
  }
  if (state_.shadow_parameter_name.empty()) {
    return absl::Status(absl::FailedPreconditionError(
        "Unable to create contact shadow receiver. "
        "No shadow parameter name given. "
        "Cannot pass shadow texture to receiver."));
  }
  return SetupReceiver();
}

absl::Status ContactShadowReceiver::Setup(
    absl::string_view target_group, absl::string_view shadow_parameter_name) {
  state_.target_group = target_group;
  state_.shadow_parameter_name = shadow_parameter_name;
  return SetupReceiver();
}

void ContactShadowReceiver::OnActiveStatusChanged(bool is_active) {
  // We want to set the shadow parameter name to the blank texture,
  // or the right texture name... based on the value of is_active
  absl::Status status = CheckPrerequisites();
  if (!status.ok()) {
    // Should this check fail, it means something has changed on
    // this node since setup or the last active status change.
    IMP_LOG(imp::INFO) << "Cannot enable ContactShadowReceiver component due to "
                 "failed prerequisite: "
              << status;
  } else {
    Material* receiver_material =
        GetNode()->GetComponent<MeshRenderer>()->GetMaterial();
    if (!is_active) {
      if (disabled_shadows_texture_ == nullptr) {
        absl::Status status = SetupTextures();
        if (!status.ok()) IMP_LOG(imp::INFO) << status;
      }
      receiver_material->SetParameter(state_.shadow_parameter_name,
                                      disabled_shadows_texture_.get());
    } else {
      receiver_material->SetParameter(state_.shadow_parameter_name,
                                      enabled_shadows_texture_);
      UpdateProjectorClipToWorld();
    }
  }
}

absl::Status ContactShadowReceiver::SetupReceiver() {
  // First check that all 4 prerequisites are met.
  absl::Status prerequisite_status = CheckPrerequisites();
  if (!prerequisite_status.ok()) {
    return prerequisite_status;
  }

  // All prerequisites are met. Retrieve the shadow texture
  // and apply to the material that is on the mesh renderer
  // that was found on the same node as this receiver.
  absl::Status textures_status = SetupTextures();
  if (!textures_status.ok()) {
    return textures_status;
  }

  GetNode()->GetComponent<MeshRenderer>()->GetMaterial()->SetParameter(
      state_.shadow_parameter_name, enabled_shadows_texture_);

  UpdateProjectorClipToWorld();
  return absl::OkStatus();
}

absl::Status ContactShadowReceiver::CheckPrerequisites() {
  // 1. Ensure that the target group is not empty, and that a projector
  // for the given target group exists.
  //
  // TODO : Free projectors and receivers from order dependency.
  // Edit this later, so that the current presence of a matching projector
  // is not necessary: rather, any unmatched receivers or projectors
  // will simply lie dormant, in case the user adds their match to the
  // world of the View later.
  if (state_.target_group.empty()) {
    return absl::Status(absl::FailedPreconditionError(
        "Unable to create contact shadow receiver without target group."));
  }
  if (!GetView()
           .GetComponentManager()
           .GetComponentSystem<ContactShadowProjector>()
           .HasProjector(state_.target_group)) {
    return absl::Status(absl::FailedPreconditionError(absl::StrFormat(
        "A matching projector was not found for target group %s.",
        state_.target_group)));
  }

  // 2. Check that this node has a mesh renderer on it.
  ComponentHandle<MeshRenderer> mesh_renderer =
      GetNode()->GetComponent<MeshRenderer>();
  if (!mesh_renderer) {
    return absl::Status(absl::FailedPreconditionError(
        "Must add mesh renderer to same node as contact shadow receiver."));
  }

  // 3. Make sure the mesh renderer has a material on it.
  Material* material = mesh_renderer->GetMaterial();
  if (material == nullptr) {
    return absl::Status(absl::FailedPreconditionError(
        "Mesh renderer must have material for contact shadow receiver."));
  }

  // 4. Make sure the material has a parameter to accept
  // the shadow texture from the projector.
  if (!material->HasParameter(state_.shadow_parameter_name)) {
    return absl::Status(absl::FailedPreconditionError(
        absl::StrFormat("Parameter '%s' not found on material "
                        "for contact shadow receiver.",
                        state_.shadow_parameter_name)));
  }

  // 5. Make sure the material has a parameter to accept the projection.
  if (!material->HasParameter("projection")) {
    return absl::Status(absl::FailedPreconditionError(
        absl::StrFormat("Parameter 'projection' not found on material "
                        "for contact shadow receiver.")));
  }

  return absl::OkStatus();
}

absl::Status ContactShadowReceiver::SetupTextures() {
  ContactShadowProjector::System& projector_system =
      GetView()
          .GetComponentManager()
          .GetComponentSystem<ContactShadowProjector>();
  std::optional<absl::string_view> get_shadow_texture_result =
      projector_system.GetShadowTexture(state_.target_group);
  if (!get_shadow_texture_result.has_value()) {
    return absl::Status(absl::UnavailableError(
        absl::StrFormat("Texture for target group '%s' "
                        "not found in Contact Shadow Projector system.",
                        state_.target_group)));
  }
  absl::string_view shadow_texture_name = get_shadow_texture_result.value();

  // Retrieve and save reference to the main shadow texture.
  enabled_shadows_texture_ =
      GetView().GetTextureRegistry().GetTexture(shadow_texture_name);
  if (enabled_shadows_texture_ == nullptr) {
    return absl::Status(absl::UnavailableError(
        absl::StrFormat("Texture '%s' from contact shadow projector "
                        "not found in Texture Registry.",
                        shadow_texture_name)));
  }

  // Generate and save a blank texture, to hide shadows on disable.
  constexpr int kDisabledShadowsTextureSize = 1;
  disabled_shadows_texture_ = GetView().GetTextureFactory().CreateTexture(
      kDisabledShadowsTextureSize, kDisabledShadowsTextureSize,
      filament::Texture::InternalFormat::RGB8);

  constexpr size_t buffer_size =
      kDisabledShadowsTextureSize * kDisabledShadowsTextureSize * 3;
  uint8_t* buffer = (uint8_t*)malloc(buffer_size);
  std::memset(buffer, 0, buffer_size);
  filament::Texture::PixelBufferDescriptor pixel_buffer(
      buffer, buffer_size, filament::Texture::Format::RGB,
      filament::Texture::Type::UBYTE,
      [](void* buf, size_t, void* data) { free((uint8_t*)buf); });
  disabled_shadows_texture_->GetTexture()->setImage(
      *GetView().GetHost()->GetEngine(), 0, std::move(pixel_buffer));

  return absl::OkStatus();
}

}  // namespace imp
