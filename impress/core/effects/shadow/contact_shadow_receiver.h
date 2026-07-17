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

#ifndef THIRD_PARTY_IMPRESS_CORE_EFFECTS_SHADOW_CONTACT_SHADOW_RECEIVER_H_
#define THIRD_PARTY_IMPRESS_CORE_EFFECTS_SHADOW_CONTACT_SHADOW_RECEIVER_H_

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/effects/shadow/contact_shadow_projector.h"
#include "core/effects/shadow/contact_shadow_receiver_state.proto.imp.h"
#include "core/ncsb/component.h"
#include "core/ncsb/isf_info.h"
#include "core/render/texture.h"
#include "core/view/utils/frame_time.h"

namespace imp {

class ContactShadowReceiver : public Component {
 public:
  void Update(const FrameTime& frame_time);

  absl::Status Setup();
  absl::Status Setup(absl::string_view target_group,
                     absl::string_view shadow_parameter_name);

  void OnActiveStatusChanged(bool is_active);

 private:
  void UpdateProjectorClipToWorld();

  absl::Status SetupReceiver();

  // Check that all prerequisites are met for the receiver to work as intended.
  // This includes a mesh renderer and a compatible material.
  absl::Status CheckPrerequisites();

  // Setup the internal textures for enabled and disabled shadows
  // used by this receiver.
  absl::Status SetupTextures();

  void SetActiveStateTexture(bool state);

  Texture* enabled_shadows_texture_;
  OwnedTexturePtr disabled_shadows_texture_;

  ContactShadowReceiverState state_;

 public:
  using IsfInfo = IsfInfo<&ContactShadowReceiver::state_,
                          IsfDependencies<ContactShadowProjector>>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_EFFECTS_SHADOW_CONTACT_SHADOW_RECEIVER_H_
