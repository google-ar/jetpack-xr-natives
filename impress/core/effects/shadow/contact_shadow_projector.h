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

#ifndef THIRD_PARTY_IMPRESS_CORE_EFFECTS_SHADOW_CONTACT_SHADOW_PROJECTOR_H_
#define THIRD_PARTY_IMPRESS_CORE_EFFECTS_SHADOW_CONTACT_SHADOW_PROJECTOR_H_

#include <optional>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/effects/shadow/contact_shadow_projector_state.proto.imp.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_system.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/node_handle.h"
#include "core/render_passes/texture_pipeline_renderer.h"
#include "core/view/framework/camera/camera_component.h"
#include "core/view/framework/render/mesh_renderer.h"
#include "core/view/utils/string_map.h"

namespace imp {

class ContactShadowProjector : public Component {
 public:
  // Tracks all instances of contact shadow projectors and their target group.
  // Ensures each projector is instantiated with a unique target group.
  class System : public ComponentSystem<ContactShadowProjector> {
   public:
    explicit System(BaseView* view);

    void AfterLastComponentRemoved() override;

    // Returns true if a contact shadow projector already exists
    // for a given target group.
    bool HasProjector(absl::string_view target_group);

    // Returns the name of the final registered texture a projector
    // for a given target group has drawn for the shadows.
    std::optional<absl::string_view> GetShadowTexture(
        absl::string_view target_group);

    // Retrieves the transformation matrix to go from world to clip space.
    absl::StatusOr<mat4f> GetClipFromWorld(absl::string_view target_group);

   private:
    // Registers the projector to the component system via the target group.
    // The target group must be unique to the projector.
    void RegisterProjector(ComponentHandle<ContactShadowProjector> projector,
                           absl::string_view target_group);

    // Unregister a projector by its target group.
    void UnregisterProjector(absl::string_view target_group);

    StringMap<ComponentHandle<ContactShadowProjector>>
        projector_by_target_group_;

    friend class ContactShadowProjector;
  };

  ~ContactShadowProjector();

  const std::string& GetShadowTexture();

  float GetBlurFactor();
  void SetBlurFactor(float blur_factor);

  void Cleanup();

  void Update();

  Future<absl::Status> Setup();

  Future<absl::Status> Setup(absl::string_view target_group);

  void OnActiveStatusChanged(bool is_active);

  // Allows manual refresh of the shadow drawn by the projector,
  // if the projector is static. By default the projector is static.
  void Refresh();

 private:
  Future<absl::Status> SetupProjector();

  ComponentHandle<MeshRenderer> DrawRenderPassToCameraQuad(
      std::string blur_pass_name, float2 blur_axis,
      AssetPtr<MaterialAsset> blur_material);

  // Name of the first vertical blur pass.
  std::string v_blur_pass_name_;
  // Name of the second horizontal blur pass.
  std::string h_blur_pass_name_;
  // The registered name of the final texture.
  std::string final_pass_texture_name_;

  std::vector<ComponentHandle<MeshRenderer>> shadow_passes_;

  float blur_factor_;

  // Parent node aka root to all camera render pass quads
  //  that the projector is using.
  NodeHandle camera_quads_root_;

  ComponentHandle<TexturePipelineRenderer> pass_renderer_;
  ComponentHandle<CameraComponent> camera_;

  ContactShadowProjectorState state_;

 public:
  using IsfInfo = IsfInfo<&ContactShadowProjector::state_>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_EFFECTS_SHADOW_CONTACT_SHADOW_PROJECTOR_H_
