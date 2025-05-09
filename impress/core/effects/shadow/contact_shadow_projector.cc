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

#include "core/effects/shadow/contact_shadow_projector.h"

#include <algorithm>
#include <optional>
#include <string>
#include <tuple>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/effects/shadow/contact_shadow_assets.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_system.h"
#include "core/ncsb/node_handle.h"
#include "core/render/texture.h"
#include "core/render_passes/texture_pipeline_renderer.h"
#include "core/render_passes/texture_pipeline_renderer_state.proto.imp.h"
#include "core/scene_handles/scene_handles.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/camera/camera_component.h"
#include "core/view/framework/camera/camera_helpers.h"
#include "core/view/framework/camera/camera_state.proto.imp.h"
#include "core/view/framework/render/material_definition.proto.imp.h"
#include "core/view/framework/render/mesh_factory.h"
#include "core/view/framework/render/mesh_renderer.h"

namespace imp {

static constexpr int kDefaultShadowTextureSize = 256;
static constexpr float kDefaultBlurFactor = .05f;
// Displacement for the quads used for the render passes.
static constexpr float3 kQuadPassDisplacement = {0, 0, -1};
static constexpr bool kIsStaticDefault = true;

// TODO: Remove dependence on an arbitrary scale value
// later, when smart positioning and scaling are implemented.
static constexpr float kDefaultSize = 1;

static constexpr CameraState::ProjectionType kDefaultProjectionType =
    CameraState::ProjectionType::ORTHOGRAPHIC;

ContactShadowProjector::System::System(BaseView* view)
    : ComponentSystem<ContactShadowProjector>(view) {}

void ContactShadowProjector::System::AfterLastComponentRemoved() {
  // This shouldn't happen, implies a bug in the Register/Unregister code.
  if (!projector_by_target_group_.empty()) {
    IMP_LOG(imp::FATAL) << "All ConstactShadowProjectors are removed, but "
                  "projector_by_target_group_ is not empty.";
  }
}

bool ContactShadowProjector::System::HasProjector(
    absl::string_view target_group) {
  return projector_by_target_group_.count(target_group) > 0;
}

std::optional<absl::string_view>
ContactShadowProjector::System::GetShadowTexture(
    absl::string_view target_group) {
  if (HasProjector(target_group)) {
    return projector_by_target_group_[std::string(target_group)]
        ->GetShadowTexture();
  }
  IMP_LOG(imp::ERROR) << "No Projector found for target group '" << target_group
             << "'. 'GetShadowTexture' failed.";
  return std::nullopt;
}

absl::StatusOr<mat4f> ContactShadowProjector::System::GetClipFromWorld(
    absl::string_view target_group) {
  if (HasProjector(target_group)) {
    ComponentHandle<CameraComponent> camera =
        projector_by_target_group_[std::string(target_group)]->camera_;
    return static_cast<mat4f>(camera->ClipFromWorld());
  }
  return absl::NotFoundError(
      absl::StrFormat("No Projector found for target group '%s'. "
                      "'GetProjectionMatrix' failed.",
                      target_group));
}

void ContactShadowProjector::System::RegisterProjector(
    ComponentHandle<ContactShadowProjector> projector,
    absl::string_view target_group) {
  if (projector_by_target_group_.count(target_group) > 0) {
    IMP_LOG(imp::FATAL) << "Cannot register projector under target group '"
               << target_group << "': already in use";
  }
  projector_by_target_group_.emplace(std::string(target_group), projector);
}

void ContactShadowProjector::System::UnregisterProjector(
    absl::string_view target_group) {
  projector_by_target_group_.erase(target_group);
}

ContactShadowProjector::~ContactShadowProjector() {
  GetView().DestroyNode(camera_quads_root_);
}

const std::string& ContactShadowProjector::GetShadowTexture() {
  return final_pass_texture_name_;
}

float ContactShadowProjector::GetBlurFactor() { return blur_factor_; }
void ContactShadowProjector::SetBlurFactor(float blur_factor) {
  blur_factor_ = std::clamp(blur_factor, 0.0f, 1.0f);
  for (const auto& shadow_pass : shadow_passes_) {
    shadow_pass->GetMaterial()->SetParameter("sizeNormalized", blur_factor_);
  }
}

void ContactShadowProjector::Cleanup() {
  ContactShadowProjector::System& projector_system =
      GetView()
          .GetComponentManager()
          .GetComponentSystem<ContactShadowProjector>();
  projector_system.UnregisterProjector(state_.target_group);
}

void ContactShadowProjector::Update() { Refresh(); }

Future<absl::Status> ContactShadowProjector::Setup() {
  if (state_.target_group.empty()) {
    return Future<absl::Status>(absl::FailedPreconditionError(
        "Unable to create contact shadow projector without a target group."));
  }
  return SetupProjector();
}

Future<absl::Status> ContactShadowProjector::Setup(
    absl::string_view target_group) {
  state_.target_group = std::string(target_group);
  return SetupProjector();
}

void ContactShadowProjector::OnActiveStatusChanged(bool is_active) {
  if (camera_quads_root_) {
    camera_quads_root_->SetEnabled(is_active);
    Refresh();
  }
}

void ContactShadowProjector::Refresh() {
  bool is_static = this->state_.is_static.value_or(kIsStaticDefault);
  if (!is_static) return;
  // Ensure the components are present, in case the refresh got called
  // during shutdown or destruction of the node.
  if (camera_) camera_.Get()->SetEnabled(true);
  if (pass_renderer_) pass_renderer_->SetEnabled(true);

  if (IsActive() &&
      !FilterNodesViewableByCamera(camera_, state_.target_group).empty()) {
    MoveIntoView(camera_, state_.target_group).IgnoreError();
  }
}

Future<absl::Status> ContactShadowProjector::SetupProjector() {
  ContactShadowProjector::System& projector_system =
      GetView()
          .GetComponentManager()
          .GetComponentSystem<ContactShadowProjector>();

  if (projector_system.HasProjector(state_.target_group)) {
    return Future<absl::Status>(absl::FailedPreconditionError(
        "Projector already exists for the specified target group."));
  }
  projector_system.RegisterProjector(GetHandle(this), state_.target_group);

  const int shadow_texture_size =
      state_.texture_size.value_or(kDefaultShadowTextureSize);

  blur_factor_ = state_.blur_factor.value_or(kDefaultBlurFactor);

  const float projection_size = state_.receiver_size.value_or(kDefaultSize);

  final_pass_texture_name_ =
      absl::StrFormat("ContactShadows_%s_ColorTexture", state_.target_group);

  CameraState::ProjectionType projection_type =
      state_.projection_type.value_or(kDefaultProjectionType);
  CameraState camera_state{.projection_type = projection_type,
                           .orthographic_scale = projection_size,
                           .locked_aspect_ratio = 1};
  camera_ = GetNode()->AddComponentWithState<CameraComponent>(camera_state);

  v_blur_pass_name_ =
      absl::StrFormat("ContactShadows_%s_VBlur", state_.target_group);
  h_blur_pass_name_ =
      absl::StrFormat("ContactShadows_%s_HBlur", state_.target_group);

  TexturePipelineRendererState pipeline_state;

  // Have the camera capture the target group.
  // The camera will swap the material with the override_material
  // and render everything to a simple white on transparent texture.
  // The name of the texture is 'kVBlurPass.'
  pipeline_state.passes.push_back(TexturePipelineRendererState::Pass{
      .group = std::string(state_.target_group),
      .camera = ComponentSceneHandle<CameraComponent>(
          std::string(GetNode()->GetName()), camera_),
      .color_texture_config =
          TexturePipelineRendererState::Texture{
              .name = v_blur_pass_name_,
              .format = TexturePipelineRendererState::Texture::R8},
      .override_material =
          MaterialDefinition{
              .asset = std::string(
                  contact_shadow_assets::kSimpleWhiteCmat.GetUrl())},
  });

  // Have the camera capture a quad with a vertically blurred pass on it.
  // This capture will be used for the next pass, which will blur horizontally.
  pipeline_state.passes.push_back(TexturePipelineRendererState::Pass{
      .group = v_blur_pass_name_,
      .camera = ComponentSceneHandle<CameraComponent>(
          std::string(GetNode()->GetName()), camera_),
      .color_texture_config =
          TexturePipelineRendererState::Texture{
              .name = h_blur_pass_name_,
              .format = TexturePipelineRendererState::Texture::R8},
  });

  // Have the camera capture a quad with the full blur.
  // This capture texture can then be passed onto any receivers.
  pipeline_state.passes.push_back(TexturePipelineRendererState::Pass{
      .group = h_blur_pass_name_,
      .camera = ComponentSceneHandle<CameraComponent>(
          std::string(GetNode()->GetName()), camera_),
      .color_texture_config =
          TexturePipelineRendererState::Texture{
              .name = final_pass_texture_name_,
              .format = TexturePipelineRendererState::Texture::RGB8},
      .texture_size = uint2{shadow_texture_size, shadow_texture_size},
  });

  // Load the texture pipeline renderer and the blur material.
  Future<ComponentHandle<TexturePipelineRenderer>>
      texture_pipeline_renderer_future =
          GetNode()->AddComponentWithState<TexturePipelineRenderer>(
              pipeline_state);
  auto blur_material_future = GetView().GetAssetManager().LoadMaterial(
      contact_shadow_assets::kSingleAxisBlurCmat);

  return texture_pipeline_renderer_future.Merge(blur_material_future)
      .Then([this](std::tuple<ComponentHandle<TexturePipelineRenderer>,
                              AssetPtr<MaterialAsset>>
                       result) {
        AssetPtr<MaterialAsset> blur_material;
        std::tie(pass_renderer_, blur_material) = result;

        // We want to turn off the pipeline renderer and camera
        // if this projector is static.
        Connect(
            [this](const imp::TexturePipelineRenderer::PostRenderEvent& event) {
              if (event.GetOriginatingNode() != this->GetNode()) return;
              bool is_static =
                  this->state_.is_static.value_or(kIsStaticDefault);
              // If the projector is static,
              // the components should not be enabled.
              bool is_enabled = !is_static;
              pass_renderer_->SetEnabled(is_enabled);
              camera_->SetEnabled(is_enabled);
            },
            this);

        shadow_passes_.push_back(DrawRenderPassToCameraQuad(
            std::move(v_blur_pass_name_), {0, 1}, blur_material));
        shadow_passes_.push_back(DrawRenderPassToCameraQuad(
            std::move(h_blur_pass_name_), {1, 0}, blur_material));

        return absl::OkStatus();
      });
}

ComponentHandle<MeshRenderer>
ContactShadowProjector::DrawRenderPassToCameraQuad(
    std::string blur_pass_name, float2 blur_axis,
    AssetPtr<MaterialAsset> blur_material) {
  if (!camera_quads_root_) {
    camera_quads_root_ = GetView().CreateNode();
    camera_quads_root_->SetParent(this->GetNode());
  }

  NodeHandle quad_node = GetView().CreateNode();
  quad_node->SetLocalPosition(kQuadPassDisplacement);
  quad_node->SetName(blur_pass_name);
  quad_node->SetParent(camera_quads_root_);
  quad_node->SetGroups({blur_pass_name});

  ComponentHandle<MeshRenderer> renderer =
      quad_node->GetOrAddComponent<MeshRenderer>(
          MeshRenderer::FrustumCullingMode::kDisabled);
  renderer->SetShadowCastingMode(MeshRenderer::ShadowMode::kNone);
  renderer->SetShadowReceivingMode(MeshRenderer::ShadowMode::kNone);

  renderer->SetMesh(GetView().GetMeshFactory().CreateQuad());

  MaterialPtr material_instance =
      this->GetView().GetMaterialFactory().CreateMaterial(blur_material);
  Texture* blur_pass_texture =
      GetView().GetTextureRegistry().GetTexture(blur_pass_name);
  if (blur_pass_texture == nullptr) {
    IMP_LOG(imp::ERROR) << blur_pass_name << " not found.";
  }

  material_instance->SetParameter("renderTargetSampler", blur_pass_texture);
  material_instance->SetParameter("sizeNormalized", blur_factor_);
  material_instance->SetParameter("blurAxis", blur_axis);
  renderer->SetMaterial(std::move(material_instance));
  return renderer;
}

}  // namespace imp
