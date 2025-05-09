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

#include "core/render_passes/on_demand_texture_pipeline_renderer.h"

#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/types/optional.h"
#include "filament/filament/include/filament/ColorGrading.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Options.h"
#include "filament/filament/include/filament/RenderTarget.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/Scene.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/Viewport.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/render/texture_registry.h"
#include "core/render_passes/on_demand_texture_pipeline_render_params.proto.imp.h"
#include "core/render_passes/texture_config.proto.imp.h"
#include "core/render_passes/texture_config_utils.h"
#include "core/view/base_view.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/utils/proto/render_settings.proto.imp.h"
#include "core/view/utils/render_setting_utils.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {
OnDemandTexturePipelineRenderer::OnDemandTexturePipelineRenderer(BaseView* view)
    : view_(view), filament_view_(view->CreateFilamentView()) {}

OnDemandTexturePipelineRenderer::~OnDemandTexturePipelineRenderer() {
  view_->DestroyFilamentView(filament_view_);
}

absl::StatusOr<OnDemandTexturePipelineRenderer::RenderResult>
OnDemandTexturePipelineRenderer::Render(
    const OnDemandTexturePipelineRenderParams& params) {
  if (params.passes.empty()) {
    return absl::FailedPreconditionError(
        "OnDemandTexturePipelineRenderer must have at least one pass.");
  }

  std::vector<RuntimePass> runtime_passes;
  runtime_passes.reserve(params.passes.size());
  for (const OnDemandTexturePipelineRenderParams::Pass& pass : params.passes) {
    if (pass.group.empty()) {
      return absl::FailedPreconditionError(
          "OnDemandTexturePipelineRenderer pass must specify a group.");
    }

    absl::StatusOr<RuntimePass> runtime_pass = CreateRuntimePass(pass);
    MP_RETURN_IF_ERROR(runtime_pass.status());
    runtime_passes.push_back(std::move(runtime_pass.value()));
  }

  OnDemandTexturePipelineRenderer::RenderResult result;
  result.textures_per_pass.reserve(runtime_passes.size());
  for (RuntimePass& runtime_pass : runtime_passes) {
    RenderPass(runtime_pass);

    TextureResult texture_result{.color_texture =
                                     std::move(runtime_pass.color_texture)};
    if (runtime_pass.depth_texture_registration.has_value()) {
      texture_result.depth_texture =
          std::move(*runtime_pass.depth_texture_registration);
    }

    result.textures_per_pass.push_back(std::move(texture_result));
  }

  return result;
}

absl::StatusOr<filament::Texture*>
OnDemandTexturePipelineRenderer::SetupColorTexture(
    const OnDemandTexturePipelineRenderParams::Pass& pass,
    RuntimePass& runtime_pass) {
  std::variant<Texture*, TextureRegistry::ScopedTextureRegistration>&
      color_texture_variant = runtime_pass.color_texture;
  filament::Texture* color_filament_texture = nullptr;

  // Case 1: Create and register the color texture with the given color texture
  // name.
  if (pass.color_texture()) {
    uint2 texture_size = pass.render_region_size;
    const TextureConfig& color_texture_proto = *pass.color_texture();

    if (color_texture_proto.name.empty()) {
      return absl::FailedPreconditionError(
          "OnDemandTexturePipelineRenderer color texture must specify a name.");
    }

    filament::Texture::InternalFormat color_format =
        FormatFromTexture(color_texture_proto.format, TextureConfig::RGBA8);

    TexturePtr color_texture = view_->GetTextureFactory().CreateTexture(
        texture_size.x, texture_size.y, color_format,
        filament::Texture::Usage::COLOR_ATTACHMENT |
            filament::Texture::Usage::SAMPLEABLE);

    color_texture_variant = view_->GetTextureRegistry().RegisterTexture(
        color_texture_proto.name, std::move(color_texture));
    color_filament_texture =
        std::get<TextureRegistry::ScopedTextureRegistration>(
            color_texture_variant)
            .GetTexture()
            ->GetTexture();
    // Case 2: Use the pre-registered color texture of the given name.
  } else if (pass.registered_color_texture()) {
    Texture* color_texture = view_->GetTextureRegistry().GetTexture(
        *pass.registered_color_texture());
    if (color_texture == nullptr) {
      return absl::InternalError(absl::StrFormat(
          "Failed to fetch texture from TextureRegistry with the name %s",
          *pass.registered_color_texture()));
    }

    color_texture_variant = color_texture;
    color_filament_texture = color_texture->GetTexture();
    // TODO: Ideally we should also check if this texture has
    // COLOR_ATTACHMENT and SAMPLABLE usage flags or otherwise it will crash.
  } else {
    return absl::FailedPreconditionError(
        "OnDemandTexturePipelineRenderer must specify either a new color "
        "texture or a registered texture.");
  }
  return color_filament_texture;
}

absl::StatusOr<OnDemandTexturePipelineRenderer::RuntimePass>
OnDemandTexturePipelineRenderer::CreateRuntimePass(
    const OnDemandTexturePipelineRenderParams::Pass& pass) {
  RuntimePass runtime_pass;
  runtime_pass.group = pass.group;
  runtime_pass.camera =
      pass.camera ? pass.camera->GetCamera()
                  : view_->GetCameraManager().GetCamera()->GetCamera();
  runtime_pass.use_main_view_settings = pass.use_main_view_settings;
  runtime_pass.render_settings = pass.render_settings;

  uint2 render_region_size = pass.render_region_size;

  // Setup the color texture.
  absl::StatusOr<filament::Texture*> color_filament_texture =
      SetupColorTexture(pass, runtime_pass);
  MP_RETURN_IF_ERROR(color_filament_texture.status());

  uint2 color_texture_size = {color_filament_texture.value()->getWidth(),
                              color_filament_texture.value()->getHeight()};

  // Setup the depth texture, if it exists.
  absl::optional<TextureRegistry::ScopedTextureRegistration>&
      depth_texture_registration = runtime_pass.depth_texture_registration;
  if (pass.depth_texture.has_value()) {
    const TextureConfig& texture_proto = *pass.depth_texture;

    if (texture_proto.name.empty()) {
      return absl::FailedPreconditionError(
          "OnDemandTexturePipelineRenderer depth texture must specify a name.");
    }

    filament::Texture::InternalFormat depth_format =
        FormatFromTexture(texture_proto.format, TextureConfig::DEPTH24);

    TexturePtr texture = view_->GetTextureFactory().CreateTexture(
        render_region_size.x, render_region_size.y, depth_format,
        filament::Texture::Usage::DEPTH_ATTACHMENT |
            filament::Texture::Usage::SAMPLEABLE,
        {.mag_filter = TextureFactory::MagFilter::NEAREST,
         .min_filter = TextureFactory::MinFilter::NEAREST});

    depth_texture_registration.emplace(
        view_->GetTextureRegistry().RegisterTexture(texture_proto.name,
                                                    std::move(texture)));
  }

  // Setup viewport.
  int2 view_port_left_bottom = pass.view_port_left_bottom.value_or(uint2{0, 0});
  filament::Viewport view_port = {view_port_left_bottom.x,
                                  view_port_left_bottom.y, render_region_size.x,
                                  render_region_size.y};
  runtime_pass.view_port = view_port;

  bool is_viewport_at_left_bottom =
      runtime_pass.view_port.left == 0 && runtime_pass.view_port.bottom == 0;
  bool is_viewport_at_full_size =
      runtime_pass.view_port.width == color_texture_size.x &&
      runtime_pass.view_port.height == color_texture_size.y;

  runtime_pass.rendering_to_subregion =
      !(is_viewport_at_left_bottom && is_viewport_at_full_size);

  // Create the render target.
  filament::RenderTarget::Builder render_target_builder;
  render_target_builder.texture(filament::RenderTarget::AttachmentPoint::COLOR,
                                color_filament_texture.value());
  if (pass.color_texture_layer.has_value()) {
    if (color_filament_texture.value()->getDepth() == 1) {
      return absl::FailedPreconditionError(
          "Color texture must have at least 2 layers to support layer "
          "selection.");
    }
    if (pass.color_texture_layer.value() >=
        color_filament_texture.value()->getDepth()) {
      return absl::FailedPreconditionError(
          absl::StrFormat("Color texture layer index %d is out of bounds. "
                          "Texture has %d layers.",
                          pass.color_texture_layer.value(),
                          color_filament_texture.value()->getDepth()));
    }
    render_target_builder.layer(filament::RenderTarget::AttachmentPoint::COLOR,
                                pass.color_texture_layer.value());
  }
  if (depth_texture_registration.has_value()) {
    render_target_builder.texture(
        filament::RenderTarget::AttachmentPoint::DEPTH,
        depth_texture_registration->GetTexture()->GetTexture());
  }
  runtime_pass.render_target =
      render_target_builder.build(*view_->GetSharedEngine());

  return runtime_pass;
}

void OnDemandTexturePipelineRenderer::RenderPass(RuntimePass& runtime_pass) {
  // Find the scene for the group this pass renders.
  filament::Scene* scene =
      view_->GetGroupsManager().GetScene(runtime_pass.group);

  // If there is no scene, return early without rendering. This is not an error,
  // it means that there are no nodes in the group for this pass, which could be
  // valid.
  if (!scene) {
    IMP_LOG(imp::INFO) << "No scene for group " << runtime_pass.group;
    return;
  }
  filament_view_->setScene(scene);

  filament::Engine* engine = view_->GetSharedEngine();

  // Retrieve the render settings of the view before override.
  const render_settings::ViewRenderSettings view_render_settings =
      GetViewRenderSettings(filament_view_);
  // Override render settings if necessary.
  if (!runtime_pass.use_main_view_settings &&
      runtime_pass.render_settings.has_value()) {
    OverrideViewRenderSettings(filament_view_, &(*runtime_pass.render_settings),
                               engine);
  }

  if (runtime_pass.use_main_view_settings) {
    render_settings::ViewRenderSettings* settings =
        runtime_pass.render_settings.has_value()
            ? &(*runtime_pass.render_settings)
            : nullptr;
    ConfigureViewRenderSettingsWithOverrides(
        render_settings::OverrideMode::OVERRIDE_MODE_OVERRIDE_FROM_SOURCE,
        filament_view_, view_->GetHost()->GetView(), settings, engine);
  }

  filament_view_->setCamera(runtime_pass.camera);
  filament_view_->setViewport(runtime_pass.view_port);
  engine->destroy(filament_view_->getRenderTarget());
  filament_view_->setRenderTarget(runtime_pass.render_target);

  filament::Renderer* renderer = view_->GetHost()->GetRenderer();
  const filament::Renderer::ClearOptions clear_options =
      renderer->getClearOptions();
  if (runtime_pass.rendering_to_subregion) {
    // If this is only rendering to a subregion of the texture, clearing will
    // be disabled.
    // TODO: Add support for manually specifying whether to clear
    // or not instead of always clearing when rendering to subregions.
    renderer->setClearOptions(
        filament::Renderer::ClearOptions{.clear = false, .discard = false});
  }
  renderer->renderStandaloneView(filament_view_);

  // Restore filament view before rendering so that it doesn't affect the next
  // render pass.
  renderer->setClearOptions(clear_options);
  engine->destroy(filament_view_->getRenderTarget());
  filament_view_->setRenderTarget(nullptr);
  // Restore render settings.
  OverrideViewRenderSettings(filament_view_, &view_render_settings, engine);
}

}  // namespace imp
