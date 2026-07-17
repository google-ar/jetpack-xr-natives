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

#include "core/render_passes/texture_pipeline_renderer.h"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/types/optional.h"
#include "absl/types/variant.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Camera.h"
#if defined(__ANDROID__)
#include "filament/filament/include/filament/Fence.h"
#endif  // defined(__ANDROID__)
#include "filament/filament/include/filament/MaterialInstance.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/Scene.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/Viewport.h"
#include "core/async/future.h"
#include "core/camera/camera_component.h"
#include "core/common/small_source_location.h"
#include "core/materials/material.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/component_system.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/render/texture_options.h"
#include "core/render_passes/texture_pipeline_renderer_projection_quad.h"
#include "core/render_passes/texture_pipeline_renderer_state.proto.imp.h"
#include "core/scene_handles/scene_handles.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/utils/device.h"
#include "core/view/utils/proto/render_settings.proto.imp.h"
#include "core/view/utils/render_setting_utils.h"
#include "core/view/view_events.h"
#include "core/window/filament_host.h"

namespace imp {

namespace {

filament::Texture::InternalFormat FormatFromTexture(
    TexturePipelineRendererState::Texture::Format format,
    TexturePipelineRendererState::Texture::Format auto_format) {
  if (format == TexturePipelineRendererState::Texture::AUTO) {
    format = auto_format;
  }

  switch (format) {
    case TexturePipelineRendererState::Texture::RGBA8:
      return filament::Texture::InternalFormat::RGBA8;
    case TexturePipelineRendererState::Texture::RGB8:
      return filament::Texture::InternalFormat::RGB8;
    case TexturePipelineRendererState::Texture::RG8:
      return filament::Texture::InternalFormat::RG8;
    case TexturePipelineRendererState::Texture::R8:
      return filament::Texture::InternalFormat::R8;
    case TexturePipelineRendererState::Texture::RGBA32F:
      return filament::Texture::InternalFormat::RGBA32F;
    case TexturePipelineRendererState::Texture::RGB32F:
      return filament::Texture::InternalFormat::RGB32F;
    case TexturePipelineRendererState::Texture::RG32F:
      return filament::Texture::InternalFormat::RG32F;
    case TexturePipelineRendererState::Texture::R32F:
      return filament::Texture::InternalFormat::R32F;
    case TexturePipelineRendererState::Texture::RGBA16F:
      return filament::Texture::InternalFormat::RGBA16F;
    case TexturePipelineRendererState::Texture::RGB16F:
      return filament::Texture::InternalFormat::RGB16F;
    case TexturePipelineRendererState::Texture::RG16F:
      return filament::Texture::InternalFormat::RG16F;
    case TexturePipelineRendererState::Texture::R16F:
      return filament::Texture::InternalFormat::R16F;
    case TexturePipelineRendererState::Texture::DEPTH24:
      return filament::Texture::InternalFormat::DEPTH24;
    case TexturePipelineRendererState::Texture::DEPTH32F:
      return filament::Texture::InternalFormat::DEPTH32F;
    case TexturePipelineRendererState::Texture::R8UI:
      return filament::Texture::InternalFormat::R8UI;
    case TexturePipelineRendererState::Texture::RG8UI:
      return filament::Texture::InternalFormat::RG8UI;
    case TexturePipelineRendererState::Texture::RGB8UI:
      return filament::Texture::InternalFormat::RGB8UI;
    case TexturePipelineRendererState::Texture::RGBA8UI:
      return filament::Texture::InternalFormat::RGBA8UI;
    case TexturePipelineRendererState::Texture::R16UI:
      return filament::Texture::InternalFormat::R16UI;
    case TexturePipelineRendererState::Texture::RG16UI:
      return filament::Texture::InternalFormat::RG16UI;
    case TexturePipelineRendererState::Texture::RGB16UI:
      return filament::Texture::InternalFormat::RGB16UI;
    case TexturePipelineRendererState::Texture::RGBA16UI:
      return filament::Texture::InternalFormat::RGBA16UI;
    case TexturePipelineRendererState::Texture::R32UI:
      return filament::Texture::InternalFormat::R32UI;
    case TexturePipelineRendererState::Texture::RG32UI:
      return filament::Texture::InternalFormat::RG32UI;
    case TexturePipelineRendererState::Texture::RGB32UI:
      return filament::Texture::InternalFormat::RGB32UI;
    case TexturePipelineRendererState::Texture::RGBA32UI:
      return filament::Texture::InternalFormat::RGBA32UI;
    default:
      // This should never happen.
      IMP_LOG(imp::FATAL) << "Unsupported texture format in TexturePipelineRenderer.";
      return filament::Texture::InternalFormat::RGBA8;
  }
}

uint2 TextureSizeFromPass(const TexturePipelineRendererState::Pass& pass,
                          BaseView& view,
                          filament::Texture* texture = nullptr) {
  return absl::visit(
      [&view, &pass, texture](const auto& size) -> uint2 {
        using TextureSizeType = std::decay_t<decltype(size)>;
        if constexpr (std::is_same_v<TextureSizeType, uint2>) {
          return size;
        } else if constexpr (std::is_same_v<TextureSizeType,
                                            TexturePipelineRendererState::
                                                AutomaticTextureSize>) {
          switch (size.mode) {
            case TexturePipelineRendererState::AutomaticTextureSize::
                DEFAULT_VIEW_SIZE_VIRTUAL_PIXELS:
              return view.GetSize() * size.factor.value_or(1.0f);
            case TexturePipelineRendererState::AutomaticTextureSize::
                VIEW_SIZE_PHYSICAL_PIXELS:
              const Device& device = view.GetDevice();
              const uint2 view_size = view.GetSize();
              const float factor = size.factor.value_or(1.0f);

              if (!device.IsPhysicalPixelRatioAvailable()) {
                return view_size * factor;
              }

              return device.PixelsToPhysicalPixels(view_size) * factor;
          }
        } else {
          if (texture && pass.registered_color_texture()) {
            return {texture->getWidth(), texture->getHeight()};
          } else {
            return view.GetSize();
          }
        }
      },
      pass.texture_size);
}

// Returns true if the pass textures need to be resized when the viewport size
// changes.
bool RequiresViewSizeChangedEvent(
    const TexturePipelineRendererState::Pass& pass) {
  return pass.automatic_size() && pass.automatic_size()->resize_with_view;
}

bool RequiresPhysicalPixelsRatio(
    const TexturePipelineRendererState::Pass& pass) {
  return pass.automatic_size() &&
         pass.automatic_size()->mode ==
             TexturePipelineRendererState::AutomaticTextureSize::
                 VIEW_SIZE_PHYSICAL_PIXELS;
}

}  // namespace

Future<absl::Status> TexturePipelineRenderer::Setup() {
  Future<absl::Status> result(absl::OkStatus());

  if (state_.passes.empty()) {
    return Future<absl::Status>(absl::FailedPreconditionError(
        "TexturePipelineRenderer must have at least one pass."));
  }

  if (split_engine::SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    // Serialize the TexturePipelineRenderer state.
    serializer->AddTexturePipelineRenderer(GetNode().GetEntity(), state_);

    // Register any output textures that don't exist yet with a placeholder
    // texture. Register those textures in the app-side texture registry so the
    // corresponding texture that the backend TexturePipelineRenderer creates
    // can be linked to it. This allows the app to use the texture from the
    // registry to assign to material parameters and they will be resolved by
    // the backend to the texture rendered to by the backend TPR mirror.
    for (size_t i = 0; i < state_.passes.size(); ++i) {
      RuntimePass& runtime_pass = runtime_passes_.emplace_back();
      const auto& pass = state_.passes[i];
      if (std::holds_alternative<imp::TexturePipelineRendererState::Texture>(
              pass.color_texture_config)) {
        const auto& tex = std::get<imp::TexturePipelineRendererState::Texture>(
            pass.color_texture_config);
        const std::string& name = tex.name;
        if (!GetView().GetTextureRegistry().GetTexture(name)) {
          // Create a 1x1 placeholder texture.
          // Note: manually creating a texture with filament::TextureBuilder to
          // avoid automatic Split Engine texture serialization.
          filament::Texture* filament_texture =
              filament::Texture::Builder()
                  .width(1)
                  .height(1)
                  .sampler(filament::Texture::Sampler::SAMPLER_2D)
                  .format(filament::Texture::InternalFormat::RGBA8)
                  .build(*BaseView::GetSharedEngine());
          OwnedTexturePtr texture =
              GetView().GetTextureFactory().WrapTexture(filament_texture);
          runtime_pass.color_texture_registration =
              GetView().GetTextureRegistry().RegisterTexture(
                  name, std::move(texture));
        }
      }
    }
    // Return early, do not run any real TexturePipelineRenderer logic on the
    // Split Engine app side.
    return result;
  }

  // Setup runtime information for each pass.
  filament::Engine* engine = BaseView::GetSharedEngine();
  bool requires_view_size_changed_event = false;
  bool any_pass_requires_physical_pixels_ratio = false;

  for (TexturePipelineRendererState::Pass& pass : state_.passes) {
    if (pass.group.empty()) {
      return Future<absl::Status>(absl::FailedPreconditionError(
          "TexturePipelineRenderer pass must specify a group."));
    }
    requires_view_size_changed_event |= RequiresViewSizeChangedEvent(pass);
    bool requires_physical_pixels_ratio = RequiresPhysicalPixelsRatio(pass);
    any_pass_requires_physical_pixels_ratio |= requires_physical_pixels_ratio;

    RuntimePass& runtime_pass = runtime_passes_.emplace_back();
    runtime_pass.view = GetView().CreateFilamentView();
    runtime_pass.view->setName(
        ("TexturePipelineRenderer_" + pass.group).c_str());
    runtime_pass.view->setCamera(
        pass.camera ? pass.camera->GetCamera()
                    : GetView().GetCameraManager().GetCamera()->GetCamera());
    runtime_pass.use_main_view_settings = pass.use_main_view_settings;
    runtime_pass.use_main_view_camera_projection_matrix =
        pass.use_main_view_camera_projection_matrix.value_or(true);
    runtime_pass.render_settings = pass.render_settings;

    if (!runtime_pass.use_main_view_settings &&
        pass.render_settings.has_value()) {
      OverrideViewRenderSettings(runtime_pass.view, &(*pass.render_settings),
                                 engine);
    }

    absl::Status initialized = InitializeTextures(pass, runtime_pass);
    if (!initialized.ok()) {
      return Future<absl::Status>(initialized);
    }

    // Setup the override material, if one is specified for this pass.
    if (pass.override_material.has_value()) {
      int pass_index = runtime_passes_.size() - 1;
      result = result.Combine(
          GetView()
              .GetMaterialFactory()
              .LoadMaterial(*pass.override_material)
              .Then([this, pass_index](OwnedMaterialPtr material) mutable {
                runtime_passes_.at(pass_index).override_material =
                    std::move(material);
              }));
    }
  }

  if (requires_view_size_changed_event ||
      (any_pass_requires_physical_pixels_ratio &&
       !GetView().GetDevice().IsPhysicalPixelRatioAvailable())) {
    GetView().GetDispatcher().Connect(
        [this](const ViewSizeChangedEvent& size_changed_event) {
          // Resize the textures even if the component isn't active/enabled so
          // that the texture sizes are up to date even as the component comes
          // out of being inactive.
          bool resized = false;
          for (int pass_index = 0; pass_index < state_.passes.size();
               ++pass_index) {
            const TexturePipelineRendererState::Pass& pass =
                state_.passes[pass_index];
            RuntimePass& runtime_pass = runtime_passes_.at(pass_index);

            if (RequiresViewSizeChangedEvent(pass) ||
                IsRenderTargetStale(pass, runtime_pass) ||
                !runtime_pass.render_target) {
              resized = true;
              absl::Status status = InitializeTextures(pass, runtime_pass);
              if (!status.ok()) {
                IMP_LOG(imp::FATAL) << "Texture creation is expected to succeed after "
                              "the initial setup. Error: "
                           << status;
              }
            }
          }
          if (resized) {
            GetNode()->Send(PostResizeEvent());
          }
        },
        this);

    GetView().GetDispatcher().Connect(
        [this](const PostResizeEvent& post_resize_event) {
          textures_marked_for_deletion_.clear();
        },
        this);
  }

  return result;
}

bool TexturePipelineRenderer::IsRenderTargetStale(
    const TexturePipelineRendererState::Pass& pass,
    const RuntimePass& runtime_pass) const {
  // If we own this texture, i.e. it is not owned by the texture registry, then
  // it can never be stale.
  if (!pass.registered_color_texture()) {
    return false;
  }
  if (!runtime_pass.color_texture_registration) {
    return true;
  }
  // If the target of this color pass texture has been recreated in the registry
  // elsewhere since we last built our RenderTarget, we have to rebuild it.
  return runtime_pass.color_texture_registration->GetId() !=
         GetView().GetTextureRegistry().GetId(*pass.registered_color_texture());
}

void TexturePipelineRenderer::SetPassEnabled(size_t pass_index, bool enabled) {
  if (pass_index >= runtime_passes_.size()) {
    IMP_LOG(imp::ERROR) << "Pass index out of bounds: " << pass_index;
    return;
  }
  runtime_passes_[pass_index].enabled = enabled;
  if (split_engine::SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    std::vector<bool> enabled_passes(runtime_passes_.size());
    for (size_t i = 0; i < runtime_passes_.size(); ++i) {
      enabled_passes[i] = runtime_passes_[i].enabled;
    }
    serializer->SetTexturePipelineRendererPassesEnabled(GetNode().GetEntity(),
                                                        enabled_passes);
  }
}

bool TexturePipelineRenderer::IsPassEnabled(size_t pass_index) const {
  if (pass_index >= runtime_passes_.size()) {
    IMP_LOG(imp::ERROR) << "Pass index out of bounds: " << pass_index;
    return false;
  }
  return runtime_passes_[pass_index].enabled;
}

absl::Status TexturePipelineRenderer::ResizePassTexture(size_t pass_index,
                                                        imp::uint2 size) {
  if (pass_index >= runtime_passes_.size()) {
    return absl::InvalidArgumentError("Pass index out of bounds");
  }
  TexturePipelineRendererState::Pass& pass = state_.passes[pass_index];

  if (split_engine::SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    // TODO: (broken link) - Implement texture resize in Split Engine.
    IMP_LOG(imp::FATAL) << "ResizePassTexture is not supported in Split Engine.";
  }

  RuntimePass& runtime_pass = runtime_passes_.at(pass_index);
  pass.texture_size = size;
  absl::Status status = InitializeTextures(pass, runtime_pass);
  if (!status.ok()) {
    return status;
  }
  return absl::OkStatus();
}

absl::Status TexturePipelineRenderer::SetPassCamera(
    size_t pass_index, ComponentHandle<CameraComponent> camera) {
  if (pass_index >= runtime_passes_.size()) {
    return absl::InvalidArgumentError("Pass index out of bounds");
  }
  if (split_engine::SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    IMP_LOG(imp::FATAL) << "SetPassCamera is not supported in Split Engine.";
  }

  state_.passes[pass_index].camera =
      camera ? ComponentSceneHandle<CameraComponent>(camera)
             : ComponentSceneHandle<CameraComponent>();
  filament::Camera* filament_camera =
      camera ? camera->GetCamera()
             : GetView().GetCameraManager().GetCamera()->GetCamera();
  runtime_passes_[pass_index].view->setCamera(filament_camera);
  return absl::OkStatus();
}

filament::View* TexturePipelineRenderer::GetFilamentView(
    size_t pass_index) const {
  if (pass_index >= runtime_passes_.size()) {
    IMP_LOG(imp::ERROR) << "Pass index out of bounds: " << pass_index;
    return nullptr;
  }
  return runtime_passes_[pass_index].view;
}

void TexturePipelineRenderer::Cleanup() {
  if (split_engine::SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    serializer->RemoveTexturePipelineRenderer(GetNode().GetEntity());
  }

  filament::Engine* engine = BaseView::GetSharedEngine();

  // Destroy filament resources.
  // Textures will be destroyed automatically when the ScopedTextureRegistration
  // falls out of scope.
  for (RuntimePass& runtime_pass : runtime_passes_) {
    engine->destroy(runtime_pass.render_target);
    GetView().DestroyFilamentView(runtime_pass.view);
  }
}

absl::Status TexturePipelineRenderer::InitializeTextures(
    const TexturePipelineRendererState::Pass& pass, RuntimePass& runtime_pass) {
  filament::Engine* engine = BaseView::GetSharedEngine();
  uint2 texture_size = TextureSizeFromPass(pass, GetView());

  // Mark old textures for deletion.
  absl::optional<TextureRegistry::ScopedTextureRegistration>&
      color_texture_registration = runtime_pass.color_texture_registration;
  if (color_texture_registration.has_value()) {
    textures_marked_for_deletion_.push_back(
        color_texture_registration->Release());
    color_texture_registration = absl::nullopt;
  }
  absl::optional<TextureRegistry::ScopedTextureRegistration>&
      depth_texture_registration = runtime_pass.depth_texture_registration;
  if (depth_texture_registration.has_value()) {
    textures_marked_for_deletion_.push_back(
        depth_texture_registration->Release());
    depth_texture_registration = absl::nullopt;
  }

  const filament::Engine::Config& config =
      GetView().GetHost()->GetEngine()->getConfig();
  // For now, multiview is only supported for main view and passes that use
  // main view settings.
  bool is_multiview = runtime_pass.use_main_view_settings &&
                      (config.stereoscopicType ==
                       filament::backend::StereoscopicType::MULTIVIEW);
  size_t stereo_depth = config.stereoscopicEyeCount;

  // Setup the color texture.
  filament::Texture* color_filament_texture = nullptr;
  if (pass.color_texture()) {
    const TexturePipelineRendererState::Texture& color_texture_proto =
        *pass.color_texture();

    filament::Texture::InternalFormat color_format =
        FormatFromTexture(color_texture_proto.format,
                          TexturePipelineRendererState::Texture::RGBA8);

    OwnedTexturePtr color_texture;
    TextureSamplerOptions color_sampler_options;
    if (filament::backend::isUnsignedIntFormat(color_format) ||
        filament::backend::isSignedIntFormat(color_format)) {
      color_sampler_options.min_filter =
          TextureSamplerOptions::MinFilter::NEAREST;
      color_sampler_options.mag_filter =
          TextureSamplerOptions::MagFilter::NEAREST;
    }

    if (is_multiview) {
      color_sampler_options.sampler_type =
          TextureSamplerOptions::SamplerType::SAMPLER_2D_ARRAY;
      TextureFactory::TextureCreationSettings settings = {
          .width = texture_size.x,
          .height = texture_size.y,
          .format = color_format,
          .depth = stereo_depth,
          .levels = 1,
          .usage = filament::Texture::Usage::COLOR_ATTACHMENT |
                   filament::Texture::Usage::SAMPLEABLE,
          .sampler_options = color_sampler_options,
      };
      color_texture = GetView().GetTextureFactory().CreateTexture(settings);
    } else {
      color_texture = GetView().GetTextureFactory().CreateTexture(
          imp::TextureFactory::TextureCreationSettings{
              .width = texture_size.x,
              .height = texture_size.y,
              .format = color_format,
              .usage = filament::Texture::Usage::COLOR_ATTACHMENT |
                       filament::Texture::Usage::SAMPLEABLE,
              .sampler_options = color_sampler_options,
          });
    }

    if (!color_texture) {
      return absl::InternalError("Failed to create color texture.");
    }

    if (color_texture_proto.name.empty()) {
      return absl::FailedPreconditionError(
          "TexturePipelineRenderer color texture must specify a name.");
    }

    color_texture->SetName(color_texture_proto.name);
    color_texture_registration.emplace(
        GetView().GetTextureRegistry().RegisterTexture(
            color_texture_proto.name, std::move(color_texture)));
    color_filament_texture =
        color_texture_registration->GetTexture()->GetTexture();
  } else if (pass.registered_color_texture()) {
    Texture* color_texture = GetView().GetTextureRegistry().GetTexture(
        *pass.registered_color_texture());
    if (color_texture == nullptr) {
      return absl::InternalError(absl::StrFormat(
          "Failed to fetch texture from TextureRegistry with the name %s",
          *pass.registered_color_texture()));
    }

    color_filament_texture = color_texture->GetTexture();
    texture_size = TextureSizeFromPass(pass, GetView(), color_filament_texture);
    // TODO: Ideally we should also check if this texture has
    // COLOR_ATTACHMENT and SAMPLABLE usage flags or otherwise it will crash.
  } else {
    return absl::FailedPreconditionError(
        "TexturePipelineRenderer must specify either a new color texture or "
        "a registered texture.");
  }

  // Setup the depth texture, if it exists.
  if (pass.depth_texture.has_value()) {
    const TexturePipelineRendererState::Texture& texture_proto =
        *pass.depth_texture;

    filament::Texture::InternalFormat depth_format = FormatFromTexture(
        texture_proto.format, TexturePipelineRendererState::Texture::DEPTH24);

    OwnedTexturePtr depth_texture;
    if (is_multiview) {
      TextureSamplerOptions sampler_options = {
          .sampler_type = TextureSamplerOptions::SamplerType::SAMPLER_2D_ARRAY,
          .mag_filter = TextureSamplerOptions::MagFilter::NEAREST,
          .min_filter = TextureSamplerOptions::MinFilter::NEAREST,
      };
      TextureFactory::TextureCreationSettings settings = {
          .width = texture_size.x,
          .height = texture_size.y,
          .format = depth_format,
          .depth = stereo_depth,
          .levels = 1,
          .usage = filament::Texture::Usage::DEPTH_ATTACHMENT |
                   filament::Texture::Usage::SAMPLEABLE,
          .sampler_options = sampler_options,
      };
      depth_texture = GetView().GetTextureFactory().CreateTexture(settings);
    } else {
      depth_texture = GetView().GetTextureFactory().CreateTexture(
          imp::TextureFactory::TextureCreationSettings{
              .width = texture_size.x,
              .height = texture_size.y,
              .format = depth_format,
              .usage = filament::Texture::Usage::DEPTH_ATTACHMENT |
                       filament::Texture::Usage::SAMPLEABLE,
              .sampler_options =
                  TextureSamplerOptions{
                      .mag_filter = TextureSamplerOptions::MagFilter::NEAREST,
                      .min_filter = TextureSamplerOptions::MinFilter::NEAREST,
                  },
          });
    }

    if (!depth_texture) {
      return absl::InternalError("Failed to create depth texture.");
    }

    if (texture_proto.name.empty()) {
      return absl::FailedPreconditionError(
          "TexturePipelineRenderer depth texture must specify a name.");
    }

    depth_texture->SetName(texture_proto.name);
    depth_texture_registration.emplace(
        GetView().GetTextureRegistry().RegisterTexture(
            texture_proto.name, std::move(depth_texture)));
  }

  // Setup viewport and subregion rendering.
  int2 view_port_left_bottom = pass.view_port_left_bottom.value_or(uint2{0, 0});
  filament::Viewport view_port = {view_port_left_bottom.x,
                                  view_port_left_bottom.y, texture_size.x,
                                  texture_size.y};
  // Verify that the viewport is not covering regions outside of the texture.
  if (view_port.left < 0 || view_port.bottom < 0 ||
      view_port.left + view_port.width > color_filament_texture->getWidth() ||
      view_port.bottom + view_port.height >
          color_filament_texture->getHeight()) {
    return absl::FailedPreconditionError(absl::StrFormat(
        "Subregion {%d, %d, %d, %d} is not within the "
        "texture size {%d, %d}",
        view_port.left, view_port.bottom, view_port.left + view_port.width,
        view_port.bottom + view_port.height, color_filament_texture->getWidth(),
        color_filament_texture->getHeight()));
  }
  runtime_pass.view->setViewport(view_port);
  runtime_pass.rendering_to_subregion =
      runtime_pass.view->getViewport().left != 0 ||
      runtime_pass.view->getViewport().bottom != 0 ||
      runtime_pass.view->getViewport().width !=
          color_filament_texture->getWidth() ||
      runtime_pass.view->getViewport().height !=
          color_filament_texture->getHeight();

  // Create the render target, deleting the old one if any.
  filament::RenderTarget::Builder render_target_builder;
  render_target_builder.texture(filament::RenderTarget::AttachmentPoint::COLOR,
                                color_filament_texture);
  if (pass.color_texture_layer.has_value()) {
    if (color_filament_texture->getDepth() == 1) {
      return absl::FailedPreconditionError(
          "Color texture must have at least 2 layers to support layer "
          "selection.");
    }
    if (pass.color_texture_layer.value() >=
        color_filament_texture->getDepth()) {
      return absl::FailedPreconditionError(
          absl::StrFormat("Color texture layer index %d is out of bounds. "
                          "Texture has %d layers.",
                          pass.color_texture_layer.value(),
                          color_filament_texture->getDepth()));
    }
    render_target_builder.layer(filament::RenderTarget::AttachmentPoint::COLOR,
                                pass.color_texture_layer.value());
  }
  if (is_multiview) {
    assert(color_filament_texture->getDepth() == stereo_depth);
    render_target_builder.multiview(
        filament::RenderTarget::AttachmentPoint::COLOR,
        color_filament_texture->getDepth(), 0);
  }

  if (depth_texture_registration.has_value()) {
    render_target_builder.texture(
        filament::RenderTarget::AttachmentPoint::DEPTH,
        depth_texture_registration->GetTexture()->GetTexture());
    if (is_multiview) {
      assert(
          depth_texture_registration->GetTexture()->GetTexture()->getDepth() ==
          stereo_depth);
      render_target_builder.multiview(
          filament::RenderTarget::AttachmentPoint::DEPTH,
          depth_texture_registration->GetTexture()->GetTexture()->getDepth(),
          0);
    }
  }
  engine->destroy(runtime_pass.render_target);
  runtime_pass.render_target = render_target_builder.build(*engine);
  runtime_pass.view->setRenderTarget(runtime_pass.render_target);
  if (is_multiview) {
    runtime_pass.view->setStereoscopicOptions({.enabled = true});
  }

  return absl::OkStatus();
}

void TexturePipelineRenderer::RenderPasses(
    filament::Renderer* filament_renderer) {
  if (GetView().GetSplitEngineSerializer()) {
    // Return early, do not run any real TexturePipelineRenderer logic on the
    // Split Engine app side.
    return;
  }

  // Loop through each pass and render it.
  assert(state_.passes.size() == runtime_passes_.size());
  for (int i = 0; i < runtime_passes_.size(); i++) {
    TexturePipelineRendererState::Pass& pass = state_.passes.at(i);
    RuntimePass& runtime_pass = runtime_passes_.at(i);
    if (!runtime_pass.enabled) {
      continue;
    }

    if (runtime_pass.use_main_view_settings) {
      render_settings::ViewRenderSettings* settings =
          runtime_pass.render_settings.has_value()
              ? &(*runtime_pass.render_settings)
              : nullptr;
      ConfigureViewRenderSettingsWithOverrides(
          render_settings::OverrideMode::OVERRIDE_MODE_OVERRIDE_FROM_SOURCE,
          runtime_pass.view, GetView().GetHost()->GetView(), settings,
          GetView().GetSharedEngine());
    }

    // Find the scene for the group this pass renders.
    filament::Scene* scene = GetView().GetGroupsManager().GetScene(pass.group);

    // If there is no scene from the group manager, it means the group has no
    // nodes. This is not an error and we still want to render in order to clear
    // the output textures, as otherwise you can run into edge cases where
    // uninitialized texture memory is used later in the rendering process
    // (especially when textures have to be re-created due to a resize event).
    // If users want to avoid rendering in this case, they can disable the pass
    // directly.
    //
    // The renderer needs a non-null scene to render, so we use the shared empty
    // scene if there is no scene for this group.
    if (!scene) {
      scene = GetView()
                  .GetComponentManager()
                  .GetComponentSystem<TexturePipelineRenderer>()
                  .GetEmptyScene();
    }

    runtime_pass.view->setScene(scene);

    // If an override material is set, then swap the materials for every
    // node within the  group that is being rendered.
    BorrowedMaterialPtr mat = {};
    std::vector<filament::MaterialInstance*> temp_mats;
    if (runtime_pass.override_material) {
      mat = runtime_pass.override_material.Borrow();
      GetView().GetGroupsManager().ForEachActiveNodeInGroup(
          pass.group, [mat, &temp_mats](NodeHandle node) {
            auto& rm = BaseView::GetSharedEngine()->getRenderableManager();
            if (rm.hasComponent(node.GetEntity())) {
              auto instance = rm.getInstance(node.GetEntity());
              for (int i = 0; i < rm.getPrimitiveCount(instance); i++) {
                temp_mats.push_back(rm.getMaterialInstanceAt(instance, i));
                rm.setMaterialInstanceAt(instance, i,
                                         mat->GetFilamentMaterialInstance());
              }
            }
          });
    }

    // Copy the current ClearOptions so we can restore them after/if we disable
    // them for subregion rendering.
    const filament::Renderer::ClearOptions clear_options =
        filament_renderer->getClearOptions();
    if (runtime_pass.rendering_to_subregion) {
      // If this is only rendering to a subregion of the texture, clearing will
      // be disabled.
      // TODO: Add support for manually specifying whether to clear
      // or not instead of always clearing when rendering to subregions.
      filament_renderer->setClearOptions(
          filament::Renderer::ClearOptions{.clear = false, .discard = false});
    } else if (pass.clear_color.has_value()) {
      filament_renderer->setClearOptions(
          {.clearColor = pass.clear_color.value(), .clear = true});
    }

    // Send the pre-pass event.
    PrePassEvent pre_pass_event;
    pre_pass_event.pass_index = i;
    GetNode()->Send(pre_pass_event);

    // Render the pass.
    GetView().GetHost()->PerformRender(
        runtime_pass.view, window::FilamentHost::RenderPassOptions{
                               .projection_quad = projection_quad_});

    // Render the pass.

    // Send the post-pass event.
    PostPassEvent post_pass_event;
    post_pass_event.pass_index = i;
    GetNode()->Send(post_pass_event);

    if (runtime_pass.rendering_to_subregion || pass.clear_color.has_value()) {
      filament_renderer->setClearOptions(clear_options);
    }

    // Restore the original materials for all nodes that had their
    // materials swapped.
    if (mat) {
      size_t temp_index = 0;
      GetView().GetGroupsManager().ForEachActiveNodeInGroup(
          pass.group, [&temp_mats, &temp_index](NodeHandle node) {
            auto& rm = BaseView::GetSharedEngine()->getRenderableManager();
            if (rm.hasComponent(node.GetEntity())) {
              auto instance = rm.getInstance(node.GetEntity());
              for (int i = 0; i < rm.getPrimitiveCount(instance); i++) {
                rm.setMaterialInstanceAt(instance, i, temp_mats.at(temp_index));
                temp_index++;
              }
            }
          });
    }
  }

  // Send out the PostRenderEvent.
  GetNode()->Send(PostRenderEvent());
}

TexturePipelineRenderer::System::System(BaseView* view)
    : ComponentSystem<TexturePipelineRenderer>(view) {}

void TexturePipelineRenderer::System::BeforeFirstComponentAdded() {
  empty_scene_ = BaseView::GetSharedEngine()->createScene();
  // Connect to the offscreen render event to render all the passes at the
  // correct point in the lifecycle of a frame.
  GetView().GetDispatcher().Connect(
      [this](const ViewPreRenderEvent& pre_render_event) {
#if defined(__ANDROID__)
        // TODO: (broken link) - first frame check using an app config flag.
        if (GetView()
                .GetConfig()
                .experimental_feature_flags
                ->enable_texture_pipeline_renderer_first_frame_fence.Value()) {
          if (!is_first_frame_rendered_) {
            // Defer TexturePipelineRenderer offscreen rendering until the first
            // main rendering has successfully spawned its GL context. Executing
            // heavy off-screen GL operations on a surfaceless EGL context
            // before the main window completes its first eglSwapBuffers() can
            // crash the graphics driver into an unrecoverable state (resulting
            // in a black screen, (broken link)). We wait for a fence created
            // after the first main frame is issued.
            if (first_frame_fence_) {
              filament::Fence::FenceStatus status =
                  first_frame_fence_->wait(filament::Fence::Mode::FLUSH, 0);
              if (status == filament::Fence::FenceStatus::CONDITION_SATISFIED) {
                is_first_frame_rendered_ = true;
                BaseView::GetSharedEngine()->destroy(first_frame_fence_);
                first_frame_fence_ = nullptr;
                IMP_LOG(imp::INFO)
                    << "[TPR] first frame rendered, now okay to perform TPR";
              } else {
                IMP_LOG(imp::INFO) << "[TPR] fence not ready yet, status: "
                          << (int)status;
              }
            }
            if (!is_first_frame_rendered_) {
              IMP_LOG(imp::INFO) << "[TPR] skipping render";
              return;
            }
          }
        }
#endif  // defined(__ANDROID__)
        RunTexturePipelines(pre_render_event.GetRenderer());
      },
      this);

#if defined(__ANDROID__)
  // TODO: (broken link) - first frame check using an app config flag.
  if (GetView()
          .GetConfig()
          .experimental_feature_flags
          ->enable_texture_pipeline_renderer_first_frame_fence.Value()) {
    // Create a fence after the first frame is rendered to ensure that offscreen
    // rendering in `RunTexturePipelines` only starts after the main view's
    // initial rendering setup is complete.
    GetView().GetDispatcher().Connect(
        [this](const ViewPostRenderEvent& post_render_event) {
          if (!is_first_frame_rendered_ && !first_frame_fence_) {
            first_frame_fence_ = BaseView::GetSharedEngine()->createFence();
            IMP_LOG(imp::INFO) << "[TPR] created first frame fence";
            GetView().GetDispatcher().Disconnect<ViewPostRenderEvent>(this);
          }
        },
        this);
  }
#endif  // defined(__ANDROID__)
}

void TexturePipelineRenderer::System::AfterLastComponentRemoved() {
  GetView().GetDispatcher().DisconnectAll(this);
  BaseView::GetSharedEngine()->destroy(empty_scene_);
  empty_scene_ = nullptr;
#if defined(__ANDROID__)
  // TODO: (broken link) - first frame check using an app config flag.
  if (GetView()
          .GetConfig()
          .experimental_feature_flags
          ->enable_texture_pipeline_renderer_first_frame_fence.Value()) {
    if (first_frame_fence_) {
      BaseView::GetSharedEngine()->destroy(first_frame_fence_);
      first_frame_fence_ = nullptr;
      IMP_LOG(imp::INFO) << "[TPR] destroyed first frame fence";
    }
  }
#endif  // defined(__ANDROID__)
}

void TexturePipelineRenderer::System::RunTexturePipelines(
    filament::Renderer* filament_renderer) {
  // TODO: Once ComponentSystem detects component add/destruction
  // this can cache the sorted results.
  std::vector<TexturePipelineRenderer*> sorted_texture_pipelines;
  sorted_texture_pipelines.reserve(
      GetView()
          .GetComponentManager()
          .GetComponentPoolById(GetComponentTypeId<TexturePipelineRenderer>())
          ->GetComponentCount());

  GetComponentManager().ForEach<TexturePipelineRenderer>(
      [&sorted_texture_pipelines](TexturePipelineRenderer* texture_pipeline) {
        if (texture_pipeline->IsActive()) {
          sorted_texture_pipelines.push_back(texture_pipeline);
        }
      });

  // if priority is unset or duplicated, ComponentManager order is used
  std::stable_sort(
      sorted_texture_pipelines.begin(), sorted_texture_pipelines.end(),
      [](TexturePipelineRenderer* lhs, TexturePipelineRenderer* rhs) {
        return lhs->state_.priority > rhs->state_.priority;
      });

  for (TexturePipelineRenderer* texture_pipeline : sorted_texture_pipelines) {
    texture_pipeline->RenderPasses(filament_renderer);
  }
}

std::optional<BorrowedMaterialPtr> TexturePipelineRenderer::GetOverrideMaterial(
    size_t pass_index, imp::SmallSourceLocation loc) const {
  if (pass_index >= runtime_passes_.size()) {
    IMP_LOG(imp::ERROR) << "Pass index out of bounds: " << pass_index;
    return std::nullopt;
  } else if (!runtime_passes_[pass_index].override_material) {
    return std::nullopt;
  }
  return runtime_passes_[pass_index].override_material.Borrow(loc);
}

void TexturePipelineRenderer::SetProjectionQuad(
    const std::optional<TexturePipelineRendererProjectionQuad>& quad) {
  if (split_engine::SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    serializer->SetTexturePipelineRendererProjectionQuad(GetNode().GetEntity(),
                                                         quad);
  }

  projection_quad_ = quad;
}

}  // namespace imp
