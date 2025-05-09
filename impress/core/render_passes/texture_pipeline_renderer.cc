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
#include <type_traits>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/types/optional.h"
#include "absl/types/variant.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/Scene.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/Viewport.h"
#include "core/async/future.h"
#include "core/common/small_source_location.h"
#include "core/materials/material.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/component_system.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/render_passes/texture_pipeline_renderer_state.proto.imp.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/utils/device.h"
#include "core/view/utils/proto/render_settings.proto.imp.h"
#include "core/view/utils/render_setting_utils.h"
#include "core/view/view_events.h"

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
              return view.GetDevice().PixelsToPhysicalPixels(view.GetSize()) *
                     size.factor.value_or(1.0f);
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
    runtime_pass.view->setCamera(
        pass.camera ? pass.camera->GetCamera()
                    : GetView().GetCameraManager().GetCamera()->GetCamera());
    runtime_pass.use_main_view_settings = pass.use_main_view_settings;
    runtime_pass.render_settings = pass.render_settings;

    if (!runtime_pass.use_main_view_settings &&
        pass.render_settings.has_value()) {
      OverrideViewRenderSettings(runtime_pass.view, &(*pass.render_settings),
                                 engine);
    }

    if (!requires_physical_pixels_ratio ||
        GetView().GetDevice().IsPhysicalPixelRatioAvailable()) {
      absl::Status initialized = InitializeTextures(pass, runtime_pass);
      if (!initialized.ok()) {
        return Future<absl::Status>(initialized);
      }
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

void TexturePipelineRenderer::SetPassEnabled(size_t pass_index, bool enabled) {
  if (pass_index >= runtime_passes_.size()) {
    IMP_LOG(imp::ERROR) << "Pass index out of bounds: " << pass_index;
    return;
  }
  runtime_passes_[pass_index].enabled = enabled;
}

bool TexturePipelineRenderer::IsPassEnabled(size_t pass_index) const {
  if (pass_index >= runtime_passes_.size()) {
    IMP_LOG(imp::ERROR) << "Pass index out of bounds: " << pass_index;
    return false;
  }
  return runtime_passes_[pass_index].enabled;
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
    if (is_multiview) {
      TextureFactory::TextureCreationSettings settings = {
          .width = texture_size.x,
          .height = texture_size.y,
          .format = color_format,
          .depth = stereo_depth,
          .levels = 1,
          .sampler_type = filament::Texture::Sampler::SAMPLER_2D_ARRAY,
          .usage = filament::Texture::Usage::COLOR_ATTACHMENT |
                   filament::Texture::Usage::SAMPLEABLE,
      };
      color_texture = GetView().GetTextureFactory().CreateTexture(settings);
    } else {
      color_texture = GetView().GetTextureFactory().CreateTexture(
          texture_size.x, texture_size.y, color_format,
          filament::Texture::Usage::COLOR_ATTACHMENT |
              filament::Texture::Usage::SAMPLEABLE);
    }

    if (color_texture_proto.name.empty()) {
      return absl::FailedPreconditionError(
          "TexturePipelineRenderer color texture must specify a name.");
    }

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
      TextureFactory::Options options = {
          .mag_filter = TextureFactory::MagFilter::NEAREST,
          .min_filter = TextureFactory::MinFilter::NEAREST,
      };
      TextureFactory::TextureCreationSettings settings = {
          .width = texture_size.x,
          .height = texture_size.y,
          .format = depth_format,
          .depth = stereo_depth,
          .levels = 1,
          .sampler_type = filament::Texture::Sampler::SAMPLER_2D_ARRAY,
          .usage = filament::Texture::Usage::DEPTH_ATTACHMENT |
                   filament::Texture::Usage::SAMPLEABLE,
          .options = options,
      };
      depth_texture = GetView().GetTextureFactory().CreateTexture(settings);
    } else {
      depth_texture = GetView().GetTextureFactory().CreateTexture(
          texture_size.x, texture_size.y, depth_format,
          filament::Texture::Usage::DEPTH_ATTACHMENT |
              filament::Texture::Usage::SAMPLEABLE,
          {.mag_filter = TextureFactory::MagFilter::NEAREST,
           .min_filter = TextureFactory::MinFilter::NEAREST});
    }

    if (texture_proto.name.empty()) {
      return absl::FailedPreconditionError(
          "TexturePipelineRenderer depth texture must specify a name.");
    }

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
    }

    // Send the pre-pass event.
    PrePassEvent pre_pass_event;
    pre_pass_event.pass_index = i;
    GetNode()->Send(pre_pass_event);

    // Render the pass.
    GetView().GetHost()->PerformRender(runtime_pass.view);

    // Send the post-pass event.
    PostPassEvent post_pass_event;
    post_pass_event.pass_index = i;
    GetNode()->Send(post_pass_event);

    if (runtime_pass.rendering_to_subregion) {
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
        RunTexturePipelines(pre_render_event.GetRenderer());
      },
      this);
}

void TexturePipelineRenderer::System::AfterLastComponentRemoved() {
  GetView().GetDispatcher().DisconnectAll(this);
  BaseView::GetSharedEngine()->destroy(empty_scene_);
  empty_scene_ = nullptr;
}

void TexturePipelineRenderer::System::RunTexturePipelines(
    filament::Renderer* filament_renderer) {
  // TODO: Once ComponentSystem detects component add/destruction
  // this can cache the sorted results.
  std::vector<TexturePipelineRenderer*> sorted_texture_pipelines;
  sorted_texture_pipelines.reserve(
      GetView()
          .GetComponentManager()
          .GetComponentPoolById(kComponentId<TexturePipelineRenderer>)
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

}  // namespace imp
