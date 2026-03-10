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

#include "core/view/platforms/xr_android/xr_opengl_swap_chain_image_handler.h"

#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>  // NOLINT
#include <GLES3/gl31.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "filament/filament/backend/src/opengl/gl_headers.h"  // NOLINT
#include "core/render/content_security_level.h"
#include "core/view/platforms/xr_android/openxr_includes.h"
#include "core/view/platforms/xr_android/xr_helpers.h"
#include "core/view/platforms/xr_android/xr_session_host.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

XrOpenGLSwapChainImageHandler::XrOpenGLSwapChainImageHandler(
    XrPlatformType* platform, XrSessionHost* host,
    std::unique_ptr<SwapchainLayers> layers,
    ContentSecurityLevel content_security_level)
    : platform_(platform),
      host_(host),
      layers_(std::move(layers)),
      content_security_level_(content_security_level) {
  layers_->active_color = host->ShouldRenderVarjoFoveationThisFrame()
                              ? &layers_->varjo_foveation_color
                              : &layers_->default_color;
}
XrOpenGLSwapChainImageHandler::~XrOpenGLSwapChainImageHandler() {}

absl::Status XrOpenGLSwapChainImageHandler::MakeCurrent(uint32_t fbo) {
  // WARNING: Called from Filament's Render Thread. It is NOT SAFE to call most
  // Impress or Filament APIs from here.
  std::pair<XrSwapChainImage, std::optional<XrSwapChainImage>> swapchain_images;
  MP_ASSIGN_OR_RETURN(swapchain_images, BeginFrameAndAcquireSwapChainImages());
  const uint32_t color_texture = swapchain_images.first.image;
  if (!swapchain_images.second.has_value()) {
    BindTexturesToFbo(fbo, color_texture, GetDepthTexture(color_texture));
  } else {
    // The second texture is depth buffer.
    BindTexturesToFbo(fbo, color_texture, swapchain_images.second->image);
  }
  return absl::OkStatus();
}

absl::Status XrOpenGLSwapChainImageHandler::Commit(uint32_t fbo) {
  // WARNING: Called from Filament's Render Thread. It is NOT SAFE to call most
  // Impress or Filament APIs from here.
  if (!host_ || layers_->active_color->handle == XR_NULL_HANDLE) {
    return absl::FailedPreconditionError(
        "Cannot commit a swapchain without a valid host and swapchain.");
  }
  BindTexturesToFbo(fbo, 0, 0);
  glFlush();
  if (xr_frames_acquired_) {
    XrSwapchainImageReleaseInfo releaseInfo{
        .type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO, .next = nullptr};
    MP_RETURN_IF_ERROR(host_->ToStatus(
        xrReleaseSwapchainImage(layers_->active_color->handle, &releaseInfo)));
    if (layers_->depth.handle != XR_NULL_HANDLE) {
      MP_RETURN_IF_ERROR(host_->ToStatus(
          xrReleaseSwapchainImage(layers_->depth.handle, &releaseInfo)));
    }
    xr_frames_acquired_ = false;
  }
  return host_->EndFrame(layers_->active_color->handle, layers_->depth.handle);
}

absl::StatusOr<
    std::pair<XrOpenGLSwapChainImageHandler::XrSwapChainImage,
              std::optional<XrOpenGLSwapChainImageHandler::XrSwapChainImage>>>
XrOpenGLSwapChainImageHandler::BeginFrameAndAcquireSwapChainImages() {
  // WARNING: Called from Filament's Render Thread. It is NOT SAFE to call most
  // Impress or Filament APIs from here.

  if (host_->ShouldRenderVarjoFoveationThisFrame()) {
    layers_->active_color = &layers_->varjo_foveation_color;
  } else {
    layers_->active_color = &layers_->default_color;
  }

  MP_RETURN_IF_ERROR(host_->BeginFrame());

  XrSwapchainImageAcquireInfo acquire_info{
      .type = XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO, .next = nullptr};

  imp::output::Xr("Acquiring Swapchain Image.");
  uint32_t image_idx;
  MP_RETURN_IF_ERROR(host_->ToStatus(xrAcquireSwapchainImage(
      layers_->active_color->handle, &acquire_info, &image_idx)));
  XrSwapchainImageWaitInfo wait_info{
      .type = XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO,
      .next = nullptr,
      .timeout = XR_INFINITE_DURATION,
  };
  MP_RETURN_IF_ERROR(host_->ToStatus(
      xrWaitSwapchainImage(layers_->active_color->handle, &wait_info)));

  std::optional<XrSwapChainImage> depth_image = std::nullopt;
  if (layers_->depth.handle != XR_NULL_HANDLE &&
      host_->IsCompositionLayerDepthEnabled()) {
    XrSwapchainImageAcquireInfo acquire_info{
        .type = XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO, .next = nullptr};
    uint32_t image_idx;
    MP_RETURN_IF_ERROR(host_->ToStatus(xrAcquireSwapchainImage(
        layers_->depth.handle, &acquire_info, &image_idx)));
    MP_RETURN_IF_ERROR(host_->ToStatus(
        xrWaitSwapchainImage(layers_->depth.handle, &wait_info)));
    
    depth_image = layers_->depth.images[image_idx];
  }

  xr_frames_acquired_ = true;

  return std::make_pair(layers_->active_color->images[image_idx], depth_image);
}

void XrOpenGLSwapChainImageHandler::BindTexturesToFbo(uint32_t fbo,
                                                      uint32_t color_texture,
                                                      uint32_t depth_texture) {
  // Get previously bound fbo.
  GLint previously_bound_fbo;
  glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &previously_bound_fbo);
  // Bind the fbo passed in.
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);

  const GLenum depth_attachment = host_->GetState()->ShouldUseStencilSwapChain()
                                      ? GL_DEPTH_STENCIL_ATTACHMENT
                                      : GL_DEPTH_ATTACHMENT;

  // Bind the textures to the fbo.
  if (host_->IsMultiviewStereo()) {
    if (host_->GetMsaaSampleCount() > 0) {
      glFramebufferTextureMultisampleMultiviewOVR(
          GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, color_texture, /*level=*/0,
          /*samples=*/host_->GetMsaaSampleCount(),
          /*baseViewIndex=*/0,
          /*numViews=*/host_->GetLogicalEyeCount());
      glFramebufferTextureMultisampleMultiviewOVR(
          GL_DRAW_FRAMEBUFFER, depth_attachment, depth_texture, /*level=*/0,
          /*samples=*/host_->GetMsaaSampleCount(),
          /*baseViewIndex=*/0,
          /*numViews=*/host_->GetLogicalEyeCount());
    } else {
      glFramebufferTextureMultiviewOVR(
          GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, color_texture, /*level=*/0,
          /*baseViewIndex=*/0,
          /*numViews=*/host_->GetLogicalEyeCount());
      glFramebufferTextureMultiviewOVR(
          GL_DRAW_FRAMEBUFFER, depth_attachment, depth_texture, /*level=*/0,
          /*baseViewIndex=*/0,
          /*numViews=*/host_->GetLogicalEyeCount());
    }
  } else {
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, color_texture, 0);
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, depth_attachment, GL_TEXTURE_2D,
                           depth_texture, 0);
  }
  // Restore the previously bound fbo.
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, previously_bound_fbo);
}

uint32_t XrOpenGLSwapChainImageHandler::GetDepthTexture(
    uint32_t color_texture) {
  // WARNING: Called from Filament's Render Thread. It is NOT SAFE to call most
  // Impress or Filament APIs from here.

  auto it = color_to_depth_texture_.find(color_texture);
  if (it != color_to_depth_texture_.end()) {
    return it->second;
  }

  // Get the previously bound texture.
  GLint previously_bound_texture;
  GLenum texture_binding_target = host_->IsMultiviewStereo()
                                      ? GL_TEXTURE_BINDING_2D_ARRAY
                                      : GL_TEXTURE_BINDING_2D;
  glGetIntegerv(texture_binding_target, &previously_bound_texture);

  GLenum texture_target =
      host_->IsMultiviewStereo() ? GL_TEXTURE_2D_ARRAY : GL_TEXTURE_2D;

  // Get the width and height of the color texture so that we can create the
  // depth texture with the same size.
  GLint width;
  GLint height;
  glBindTexture(texture_target, color_texture);
  glGetTexLevelParameteriv(texture_target, 0, GL_TEXTURE_WIDTH, &width);
  glGetTexLevelParameteriv(texture_target, 0, GL_TEXTURE_HEIGHT, &height);

  // Create the depth texture.
  uint32_t depth_texture;
  glGenTextures(1, &depth_texture);
  glBindTexture(texture_target, depth_texture);
  glTexParameteri(texture_target, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(texture_target, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(texture_target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(texture_target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
#if IMP_PLATFORM(ANDROID)
  if (content_security_level_ == ContentSecurityLevel::kProtected) {
    glTexParameteri(texture_target, GL_TEXTURE_PROTECTED_EXT, 1);
  }
#endif

  const GLenum internal_format = host_->GetState()->ShouldUseStencilSwapChain()
                                     ? kDepthStencilFormat
                                     : kDepthFormat;

  if (texture_target == GL_TEXTURE_2D_ARRAY) {
    // NOLINTNEXTLINE(misc-include-cleaner)
    glTexStorage3D(texture_target, 1, internal_format, width, height,
                   host_->GetLogicalEyeCount());
  } else {
    // NOLINTNEXTLINE(misc-include-cleaner)
    glTexStorage2D(texture_target, 1, internal_format, width, height);
  }

  color_to_depth_texture_.insert(std::make_pair(color_texture, depth_texture));

  // Restore the previously bound texture.
  glBindTexture(texture_target, previously_bound_texture);

  return depth_texture;
}
}  // namespace imp
