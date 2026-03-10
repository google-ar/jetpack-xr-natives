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


#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_OPENGL_SWAP_CHAIN_IMAGE_HANDLER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_OPENGL_SWAP_CHAIN_IMAGE_HANDLER_H_
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "filament/filament/backend/src/opengl/gl_headers.h"  // NOLINT
#include "core/render/content_security_level.h"
#include "core/view/platforms/xr_android/openxr_includes.h"
#include "core/view/platforms/xr_android/xr_opengl_platform.h"
#include "core/view/platforms/xr_android/xr_session_host.h"
#include "robin_map/include/tsl/robin_map.h"

namespace imp {

// This class handles the swapchain images for the XrSwapChain when the backing
// graphics API is OpenGL. It also holds the image type, image format, depth
// format, and foveation flag needed by the XrSwapChain on the GL thread.
class XrOpenGLSwapChainImageHandler {
 public:
  static constexpr int64_t kImageFormat = GL_SRGB8_ALPHA8;
  static constexpr int32_t kDepthFormat = GL_DEPTH_COMPONENT24;
  static constexpr int32_t kDepthStencilFormat = GL_DEPTH24_STENCIL8;
  static constexpr XrStructureType kImageType =
      XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_ES_KHR;
  static constexpr int64_t kFoveationFlag =
      XR_SWAPCHAIN_CREATE_FOVEATION_SCALED_BIN_BIT_FB;

  using XrPlatformType = XrOpenGLPlatform;
  using XrSwapChainImage = XrSwapchainImageOpenGLESKHR;
  // TODO:((broken link)) Test possible optimization later,
  // where we destroy and rebuild the swapchains
  // when changing the security level.
  struct SwapchainData {
    XrSwapchain handle = XR_NULL_HANDLE;
    std::vector<XrSwapChainImage> images;
  };
  struct SwapchainLayers {
    imp::XrOpenGLSwapChainImageHandler::SwapchainData default_color = {
        XR_NULL_HANDLE, {}};
    // Used when varjo foveated rendering is enabled and render gaze is
    // available. If varjo foveated rendering is enabled but render gaze is
    // not available, the program will fall back to using default_color at
    // runtime.
    imp::XrOpenGLSwapChainImageHandler::SwapchainData varjo_foveation_color = {
        XR_NULL_HANDLE, {}};
    // The active color swapchain is the one that is currently being used.
    SwapchainData* active_color = nullptr;
    // depth_swapchain_ will remain XR_NULL_HANDLE unless
    // XrSessionHost::IsCompositionLayerDepthEnabled is true.
    // depth_swapchain_images_ will remain empty unless
    // XrSessionHost::IsCompositionLayerDepthEnabled is true.
    SwapchainData depth = {XR_NULL_HANDLE, {}};
  };
  XrOpenGLSwapChainImageHandler(XrPlatformType* platform, XrSessionHost* host,
                                std::unique_ptr<SwapchainLayers> layers,
                                ContentSecurityLevel);
  ~XrOpenGLSwapChainImageHandler();
  XrOpenGLSwapChainImageHandler(const XrOpenGLSwapChainImageHandler&) = delete;
  XrOpenGLSwapChainImageHandler& operator=(
      const XrOpenGLSwapChainImageHandler&) = delete;
  // Called from Filament's render thread by XrOpenGLPlatform::makeCurrent. This
  // is called on the GL thread.
  absl::Status MakeCurrent(uint32_t fbo);
  // Called from Filament's render thread by XrOpenGLPlatform::commit. This is
  // called on the GL thread.
  absl::Status Commit(uint32_t fbo);
  std::unique_ptr<SwapchainLayers>& GetSwapchainLayers() { return layers_; }

 private:
  // Returns the color and depth swapchain images for the current frame on the
  // GL thread.
  absl::StatusOr<std::pair<XrSwapChainImage, std::optional<XrSwapChainImage>>>
  BeginFrameAndAcquireSwapChainImages();
  // Returns the depth texture for the given color texture on the GL thread.
  uint32_t GetDepthTexture(uint32_t color_texture);
  // Binds the color and depth textures to the fbo on the GL thread.
  void BindTexturesToFbo(uint32_t fbo, uint32_t color_texture,
                         uint32_t depth_texture);

  XrPlatformType* platform_;
  XrSessionHost* host_;
  std::unique_ptr<SwapchainLayers> layers_;
  ContentSecurityLevel content_security_level_;
  tsl::robin_map<uint32_t, uint32_t> color_to_depth_texture_;
  // The `BeginFrameAndAcquireSwapChainImages` and `Commit` methods are called
  // always in pairs for every frame even if the former method returns an error.
  // This value is to ensure that the swapchain images are released only when
  // it's needed. ((broken link))
  bool xr_frames_acquired_ = false;
};
}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_OPENGL_SWAP_CHAIN_IMAGE_HANDLER_H_
