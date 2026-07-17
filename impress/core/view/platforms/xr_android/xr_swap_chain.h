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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_SWAP_CHAIN_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_SWAP_CHAIN_H_

#include <backend/Platform.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/config.h"
#include "core/render/content_security_level.h"
#include "core/view/platforms/xr_android/openxr_includes.h"
#if IMP_MATERIAL_API(VULKAN)
#include "core/view/platforms/xr_android/xr_vulkan_swap_chain_image_handler.h"
#else
#include "core/view/platforms/xr_android/xr_opengl_swap_chain_image_handler.h"
#endif
#include "core/view/platforms/xr_android/xr_session_host.h"

namespace imp {

// An implementation of a Filament SwapChain for use with OpenXR.
//
// This class wraps an OpenXR XrSwapchain, along with swap chain images and
// depth textures.
class XrSwapChain : public filament::backend::Platform::SwapChain {
 public:
#if IMP_MATERIAL_API(VULKAN)
  using ImageHandlerType = imp::XrVulkanSwapChainImageHandler;
#else
  using ImageHandlerType = imp::XrOpenGLSwapChainImageHandler;
#endif

  static absl::StatusOr<std::unique_ptr<XrSwapChain>> Create(
      ImageHandlerType::XrPlatformType* platform, XrSessionHost* host,
      XrSwapchainCreateFlags flags = 0);

  ~XrSwapChain();

  // Returns the swapchain image handler.
  ImageHandlerType& GetSwapchainImageHandler() { return image_handler_; }

  // Returns the xr session host.
  XrSessionHost* GetHost() { return host_; }

  // Returns the content security level of the swapchain.
  ContentSecurityLevel GetContentSecurityLevel();

  // Updates the foveated rendering properties in OpenXR, if needed.
  absl::Status UpdateFoveationProperties();

 private:
  XrSwapChain(ImageHandlerType::XrPlatformType* platform, XrSessionHost* host,
              ImageHandlerType::SwapchainLayers layers,
              ContentSecurityLevel content_security_level);

  PFN_xrUpdateSwapchainFB xr_update_swapchain_fn_;
  PFN_xrCreateFoveationProfileFB xr_create_foveation_profile_fn_;
  PFN_xrDestroyFoveationProfileFB xr_destroy_foveation_profile_fn_;

  absl::Status ResolveFoveationFunctionPointers();

  XrSessionHost* host_ = nullptr;
  ContentSecurityLevel content_security_level_;
  ImageHandlerType image_handler_;

  XrFoveationLevelFB current_foveation_level_ = XR_FOVEATION_LEVEL_NONE_FB;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_SWAP_CHAIN_H_
