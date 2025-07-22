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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_VULKAN_SWAP_CHAIN_IMAGE_HANDLER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_VULKAN_SWAP_CHAIN_IMAGE_HANDLER_H_

#include <dlfcn.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "core/math/vec.h"
#include "core/render/content_security_level.h"
#include "core/view/platforms/xr_android/openxr_includes.h"
#include "core/view/platforms/xr_android/xr_session_host.h"

namespace imp {

// This class handles the swapchain images for the XrSwapChain when the backing
// graphics API is Vulkan. It also holds the image type, image format, depth
// format, and foveation flag needed by the `XrSwapChain` on the Vulkan thread.
class XrVulkanSwapChainImageHandler {
 public:
  static constexpr VkFormat kVkImageFormat = VK_FORMAT_R8G8B8A8_SRGB;
  static constexpr int64_t kImageFormat = kVkImageFormat;
  static constexpr VkFormat kVkDepthFormat = VK_FORMAT_D32_SFLOAT;
  static constexpr int32_t kDepthFormat = kVkDepthFormat;
  static constexpr XrStructureType kImageType =
      XR_TYPE_SWAPCHAIN_IMAGE_VULKAN_KHR;
  static constexpr int64_t kFoveationFlag =
      XR_SWAPCHAIN_CREATE_FOVEATION_FRAGMENT_DENSITY_MAP_BIT_FB;

  using XrPlatformType = XrVulkanPlatform;
  using XrSwapChainImage = XrSwapchainImageVulkan2KHR;

  // TODO:((broken link)) Test possible optimization later,
  // where we destroy and rebuild the swapchains
  // when changing the security level.
  struct SwapchainData {
    XrSwapchain handle = XR_NULL_HANDLE;
    std::vector<XrSwapChainImage> images;
    std::vector<VkDeviceMemory> vulkan_memories;
  };

  struct SwapchainLayers {
    imp::XrVulkanSwapChainImageHandler::SwapchainData default_color = {
        XR_NULL_HANDLE, {}};
    // Used when varjo foveated rendering is enabled and render gaze is
    // available. If varjo foveated rendering is enabled but render gaze is
    // not available, the program will fall back to using default_color at
    // runtime.
    imp::XrVulkanSwapChainImageHandler::SwapchainData varjo_foveation_color = {
        XR_NULL_HANDLE, {}};
    // The active color swapchain is the one that is currently being used.
    SwapchainData* active_color = nullptr;
    // depth_swapchain_ will remain XR_NULL_HANDLE unless
    // XrSessionHost::IsCompositionLayerDepthEnabled is true.
    // depth_swapchain_images_ will remain empty unless
    // XrSessionHost::IsCompositionLayerDepthEnabled is true.
    SwapchainData depth = {XR_NULL_HANDLE, {}};
    // The current display size of the swapchain being used.
    uint2 display_size = {0, 0};
  };

  XrVulkanSwapChainImageHandler(XrPlatformType* platform, XrSessionHost* host,
                                std::unique_ptr<SwapchainLayers> layers,
                                ContentSecurityLevel);
  ~XrVulkanSwapChainImageHandler();
  XrVulkanSwapChainImageHandler(const XrVulkanSwapChainImageHandler&) = delete;
  XrVulkanSwapChainImageHandler& operator=(
      const XrVulkanSwapChainImageHandler&) = delete;

  std::unique_ptr<SwapchainLayers>& GetSwapchainLayers() { return layers_; }
  void SwitchSwapchainLayers(bool use_varjo_foveation);
  VkResult acquire(XrVulkanPlatform::ImageSyncData* outImageSyncData);
  VkResult present(uint32_t index, VkSemaphore finishedDrawing);

 private:
  XrPlatformType* platform_;
  XrSessionHost* host_;
  std::unique_ptr<SwapchainLayers> layers_;
  ContentSecurityLevel content_security_level_;

  // Creates a depth image for the given vulkan device.
  void CreateVulkanDepthImage(XrVulkanPlatform* platform);

  VkDevice vulkan_device_;
  bool is_composition_layer_depth_enabled_;
};
}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_VULKAN_SWAP_CHAIN_IMAGE_HANDLER_H_
