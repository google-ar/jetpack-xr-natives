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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_VULKAN_PLATFORM_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_VULKAN_PLATFORM_H_

// Enable the VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PORTABILITY_SUBSET_FEATURES_KHR
// extension.
#define VK_ENABLE_BETA_EXTENSIONS

#include <jni.h>

#include <cstdint>

#include "filament/filament/backend/include/backend/Platform.h"
#include "filament/filament/backend/include/backend/platforms/VulkanPlatformAndroid.h"
#include "filament/libs/bluevk/include/vulkan/vulkan_core.h"
#include "core/config.h"
#include "core/view/platforms/xr_android/openxr_includes.h"

namespace imp {
using XrPlatformBase = filament::backend::VulkanPlatformAndroid;

// Custom filament platform for rendering in OpenXR using Vulkan.
//
// Provides functionality for creating & using Xr specific swapchains.
class XrVulkanPlatform : public XrPlatformBase {
 public:
  XrVulkanPlatform();

  filament::backend::Driver* createDriver(
      void* sharedContext,
      const Platform::DriverConfig& driverConfig) noexcept override;

  void bindVulkanInstance(VkInstance instance);
  XrGraphicsBindingVulkan2KHR GetGraphicsBinding();
  void setXrInstance(XrInstance instance);
  void setXrSystemId(XrSystemId systemId);
  void setVulkanSharedContext(VulkanSharedContext context);

  Customization getCustomization() const noexcept override;

  SwapChainBundle getSwapChainBundle(SwapChainPtr handle) noexcept override;
  bool hasResized(SwapChainPtr handle) noexcept override;
  VkResult recreate(SwapChainPtr handle) noexcept override;
  VkResult acquire(SwapChainPtr handle,
                   ImageSyncData* outImageSyncData) noexcept override;
  VkResult present(SwapChainPtr handle, uint32_t index,
                   VkSemaphore finishedDrawing) noexcept override;
  VkInstance createVulkanInstance();
  VkPhysicalDevice getVulkanPhysicalDevice(VkInstance instance);
  uint32_t identifyVulkanGraphicsQueueFamilyIndex(
      VkPhysicalDevice physicalDevice);
  uint32_t identifyVulkanProtectedGraphicsQueueFamilyIndex(
      VkPhysicalDevice physicalDevice);
  VkDevice createVulkanLogicalDevice(VkPhysicalDevice physicalDevice,
                                     VkInstance instance,
                                     uint32_t graphicsQueueFamilyIndex,
                                     uint32_t protectedGraphicsQueueFamilyIndex,
                                     bool enableMultiview);
  XrPlatformBase::VulkanSharedContext getVulkanSharedContext() {
    return vulkan_shared_context_;
  }
  // For OpenXR, nativeWindow is required to be an XrSessionHost.
  // This is passed into Filament through FilamentHost::CreateSwapChain.
  SwapChain* createSwapChain(void* nativewindow, uint64_t flags,
                             VkExtent2D extent) noexcept override;

  bool isCompositorTimingSupported() const noexcept override;

  bool setPresentFrameId(SwapChain const* swapchain,
                         uint64_t frameId) noexcept override;

  bool queryFrameTimestamps(
      SwapChain const* swapchain, uint64_t frameId,
      FrameTimestamps* outFrameTimestamps) const noexcept override;

  // Check if the swapchain is protected.
  bool isProtected(SwapChain* swapChain) noexcept override;

  // Query if transient attachments are supported by the backend.
  bool isTransientAttachmentSupported() const;

  void destroy(SwapChain* swapChain) noexcept override;

 private:
  XrInstance xrInstance = XR_NULL_HANDLE;
  XrSystemId xrSystemId = XR_NULL_SYSTEM_ID;
  VkPhysicalDevice physicalDevice_ = VK_NULL_HANDLE;
  XrPlatformBase::VulkanSharedContext vulkan_shared_context_;

  // Flag to indicate if it's rendering frames using a varjo foveation swapchain
  // or not.
  bool rendering_with_varjo_foveation_ = false;
  filament::backend::Platform::GpuContextPriority gpu_context_priority_ =
      filament::backend::Platform::GpuContextPriority::DEFAULT;
};

}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_VULKAN_
        // PLATFORM_H_
