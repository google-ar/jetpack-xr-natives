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

#include "core/view/platforms/xr_android/xr_vulkan_swap_chain_image_handler.h"

#include <bluevk/BlueVK.h>

#include <cstdint>
#include <memory>
#include <utility>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "core/render/content_security_level.h"
#include "core/view/platforms/xr_android/xr_session_host.h"

namespace imp {

XrVulkanSwapChainImageHandler::XrVulkanSwapChainImageHandler(
    XrPlatformType* platform, XrSessionHost* host,
    std::unique_ptr<SwapchainLayers> layers,
    ContentSecurityLevel content_security_level)
    : platform_(platform),
      host_(host),
      layers_(std::move(layers)),
      content_security_level_(content_security_level) {
  bluevk::bindInstance(platform->getVulkanSharedContext().instance);
  if (host->ShouldRenderVarjoFoveationThisFrame()) {
    layers_->active_color = &layers_->varjo_foveation_color;
    layers_->display_size = host->GetVarjoFoveationDisplaySize();
  } else {
    layers_->active_color = &layers_->default_color;
    layers_->display_size = host->GetDisplaySize();
  }
  XrGraphicsBindingVulkan2KHR graphicsBinding = platform->GetGraphicsBinding();
  vulkan_device_ = graphicsBinding.device;
  if (!host->IsCompositionLayerDepthEnabled()) {
    CreateVulkanDepthImage(platform);
  }
}

XrVulkanSwapChainImageHandler::~XrVulkanSwapChainImageHandler() {
  if (!layers_->depth.images.empty()) {
    for (int i = 0; i < layers_->depth.images.size(); ++i) {
      XrSwapchainImageVulkan2KHR xrImage = layers_->depth.images[i];
      if (xrImage.image != VK_NULL_HANDLE) {
        bluevk::vkDestroyImage(vulkan_device_, xrImage.image, nullptr);
        bluevk::vkFreeMemory(vulkan_device_, layers_->depth.vulkan_memories[i],
                             nullptr);
      }
    }
  }
}

void XrVulkanSwapChainImageHandler::SwitchSwapchainLayers(
    bool use_varjo_foveation) {
  if (use_varjo_foveation) {
    layers_->active_color = &layers_->varjo_foveation_color;
    layers_->display_size = host_->GetVarjoFoveationDisplaySize();
  } else {
    layers_->active_color = &layers_->default_color;
    layers_->display_size = host_->GetDisplaySize();
  }
}

VkResult XrVulkanSwapChainImageHandler::acquire(
    XrVulkanPlatform::ImageSyncData* outImageSyncData) {
  absl::Status status = host_->BeginFrame();
  if (!status.ok()) {
    return VK_INCOMPLETE;
  }

  XrSwapchainImageAcquireInfo acquire_info = {
      .type = XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO,
      .next = nullptr,
  };

  uint32_t color_idx;
  xrAcquireSwapchainImage(layers_->active_color->handle, &acquire_info,
                          &color_idx);

  if (layers_->depth.handle != XR_NULL_HANDLE) {
    XrSwapchainImageAcquireInfo acquire_info{
        .type = XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO,
        .next = nullptr,
    };
    uint32_t depth_idx;
    xrAcquireSwapchainImage(layers_->depth.handle, &acquire_info, &depth_idx);
    
  }

  outImageSyncData->imageIndex = color_idx;
  return VK_SUCCESS;
}

VkResult XrVulkanSwapChainImageHandler::present(uint32_t index,
                                                VkSemaphore finishedDrawing) {
  if (!host_ || layers_->active_color->handle == XR_NULL_HANDLE) {
    return VK_ERROR_NOT_PERMITTED_KHR;
  }
  XrSwapchainImageReleaseInfo releaseInfo{
      .type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO, .next = nullptr};
  xrReleaseSwapchainImage(layers_->active_color->handle, &releaseInfo);

  if (layers_->depth.handle != XR_NULL_HANDLE) {
    xrReleaseSwapchainImage(layers_->depth.handle, &releaseInfo);
  }

  absl::Status status =
      host_->EndFrame(layers_->active_color->handle, layers_->depth.handle);

  if (!status.ok()) {
    return VK_INCOMPLETE;
  }
  return VK_SUCCESS;
}

inline uint32_t selectMemoryType(
    VkPhysicalDeviceMemoryProperties memoryProperties, uint32_t flags,
    VkFlags reqs) {
  for (uint32_t i = 0; i < VK_MAX_MEMORY_TYPES; i++) {
    if (flags & 1) {
      if ((memoryProperties.memoryTypes[i].propertyFlags & reqs) == reqs) {
        return i;
      }
    }

    flags >>= 1;
  }

  return (uint32_t)VK_MAX_MEMORY_TYPES;
}

void XrVulkanSwapChainImageHandler::CreateVulkanDepthImage(
    XrVulkanPlatform* platform) {
  VkImage depth_image;

  VkImageCreateInfo depth_image_info = {
      .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
      .pNext = nullptr,
      .imageType = VK_IMAGE_TYPE_2D,
      .format = kVkDepthFormat,
      .extent =
          {
              .width = host_->GetDisplaySize().x,
              .height = host_->GetDisplaySize().y,
              .depth = 1,
          },
      .mipLevels = 1,
      .arrayLayers =
          host_->IsMultiviewStereo() ? host_->GetLogicalEyeCount() : 1,
      .samples = VK_SAMPLE_COUNT_1_BIT,
      .tiling = VK_IMAGE_TILING_OPTIMAL,
      .usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT |
               VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT,
      .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED,
  };

  bluevk::vkCreateImage(vulkan_device_, &depth_image_info, nullptr,
                        &depth_image);
  VkMemoryRequirements memRequirements;
  bluevk::vkGetImageMemoryRequirements(vulkan_device_, depth_image,
                                       &memRequirements);

  VkMemoryAllocateInfo allocInfo{};
  allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  allocInfo.allocationSize = memRequirements.size;
  VkPhysicalDeviceMemoryProperties memoryProperties;

  bluevk::vkGetPhysicalDeviceMemoryProperties(
      platform->getVulkanSharedContext().physicalDevice, &memoryProperties);

  allocInfo.memoryTypeIndex =
      selectMemoryType(memoryProperties, memRequirements.memoryTypeBits,
                       VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT |
                           VK_MEMORY_PROPERTY_LAZILY_ALLOCATED_BIT);

  VkDeviceMemory vulkan_memory;
  bluevk::vkAllocateMemory(vulkan_device_, &allocInfo, nullptr, &vulkan_memory);
  bluevk::vkBindImageMemory(vulkan_device_, depth_image, vulkan_memory,
                            /*memoryOffset=*/0);

  XrSwapchainImageVulkan2KHR depth_image_vulkan2khr{
      .type = XR_TYPE_SWAPCHAIN_IMAGE_VULKAN_KHR,
      .image = depth_image,
  };

  layers_->depth.images.push_back(depth_image_vulkan2khr);
  layers_->depth.vulkan_memories.push_back(vulkan_memory);
}
}  // namespace imp
