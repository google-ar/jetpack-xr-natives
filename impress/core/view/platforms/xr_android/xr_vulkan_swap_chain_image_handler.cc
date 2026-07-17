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
#include "core/common/log.h"
#include "absl/status/status.h"
#include "filament/libs/bluevk/include/vulkan/vulkan_core.h"
#include "core/math/vec.h"
#include "core/render/content_security_level.h"
#include "core/view/platforms/xr_android/xr_session_host.h"

namespace imp {

XrVulkanSwapChainImageHandler::XrVulkanSwapChainImageHandler(
    XrPlatformType* platform, XrSessionHost* host,
    std::unique_ptr<SwapchainLayers> layers,
    ContentSecurityLevel /*content_security_level*/)
    : platform_(platform), host_(host), layers_(std::move(layers)) {
  if (!host->IsCompositionLayerDepthEnabled()) {
    CreateDepthSwapchains();
  }

  if (host->ShouldRenderVarjoFoveationThisFrame()) {
    layers_->active_color = &layers_->varjo_foveation_color;
    layers_->active_depth = &layers_->varjo_foveation_depth;
    layers_->display_size = host->GetVarjoFoveationDisplaySize();
  } else {
    layers_->active_color = &layers_->default_color;
    layers_->active_depth = &layers_->depth;
    layers_->display_size = host->GetDisplaySize();
  }
}

XrVulkanSwapChainImageHandler::~XrVulkanSwapChainImageHandler() {
  for (int i = 0; i < layers_->depth.images.size(); ++i) {
    XrSwapchainImageVulkan2KHR xrImage = layers_->depth.images[i];
    if (xrImage.image != VK_NULL_HANDLE) {
      bluevk::vkDestroyImage(platform_->getDevice(), xrImage.image, nullptr);
      bluevk::vkFreeMemory(platform_->getDevice(),
                           layers_->depth.vulkan_memories[i], nullptr);
    }
  }

  for (int i = 0; i < layers_->varjo_foveation_depth.images.size(); ++i) {
    XrSwapchainImageVulkan2KHR xrImage =
        layers_->varjo_foveation_depth.images[i];
    if (xrImage.image != VK_NULL_HANDLE) {
      bluevk::vkDestroyImage(platform_->getDevice(), xrImage.image, nullptr);
      bluevk::vkFreeMemory(platform_->getDevice(),
                           layers_->varjo_foveation_depth.vulkan_memories[i],
                           nullptr);
    }
  }
}

void XrVulkanSwapChainImageHandler::SwitchSwapchainLayers(
    bool use_varjo_foveation) {
  if (use_varjo_foveation) {
    layers_->active_color = &layers_->varjo_foveation_color;
    layers_->active_depth = &layers_->varjo_foveation_depth;
    layers_->display_size = host_->GetVarjoFoveationDisplaySize();
  } else {
    layers_->active_color = &layers_->default_color;
    layers_->active_depth = &layers_->depth;
    layers_->display_size = host_->GetDisplaySize();
  }
}

VkResult XrVulkanSwapChainImageHandler::acquire(
    ImageSyncData* outImageSyncData) {
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

  if (layers_->active_depth->handle != XR_NULL_HANDLE) {
    XrSwapchainImageAcquireInfo acquire_info{
        .type = XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO,
        .next = nullptr,
    };
    uint32_t depth_idx;
    xrAcquireSwapchainImage(layers_->active_depth->handle, &acquire_info,
                            &depth_idx);
    
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
    // In the case of an error in `xrEndFrame`, just print the error and
    // continue as there were no problem since all the work has already been
    // submitted to the GPU.
    //
    // The error will manifest as a reprojected frame or a black frame if no
    // valid frame was submitted previously.
    //
    // An error at this point in time is only related to OpenXR and has no
    // relation to Vulkan, so it's safe to return VK_SUCCESS.
    IMP_LOG(imp::ERROR) << "Error calling xrEndFrame: " << status;
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

void XrVulkanSwapChainImageHandler::CreateDepthSwapchains() {
  uint32_t layers =
      host_->IsMultiviewStereo() ? host_->GetLogicalEyeCount() : 1;
  layers_->depth = CreateDepthSwapchain(host_->GetDisplaySize(), layers);

  if (host_->IsXrVarjoFoveatedRenderingEnabled()) {
    layers_->varjo_foveation_depth =
        CreateDepthSwapchain(host_->GetVarjoFoveationDisplaySize(), layers);
  }
}

XrVulkanSwapChainImageHandler::SwapchainData
XrVulkanSwapChainImageHandler::CreateDepthSwapchain(uint2 display_size,
                                                    uint32_t layers) {
  VkImageUsageFlags usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
  if (platform_->isTransientAttachmentSupported()) {
    usage |= VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT;
  }

  VkImage image = VK_NULL_HANDLE;
  VkImageCreateInfo create_info = {
      .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
      .pNext = nullptr,
      .imageType = VK_IMAGE_TYPE_2D,
      .format = host_->GetState()->ShouldUseStencilSwapChain()
                    ? kVkDepthStencilFormat
                    : kVkDepthFormat,
      .extent =
          {
              .width = display_size.x,
              .height = display_size.y,
              .depth = 1,
          },
      .mipLevels = 1,
      .arrayLayers = layers,
      .samples = VK_SAMPLE_COUNT_1_BIT,
      .tiling = VK_IMAGE_TILING_OPTIMAL,
      .usage = usage,
      .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED,
  };

  VkResult result = bluevk::vkCreateImage(platform_->getDevice(), &create_info,
                                          nullptr, &image);
  

  VkMemoryRequirements memRequirements = {};
  bluevk::vkGetImageMemoryRequirements(platform_->getDevice(), image,
                                       &memRequirements);

  VkMemoryAllocateInfo allocInfo{};
  allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  allocInfo.allocationSize = memRequirements.size;
  VkPhysicalDeviceMemoryProperties memoryProperties;

  bluevk::vkGetPhysicalDeviceMemoryProperties(platform_->getPhysicalDevice(),
                                              &memoryProperties);

  VkMemoryPropertyFlags memory_properties = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
  if (platform_->isTransientAttachmentSupported()) {
    memory_properties |= VK_MEMORY_PROPERTY_LAZILY_ALLOCATED_BIT;
  }

  // There's no need to use VK_MEMORY_PROPERTY_PROTECTED_BIT here because it's a
  // transient attachment and the output will be discarded after being use in a
  // render pass.
  // That's why `VK_MEMORY_PROPERTY_LAZILY_ALLOCATED_BIT |
  // VK_MEMORY_PROPERTY_PROTECTED_BIT` is not a supported combination in vulkan.
  allocInfo.memoryTypeIndex = selectMemoryType(
      memoryProperties, memRequirements.memoryTypeBits, memory_properties);
  

  VkDeviceMemory vulkan_memory;
  result = bluevk::vkAllocateMemory(platform_->getDevice(), &allocInfo, nullptr,
                                    &vulkan_memory);
  

  result = bluevk::vkBindImageMemory(platform_->getDevice(), image,
                                     vulkan_memory, /*memoryOffset=*/0);
  

  XrSwapchainImageVulkan2KHR depth_image_vulkan2khr{
      .type = XR_TYPE_SWAPCHAIN_IMAGE_VULKAN_KHR,
      .image = image,
  };

  return {
      .handle = XR_NULL_HANDLE,
      .images = {depth_image_vulkan2khr},
      .vulkan_memories = {vulkan_memory},
  };
}

}  // namespace imp
