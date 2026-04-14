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

#include "core/view/platforms/xr_android/xr_vulkan_platform.h"

#include <bluevk/BlueVK.h>
#include <dlfcn.h>
#include <jni.h>

#include <array>
#include <cstdint>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "filament/filament/backend/include/backend/Platform.h"
#include "filament/filament/backend/include/backend/platforms/VulkanPlatform.h"
#include "filament/filament/include/filament/SwapChain.h"
#include "filament/libs/bluevk/include/vulkan/vulkan_core.h"
#include "filament/libs/utils/include/utils/CString.h"
#include "filament/libs/utils/include/utils/FixedCapacityVector.h"
#include "filament/libs/utils/include/utils/Invocable.h"
#include "filament/libs/utils/include/utils/Panic.h"
#include "filament/libs/utils/include/utils/compiler.h"
#include "filament/libs/utils/include/utils/debug.h"
#include "core/common/platform_helpers.h"
#include "core/common/trace.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/render/content_security_level.h"
#include "core/view/platforms/xr_android/openxr_includes.h"
#include "core/view/platforms/xr_android/xr_session_host.h"
#include "core/view/platforms/xr_android/xr_swap_chain.h"

#if IMP_PLATFORM(ANDROID)
#include <android/hardware_buffer.h>
#endif  // IMP_PLATFORM(ANDROID)

namespace imp {

namespace {

// To avoid hitching when compiling samplers with external samplers,
// we can proactively compile them with specific YCbCr formats. Here, we
// define which those are.
struct ExternalSamplerFormatDescription {
  AHardwareBuffer_Format ahbFormat;
  uint64_t ahbUsage;
  VkSamplerYcbcrRange ycbcrRange;
  VkSamplerYcbcrModelConversion ycbcrModel;
};

// Until we update our build target, this value will not exist.
// We'll leave an error so that this can be removed in the future.
#if __ANDROID_API__ < 36
#define AHARDWAREBUFFER_FORMAT_YCbCr_P210 \
  static_cast<AHardwareBuffer_Format>(0x3c)
#endif

constexpr std::array kExternalSamplerFormats = {
    // Standard video playback
    ExternalSamplerFormatDescription{
        AHARDWAREBUFFER_FORMAT_Y8Cb8Cr8_420,
        AHARDWAREBUFFER_USAGE_GPU_SAMPLED_IMAGE,
        VK_SAMPLER_YCBCR_RANGE_ITU_NARROW,
        VK_SAMPLER_YCBCR_MODEL_CONVERSION_YCBCR_709,
    },
    // HDR video playback
    ExternalSamplerFormatDescription{
        AHARDWAREBUFFER_FORMAT_YCbCr_P010,
        AHARDWAREBUFFER_USAGE_GPU_SAMPLED_IMAGE,
        VK_SAMPLER_YCBCR_RANGE_ITU_NARROW,
        VK_SAMPLER_YCBCR_MODEL_CONVERSION_YCBCR_2020,
    },
    // Camera preview. Note: ITU_FULL may cause linker errors with certain
    // formats,
    // which often come up if AHARDWAREBUFFER_USAGE_VIDEO_ENCODE is used. If
    // there are
    // crashes on older devices, it may be that we need to remove some of the
    // ITU_FULL
    // entries, as those conversions may not exist. That case is very unlikely.
    ExternalSamplerFormatDescription{
        AHARDWAREBUFFER_FORMAT_Y8Cb8Cr8_420,
        AHARDWAREBUFFER_USAGE_GPU_SAMPLED_IMAGE,
        VK_SAMPLER_YCBCR_RANGE_ITU_FULL,
        VK_SAMPLER_YCBCR_MODEL_CONVERSION_YCBCR_709,
    },
    ExternalSamplerFormatDescription{
        AHARDWAREBUFFER_FORMAT_Y8Cb8Cr8_420,
        AHARDWAREBUFFER_USAGE_GPU_SAMPLED_IMAGE,
        VK_SAMPLER_YCBCR_RANGE_ITU_FULL,
        VK_SAMPLER_YCBCR_MODEL_CONVERSION_YCBCR_601,
    },
    // If supported
    ExternalSamplerFormatDescription{
        AHARDWAREBUFFER_FORMAT_YCbCr_P210,
        AHARDWAREBUFFER_USAGE_GPU_SAMPLED_IMAGE,
        VK_SAMPLER_YCBCR_RANGE_ITU_FULL,
        VK_SAMPLER_YCBCR_MODEL_CONVERSION_YCBCR_2020,
    },
};

}  // namespace

XrVulkanPlatform::XrVulkanPlatform() { bluevk::initialize(); }

filament::backend::Driver* XrVulkanPlatform::createDriver(
    void* sharedContext, const Platform::DriverConfig& driverConfig) noexcept {
  // TODO: (broken link) - Remove this once impress supports filament feature
  // flags
  Platform::DriverConfig vk_driver_config = driverConfig;
  vk_driver_config.vulkanEnableAsyncPipelineCachePrewarming = true;
  vk_driver_config.vulkanEnableStagingBufferBypass = true;

  filament::backend::Driver* driver =
      XrPlatformBase::createDriver(sharedContext, vk_driver_config);

  // This loads several AHardwareBuffers to fetch their external formats,
  // and store them in a list for async cache prewarming.
  // We're currently setting this flag a few lines prior to this, so it seems
  // odd; there are plans for that to change as soon as the Impress-Filament
  // feature flag system is fixed.
  if (vk_driver_config.vulkanEnableAsyncPipelineCachePrewarming) {
    registerAndroidExternalFormatsForCachePrewarm();
  }

  return driver;
}

XrGraphicsBindingVulkan2KHR XrVulkanPlatform::GetGraphicsBinding() {
#if IMP_PLATFORM(ANDROID)
  return XrGraphicsBindingVulkan2KHR{
      .type = XR_TYPE_GRAPHICS_BINDING_VULKAN2_KHR,
      .next = nullptr,
      .instance = getInstance(),
      .physicalDevice = getPhysicalDevice(),
      .device = getDevice(),
      .queueFamilyIndex = getGraphicsQueueFamilyIndex(),
      .queueIndex = getGraphicsQueueIndex(),
  };
#else
  return {};
#endif
}

void XrVulkanPlatform::setXrInstance(XrInstance instance) {
  xrInstance = instance;
}

void XrVulkanPlatform::setXrSystemId(XrSystemId systemId) {
  xrSystemId = systemId;
}

XrVulkanPlatform::Customization XrVulkanPlatform::getCustomization()
    const noexcept {
  return {
      .isSRGBSwapChainSupported = true,
      .flushAndWaitOnWindowResize = true,
      .transitionSwapChainImageLayoutForPresent = false,
  };
}

XrVulkanPlatform::SwapChainBundle XrVulkanPlatform::getSwapChainBundle(
    SwapChainPtr handle) noexcept {
  auto* swap_chain = static_cast<XrSwapChain*>(handle);
  uint32_t active_color_image_count = swap_chain->GetSwapchainImageHandler()
                                          .GetSwapchainLayers()
                                          ->active_color->images.size();
  
  utils::FixedCapacityVector<VkImage> colors;
  colors.reserve(active_color_image_count);
  for (XrSwapchainImageVulkan2KHR xr_sc_image :
       swap_chain->GetSwapchainImageHandler()
           .GetSwapchainLayers()
           ->active_color->images) {
    colors.push_back(xr_sc_image.image);
  }
  XrVulkanPlatform::SwapChainBundle bundle;
  bundle.depthFormat = XrVulkanSwapChainImageHandler::kVkDepthFormat;
  VkImage depth_image = swap_chain->GetSwapchainImageHandler()
                            .GetSwapchainLayers()
                            ->active_depth->images.front()
                            .image;
  uint2 display_size =
      swap_chain->GetSwapchainImageHandler().GetSwapchainLayers()->display_size;
  
  bundle.depth = depth_image;
  bundle.colors = colors;
  bundle.colorFormat = XrVulkanSwapChainImageHandler::kVkImageFormat;
  bundle.layerCount = swap_chain->GetHost()->IsMultiviewStereo()
                          ? swap_chain->GetHost()->GetLogicalEyeCount()
                          : 1;
  bundle.extent = {
      display_size.x,
      display_size.y,
  };
  return bundle;
}

bool XrVulkanPlatform::hasResized(SwapChainPtr handle) noexcept {
  auto* swap_chain = static_cast<XrSwapChain*>(handle);
  bool should_render_with_varjo_foveation =
      swap_chain->GetHost()->ShouldRenderVarjoFoveationThisFrame();
  // TODO: (broken link) - Implement a better way to handle this which handles
  // switching between swapchains without forcing a resize.

  // Force filament to update the swapchain when the varjo foveation state
  // is different from the last rendered frame. This will make filament to
  // request a new SwapchainBundle.
  if (should_render_with_varjo_foveation != rendering_with_varjo_foveation_) {
    swap_chain->GetSwapchainImageHandler().SwitchSwapchainLayers(
        should_render_with_varjo_foveation);
    rendering_with_varjo_foveation_ = should_render_with_varjo_foveation;
    return true;
  }
  return false;
}

VkResult XrVulkanPlatform::recreate(SwapChainPtr handle) noexcept {
  return VK_SUCCESS;
}

VkResult XrVulkanPlatform::acquire(SwapChainPtr handle,
                                   ImageSyncData* outImageSyncData) noexcept {
  auto* swap_chain = static_cast<XrSwapChain*>(handle);
  VkResult result =
      swap_chain->GetSwapchainImageHandler().acquire(outImageSyncData);

  if (result == VK_SUCCESS || result == VK_SUBOPTIMAL_KHR) {
    XrSwapchainImageWaitInfo wait_info = {
        .type = XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO,
        .next = nullptr,
        .timeout = XR_INFINITE_DURATION,
    };
    // for color image
    xrWaitSwapchainImage(swap_chain->GetSwapchainImageHandler()
                             .GetSwapchainLayers()
                             ->active_color->handle,
                         &wait_info);

    // for depth image if any
    XrSwapchain depth_swapchain = swap_chain->GetSwapchainImageHandler()
                                      .GetSwapchainLayers()
                                      ->depth.handle;
    if (depth_swapchain != XR_NULL_HANDLE) {
      xrWaitSwapchainImage(depth_swapchain, &wait_info);
    }
  }

  return result;
}

VkResult XrVulkanPlatform::present(SwapChainPtr handle, uint32_t index,
                                   VkSemaphore finishedDrawing) noexcept {
  auto* swap_chain = static_cast<XrSwapChain*>(handle);
  return swap_chain->GetSwapchainImageHandler().present(index, finishedDrawing);
}

VkInstance XrVulkanPlatform::createVkInstance(
    const VkInstanceCreateInfo& createInfo) {
  XrVulkanInstanceCreateInfoKHR xrVulkanInstanceCreateInfo = {
      .type = XR_TYPE_VULKAN_INSTANCE_CREATE_INFO_KHR,
      .next = nullptr,
      .systemId = xrSystemId,
      .createFlags = 0,
      .pfnGetInstanceProcAddr = bluevk::vkGetInstanceProcAddr,
      .vulkanCreateInfo = &createInfo,
      .vulkanAllocator = nullptr};

  PFN_xrCreateVulkanInstanceKHR xrCreateVulkanInstanceKHR = nullptr;
  xrGetInstanceProcAddr(
      xrInstance, "xrCreateVulkanInstanceKHR",
      reinterpret_cast<PFN_xrVoidFunction*>(&xrCreateVulkanInstanceKHR));

  VkResult vkResult;
  VkInstance instance = VK_NULL_HANDLE;
  XrResult xrResult = xrCreateVulkanInstanceKHR(
      xrInstance, &xrVulkanInstanceCreateInfo, &instance, &vkResult);

  ASSERT_POSTCONDITION(vkResult == VK_SUCCESS,
                       "Unable to create Vulkan instance. Result=%d", vkResult);
  ASSERT_POSTCONDITION(xrResult == XR_SUCCESS,
                       "Unable to create Vulkan instance. Result=%d", xrResult);
  return instance;
}

VkPhysicalDevice XrVulkanPlatform::selectVkPhysicalDevice(VkInstance instance) {
  XrVulkanGraphicsDeviceGetInfoKHR vulkanGraphicsDeviceGetInfo = {
      .type = XR_TYPE_VULKAN_GRAPHICS_DEVICE_GET_INFO_KHR,
      .next = nullptr,
      .systemId = xrSystemId,
      .vulkanInstance = instance,
  };

  PFN_xrGetVulkanGraphicsDevice2KHR xrGetVulkanGraphicsDevice2KHR = nullptr;
  xrGetInstanceProcAddr(
      xrInstance, "xrGetVulkanGraphicsDevice2KHR",
      reinterpret_cast<PFN_xrVoidFunction*>(&xrGetVulkanGraphicsDevice2KHR));

  VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
  XrResult xrResult = xrGetVulkanGraphicsDevice2KHR(
      xrInstance, &vulkanGraphicsDeviceGetInfo, &physicalDevice);
  ASSERT_POSTCONDITION(xrResult == XR_SUCCESS,
                       "Unable to get Vulkan graphics device. Result=%d",
                       xrResult);
  return physicalDevice;
}

VkDevice XrVulkanPlatform::createVkDevice(
    const VkDeviceCreateInfo& createInfo) {
  XrVulkanDeviceCreateInfoKHR vulkanDeviceCreateInfo = {
      .type = XR_TYPE_VULKAN_DEVICE_CREATE_INFO_KHR,
      .next = nullptr,
      .systemId = xrSystemId,
      .createFlags = 0,
      .pfnGetInstanceProcAddr = bluevk::vkGetInstanceProcAddr,
      .vulkanPhysicalDevice = getPhysicalDevice(),
      .vulkanCreateInfo = &createInfo,
      .vulkanAllocator = nullptr,
  };

  PFN_xrCreateVulkanDeviceKHR xrCreateVulkanDeviceKHR = nullptr;
  xrGetInstanceProcAddr(
      xrInstance, "xrCreateVulkanDeviceKHR",
      reinterpret_cast<PFN_xrVoidFunction*>(&xrCreateVulkanDeviceKHR));

  VkResult vkResult;
  VkDevice device = VK_NULL_HANDLE;
  XrResult xrResult = xrCreateVulkanDeviceKHR(
      xrInstance, &vulkanDeviceCreateInfo, &device, &vkResult);
  ASSERT_POSTCONDITION(xrResult == XR_SUCCESS,
                       "Unable to create Vulkan device. Result=%d", xrResult);
  ASSERT_POSTCONDITION(vkResult == VK_SUCCESS,
                       "Unable to create Vulkan device. Result=%d", vkResult);
  return device;
}

filament::backend::Platform::SwapChain* XrVulkanPlatform::createSwapChain(
    void* nativewindow, uint64_t flags, VkExtent2D extent) noexcept {
  IMP_TRACE();
  XrSessionHost* host = reinterpret_cast<XrSessionHost*>(nativewindow);
  auto status = host->SetThreadType(XR_ANDROID_THREAD_TYPE_RENDERER_MAIN_KHR);
  if (!status.ok()) {
    IMP_LOG(imp::INFO) << "Failed to reported thread type to OpenXR due to " << status;
  }
  // BUG((broken link)): The thread name should already be "FEngine::loop", but
  // SysUI renames threads.  This puts the original setting back.
  SetThreadName("FEngine::loop");
  XrSwapchainCreateFlags xr_flags = 0;
  if (flags & filament::SwapChain::CONFIG_PROTECTED_CONTENT) {
    xr_flags = XR_SWAPCHAIN_CREATE_PROTECTED_CONTENT_BIT;
  }
  absl::StatusOr<std::unique_ptr<XrSwapChain>> swap_chain =
      XrSwapChain::Create(this, host, xr_flags);
  if (!swap_chain.ok()) {
    IMP_LOG(imp::FATAL) << "Unable to create XrSwapChain: " << swap_chain.status();
  }
  return swap_chain.value().release();
}

bool XrVulkanPlatform::isCompositorTimingSupported() const noexcept {
  return false;
}

bool XrVulkanPlatform::setPresentFrameId(SwapChain const* swapchain,
                                         uint64_t frameId) noexcept {
  return false;
}

bool XrVulkanPlatform::queryFrameTimestamps(
    SwapChain const* swapchain, uint64_t frameId,
    FrameTimestamps* outFrameTimestamps) const noexcept {
  return false;
}

bool XrVulkanPlatform::isProtected(SwapChain* swapChain) noexcept {
  return static_cast<XrSwapChain*>(swapChain)->GetContentSecurityLevel() ==
         ContentSecurityLevel::kProtected;
}

bool XrVulkanPlatform::isTransientAttachmentSupported() const {
  return VulkanPlatform::isTransientAttachmentSupported();
}

void XrVulkanPlatform::destroy(SwapChain* swapChain) noexcept {
  IMP_TRACE();
  // Destroyed when it falls out of scope.
  std::unique_ptr<XrSwapChain> swap_chain(static_cast<XrSwapChain*>(swapChain));
}

void XrVulkanPlatform::registerAndroidExternalFormatsForCachePrewarm() {
  if (__builtin_available(android 26, *)) {
    for (const auto& externalFormat : kExternalSamplerFormats) {
      AHardwareBuffer_Desc desc{
          .width = 2,
          .height = 2,
          .layers = 1,
          .format = externalFormat.ahbFormat,
          .usage = externalFormat.ahbUsage,
      };

      // Some formats may not be supported, depending on the runtime API
      // version. Try to check if the format is supported to avoid a more
      // expensive call to allocate.
      if (__builtin_available(android 29, *)) {
        if (!AHardwareBuffer_isSupported(&desc)) {
          IMP_LOG(imp::ERROR) << "Skipping unsupported ahb format "
                     << std::to_string(externalFormat.ahbFormat)
                     << " for cache prewarming.";
          continue;
        }
      }

      // Try to create a fake buffer, so we can fetch the external format
      // number.
      AHardwareBuffer* buffer = nullptr;
      if (int rc = AHardwareBuffer_allocate(&desc, &buffer); rc != 0) {
        IMP_LOG(imp::ERROR) << "Failed to allocate fake AHardwareBuffer to "
                      "check external format constant, not registering "
                   << std::to_string(externalFormat.ahbFormat) << " / "
                   << std::to_string(externalFormat.ahbUsage) << " (rc = " << rc
                   << ")";
        continue;
      }

      // Get the format properties from Vulkan.
      VkAndroidHardwareBufferFormatPropertiesANDROID formatProps = {
          .sType =
              VK_STRUCTURE_TYPE_ANDROID_HARDWARE_BUFFER_FORMAT_PROPERTIES_ANDROID,  // NOLINT
      };
      VkAndroidHardwareBufferPropertiesANDROID props = {
          .sType = VK_STRUCTURE_TYPE_ANDROID_HARDWARE_BUFFER_PROPERTIES_ANDROID,
          .pNext = &formatProps,
      };
      VkResult result = bluevk::vkGetAndroidHardwareBufferPropertiesANDROID(
          getDevice(), buffer, &props);
      if (result == VK_SUCCESS) {
        IMP_LOG(imp::INFO) << "Registered external format for cache prewarming: "
                  << std::to_string(externalFormat.ahbFormat) << " / "
                  << std::to_string(externalFormat.ahbUsage);
        registerPipelineCachePrewarmExternalFormat({
            .externalFormat = formatProps.externalFormat,
            .ycbcrModelConversion = externalFormat.ycbcrModel,
            .ycbcrRange = externalFormat.ycbcrRange,
        });
      } else {
        IMP_LOG(imp::ERROR)
            << "Failed to fetch format props for fake AHardwareBuffer, not "
            << "registering " << std::to_string(externalFormat.ahbFormat)
            << " / " << std::to_string(externalFormat.ahbUsage)
            << " (rc = " << result << ")";
      }

      AHardwareBuffer_release(buffer);
    }
  }
}

}  // namespace imp
