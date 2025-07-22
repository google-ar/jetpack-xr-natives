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

#include <cstdint>
#include <memory>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "filament/filament/backend/include/backend/Platform.h"
#include "filament/filament/include/filament/SwapChain.h"
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

namespace imp {

namespace {

constexpr uint32_t const kInvalidVkIndex = 0xFFFFFFFF;

uint32_t identifyQueueFamilyIndex(VkPhysicalDevice physical_device,
                                  uint32_t queue_flags) {
  uint32_t queue_families_count = 0;
  bluevk::vkGetPhysicalDeviceQueueFamilyProperties(
      physical_device, &queue_families_count,
      /*pQueueFamilyProperties=*/nullptr);
  utils::FixedCapacityVector<VkQueueFamilyProperties> queue_families_properties(
      queue_families_count);
  if (queue_families_count > 0) {
    bluevk::vkGetPhysicalDeviceQueueFamilyProperties(
        physical_device, &queue_families_count,
        queue_families_properties.data());
  }

  uint32_t family_index = kInvalidVkIndex;
  for (uint32_t index = 0; index < queue_families_properties.size(); ++index) {
    const VkQueueFamilyProperties& properties =
        queue_families_properties[index];
    if (properties.queueCount != 0 && (properties.queueFlags & queue_flags)) {
      family_index = index;
      break;
    }
  }
  return family_index;
}

}  // namespace

XrVulkanPlatform::XrVulkanPlatform() { bluevk::initialize(); }

void XrVulkanPlatform::bindVulkanInstance(VkInstance instance) {
  bluevk::bindInstance(instance);
}

XrGraphicsBindingVulkan2KHR XrVulkanPlatform::GetGraphicsBinding() {
#if IMP_PLATFORM(ANDROID)
  return XrGraphicsBindingVulkan2KHR{
      .type = XR_TYPE_GRAPHICS_BINDING_VULKAN2_KHR,
      .next = nullptr,
      .instance = getInstance(),
      .physicalDevice = physicalDevice_,
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

void XrVulkanPlatform::setVulkanSharedContext(
    XrPlatformBase::VulkanSharedContext context) {
  vulkan_shared_context_ = context;
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
                            ->depth.images.front()
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

VkInstance XrVulkanPlatform::createVulkanInstance() {
  VkInstance instance;
  VkInstanceCreateInfo vulkanInstanceCreateInfo = {};
  bool validationFeaturesSupported = false;

  // The Platform class requires at most 2 instance extensions, so a max of 3.
  static constexpr uint32_t MAX_INSTANCE_EXTENSION_COUNT = 3;
  const char* ppEnabledExtensions[MAX_INSTANCE_EXTENSION_COUNT];

  // Request platform-specific extensions.
  VulkanPlatform::ExtensionSet const TARGET_EXTS = {
      VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME,
      VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME,
  };

  uint32_t extensionCount = 0;
  bluevk::vkEnumerateInstanceExtensionProperties(
      static_cast<char const*>(nullptr) /* pLayerName */, &extensionCount,
      nullptr);

  std::vector<VkExtensionProperties> availableExtensions;
  availableExtensions.resize(extensionCount);
  bluevk::vkEnumerateInstanceExtensionProperties(
      static_cast<char const*>(nullptr) /* pLayerName */, &extensionCount,
      availableExtensions.data());

  uint32_t enabledExtensionCount = 0;
  if (validationFeaturesSupported) {
    ppEnabledExtensions[enabledExtensionCount++] =
        VK_EXT_VALIDATION_FEATURES_EXTENSION_NAME;
  }

  for (auto const& extensionProperties : availableExtensions) {
    assert_invariant(enabledExtensionCount < MAX_INSTANCE_EXTENSION_COUNT);
    utils::CString name{extensionProperties.extensionName};
    // To workaround an Adreno bug where the extension name could be of 0
    // length.
    if (name.size() == 0) {
      continue;
    }

    if (TARGET_EXTS.find(name) != TARGET_EXTS.end()) {
      ppEnabledExtensions[enabledExtensionCount++] =
          extensionProperties.extensionName;

      if (name == VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME) {
        vulkanInstanceCreateInfo.flags =
            VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
      }
    }
  }

  // Do not support the filament debug utils extension.
  vulkan_shared_context_.debugUtilsSupported = false;

  // Create the Vulkan instance.
  VkApplicationInfo appInfo = {};
  appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  appInfo.pEngineName = "Filament";
  // TODO Replace 1s with FVK_REQUIRED_VERSION_MINOR and
  // FVK_REQUIRED_VERSION_MAJOR
  appInfo.apiVersion = VK_MAKE_API_VERSION(0, 1, 1, 0);
  vulkanInstanceCreateInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  vulkanInstanceCreateInfo.pApplicationInfo = &appInfo;
  vulkanInstanceCreateInfo.enabledExtensionCount = enabledExtensionCount;
  vulkanInstanceCreateInfo.ppEnabledExtensionNames = ppEnabledExtensions;

  VkValidationFeaturesEXT features = {};
  VkValidationFeatureEnableEXT enables[] = {
      VK_VALIDATION_FEATURE_ENABLE_BEST_PRACTICES_EXT,
      VK_VALIDATION_FEATURE_ENABLE_SYNCHRONIZATION_VALIDATION_EXT,
  };

  if (validationFeaturesSupported) {
    features.sType = VK_STRUCTURE_TYPE_VALIDATION_FEATURES_EXT;
    features.enabledValidationFeatureCount =
        sizeof(enables) / sizeof(enables[0]);
    features.pEnabledValidationFeatures = enables;
    vulkanInstanceCreateInfo.pNext = &features;
  }

  XrVulkanInstanceCreateInfoKHR xrVulkanInstanceCreateInfo = {
      .type = XR_TYPE_VULKAN_INSTANCE_CREATE_INFO_KHR,
      .next = nullptr,
      .systemId = xrSystemId,
      .createFlags = 0,
      .pfnGetInstanceProcAddr = bluevk::vkGetInstanceProcAddr,
      .vulkanCreateInfo = &vulkanInstanceCreateInfo,
      .vulkanAllocator = nullptr};

  VkResult vkResult;

  PFN_xrCreateVulkanInstanceKHR xrCreateVulkanInstanceKHR = nullptr;
  xrGetInstanceProcAddr(
      xrInstance, "xrCreateVulkanInstanceKHR",
      reinterpret_cast<PFN_xrVoidFunction*>(&xrCreateVulkanInstanceKHR));

  XrResult xrResult = xrCreateVulkanInstanceKHR(
      xrInstance, &xrVulkanInstanceCreateInfo, &instance, &vkResult);

  ASSERT_POSTCONDITION(vkResult == VK_SUCCESS,
                       "Unable to create Vulkan instance. Result=%d", vkResult);
  ASSERT_POSTCONDITION(xrResult == XR_SUCCESS,
                       "Unable to create Vulkan instance. Result=%d", xrResult);
  return instance;
}

VkPhysicalDevice XrVulkanPlatform::getVulkanPhysicalDevice(
    VkInstance instance) {
  uint32_t deviceCount = 1;  // We want to enumerate only one device
  // VkPhysicalDevice physicalDevice;
  bluevk::vkEnumeratePhysicalDevices(instance, &deviceCount, &physicalDevice_);
  return physicalDevice_;
}

uint32_t XrVulkanPlatform::identifyVulkanGraphicsQueueFamilyIndex(
    VkPhysicalDevice physicalDevice) {
  return identifyQueueFamilyIndex(physicalDevice, VK_QUEUE_GRAPHICS_BIT);
}

uint32_t XrVulkanPlatform::identifyVulkanProtectedGraphicsQueueFamilyIndex(
    VkPhysicalDevice physicalDevice) {
  return identifyQueueFamilyIndex(
      physicalDevice, VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_PROTECTED_BIT);
}

VkDevice XrVulkanPlatform::createVulkanLogicalDevice(
    VkPhysicalDevice physicalDevice, VkInstance instance,
    uint32_t graphicsQueueFamilyIndex,
    uint32_t protectedGraphicsQueueFamilyIndex, bool enableMultiview) {
  // Platform-specific extensions.
  VulkanPlatform::ExtensionSet const TARGET_EXTS = {
      VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME,
      VK_KHR_MAINTENANCE1_EXTENSION_NAME,
      VK_KHR_MAINTENANCE2_EXTENSION_NAME,
      VK_KHR_MAINTENANCE3_EXTENSION_NAME,
      VK_KHR_MULTIVIEW_EXTENSION_NAME,
      VK_KHR_IMAGE_FORMAT_LIST_EXTENSION_NAME,
  };

  uint32_t deviceExtensionCount = 0;
  // Identify supported physical device extensions
  bluevk::vkEnumerateDeviceExtensionProperties(
      physicalDevice, static_cast<const char*>(nullptr) /* pLayerName */,
      &deviceExtensionCount, nullptr);

  std::vector<VkExtensionProperties> availableDeviceExtensions;
  availableDeviceExtensions.resize(deviceExtensionCount);
  bluevk::vkEnumerateDeviceExtensionProperties(
      physicalDevice, static_cast<const char*>(nullptr) /* pLayerName */,
      &deviceExtensionCount, availableDeviceExtensions.data());

  void* pNext = nullptr;

  VkPhysicalDeviceProtectedMemoryFeatures protectedMemory = {
      .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROTECTED_MEMORY_FEATURES,
      .pNext = pNext,
      .protectedMemory = VK_TRUE,
  };
  pNext = &protectedMemory;

  VkPhysicalDeviceSamplerYcbcrConversionFeatures ycbcrConversion = {
      .sType =
          VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SAMPLER_YCBCR_CONVERSION_FEATURES,
      .pNext = pNext,
      .samplerYcbcrConversion = VK_TRUE,
  };
  pNext = &ycbcrConversion;

  VkPhysicalDevicePortabilitySubsetFeaturesKHR portability = {
      .sType =
          VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PORTABILITY_SUBSET_FEATURES_KHR,
      .pNext = nullptr,
      .imageViewFormatSwizzle = VK_TRUE,
      .mutableComparisonSamplers = VK_TRUE,
  };

  VkPhysicalDeviceMultiviewFeaturesKHR multiview = {
      .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MULTIVIEW_FEATURES_KHR,
      .pNext = nullptr,
      .multiview = VK_TRUE,
      .multiviewGeometryShader = VK_FALSE,
      .multiviewTessellationShader = VK_FALSE};

  std::vector<const char*> enabledExtensions;
  for (auto const& extensionProperties : availableDeviceExtensions) {
    utils::CString name{extensionProperties.extensionName};
    // To workaround an Adreno bug where the extension name could be of 0
    // length.
    if (name.size() == 0) {
      continue;
    }

    if (TARGET_EXTS.find(name) != TARGET_EXTS.end()) {
      enabledExtensions.push_back(extensionProperties.extensionName);

      if (name == VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME) {
        portability.pNext = pNext;
        pNext = &portability;
      } else if (name == VK_KHR_MULTIVIEW_EXTENSION_NAME) {
        multiview.pNext = pNext;
        pNext = &multiview;
        vulkan_shared_context_.multiviewSupported = enableMultiview;
      }
    }
  }

  // Do not support the filament debug markers extension.
  vulkan_shared_context_.debugMarkersSupported = false;

  VkDeviceQueueCreateInfo deviceQueueCreateInfo[2] = {};
  const float queuePriority[] = {1.0f};
  VkDeviceCreateInfo deviceCreateInfo = {};
  deviceQueueCreateInfo[0].sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
  deviceQueueCreateInfo[0].queueFamilyIndex = graphicsQueueFamilyIndex;
  deviceQueueCreateInfo[0].queueCount = 1;
  deviceQueueCreateInfo[0].pQueuePriorities = &queuePriority[0];

  deviceQueueCreateInfo[1].sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
  deviceQueueCreateInfo[1].flags = VK_DEVICE_QUEUE_CREATE_PROTECTED_BIT;
  deviceQueueCreateInfo[1].queueFamilyIndex = protectedGraphicsQueueFamilyIndex;
  deviceQueueCreateInfo[1].queueCount = 1;
  deviceQueueCreateInfo[1].pQueuePriorities = &queuePriority[0];

  deviceCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
  deviceCreateInfo.queueCreateInfoCount =
      protectedGraphicsQueueFamilyIndex != kInvalidVkIndex ? 2 : 1;
  deviceCreateInfo.pQueueCreateInfos = deviceQueueCreateInfo;
  deviceCreateInfo.pNext = pNext;

  // We could simply enable all supported features, but since that may have
  // performance consequences let's just enable the features we need. Get these
  // from the physical device.
  VkPhysicalDeviceFeatures enabledFeatures{
      .samplerAnisotropy = true,
      .textureCompressionETC2 = true,
      .textureCompressionBC = true,
      .shaderClipDistance = true,
  };

  deviceCreateInfo.pEnabledFeatures = &enabledFeatures;
  deviceCreateInfo.enabledExtensionCount =
      static_cast<uint32_t>(enabledExtensions.size());
  deviceCreateInfo.ppEnabledExtensionNames = enabledExtensions.data();

  VkDevice device;
  XrVulkanDeviceCreateInfoKHR vulkanDeviceCreateInfo = {
      .type = XR_TYPE_VULKAN_DEVICE_CREATE_INFO_KHR,
      .next = nullptr,
      .systemId = xrSystemId,
      .createFlags = 0,
      .pfnGetInstanceProcAddr = bluevk::vkGetInstanceProcAddr,
      .vulkanPhysicalDevice = physicalDevice,
      .vulkanCreateInfo = &deviceCreateInfo,
      .vulkanAllocator = nullptr};

  XrVulkanGraphicsDeviceGetInfoKHR vulkanGraphicsDeviceGetInfo = {
      .type = XR_TYPE_VULKAN_GRAPHICS_DEVICE_GET_INFO_KHR,
      .next = nullptr,
      .systemId = xrSystemId,
      .vulkanInstance = instance};

  VkResult vkResult;

  PFN_xrGetVulkanGraphicsDevice2KHR xrGetVulkanGraphicsDevice2KHR = nullptr;
  xrGetInstanceProcAddr(
      xrInstance, "xrGetVulkanGraphicsDevice2KHR",
      reinterpret_cast<PFN_xrVoidFunction*>(&xrGetVulkanGraphicsDevice2KHR));

  XrResult xrResult = xrGetVulkanGraphicsDevice2KHR(
      xrInstance, &vulkanGraphicsDeviceGetInfo, &physicalDevice);
  ASSERT_POSTCONDITION(xrResult == XR_SUCCESS,
                       "Unable to get Vulkan graphics device. Result=%d",
                       xrResult);

  PFN_xrCreateVulkanDeviceKHR xrCreateVulkanDeviceKHR = nullptr;
  xrGetInstanceProcAddr(
      xrInstance, "xrCreateVulkanDeviceKHR",
      reinterpret_cast<PFN_xrVoidFunction*>(&xrCreateVulkanDeviceKHR));
  xrResult = xrCreateVulkanDeviceKHR(xrInstance, &vulkanDeviceCreateInfo,
                                     &device, &vkResult);
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

bool XrVulkanPlatform::isProtected(SwapChain* swapChain) noexcept {
  return static_cast<XrSwapChain*>(swapChain)->GetContentSecurityLevel() ==
         ContentSecurityLevel::kProtected;
}

void XrVulkanPlatform::destroy(SwapChain* swapChain) noexcept {
  IMP_TRACE();
  // Destroyed when it falls out of scope.
  std::unique_ptr<XrSwapChain> swap_chain(static_cast<XrSwapChain*>(swapChain));
}
}  // namespace imp
