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

#include "core/view/platforms/xr_android/xr_swap_chain.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/render/content_security_level.h"
#include "core/view/platforms/xr_android/openxr_includes.h"
#if IMP_MATERIAL_API(VULKAN)
#include "core/view/platforms/xr_android/xr_vulkan_swap_chain_image_handler.h"
#else
#include "core/view/platforms/xr_android/xr_opengl_swap_chain_image_handler.h"
#endif
#include "core/view/platforms/xr_android/xr_session_host.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

bool IsFormatSupported(int64_t format) {
  return format == XrSwapChain::ImageHandlerType::kImageFormat;
}

absl::StatusOr<uint32_t> GetSwapchainColorFormat(XrSessionHost* host) {
  XrSession session = host->GetXrSession();

  uint32_t swapchain_format_count;
  MP_RETURN_IF_ERROR(host->ToStatus(xrEnumerateSwapchainFormats(
      session, 0, &swapchain_format_count, nullptr)));
  std::vector<int64_t> swapchain_formats(swapchain_format_count);
  MP_RETURN_IF_ERROR(host->ToStatus(xrEnumerateSwapchainFormats(
      session, swapchain_formats.size(), &swapchain_format_count,
      swapchain_formats.data())));

  std::vector<int64_t>::iterator iter = std::find_if(
      swapchain_formats.begin(), swapchain_formats.end(), IsFormatSupported);
  if (iter == swapchain_formats.end()) {
    return absl::InternalError("No supported swapchain color format found");
  }
  return *iter;
}

absl::StatusOr<XrSwapchain> CreateSwapChain(XrSessionHost* host,
                                            XrSwapchainCreateFlags flags,
                                            bool use_varjo_foveation = false) {
  // WARNING: Called from Filament's Render Thread. It is NOT SAFE to call most
  // Impress or Filament APIs from here.

  MP_ASSIGN_OR_RETURN(uint32_t swapchain_color_format,
                   GetSwapchainColorFormat(host));

  uint2 display_size = host->GetDisplaySize();
  if (use_varjo_foveation) {
    display_size = host->GetVarjoFoveationDisplaySize();
  }

  XrSwapchainCreateInfo swapchain_create_info = {
      .type = XR_TYPE_SWAPCHAIN_CREATE_INFO,
      .next = nullptr,
      .createFlags = flags,
      .usageFlags = XR_SWAPCHAIN_USAGE_SAMPLED_BIT |
                    XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT,
      .format = swapchain_color_format,
      .sampleCount = host->GetViewSampleCount(),
      .width = display_size.x,
      .height = display_size.y,
      .faceCount = 1,
      .arraySize = host->IsMultiviewStereo() ? host->GetLogicalEyeCount() : 1,
      .mipCount = 1,
  };

  if (host->IsXrFbFoveationEnabled()) {
    XrSwapchainCreateInfoFoveationFB swapChainFoveationCreateInfo;
    memset(&swapChainFoveationCreateInfo, 0,
           sizeof(swapChainFoveationCreateInfo));
    swapChainFoveationCreateInfo.type =
        XR_TYPE_SWAPCHAIN_CREATE_INFO_FOVEATION_FB;
    swapChainFoveationCreateInfo.flags =
        XrSwapChain::ImageHandlerType::kFoveationFlag;
    swapchain_create_info.next = &swapChainFoveationCreateInfo;
  }

  XrSwapchain swapchain = XR_NULL_HANDLE;

  MP_RETURN_IF_ERROR(host->ToStatus(xrCreateSwapchain(
      host->GetXrSession(), &swapchain_create_info, &swapchain)));

  return swapchain;
}

absl::StatusOr<XrSwapchain> CreateDepthSwapChain(
    XrSessionHost* host, XrSwapchainCreateFlags flags = 0) {
  // WARNING: Called from Filament's Render Thread. It is NOT SAFE to call most
  // Impress or Filament APIs from here.

  uint2 display_size = host->GetDisplaySize();

  const int64_t depth_format =
      host->GetState()->ShouldUseStencilSwapChain()
          ? XrSwapChain::ImageHandlerType::kDepthStencilFormat
          : XrSwapChain::ImageHandlerType::kDepthFormat;

  XrSwapchainCreateInfo swapchain_create_info = {
      .type = XR_TYPE_SWAPCHAIN_CREATE_INFO,
      .next = nullptr,
      .createFlags = flags,
      .usageFlags = XR_SWAPCHAIN_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
      .format = depth_format,
      .sampleCount = host->GetViewSampleCount(),
      .width = display_size.x,
      .height = display_size.y,
      .faceCount = 1,
      .arraySize = host->IsMultiviewStereo() ? host->GetLogicalEyeCount() : 1,
      .mipCount = 1,
  };

  XrSwapchain depth_swapchain = XR_NULL_HANDLE;
  MP_RETURN_IF_ERROR(host->ToStatus(xrCreateSwapchain(
      host->GetXrSession(), &swapchain_create_info, &depth_swapchain)));

  return depth_swapchain;
}

absl::StatusOr<std::vector<XrSwapChain::ImageHandlerType::XrSwapChainImage>>
CreateSwapChainImages(XrSessionHost* host, XrSwapchain swapchain) {
  // WARNING: Called from Filament's Render Thread. It is NOT SAFE to call most
  // Impress or Filament APIs from here.

  uint32_t image_count;
  MP_RETURN_IF_ERROR(host->ToStatus(
      xrEnumerateSwapchainImages(swapchain, 0, &image_count, nullptr)));

  std::vector<XrSwapChain::ImageHandlerType::XrSwapChainImage> image_buffer(
      image_count, {.type = XrSwapChain::ImageHandlerType::kImageType});

  MP_RETURN_IF_ERROR(host->ToStatus(xrEnumerateSwapchainImages(
      swapchain, image_count, &image_count,
      reinterpret_cast<XrSwapchainImageBaseHeader*>(image_buffer.data()))));

  return image_buffer;
}

absl::StatusOr<std::unique_ptr<XrSwapChain>> XrSwapChain::Create(
    XrSwapChain::ImageHandlerType::XrPlatformType* platform,
    XrSessionHost* host, XrSwapchainCreateFlags flags) {
  // WARNING: Called from Filament's Render Thread. It is NOT SAFE to call most
  // Impress or Filament APIs from here.

  MP_ASSIGN_OR_RETURN(XrSwapchain swapchain_color, CreateSwapChain(host, flags));
  MP_ASSIGN_OR_RETURN(std::vector<XrSwapChain::ImageHandlerType::XrSwapChainImage>
                       swapchain_color_images,
                   CreateSwapChainImages(host, swapchain_color));

  XrSwapchain swapchain_varjo_foveation_color = XR_NULL_HANDLE;
  std::vector<XrSwapChain::ImageHandlerType::XrSwapChainImage>
      swapchain_varjo_foveation_color_images;
  if (host->IsXrVarjoFoveatedRenderingEnabled()) {
    MP_ASSIGN_OR_RETURN(
        swapchain_varjo_foveation_color,
        CreateSwapChain(host, flags, /*use_varjo_foveation=*/true));
    MP_ASSIGN_OR_RETURN(
        swapchain_varjo_foveation_color_images,
        CreateSwapChainImages(host, swapchain_varjo_foveation_color));
  }

  XrSwapchain swapchain_depth = XR_NULL_HANDLE;
  std::vector<XrSwapChain::ImageHandlerType::XrSwapChainImage>
      swapchain_depth_images;
  if (host->IsCompositionLayerDepthEnabled()) {
    MP_ASSIGN_OR_RETURN(swapchain_depth, CreateDepthSwapChain(host, flags));
    MP_ASSIGN_OR_RETURN(swapchain_depth_images,
                     CreateSwapChainImages(host, swapchain_depth));
  }

  XrSwapChain::ImageHandlerType::SwapchainLayers swapchain_layers;
  swapchain_layers.default_color = {swapchain_color,
                                    std::move(swapchain_color_images)};
  swapchain_layers.varjo_foveation_color = {
      swapchain_varjo_foveation_color,
      std::move(swapchain_varjo_foveation_color_images)};
  swapchain_layers.depth = {swapchain_depth, std::move(swapchain_depth_images)};

  std::unique_ptr<XrSwapChain> xr_swapchain = absl::WrapUnique(
      new XrSwapChain(platform, host, swapchain_layers,
                      flags & XR_SWAPCHAIN_CREATE_PROTECTED_CONTENT_BIT
                          ? ContentSecurityLevel::kProtected
                          : ContentSecurityLevel::kNone));
  if (host->IsXrFbFoveationEnabled()) {
    MP_RETURN_IF_ERROR(xr_swapchain->ResolveFoveationFunctionPointers());
  }

  return xr_swapchain;
}

XrSwapChain::XrSwapChain(
    XrSwapChain::ImageHandlerType::XrPlatformType* platform,
    XrSessionHost* host, XrSwapChain::ImageHandlerType::SwapchainLayers layers,
    ContentSecurityLevel content_security_level)
    : host_(host),
      content_security_level_(content_security_level),
      image_handler_(
          platform, host,
          std::make_unique<XrSwapChain::ImageHandlerType::SwapchainLayers>(
              layers),
          content_security_level) {}

XrSwapChain::~XrSwapChain() {
  if (image_handler_.GetSwapchainLayers()->default_color.handle !=
      XR_NULL_HANDLE) {
    absl::Status status = host_->ToStatus(xrDestroySwapchain(
        image_handler_.GetSwapchainLayers()->default_color.handle));
    if (!status.ok()) {
      IMP_LOG(imp::FATAL) << "Unable to destroy XrSwapchain: " << status;
    }
  }

  if (image_handler_.GetSwapchainLayers()->varjo_foveation_color.handle !=
      XR_NULL_HANDLE) {
    absl::Status status = host_->ToStatus(xrDestroySwapchain(
        image_handler_.GetSwapchainLayers()->varjo_foveation_color.handle));
    if (!status.ok()) {
      IMP_LOG(imp::FATAL) << "Unable to destroy XrSwapchain of foveation texture: "
                 << status;
    }
  }

  if (image_handler_.GetSwapchainLayers()->depth.handle != XR_NULL_HANDLE &&
      host_->IsCompositionLayerDepthEnabled()) {
    absl::Status status = host_->ToStatus(
        xrDestroySwapchain(image_handler_.GetSwapchainLayers()->depth.handle));
    if (!status.ok()) {
      IMP_LOG(imp::FATAL) << "Unable to destroy XrSwapchain of depth texture: "
                 << status;
    }
  }
}

absl::Status XrSwapChain::ResolveFoveationFunctionPointers() {
  MP_RETURN_IF_ERROR(host_->ToStatus(xrGetInstanceProcAddr(
      host_->GetXrInstance(), "xrUpdateSwapchainFB",
      reinterpret_cast<PFN_xrVoidFunction*>(&xr_update_swapchain_fn_))));
  MP_RETURN_IF_ERROR(host_->ToStatus(xrGetInstanceProcAddr(
      host_->GetXrInstance(), "xrCreateFoveationProfileFB",
      reinterpret_cast<PFN_xrVoidFunction*>(
          &xr_create_foveation_profile_fn_))));
  MP_RETURN_IF_ERROR(host_->ToStatus(xrGetInstanceProcAddr(
      host_->GetXrInstance(), "xrDestroyFoveationProfileFB",
      reinterpret_cast<PFN_xrVoidFunction*>(
          &xr_destroy_foveation_profile_fn_))));
  return absl::OkStatus();
}

absl::Status XrSwapChain::UpdateFoveationProperties() {
  XrFoveationLevelFB host_foveation_level = host_->GetCurrentFoveationLevel();
  if (host_foveation_level == current_foveation_level_) {
    return absl::OkStatus();
  }
  current_foveation_level_ = host_foveation_level;

  XrFoveationLevelProfileCreateInfoFB levelProfileCreateInfo{
      .type = XR_TYPE_FOVEATION_LEVEL_PROFILE_CREATE_INFO_FB,
      .level = current_foveation_level_,
      .verticalOffset = 0.0f,
      .dynamic = XR_FOVEATION_DYNAMIC_DISABLED_FB};

  XrFoveationProfileCreateInfoFB profileCreateInfo{
      .type = XR_TYPE_FOVEATION_PROFILE_CREATE_INFO_FB,
      .next = &levelProfileCreateInfo,
  };

  XrFoveationProfileFB foveationProfile;
  MP_RETURN_IF_ERROR(host_->ToStatus(xr_create_foveation_profile_fn_(
      host_->GetXrSession(), &profileCreateInfo, &foveationProfile)));

  XrSwapchainStateFoveationFB foveationUpdateState{
      .type = XR_TYPE_SWAPCHAIN_STATE_FOVEATION_FB,
      .flags = 0,
      .profile = foveationProfile,
  };

  MP_RETURN_IF_ERROR(host_->ToStatus(xr_update_swapchain_fn_(
      image_handler_.GetSwapchainLayers()->default_color.handle,
      reinterpret_cast<XrSwapchainStateBaseHeaderFB*>(&foveationUpdateState))));

  xr_destroy_foveation_profile_fn_(foveationProfile);
  return absl::OkStatus();
}

ContentSecurityLevel XrSwapChain::GetContentSecurityLevel() {
  return content_security_level_;
}

}  // namespace imp
