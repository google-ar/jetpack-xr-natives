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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_OPENGL_PLATFORM_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_OPENGL_PLATFORM_H_

#include <jni.h>

#include <array>
#include <cstdint>

#include "filament/filament/backend/include/backend/Platform.h"
#include "core/config.h"
#include "core/view/platforms/xr_android/openxr_includes.h"

namespace imp {

// It is necessary for this class to compile outside of Android to get past
// presubmit.
// TODO: Investigate why this is necessary.
#if IMP_PLATFORM(ANDROID)
using XrPlatformBase = filament::backend::PlatformEGLAndroid;
#else
using XrPlatformBase = filament::backend::OpenGLPlatform;
#endif

// Custom filament platform for rendering in OpenXR.
//
// Provides functionality for creating & using Xr specific swapchains.
class XrOpenGLPlatform : public XrPlatformBase {
 public:
  XrGraphicsBindingOpenGLESAndroidKHR GetGraphicsBinding();

  filament::backend::Driver* createDriver(
      void* sharedContext,
      const Platform::DriverConfig& driverConfig) noexcept override;

  uint32_t getDefaultFramebufferObject() noexcept override;

  // For OpenXR, nativeWindow is required to be an XrSessionHost.
  // This is passed into Filament through FilamentHost::CreateSwapChain.
  SwapChain* createSwapChain(void* nativewindow,
                             uint64_t flags) noexcept override;

  bool isSwapChainProtected(SwapChain* swapChain) noexcept override;

  void destroySwapChain(SwapChain* swapChain) noexcept override;

  // TODO: Re-enable once issues are fixed.
  bool isCompositorTimingSupported() const noexcept override;

  // TODO: Re-enable once issues with getFrameId are fixed.
  bool setPresentFrameId(SwapChain const* swapchain,
                         uint64_t frameId) noexcept override;
  // TODO: Re-enable once issues with getFrameId are fixed.
  bool queryFrameTimestamps(
      SwapChain const* swapchain, uint64_t frameId,
      FrameTimestamps* outFrameTimestamps) const noexcept override;

  // Called on Filament's rendering thread to bind the swap chain as the current
  // target being rendered into for the frame.
  bool makeCurrent(ContextType type, Platform::SwapChain* drawSwapChain,
                   Platform::SwapChain* readSwapChain) noexcept override;

  // Called on Filament's rendering thread to commit the frame.
  // Ultimately will call xrEndFrame.
  void commit(Platform::SwapChain* swapChain) noexcept override;

// This is necessary for this class to compile outside of Android to get past
// presubmit.
// TODO: Investigate why this is necessary.
#if !IMP_PLATFORM(ANDROID)
  int getOSVersion() const noexcept override { return 0; }
  void terminate() noexcept override {}
  SwapChain* createSwapChain(uint32_t width, uint32_t height,
                             uint64_t flags) noexcept override {
    return nullptr;
  }
#endif

 private:
  std::array<uint32_t, 2> default_fbos_ = {0, 0};
  ContextType current_context_type_ = ContextType::UNPROTECTED;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_OPENGL_PLATFORM_H_
