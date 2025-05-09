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

#include "core/view/platforms/xr_android/xr_opengl_platform.h"

#include <cstddef>
#include <cstdint>
#include <memory>

#include "core/common/log.h"
#include "filament/filament/include/filament/SwapChain.h"
#include "core/render/content_security_level.h"
// Fix for EGL & X11 defining a Status macro that interferes with absl::Status.
#if defined(Status)
#undef Status
typedef int Status;
#endif
// Fix for EGL & X11 defining a Bool macro that interferes with absl::Status.
#if defined(Bool)
#undef Bool
typedef int Bool;
#endif
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "filament/filament/backend/include/backend/Platform.h"
#include "filament/filament/backend/src/opengl/OpenGLContext.h"
#include "core/common/platform_helpers.h"
#include "core/common/trace.h"
#include "core/config.h"
#include "core/view/platforms/xr_android/openxr_includes.h"
#include "core/view/platforms/xr_android/xr_session_host.h"
#include "core/view/platforms/xr_android/xr_swap_chain.h"
namespace imp {
XrGraphicsBindingOpenGLESAndroidKHR XrOpenGLPlatform::GetGraphicsBinding() {
#if IMP_PLATFORM(ANDROID)
  return XrGraphicsBindingOpenGLESAndroidKHR{
      .type = XR_TYPE_GRAPHICS_BINDING_OPENGL_ES_ANDROID_KHR,
      .next = nullptr,
      .display = mEGLDisplay,
      .config = mEGLConfig,
      .context = mEGLContext,
  };
#else
  return {};
#endif
}

filament::backend::Driver* XrOpenGLPlatform::createDriver(
    void* sharedContext, const Platform::DriverConfig& driverConfig) noexcept {
#if IMP_PLATFORM(ANDROID)
  filament::backend::Driver* driver =
      XrPlatformBase::createDriver(sharedContext, driverConfig);

  return driver;
#else
  // This is necessary for this class to compile outside of Android to get past
  // presubmit.
  // TODO: Investigate why this is necessary.
  return nullptr;
#endif
}

size_t GetFboIndex(
    filament::backend::OpenGLPlatform::ContextType context_type) noexcept {
  switch (context_type) {
    case filament::backend::OpenGLPlatform::ContextType::PROTECTED:
      return 1;
      break;
    default:
      return 0;
  }
}

uint32_t XrOpenGLPlatform::getDefaultFramebufferObject() noexcept {
  ContextType context_type = getCurrentContextType();
  size_t fbo_index = GetFboIndex(context_type);
  return default_fbos_[fbo_index];
}

filament::backend::Platform::SwapChain* XrOpenGLPlatform::createSwapChain(
    void* nativewindow, uint64_t flags) noexcept {
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

bool XrOpenGLPlatform::isSwapChainProtected(SwapChain* swapChain) noexcept {
  return static_cast<XrSwapChain*>(swapChain)->GetContentSecurityLevel() ==
         ContentSecurityLevel::kProtected;
}

void XrOpenGLPlatform::destroySwapChain(SwapChain* swapChain) noexcept {
  IMP_TRACE();
  // Destroyed when it falls out of scope.
  std::unique_ptr<XrSwapChain> swap_chain(static_cast<XrSwapChain*>(swapChain));
}

bool XrOpenGLPlatform::makeCurrent(ContextType type, SwapChain* drawSwapChain,
                                   SwapChain* readSwapChain) noexcept {
#if IMP_PLATFORM(ANDROID)
  IMP_TRACE();

  EGLContext context = getContextForType(type);
  EGLBoolean const success =
      filament::backend::PlatformEGL::makeCurrent(context);
  if (!success) {
    IMP_LOG(imp::ERROR) << "Error calling eglMakeCurrent";
    return false;
  }
  if (type != current_context_type_ &&
      current_context_type_ == ContextType::PROTECTED) {
    default_fbos_[1] = 0;
  }
  current_context_type_ = type;
  size_t fbo_index = GetFboIndex(type);
  if (default_fbos_[fbo_index] == 0) {
    // TODO: Diagnose real source of opengl error on filament
    clearGlError();
    GLuint framebuffer;
    glGenFramebuffers(1, &framebuffer);
    if (glGetError() != GL_NO_ERROR) {
      IMP_LOG(imp::FATAL) << "Filament: Error creating framebuffer";
      return false;
    }
    default_fbos_[fbo_index] = framebuffer;
  }
  uint32_t fbo = default_fbos_[fbo_index];
  absl::Status status = static_cast<XrSwapChain*>(drawSwapChain)
                            ->GetSwapchainImageHandler()
                            .MakeCurrent(fbo);
  if (!status.ok()) {
    IMP_LOG(imp::ERROR) << "Error calling XrSwapChain::MakeCurrent: " << status;
    return false;
  }
#endif
  return true;
}

void XrOpenGLPlatform::commit(Platform::SwapChain* swapChain) noexcept {
  IMP_TRACE();
  XrSwapChain* xr_swap_chain = static_cast<XrSwapChain*>(swapChain);
  absl::Status status = xr_swap_chain->GetSwapchainImageHandler().Commit(
      getDefaultFramebufferObject());
  if (!status.ok()) {
    IMP_LOG(imp::ERROR) << "Error calling XrSwapChain::Commit: " << status;
  }
  if (xr_swap_chain->GetHost()->IsXrFbFoveationEnabled()) {
    status = xr_swap_chain->UpdateFoveationProperties();
    if (!status.ok()) {
      IMP_LOG(imp::ERROR) << "Error calling XrSwapChain::UpdateFoveationProperties: "
                 << status;
    }
  }
}

}  // namespace imp
