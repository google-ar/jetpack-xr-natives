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

#include "core/view/platforms/android/wrappers/surface.h"

#include <dlfcn.h>
#include <jni.h>
#include <sys/types.h>

#include <memory>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "core/common/context.h"
#include "core/common/jni_helpers.h"
#include "core/config.h"
#include "core/render/content_security_level.h"
#include "core/view/platforms/android/wrappers/canvas.h"
#include "core/view/platforms/android/wrappers/surface_texture.h"
#include "mediapipe/framework/port/status_macros.h"

#if IMP_PLATFORM(ANDROID)
#include <android/hardware_buffer.h>
#include <android/native_window.h>
#include <android/native_window_jni.h>
#include <errno.h>

#include <cstdint>
#endif

namespace imp::android {

FPANativeWindow_setUsage Surface::ANativeWindow_setUsage_ = nullptr;
FPANativeWindow_release Surface::ANativeWindow_release_ = nullptr;

Surface::Surface(const Context& context, jobject j_surface)
    : JavaWrapper(context.GetJniEnv(), j_surface) {
  InitializeJniHandles();
}

Surface::Surface(const Context& context, SurfaceTexture& surface_texture)
    : JavaWrapper(context, "android/view/Surface",
                  "(Landroid/graphics/SurfaceTexture;)V",
                  surface_texture.WeakReference()) {
  InitializeJniHandles();
}

void Surface::InitializeJniHandles() {
  lock_hardware_canvas_ =
      GetMethodHandle("lockHardwareCanvas", "()Landroid/graphics/Canvas;");
  unlock_canvas_and_post_ =
      GetMethodHandle("unlockCanvasAndPost", "(Landroid/graphics/Canvas;)V");
}

absl::StatusOr<std::unique_ptr<Surface>> Surface::Create(
    const Context& context, jobject j_surface,
    ContentSecurityLevel security_level) {
  std::unique_ptr<Surface> surface =
      absl::WrapUnique(new Surface(context, j_surface, security_level));
  MP_RETURN_IF_ERROR(surface->Initialize());
  return surface;
}

absl::StatusOr<std::unique_ptr<Surface>> Surface::Create(
    const Context& context, SurfaceTexture& surface_texture,
    ContentSecurityLevel security_level) {
  std::unique_ptr<Surface> surface =
      absl::WrapUnique(new Surface(context, surface_texture, security_level));
  MP_RETURN_IF_ERROR(surface->Initialize());
  return surface;
}

Surface::Surface(const Context& context, jobject j_surface,
                 ContentSecurityLevel security_level)
    : JavaWrapper(context.GetJniEnv(), j_surface),
      security_level_(security_level) {}

Surface::Surface(const Context& context, SurfaceTexture& surface_texture,
                 ContentSecurityLevel security_level)
    : JavaWrapper(context, "android/view/Surface",
                  "(Landroid/graphics/SurfaceTexture;)V",
                  surface_texture.WeakReference()),
      security_level_(security_level) {}

absl::Status Surface::Initialize() {
  InitializeJniHandles();

#if IMP_PLATFORM(ANDROID)
  if (security_level_ == ContentSecurityLevel::kProtected) {
    // Load the necessary runtime symbols.
    if (LoadRuntimeLibraries() != absl::OkStatus() ||
        Surface::LoadRuntimeLibraries() != absl::OkStatus()) {
      return absl::InternalError(
          "Cannot load the required runtime symbols. Can not support protected "
          "content.");
    }

    // Get the native window.
    ANativeWindow* nativeWindow =
        ANativeWindow_fromSurface(Env(), WeakReference());
    if (nativeWindow == nullptr) {
      return absl::InternalError("Failed to get native window.");
    }

    // Set the protected usage flag.
    uint64_t usage = AHARDWAREBUFFER_USAGE_GPU_SAMPLED_IMAGE |  // default
                     AHARDWAREBUFFER_USAGE_GPU_COLOR_OUTPUT |   // default
                     AHARDWAREBUFFER_USAGE_PROTECTED_CONTENT;
    if (-ENOENT == ANativeWindow_setUsage_(nativeWindow, usage)) {
      return absl::InternalError("Failed to set protected usage flag.");
    }

    // Release the native window.
    ANativeWindow_release_(nativeWindow);
  }
#endif
  return absl::OkStatus();
}

Canvas Surface::LockHardwareCanvas() {
  return Canvas(Env(), CallObjectMethod(lock_hardware_canvas_));
}

void Surface::UnlockCanvasAndPost(Canvas& canvas) {
  CallVoidMethod(unlock_canvas_and_post_, canvas.WeakReference());
}

absl::Status Surface::LoadRuntimeLibraries() {
  // Static initializers are guaranteed to be evaluated only once.
  static absl::Status initialized = [] {
    // Closes the library if an error occurred. Used as the deleter for
    // unique_ptr.
    auto library_deleter = [](void* library) { dlclose(library); };

    // Load native window library which is used by the media NDK.
    std::unique_ptr<void, decltype(library_deleter)> libnativewindow(
        dlopen("libnativewindow.so", RTLD_NOW), library_deleter);
    if (!libnativewindow) {
      return absl::InternalError(
          absl::StrCat("Unable to open libnativewindow.so: ", dlerror()));
    }

    ANativeWindow_setUsage_ = reinterpret_cast<FPANativeWindow_setUsage>(
        dlsym(libnativewindow.get(), "ANativeWindow_setUsage"));
    if (!ANativeWindow_setUsage_) {
      return absl::InternalError("Unable to load ANativeWindow_setUsage");
    }

    ANativeWindow_release_ = reinterpret_cast<FPANativeWindow_release>(
        dlsym(libnativewindow.get(), "ANativeWindow_release"));
    if (!ANativeWindow_release_) {
      return absl::InternalError("Unable to load ANativeWindow_release");
    }

    libnativewindow.release();
    return absl::OkStatus();
  }();
  return initialized;
}

ContentSecurityLevel Surface::GetContentSecurityLevel() const {
  return security_level_;
}

}  // namespace imp::android
