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
#include "core/render/content_security_level.h"
#include "core/view/platforms/android/wrappers/canvas.h"
#include "core/view/platforms/android/wrappers/surface_texture.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::android {

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
  return absl::OkStatus();
}

Canvas Surface::LockHardwareCanvas() {
  return Canvas(Env(), CallObjectMethod(lock_hardware_canvas_));
}

void Surface::UnlockCanvasAndPost(Canvas& canvas) {
  CallVoidMethod(unlock_canvas_and_post_, canvas.WeakReference());
}

ContentSecurityLevel Surface::GetContentSecurityLevel() const {
  return security_level_;
}

}  // namespace imp::android
