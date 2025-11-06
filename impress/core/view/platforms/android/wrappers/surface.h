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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_SURFACE_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_SURFACE_H_

#include <jni.h>

#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/common/context.h"
#include "core/common/jni_helpers.h"
#include "core/render/content_security_level.h"
#include "core/view/platforms/android/wrappers/canvas.h"
#include "core/view/platforms/android/wrappers/rect.h"
#include "core/view/platforms/android/wrappers/surface_texture.h"

namespace imp::android {

// JNI wrapper for the Android Surface class.
class Surface : public JavaWrapper {
 public:
  static absl::StatusOr<std::unique_ptr<Surface>> Create(
      const Context& context, jobject j_surface,
      ContentSecurityLevel security_level);

  static absl::StatusOr<std::unique_ptr<Surface>> Create(
      const Context& context, SurfaceTexture& surface_texture,
      ContentSecurityLevel security_level);

  Surface(const Context& context, jobject j_surface);
  Surface(const Context& context, SurfaceTexture& surface_texture);

  // Locks the canvas for drawing. The entire surface is marked as dirty, so all
  // existing content will be cleared. If there's a need to preserve existing
  // content, use LockCanvas(android::Rect& bounds) to specify the dirty region.
  Canvas LockCanvas();
  // Locks the canvas for drawing. The specified region is marked as dirty, so
  // existing content will be preserved outside of the specified bounds.
  Canvas LockCanvas(android::Rect& bounds);
  Canvas LockHardwareCanvas();
  void UnlockCanvasAndPost(Canvas& canvas);
  ContentSecurityLevel GetContentSecurityLevel() const;

 private:
  Surface(const Context& context, jobject j_surface,
          ContentSecurityLevel security_level);
  Surface(const Context& context, SurfaceTexture& surface_texture,
          ContentSecurityLevel security_level);

  absl::Status Initialize();
  void InitializeJniHandles();

  JniHandle lock_canvas_;
  JniHandle lock_hardware_canvas_;
  JniHandle unlock_canvas_and_post_;

  ContentSecurityLevel security_level_;
};

}  // namespace imp::android

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_SURFACE_H_
