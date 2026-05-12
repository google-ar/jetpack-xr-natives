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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_SURFACE_TEXTURE_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_SURFACE_TEXTURE_H_

#include <cstdint>

#include "absl/status/status.h"
#include "core/common/context.h"
#include "core/common/jni_helpers.h"
#include "core/math/mat.h"
#include "core/math/vec.h"

namespace imp::android {

// JNI wrapper for the Android SurfaceTexture class.
class SurfaceTexture : public JavaWrapper {
 public:
  explicit SurfaceTexture(const Context& context, bool enable_memory_leak_fix);

  SurfaceTexture(const Context& context, uint32_t texture_id, bool is_secure,
                 bool enable_memory_leak_fix);

  ~SurfaceTexture() override;

  absl::Status SetDefaultBufferSize(int2 size);
  void UpdateTexImage();
  void UpdateTexImageGLContext();
  mat4f GetTransformMatrix();
  int32_t GetDataSpace();

 private:
  JniHandle set_default_buffer_size_;
  JniHandle update_tex_image_;
  JniHandle attach_to_gl_context_;
  JniHandle detach_from_gl_context_;
  JniHandle get_transform_matrix_;
  JniHandle release_;
  uint32_t texture_id_ = 0;
  bool is_secure_ = false;
  bool is_attached_to_secure_gl_context_ = false;
  bool enable_memory_leak_fix_;
#if __ANDROID_API__ >= 33
  JniHandle get_data_space_;
#endif  // __ANDROID_API__ >= 33
};

}  // namespace imp::android

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_SURFACE_TEXTURE_H_
