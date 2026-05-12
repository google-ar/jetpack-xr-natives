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

#include "core/view/platforms/android/wrappers/surface_texture.h"

#include <cstdint>

#include "absl/status/status.h"
#include "core/common/context.h"
#include "core/common/jni_helpers.h"
#include "core/math/mat.h"
#include "core/math/vec.h"

namespace imp::android {

SurfaceTexture::SurfaceTexture(const Context& context,
                               bool enable_memory_leak_fix)
    : JavaWrapper(context, "android/graphics/SurfaceTexture", "(Z)V", false),
      enable_memory_leak_fix_(enable_memory_leak_fix) {
  set_default_buffer_size_ = GetMethodHandle("setDefaultBufferSize", "(II)V");
  update_tex_image_ = GetMethodHandle("updateTexImage", "()V");
  attach_to_gl_context_ = GetMethodHandle("attachToGLContext", "(I)V");
  detach_from_gl_context_ = GetMethodHandle("detachFromGLContext", "()V");
  get_transform_matrix_ = GetMethodHandle("getTransformMatrix", "([F)V");
  release_ = GetMethodHandle("release", "()V");
#if __ANDROID_API__ >= 33
  get_data_space_ = GetMethodHandle("getDataSpace", "()I");
#endif  // __ANDROID_API__ >= 33
}

SurfaceTexture::SurfaceTexture(const Context& context, uint32_t texture_id,
                               bool is_secure, bool enable_memory_leak_fix)
    : JavaWrapper(context, "android/graphics/SurfaceTexture", "(I)V",
                  texture_id),
      texture_id_(texture_id),
      is_secure_(is_secure),
      enable_memory_leak_fix_(enable_memory_leak_fix) {
  set_default_buffer_size_ = GetMethodHandle("setDefaultBufferSize", "(II)V");
  update_tex_image_ = GetMethodHandle("updateTexImage", "()V");
  attach_to_gl_context_ = GetMethodHandle("attachToGLContext", "(I)V");
  detach_from_gl_context_ = GetMethodHandle("detachFromGLContext", "()V");
  release_ = GetMethodHandle("release", "()V");
#if __ANDROID_API__ >= 33
  get_data_space_ = GetMethodHandle("getDataSpace", "()I");
#endif  // __ANDROID_API__ >= 33
}

SurfaceTexture::~SurfaceTexture() {
  // TODO: find the correct way of releasing the surface texture
  // that's still being used. We cannot call Release() here because it breaks
  // detachFromContext() which happened later in the Filament rendering
  // thread.
  // The fix is skip Release() call.
  if (!enable_memory_leak_fix_) {
    CallVoidMethod(release_);
  }
}

absl::Status SurfaceTexture::SetDefaultBufferSize(int2 size) {
  CallVoidMethod(set_default_buffer_size_, size.x, size.y);
  return absl::OkStatus();
}

void SurfaceTexture::UpdateTexImage() {
  // Update the GL context if the texture is not attached to a secure GL
  // context.
  if (is_secure_ && !is_attached_to_secure_gl_context_) {
    UpdateTexImageGLContext();
    is_attached_to_secure_gl_context_ = true;
  }
  CallVoidMethod(update_tex_image_);
}

void SurfaceTexture::UpdateTexImageGLContext() {
  CallVoidMethod(detach_from_gl_context_);
  CallVoidMethod(attach_to_gl_context_, texture_id_);
}

mat4f SurfaceTexture::GetTransformMatrix() {
  mat4f out = mat4f();
  auto mat = Env()->NewFloatArray(16);
  CallVoidMethod(get_transform_matrix_, mat);
  Env()->GetFloatArrayRegion(mat, 0, 16, &out[0][0]);
  Env()->DeleteLocalRef(mat);
  return out;
}

int32_t SurfaceTexture::GetDataSpace() {
#if __ANDROID_API__ >= 33
  return CallIntMethod(get_data_space_);
#else
  return 0;  // ADATASPACE_UNKNOWN
#endif  // __ANDROID_API__ >= 33
}

}  // namespace imp::android
