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

#include "core/render/shader_cache_system.h"

#include <sys/stat.h>

#include <string>

#include "core/common/log.h"
#include "absl/strings/escaping.h"
#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/Platform.h"
#include "core/common/buffer_access.h"
#include "core/common/file_helpers.h"
#include "core/common/platform_helpers.h"
#include "core/ncsb/system.h"
#include "core/view/base_view.h"

#if IMP_PLATFORM(ANDROID)
#include "core/view/platforms/android/wrappers/activity_context.h"
#endif

namespace imp {

constexpr bool ShaderCacheSystem::IsSupportedOnCurrentPlatform(
    const BaseView* view) {
#if IMP_PLATFORM(ANDROID)
  // Only supported on Android if an Activity Context is available.
  return view->GetContext().GetActivityContext() != nullptr;
#else
  return false;
#endif
}

void ShaderCacheSystem::WriteCache(const void* key, size_t keySize,
                                   const void* value, size_t valueSize) {
  std::string filename = absl::WebSafeBase64Escape(
      absl::string_view(static_cast<const char*>(key), keySize));
  std::string cache_path = JoinPath(cache_dir_, filename);

  BufferAccess buffer =
      BufferAccess::Wrap(static_cast<const uint8_t*>(value), valueSize);
  absl::Status save_status = SaveBinary(cache_path, buffer);
  if (!save_status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to save cache: " << save_status;
  }
}

size_t ShaderCacheSystem::ReadCache(const void* key, size_t keySize,
                                    void* value, size_t valueSize) {
  std::string filename = absl::WebSafeBase64Escape(
      absl::string_view(static_cast<const char*>(key), keySize));
  std::string cache_path = JoinPath(cache_dir_, filename);

  BufferAccess buffer;
  absl::Status load_status = LoadBinary(cache_path, &buffer);
  if (load_status.ok()) {
    // Need to make sure the allocated `value` buffer has enough space for the
    // shader binary.
    // According to Filament documentation, the value should only be written
    // when the loaded shader binary size is smaller or equal to the provided
    // buffer size `valueSize`.
    if (buffer.Size() <= valueSize) {
      memcpy(value, buffer.Data(), buffer.Size());
    }
    return buffer.Size();
  }

  return 0;
}

ShaderCacheSystem::ShaderCacheSystem(BaseView* view)
    : System(view), view_(view) {}

void ShaderCacheSystem::Setup() {
  if (!IsSupportedOnCurrentPlatform(view_)) {
    return;
  }

#if IMP_PLATFORM(ANDROID)
  const Context& context = view_->GetContext();
  JNIEnv* env = context.GetJniEnv();
  jobject context_object = context.GetActivityContext();
  android::ActivityContext activity_context(env, context_object);
  cache_dir_ = activity_context.GetCacheDir().GetPath();
#endif
}

void ShaderCacheSystem::EnableShaderCaching() {
  // Setup filament platform function pointers.
  view_->GetSharedEngine()->getPlatform()->setBlobFunc(
      [this](const void* key, size_t keySize, const void* value,
             size_t valueSize) { WriteCache(key, keySize, value, valueSize); },
      [this](const void* key, size_t keySize, void* value, size_t valueSize)
          -> size_t { return ReadCache(key, keySize, value, valueSize); });
}

void ShaderCacheSystem::DisableShaderCaching() {
  view_->GetSharedEngine()->getPlatform()->setBlobFunc({}, {});
  if (view_->GetSharedEngine()->getPlatform()->hasBlobFunc()) {
    IMP_LOG(imp::FATAL) << "Unable to unset blob func.";
  }
}

}  // namespace imp
