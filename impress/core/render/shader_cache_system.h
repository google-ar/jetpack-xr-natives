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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_SHADER_CACHE_SYSTEM_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_SHADER_CACHE_SYSTEM_H_

#include <cstddef>
#include <string>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/ncsb/system.h"
#include "core/view/base_view.h"

namespace imp {

// ShaderCacheSystem caches compiled shader programs as binary files and loads
// them back when requested.
//
// Internally, this class utilizes filament's platform APIs.
// filament::backend::Platform::setBlobFunc() allows us to provide custom
// functions for handling shader program binary caching and loading.
//
// TODO: Add testing for ShaderCacheSystem.
// TODO: Add support for iOS.
// TODO: Add support for WASM.
// TODO: Add support for Vulkan.
// TODO: Cap the cache blob size.
class ShaderCacheSystem : public System {
 public:
  explicit ShaderCacheSystem(BaseView* view);

  ~ShaderCacheSystem();

  // Returns true if shader caching is supported on the current platform.
  constexpr static bool IsSupportedOnCurrentPlatform(const BaseView* view);

  // Sets up the ShaderCacheSystem. Shader caching will only be enabled after
  // calling Setup().
  void Setup();

  // Enables shader caching. Please note that this function should only be
  // called within or after imp::View::Setup().
  void EnableShaderCaching();

  // Disables shader caching.
  void DisableShaderCaching();

 private:
  using ShaderCacheBlobs = absl::flat_hash_map<std::string, absl::Cord>;

  void WriteCache(const void* key, size_t keySize, const void* value,
                  size_t valueSize);

  size_t ReadCache(const void* key, size_t keySize, void* value,
                   size_t valueSize);

  // Load a single binary blob that contains all the shader cache.
  static Future<ShaderCacheBlobs> LoadCacheFromFile(
      absl::string_view file_path);

  BaseView* view_;
  std::string cache_dir_;
  std::string resolved_cache_path_;

  absl::Mutex cache_lock_;
  // Map of shader keys to their owned compiled binary blobs.
  // Loaded from disk at startup and updated dynamically at compile time.
  ShaderCacheBlobs cache_blobs_ ABSL_GUARDED_BY(cache_lock_);
  // Used to write shader cached blobs in WriteCache.
  // WriteCache can be called from Filament's CompilerThread or the main render
  // thread. When called on the render thread, we must run the write in the
  // background to avoid blocking rendering. We cache the executor during
  // Setup() to ensure it is available from either calling thread.
  Executor* background_executor_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_SHADER_CACHE_SYSTEM_H_
