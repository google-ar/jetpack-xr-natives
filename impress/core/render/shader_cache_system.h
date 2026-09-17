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
#include <cstdint>
#include <list>
#include <optional>
#include <string>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/optional.h"
#include "core/async/future.h"
#include "core/ncsb/system.h"
#include "core/view/base_view.h"

namespace imp {

// ShaderCacheSystem caches compiled shader programs as a single binary file and
// loads them in memory on Setup, and writes them to disk on ViewPaused event.
// It evicts entries using an LRU policy when the cache size reaches
// max_num_cache_entry_.
//
// Internally, this class utilizes filament's platform APIs.
// filament::backend::Platform::setBlobFunc() allows us to provide custom
// functions for handling shader program binary caching and loading.
//
// TODO: Add support for iOS.
// TODO: Add support for WASM.
// TODO: Add support for Vulkan.
class ShaderCacheSystem : public System {
 public:
  explicit ShaderCacheSystem(
      BaseView* view,
      absl::optional<int32_t> max_num_cache_entry = std::nullopt);

  ~ShaderCacheSystem();

  // Sets up the ShaderCacheSystem. Shader caching will only be enabled after
  // calling Setup().
  void Setup();

  // Enables shader caching. Please note that this function should only be
  // called within or after imp::View::Setup().
  void EnableShaderCaching();

  // Disables shader caching.
  void DisableShaderCaching();

 private:
  struct ShaderCacheEntry {
    // Actual compiled shader blob.
    absl::Cord blob;
    // Iterator pointing to the key's position in lru_keys_. Without this we
    // need to look up twice: one for the cache_blob_ and the other for
    // lru_keys_. (lru_keys_ look up is esp expensive since it's O(N))
    std::list<std::string>::iterator lru_it;
  };
  using ShaderCacheBlobs = absl::flat_hash_map<std::string, ShaderCacheEntry>;

  // Note: WriteCache and ReadCache can be called from multiple threads
  // (CompilerThread or main thread).
  void WriteCache(const void* key, size_t keySize, const void* value,
                  size_t valueSize);

  size_t ReadCache(const void* key, size_t keySize, void* value,
                   size_t valueSize);

  static Future<absl::Status> WriteCacheToDisk(std::string file_path,
                                               ShaderCacheBlobs cache_blobs,
                                               std::list<std::string> lru_keys);

  // Load a single binary blob that contains all the shader cache.
  static Future<absl::flat_hash_map<std::string, absl::Cord>> LoadCacheFromDisk(
      absl::string_view file_path);

  // Returns true if shader caching is supported on the current platform.
  constexpr static bool IsSupportedOnCurrentPlatform(const BaseView* view);

  BaseView* view_;
  std::string cache_dir_;
  std::string resolved_cache_path_;
  const size_t max_num_cache_entry_;
  bool cache_loaded_ ABSL_GUARDED_BY(cache_lock_) = false;

  absl::Mutex cache_lock_;
  // Map of shader keys to their owned compiled binary blobs.
  // Loaded from disk at startup and updated dynamically at compile time.
  ShaderCacheBlobs cache_blobs_ ABSL_GUARDED_BY(cache_lock_);
  // Tracks the least recently used cache entries. Ordered in most to least
  // recent entries.
  std::list<std::string> lru_keys_ ABSL_GUARDED_BY(cache_lock_);

  friend struct ShaderCacheSystemTestPeer;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_SHADER_CACHE_SYSTEM_H_
