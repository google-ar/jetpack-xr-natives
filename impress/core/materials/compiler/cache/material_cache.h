/*
 * Copyright 2025 Google LLC
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
#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_CACHE_MATERIAL_CACHE_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_CACHE_MATERIAL_CACHE_H_

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "core/async/future.h"
#include "core/common/context.h"
#include "core/materials/compiler/cache/file_utilities.h"

namespace imp {

// A 256-bit hash.
using MaterialHash = std::array<uint8_t, 32>;

class MaterialCache {
 public:
  static Future<std::unique_ptr<MaterialCache>> Create(
      std::unique_ptr<FileUtilities> file_utils);
  static Future<std::unique_ptr<MaterialCache>> Create(const Context& context);

  ~MaterialCache() = default;

  // The caller is responsible for hashing, so that the same material only needs
  // to be hashed once for use with Get() and Store().
  MaterialHash Hash(absl::string_view material_source);

  // Returns the compiled material bytes for the given material hash.
  absl::StatusOr<std::vector<uint8_t>> Get(const MaterialHash& hash);

  // Stores the compiled material bytes for the given material hash.
  absl::Status Store(const MaterialHash& hash,
                     std::vector<uint8_t> compiled_material_bytes);

  // Returns the file path at which the material with the given hash may be
  // stored. Adds the filament version to the file name, so that we recompile
  // the material when the filament version changes.
  std::string GetFilePath(
      const MaterialHash& hash,
      int material_version = filament::MATERIAL_VERSION) const;

 private:
  explicit MaterialCache(std::unique_ptr<FileUtilities> file_utils);
  explicit MaterialCache(const Context& context);
  // If it's not already initialized, sets the cache directory, creates the
  // folder if it doesn't exist, and deletes all stale cache entries.
  absl::Status Initialize();

  // Checks how much space the cached materials take up.
  void InitializeCacheSize() ABSL_LOCKS_EXCLUDED(mutex_);

  // Deletes all cache entries that were compiled with an older Filament
  // material version.
  void DeleteAllStaleCacheEntries() ABSL_LOCKS_EXCLUDED(mutex_);

  // Checks if the cache has enough space for the given bytes, and if not,
  // deletes cache entries, least recently used ones first, to make room.
  void EnsureCapacity(int64_t bytes_needed) ABSL_LOCKS_EXCLUDED(mutex_);

  // Waits for the file for the given hash to be accessible and then marks it
  // as being in use. Make sure to call UnlockFile when done.
  void LockFile(const MaterialHash& hash) ABSL_LOCKS_EXCLUDED(mutex_);

  // Releases the file for the given hash so that it can be accessed again.
  void UnlockFile(const MaterialHash& hash) ABSL_LOCKS_EXCLUDED(mutex_);

  std::unique_ptr<FileUtilities> file_utils_;
  absl::Mutex mutex_;
  std::string cache_dir_;
  int64_t current_cache_size_bytes_ ABSL_GUARDED_BY(mutex_) = -1;

  // Set of materials that are currently being worked on. We add the hash that
  // represents a material file to the hashset to mark that this file is being
  // worked on. This is used instead of a global mutex which would block
  // concurrent reads and writes to different files.
  absl::flat_hash_set<MaterialHash> file_locks_ ABSL_GUARDED_BY(mutex_);
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_CACHE_MATERIAL_CACHE_H_
