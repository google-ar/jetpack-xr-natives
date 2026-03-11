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
#include "core/materials/compiler/cache/material_cache.h"

#include <sys/stat.h>
#include <utime.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "core/async/future.h"
#include "core/common/context.h"
#include "core/materials/compiler/cache/file_utilities.h"
#include "third_party/openssl/sha2.h"

namespace {

constexpr int64_t kMaxCacheSizeBytes = 16 * 1024 * 1024;  // 16MB
inline constexpr const char* kCacheFileSuffix = ".cmat";
inline constexpr const char* kMaterialCacheFolderName = "/material-cache";

struct CacheEntry {
  std::string path;
  int64_t size;
  time_t mtime;
  bool operator<(const CacheEntry& other) const { return mtime < other.mtime; }
};

}  // namespace

namespace imp {

Future<std::unique_ptr<MaterialCache>> MaterialCache::Create(
    std::unique_ptr<FileUtilities> file_utils) {
  Future<std::unique_ptr<MaterialCache>> create_cache_future;
  std::unique_ptr<MaterialCache> cache =
      absl::WrapUnique(new MaterialCache(std::move(file_utils)));
  if (absl::Status status = cache->Initialize(); !status.ok()) {
    create_cache_future.Return(status);
  } else {
    create_cache_future.Return(std::move(cache));
  }
  return create_cache_future;
}

Future<std::unique_ptr<MaterialCache>> MaterialCache::Create(
    const Context& context) {
  Future<std::unique_ptr<MaterialCache>> create_cache_future;
  std::unique_ptr<MaterialCache> cache =
      absl::WrapUnique(new MaterialCache(CreateFileUtilities(context)));
  if (absl::Status status = cache->Initialize(); !status.ok()) {
    create_cache_future.Return(status);
  } else {
    create_cache_future.Return(std::move(cache));
  }
  return create_cache_future;
}

MaterialCache::MaterialCache(const Context& context)
    : MaterialCache(CreateFileUtilities(context)) {}
MaterialCache::MaterialCache(std::unique_ptr<FileUtilities> file_utils)
    : file_utils_(std::move(file_utils)) {}

// NOTE: We use a SHA256 cryptographically secure hash so that a malicious actor
// cannot create a material with the same hash and lead other apps to using a
// wrong material. If we wanted to use a simpler hash function, we could get the
// application name from the bridge and store materials in a folder per app to
// avoid the issue of hash collision across apps.
MaterialHash MaterialCache::Hash(std::string_view material_source) {
  uint8_t sha_hash[kHashByteSize];
  const uint8_t* data =
      reinterpret_cast<const uint8_t*>(material_source.data());
  SHA256(data, material_source.size(), sha_hash);
  MaterialHash mat_hash;
  // Copy the hash bytes into the MaterialHash array.
  std::copy(std::begin(sha_hash), std::end(sha_hash), std::begin(mat_hash));
  return mat_hash;
}

absl::StatusOr<std::vector<uint8_t>> MaterialCache::Get(MaterialHash hash) {
  LockFile(hash);

  // Check if the material is stored as a .cmat file.
  const std::string file_path = GetFilePath(hash);
  if (!file_utils_->Exists(file_path)) {
    UnlockFile(hash);
    return absl::NotFoundError("Material not found in cache.");
  }

  std::vector<uint8_t> buffer;
  if (absl::Status status = file_utils_->ReadFile(file_path, buffer);
      !status.ok()) {
    UnlockFile(hash);
    return status;
  }

  // Update modification time to mark it as recently used.
  // NOTE: If we care to always exactly remove the least recently used files
  // when the cache is full, we would need to lock mutex_ here, because the
  // modification time is used to determine which file gets deleted in
  // EnsureCapacity(). However, locking the mutex reduces read performance, so
  // we decide to not lock the mutex here, trading accuracy of LRU for better
  // performance.
  if (utime(file_path.c_str(), nullptr) != 0) {
    // This is not a critical error, because we can still return the material
    // and use it, so we don't return an error.
    IMP_LOG(imp::ERROR) << "Failed to update modification time for file: " << file_path;
  }

  UnlockFile(hash);
  return buffer;
}

absl::Status MaterialCache::Store(
    MaterialHash hash, std::vector<uint8_t> compiled_material_bytes) {
  // Make sure that the file is not currently compiled or stored.
  LockFile(hash);

  const std::string file_path = GetFilePath(hash);
  int64_t old_size = file_utils_->GetFileSize(file_path);
  int64_t size_increase = compiled_material_bytes.size() - old_size;
  EnsureCapacity(size_increase);

  if (absl::Status status =
          file_utils_->WriteFile(file_path, compiled_material_bytes);
      !status.ok()) {
    UnlockFile(hash);
    return status;
  }

  UnlockFile(hash);

  absl::MutexLock lock(mutex_);
  current_cache_size_bytes_ += size_increase;
  return absl::OkStatus();
}

absl::Status MaterialCache::Initialize() {
  cache_dir_ = file_utils_->GetCacheDir() + kMaterialCacheFolderName;

  // Create the folder for cache_dir_ if it doesn't exist.
  if (absl::Status status = file_utils_->MkdirRecursive(cache_dir_);
      !status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to create cache directory: " << cache_dir_
               << " with status: " << status;
    return status;
  }

  InitializeCacheSize();
  DeleteAllStaleCacheEntries();

  return absl::OkStatus();
}

void MaterialCache::InitializeCacheSize() ABSL_LOCKS_EXCLUDED(mutex_) {
  absl::MutexLock lock(mutex_);
  int64_t cache_size = 0;
  file_utils_->ForEachFileInDir(
      cache_dir_, kCacheFileSuffix,
      [&](const std::string& path, const struct stat& statbuf) {
        cache_size += statbuf.st_size;
      });

  current_cache_size_bytes_ = cache_size;
}

void MaterialCache::DeleteAllStaleCacheEntries() ABSL_LOCKS_EXCLUDED(mutex_) {
  absl::MutexLock lock(mutex_);
  // If a file with an older Filament version exists, delete it.
  int64_t deleted_bytes = 0;
  file_utils_->ForEachFileInDir(
      cache_dir_, kCacheFileSuffix,
      [&](const std::string& path, const struct stat& statbuf) {
        int file_version = 0;

        // Parse the file name to check for "hash_version", e.g. "123456789_1".
        std::string stem = file_utils_->Stem(path);
        size_t last_underscore = stem.find_last_of('_');
        bool parse_success = last_underscore != absl::string_view::npos;
        if (!parse_success) {
          IMP_LOG(imp::ERROR) << "No \"_version\" found in filename: " << path;
        } else {
          std::string version_str = stem.substr(last_underscore + 1);
          parse_success = absl::SimpleAtoi(version_str, &file_version);
        }

        if (!parse_success || file_version < filament::MATERIAL_VERSION) {
          if (std::remove(path.c_str()) == 0) {
            deleted_bytes += statbuf.st_size;
          } else {
            IMP_LOG(imp::ERROR) << "Failed to delete old cache file: " << path;
          }
        }
      });

  current_cache_size_bytes_ -= deleted_bytes;
}

std::string MaterialCache::GetFilePath(MaterialHash hash) {
  // Add the filament version to the file name, so that we recompile the
  // material if the filament version changes.
  return absl::StrFormat("%s/%s_%d.cmat", cache_dir_, absl::StrJoin(hash, ""),
                         filament::MATERIAL_VERSION);
}

void MaterialCache::EnsureCapacity(int64_t bytes_needed)
    ABSL_LOCKS_EXCLUDED(mutex_) {
  absl::MutexLock lock(mutex_);

  if (current_cache_size_bytes_ + bytes_needed <= kMaxCacheSizeBytes) {
    return;
  }

  // Get all entries in the cache directory
  std::vector<CacheEntry> entries;
  file_utils_->ForEachFileInDir(
      cache_dir_, kCacheFileSuffix,
      [&](const std::string& path, const struct stat& statbuf) {
        entries.push_back({path, statbuf.st_size, statbuf.st_mtime});
      });

  // Sort entries by modification time, so that we delete the least recently
  // used files first.
  std::sort(entries.begin(), entries.end());

  for (const auto& entry : entries) {
    if (current_cache_size_bytes_ + bytes_needed <= kMaxCacheSizeBytes) {
      break;
    }
    if (std::remove(entry.path.c_str()) == 0) {
      current_cache_size_bytes_ -= entry.size;
    } else {
      IMP_LOG(imp::ERROR) << "Failed to delete cache file: " << entry.path;
    }
  }
}

void MaterialCache::LockFile(MaterialHash hash) ABSL_LOCKS_EXCLUDED(mutex_) {
  absl::MutexLock lock(mutex_);
  auto is_file_unlocked = [this, hash]() {
    mutex_.AssertHeld();
    return !file_locks_.contains(hash);
  };
  mutex_.Await(absl::Condition(&is_file_unlocked));
  file_locks_.insert(hash);
}

void MaterialCache::UnlockFile(MaterialHash hash) ABSL_LOCKS_EXCLUDED(mutex_) {
  absl::MutexLock lock(mutex_);
  file_locks_.erase(hash);
}

}  // namespace imp
