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

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <list>
#include <optional>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/Platform.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/common/buffer_access.h"
#include "core/common/file_helpers.h"
#include "core/common/trace.h"
#include "core/config.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/system.h"
#include "core/view/base_view.h"
#include "core/view/view_events.h"

#if IMP_PLATFORM(ANDROID)
#include "core/view/platforms/android/wrappers/activity_context.h"
#endif

namespace {

constexpr absl::string_view kShaderCacheFileName = "shader_blob_cache.bin";
constexpr size_t kKeySize = sizeof(uint32_t);
constexpr size_t kBlobSize = sizeof(uint32_t);
static std::atomic<size_t> cache_write_counter = 0;

// We store all compiled shader binaries inside a single database file.
// The file is structured as a sequential Key-Length-Value (KLV) binary stream
//
// [key1_size (4B)] [key1_data (key_size B)] [blob1_size (4B)]
// [blob1_data (blob_size B)],
// [key2_size (4B)] [key2_data (key_size B)] [blob2_size (4B)]
// [blob2_data (blob_size B)],
// ...
// This format was chosen over a directory index header because it avoids
// having to rewrite the entire file when appending new cache entries.
struct BlobEntry {
  // Shader key.
  absl::string_view key;
  // Pointer to the blob.
  absl::string_view blob;
};

std::optional<BlobEntry> ReadNextEntry(absl::Span<const uint8_t> data,
                                       size_t& offset) {
  // Reached end of file.
  if (offset + kKeySize > data.size()) {
    return std::nullopt;
  }

  // Read the key size first.
  uint32_t key_size = 0;
  std::memcpy(&key_size, data.data() + offset, kKeySize);

  // Move the cursor at the start of key data.
  offset += kKeySize;

  if (offset + key_size + kBlobSize > data.size()) {
    IMP_LOG(imp::ERROR) << "Error reading shader cache key: size exceeds buffer limit.";
    return std::nullopt;
  }

  // Read key data.
  // Note: The data does not need to be null-terminated because we pass key_size
  // explicitly when constructing absl::string_view.
  absl::string_view key_view(
      reinterpret_cast<const char*>(data.data() + offset), key_size);
  offset += key_size;

  // Get blob size.
  uint32_t blob_size = 0;
  std::memcpy(&blob_size, data.data() + offset, kBlobSize);
  offset += kBlobSize;

  if (offset + blob_size > data.size()) {
    IMP_LOG(imp::ERROR) << "Error reading shader cache value: data is truncated.";
    return std::nullopt;
  }

  // Get blob data.
  // Note: The data does not need to be null-terminated as we pass the size
  // explicitly when constructing absl::string_view. Also the binary blob data
  // can contain arbitrary zero bytes (0x00).
  absl::string_view blob_view = absl::string_view(
      reinterpret_cast<const char*>(data.data() + offset), blob_size);
  offset += blob_size;

  return BlobEntry{.key = key_view, .blob = blob_view};
}

size_t GetMaxNumCacheEntry(absl::optional<int32_t> max_num_cache_entry) {
  // Each shader cache entry size 20-25KB. This will roughly consume 20-25MB of
  // disk storage.
  static constexpr size_t kDefaultMaxNumEntry = 1000;

  if (max_num_cache_entry.has_value()) {
    
    return *max_num_cache_entry;
  }
  return kDefaultMaxNumEntry;
}

};  // namespace

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
  IMP_TRACE();
  // Insert the entry into the in-memory map first.
  std::string key_str(static_cast<const char*>(key), keySize);
  absl::Cord value_cord(
      absl::string_view(static_cast<const char*>(value), valueSize));
  {
    // The key exists. Update the entry in the tracker to the front.
    absl::MutexLock lock(cache_lock_);
    if (auto map_it = cache_blobs_.find(key_str);
        map_it != cache_blobs_.end()) {
      auto lru_it = map_it->second.lru_it;
      lru_keys_.splice(lru_keys_.begin(), lru_keys_, lru_it);
      return;
    }

    // The key is new. Check if we are at capacity.
    if (lru_keys_.size() >= max_num_cache_entry_) {
      auto evict_key = lru_keys_.back();
      if (auto evict_it = cache_blobs_.find(evict_key);
          evict_it != cache_blobs_.end()) {
        cache_blobs_.erase(evict_it);
      }
      // Also remove the key from the LRU keys.
      lru_keys_.pop_back();
    }
    // Now we insert the entry into the map and the lru keys.
    lru_keys_.emplace_front(key_str);
    cache_blobs_.try_emplace(key_str,
                             ShaderCacheEntry{value_cord, lru_keys_.begin()});
  }
}

size_t ShaderCacheSystem::ReadCache(const void* key, size_t keySize,
                                    void* value, size_t valueSize) {
  IMP_TRACE();
  // Look up the key in the map.
  absl::string_view key_view(static_cast<const char*>(key), keySize);
  absl::MutexLock lock(cache_lock_);
  auto it = cache_blobs_.find(key_view);
  if (it == cache_blobs_.end()) {
    return 0;
  }

  absl::Cord shader_data = it->second.blob;
  // Update the order of the lru_keys.
  lru_keys_.splice(lru_keys_.begin(), lru_keys_, it->second.lru_it);

  // Need to make sure the allocated `value` buffer has enough space for the
  // shader binary.
  // According to Filament documentation, the value should only be written
  // when the loaded shader binary size is smaller or equal to the provided
  // buffer size `valueSize`.
  if (shader_data.size() <= valueSize) {
    // Note: ideally we should use absl::CopyCordToSpan, but it doesn't exist on
    // Moohan.
    size_t offset = 0;
    for (absl::string_view chunk : shader_data.Chunks()) {
      memcpy(static_cast<char*>(value) + offset, chunk.data(), chunk.size());
      offset += chunk.size();
    }
    
  }

  return shader_data.size();
}

Future<absl::flat_hash_map<std::string, absl::Cord>>
ShaderCacheSystem::LoadCacheFromDisk(absl::string_view file_path) {
  return Future<absl::flat_hash_map<std::string, absl::Cord>>::Schedule(
      [file_path = std::string(file_path)]()
          -> absl::StatusOr<absl::flat_hash_map<std::string, absl::Cord>> {
        IMP_TRACE();
        BufferAccess buffer;
        absl::Status load_status = LoadBinary(file_path, &buffer);

        if (!load_status.ok()) {
          // Logged as INFO. Missing shader cache on initial run is expected.
          IMP_LOG(imp::INFO) << "Failed to load shader cache blob: "
                    << load_status.ToString();
          return load_status;
        }
        absl::flat_hash_map<std::string, absl::Cord> cache_blobs;
        size_t offset = 0;
        while (std::optional<BlobEntry> entry =
                   ReadNextEntry(buffer.View(), offset)) {
          cache_blobs.try_emplace(std::string(entry->key),
                                  std::string(entry->blob));
        }

        IMP_LOG(imp::INFO) << "Loaded shader cache: "
                   << (buffer.Size() / 1024.0 / 1024.0) << " MB ("
                   << buffer.Size() << " bytes), " << cache_blobs.size()
                   << " entries.";

        return cache_blobs;
      },
      Executor::Type::kBackground);
}

Future<absl::Status> ShaderCacheSystem::WriteCacheToDisk(
    std::string file_path, ShaderCacheBlobs cache_blobs,
    std::list<std::string> lru_keys) {
  return Future<absl::Status>::Schedule(
      [file_path = std::move(file_path), cache_blobs = std::move(cache_blobs),
       lru_keys = std::move(lru_keys)]() {
        size_t buffer_size = 0;
        size_t num_entries = 0;
        num_entries = lru_keys.size();
        // Note: we iterate over the lru_keys_ to preserve the LRU order.
        // 1) Get the size of the BufferAcess.
        for (const std::string& key : lru_keys) {
          auto map_it = cache_blobs.find(key);
          // If the key exists in lru_keys, it should exist in the cache_blobs.
          
          const ShaderCacheEntry& entry = map_it->second;

          uint32_t key_len = static_cast<uint32_t>(key.size());
          uint32_t val_len = static_cast<uint32_t>(entry.blob.size());
          buffer_size += kKeySize;
          buffer_size += key_len;
          buffer_size += kBlobSize;
          buffer_size += val_len;
        }

        // 2) Now actually fill in the buffer.
        BufferAccess buffer;
        uint8_t* write_ptr = BufferAccess::Create(buffer_size, &buffer);
        for (const std::string& key : lru_keys) {
          auto map_it = cache_blobs.find(key);
          // If the key exists in lru_keys, it should exist in the cache_blobs.
          
          const ShaderCacheEntry& entry = map_it->second;

          uint32_t key_len = static_cast<uint32_t>(key.size());
          uint32_t val_len = static_cast<uint32_t>(entry.blob.size());
          std::memcpy(write_ptr, &key_len, kKeySize);
          write_ptr += kKeySize;

          std::memcpy(write_ptr, key.data(), key_len);
          write_ptr += key_len;

          std::memcpy(write_ptr, &val_len, kBlobSize);
          write_ptr += kBlobSize;

          // Note: ideally we should use absl::CopyCordToSpan, but it doesn't
          // exist on Moohan.
          size_t offset = 0;
          for (absl::string_view chunk : entry.blob.Chunks()) {
            memcpy(write_ptr + offset, chunk.data(), chunk.size());
            offset += chunk.size();
          }
          
          write_ptr += val_len;
        }

        // There may be a race condition where WriteCacheToDisk is called before
        // the current WriteCacheToDisk has been completed. The policy is to
        // write cache for each call, but with a temp file name, then rename it
        // to the actual file name. With this, we don't have to manage states
        // checking if it's writing, if there's any pending writing etc.
        // The downside of this approach is that there's no guarantee that the
        // lastest WriteCacheToDisk will be the one preserved on disk. However,
        // this is acceptable given that multiple pause events within a short
        // period of time are rare in practice.
        std::string temp_file_path =
            absl::StrCat(file_path, ".", ++cache_write_counter, ".tmp");

        // 3) Save binary (no need to clear the file, `SaveBinary` uses "wb")
        absl::Status write_status = SaveBinary(temp_file_path, buffer);
        if (write_status.ok()) {
          // 4) Then rename the file to the correct one.
          if (std::rename(temp_file_path.c_str(), file_path.c_str())) {
            IMP_LOG(imp::ERROR) << "Failed to rename the shader cache file.";
            return absl::InternalError(
                "Failed to rename the shader cache file.");
          }
          IMP_LOG(imp::INFO) << "Successfully wrote shader cache: "
                     << (buffer.Size() / 1024.0 / 1024.0) << " MB ("
                     << buffer.Size() << " bytes), " << num_entries
                     << " entries to " << file_path;
        } else {
          IMP_LOG(imp::ERROR) << "Failed to write shader cache to disk: "
                     << write_status.ToString();
        }
        return write_status;
      },
      Executor::Type::kBackground);
}

ShaderCacheSystem::ShaderCacheSystem(
    BaseView* view, absl::optional<int32_t> max_num_cache_entry)
    : System(view),
      view_(view),
      max_num_cache_entry_(GetMaxNumCacheEntry(max_num_cache_entry)) {}

ShaderCacheSystem::~ShaderCacheSystem() {
  // Without calling this, the LoadCacheFromFile future in Setup will be
  // cancelled when the base class (Rememberer) is destroyed, which is not
  // desired as the future will be cleared after cache_blobs_ is destroyed.
  ClearRemembered();
}

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
  resolved_cache_path_ = JoinPath(cache_dir_, kShaderCacheFileName);
  // TODO: Investigate if we can avoid a race condition where the
  // user requests shader before the shader cache blob is loaded.
  LoadCacheFromDisk(resolved_cache_path_)
      .Then([this](absl::StatusOr<absl::flat_hash_map<std::string, absl::Cord>>
                       cache_blobs) {
        {
          absl::MutexLock lock(cache_lock_);
          if (cache_blobs.ok()) {
            size_t count = 0;
            for (auto& [key, blob] : *cache_blobs) {
              if (count >= max_num_cache_entry_) break;
              lru_keys_.emplace_front(key);
              cache_blobs_.try_emplace(
                  key, ShaderCacheEntry{std::move(blob), lru_keys_.begin()});
              count++;
            }
          }
          // Regardless of whether the load was successful, we need to mark it
          // as true to allow disk writes.
          cache_loaded_ = true;
        }
      })
      .KeptBy(this);

  view_->GetDispatcher().Connect(
      [this](const ViewPausedEvent& event) {
        ShaderCacheBlobs cache_blobs;
        std::list<std::string> lru_keys;

        // Since WriteCacheToDisk is an async task, it may run after
        // ShaderCacheSystem has been destroyed. To avoid accessing the members
        // (cache_blobs_ and lru_keys_) of a deleted object, we need to copy
        // the map and the list.
        {
          absl::MutexLock lock(cache_lock_);
          if (!cache_loaded_) {
            return;
          }
          cache_blobs = cache_blobs_;
          lru_keys = lru_keys_;
        }

        WriteCacheToDisk(resolved_cache_path_, std::move(cache_blobs),
                         std::move(lru_keys))
            .KeptBy(this);
      },
      this);
}

void ShaderCacheSystem::EnableShaderCaching() {
  if (!IsSupportedOnCurrentPlatform(view_)) {
    return;
  }
  // Setup filament platform function pointers.
  view_->GetSharedEngine()->getPlatform()->setBlobFunc(
      [this](const void* key, size_t keySize, const void* value,
             size_t valueSize) { WriteCache(key, keySize, value, valueSize); },
      [this](const void* key, size_t keySize, void* value, size_t valueSize)
          -> size_t { return ReadCache(key, keySize, value, valueSize); });
}

void ShaderCacheSystem::DisableShaderCaching() {
  if (!IsSupportedOnCurrentPlatform(view_)) {
    return;
  }
  view_->GetSharedEngine()->getPlatform()->setBlobFunc({}, {});
  if (view_->GetSharedEngine()->getPlatform()->hasBlobFunc()) {
    IMP_LOG(imp::FATAL) << "Unable to unset blob func.";
  }
}

}  // namespace imp
