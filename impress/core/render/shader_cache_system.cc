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

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/Platform.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/common/buffer_access.h"
#include "core/common/file_helpers.h"
#include "core/common/trace.h"
#include "core/config.h"
#include "core/ncsb/system.h"
#include "core/view/base_view.h"

#if IMP_PLATFORM(ANDROID)
#include "core/view/platforms/android/wrappers/activity_context.h"
#endif

namespace {

constexpr absl::string_view kShaderCacheFileName = "shader_blob_cache.bin";
constexpr size_t kKeySize = sizeof(uint32_t);
constexpr size_t kBlobSize = sizeof(uint32_t);

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
    absl::MutexLock lock(cache_lock_);
    auto [it, inserted] = cache_blobs_.try_emplace(key_str, value_cord);
    if (!inserted) {
      IMP_LOG(imp::ERROR) << "Failed to insert shader cache blob into in-memory map.";
      return;
    }
  }

  // Write it to disk in the background.
  background_executor_->Schedule([resolved_cache_path = resolved_cache_path_,
                                  key_str = std::move(key_str), value_cord]() {
    // Cast sizes to uint32_t to keep the binary file format consistent.
    uint32_t key_len = static_cast<uint32_t>(key_str.size());
    uint32_t val_len = static_cast<uint32_t>(value_cord.size());

    // Pack the entry data into a single contiguous buffer.
    BufferAccess buffer;
    const size_t total_size = kKeySize + key_len + kBlobSize + val_len;
    uint8_t* write_ptr = BufferAccess::Create(total_size, &buffer);

    std::memcpy(write_ptr, &key_len, kKeySize);
    write_ptr += kKeySize;

    std::memcpy(write_ptr, key_str.data(), key_len);
    write_ptr += key_len;

    std::memcpy(write_ptr, &val_len, kBlobSize);
    write_ptr += kBlobSize;

    // Note: ideally we should use absl::CopyCordToSpan, but it doesn't exist on
    // Moohan.
    size_t offset = 0;
    for (absl::string_view chunk : value_cord.Chunks()) {
      memcpy(write_ptr + offset, chunk.data(), chunk.size());
      offset += chunk.size();
    }
    

    absl::Status append_status =
        AppendBinary(resolved_cache_path, buffer.Data(), buffer.Size());

    if (!append_status.ok()) {
      IMP_LOG(imp::ERROR) << "Failed to append cache entry: "
                 << append_status.ToString();
      // Note: we don't necessarily remove the entry from the in-memory map.
    }
  });
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

  absl::Cord shader_data = it->second;

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

Future<ShaderCacheSystem::ShaderCacheBlobs>
ShaderCacheSystem::LoadCacheFromFile(absl::string_view file_path) {
  return Future<ShaderCacheBlobs>::Schedule(
      [file_path =
           std::string(file_path)]() -> absl::StatusOr<ShaderCacheBlobs> {
        IMP_TRACE();
        BufferAccess buffer;
        absl::Status load_status = LoadBinary(file_path, &buffer);

        if (!load_status.ok()) {
          // Logged as INFO. Missing shader cache on initial run is expected.
          IMP_LOG(imp::INFO) << "Failed to load shader cache blob: "
                    << load_status.ToString();
          return load_status;
        }
        ShaderCacheBlobs cache_blobs;
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

ShaderCacheSystem::ShaderCacheSystem(BaseView* view)
    : System(view), view_(view) {}

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

  background_executor_ = imp::Executor::BackgroundExecutor();
  
  // TODO: Investigate if we can avoid a race condition where the
  // user requests shader before the shader cache blob is loaded.
  LoadCacheFromFile(resolved_cache_path_)
      .Then([this](ShaderCacheBlobs&& cache_blobs) {
        {
          absl::MutexLock lock(cache_lock_);
          cache_blobs_ = std::move(cache_blobs);
        }
      })
      .KeptBy(this);
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
