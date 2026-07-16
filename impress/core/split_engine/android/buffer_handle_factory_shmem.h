/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_BUFFER_HANDLE_FACTORY_SHMEM_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_BUFFER_HANDLE_FACTORY_SHMEM_H_

#include <cstddef>
#include <memory>
#include <utility>

#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "core/split_engine/android/buffer_handle_factory.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"

namespace imp::split_engine {

// SharedMemoryBufferHandleFactory uses SplitEngineSharedMemoryBridgeClient
// to register shared memory buffers with the renderer.
class SharedMemoryBufferHandleFactory : public BufferHandleFactory {
 public:
  SharedMemoryBufferHandleFactory(SplitEngineSharedMemoryBridgeClient& client)
      : client_(client) {}

  std::unique_ptr<BufferHandle> Create(int fd,
                                       size_t buffer_size_bytes) override {
    absl::StatusOr<std::unique_ptr<BufferHandle>> handle =
        client_.RegisterBuffer(fd, buffer_size_bytes);
    if (!handle.ok()) {
      IMP_LOG(imp::FATAL) << "Failed to register bridge buffer: " << handle.status();
    }

    return *std::move(handle);
  }

 private:
  SplitEngineSharedMemoryBridgeClient& client_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_BUFFER_HANDLE_FACTORY_SHMEM_H_
