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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_BRIDGEBUFFER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_BRIDGEBUFFER_H_

#include <cstddef>
#include <memory>

#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"

namespace imp::split_engine {

// Manages a shared memory bridge buffer.
class BridgeBuffer {
 public:
  BridgeBuffer(SplitEngineSharedMemoryBridgeClient& bridge,
               size_t buffer_size_bytes);
  BridgeBuffer(BridgeBuffer&& other);
  BridgeBuffer(const BridgeBuffer&) = delete;
  BridgeBuffer& operator=(const BridgeBuffer&) = delete;

  ~BridgeBuffer();

  void* Data() { return mmapped_ptr_; }
  const SplitEngineSharedMemoryBridgeClient::BufferHandle& Handle() {
    return *handle_;
  }

 private:
  std::unique_ptr<SplitEngineSharedMemoryBridgeClient::BufferHandle> handle_;

  int shared_memory_region_fd_;
  void* mmapped_ptr_;
  size_t size_in_bytes_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_BRIDGEBUFFER_H_
