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
#include <cstdint>
#include <memory>

#include "core/split_engine/android/buffer_handle_factory.h"

namespace imp::split_engine {

// Manages a shared memory bridge buffer.
class BridgeBuffer {
  using BufferHandle = BufferHandleFactory::BufferHandle;

 public:
  BridgeBuffer(BufferHandleFactory& handle_factory, size_t buffer_size_bytes);
  BridgeBuffer(BridgeBuffer&& other);
  BridgeBuffer(const BridgeBuffer&) = delete;
  BridgeBuffer& operator=(const BridgeBuffer&) = delete;

  ~BridgeBuffer();

  void* Data() { return mmapped_ptr_; }
  template <typename T>
  const T* DataAs() const {
    return reinterpret_cast<const T*>(mmapped_ptr_);
  }
  const BufferHandle& GetHandle() const { return *handle_; }

  bool IsValidBlock(const uint8_t* data, size_t data_size_in_bytes) const {
    return data >= DataAs<uint8_t>() &&
           (data + data_size_in_bytes) <= (DataAs<uint8_t>() + size_in_bytes_);
  }

 private:
  std::unique_ptr<BufferHandle> handle_;

  int shared_memory_region_fd_;
  void* mmapped_ptr_;
  size_t size_in_bytes_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_BRIDGEBUFFER_H_
