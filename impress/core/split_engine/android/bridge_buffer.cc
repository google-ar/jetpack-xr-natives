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

#include "core/split_engine/android/bridge_buffer.h"

#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstring>
#include <memory>
#include <utility>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "filament/libs/utils/include/utils/ashmem.h"
#include "core/async/executor.h"
#include "core/common/trace.h"
#include "core/split_engine/android/buffer_handle_factory.h"

namespace imp::split_engine {

BridgeBuffer::BridgeBuffer(BufferHandleFactory& handle_factory,
                           size_t buffer_size_bytes)
    : shared_memory_region_fd_(0),
      mmapped_ptr_(nullptr),
      size_in_bytes_(buffer_size_bytes) {
  IMP_TRACE();

  shared_memory_region_fd_ =
      utils::ashmem_create_region("RenderingBridgeBuffer", size_in_bytes_);
  if (shared_memory_region_fd_ == 0) {
    IMP_LOG(imp::FATAL) << "Failed to allocate render bridge buffer";
  }

  mmapped_ptr_ = ::mmap(nullptr, size_in_bytes_, PROT_READ | PROT_WRITE,
                        MAP_SHARED, shared_memory_region_fd_, 0);
  if (mmapped_ptr_ == MAP_FAILED) {
    IMP_LOG(imp::FATAL) << "Failed to mmap RenderingBridgeAssetBuffer";
  }

  handle_ = handle_factory.Create(shared_memory_region_fd_, size_in_bytes_);
}

BridgeBuffer::BridgeBuffer(BridgeBuffer&& other)
    : handle_(std::move(other.handle_)),
      shared_memory_region_fd_(std::move(other.shared_memory_region_fd_)),
      mmapped_ptr_(other.mmapped_ptr_),
      size_in_bytes_(other.size_in_bytes_) {
  other.shared_memory_region_fd_ = 0;
  other.mmapped_ptr_ = nullptr;
  other.size_in_bytes_ = 0;
}

BridgeBuffer::~BridgeBuffer() {
  IMP_TRACE();
  if (mmapped_ptr_ == nullptr) {
    return;
  }

  // munmap is blocking and might be expensive to run on the main thread.
  
  Executor::BackgroundExecutor()->Schedule([ptr = mmapped_ptr_,
                                            size = size_in_bytes_,
                                            fd = shared_memory_region_fd_]() {
    ::munmap(ptr, size);
    close(fd);
  });
}

}  // namespace imp::split_engine
