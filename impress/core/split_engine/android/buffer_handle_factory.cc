// Copyright 2025 Google LLC
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

#include "core/split_engine/android/buffer_handle_factory.h"

#include <cstddef>
#include <memory>

namespace imp::split_engine {
std::unique_ptr<BufferHandleFactory::BufferHandle> BufferHandleFactory::Create(
    int fd, size_t buffer_size_bytes) {
  // Default implementation does nothing.
  class EmptyBufferHandle : public BufferHandle {
   public:
    ~EmptyBufferHandle() override = default;
  };
  return std::make_unique<EmptyBufferHandle>();
}

}  // namespace imp::split_engine
