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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_BUFFER_HANDLE_FACTORY_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_BUFFER_HANDLE_FACTORY_H_

#include <cstddef>
#include <memory>

namespace imp::split_engine {

// BufferHandleFactory hides the knowledge from BridgeBuffer whether buffer
// information shall be shared with remote entity or not.
//
// The default implementation does nothing - this should be good enough for any
// implementation that does not share buffer information with remote entity.
//
// Derived classes should provide proper implementation of buffer handle
// creation (e.g. Android implementation shall call RegisterBuffer AIDL method).
class BufferHandleFactory {
 public:
  // An interface for a handle to a shared memory buffer.
  class BufferHandle {
   public:
    virtual ~BufferHandle() = default;
  };

  virtual ~BufferHandleFactory() = default;

  virtual std::unique_ptr<BufferHandle> Create(int fd,
                                               size_t buffer_size_bytes);
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_BUFFER_HANDLE_FACTORY_H_
