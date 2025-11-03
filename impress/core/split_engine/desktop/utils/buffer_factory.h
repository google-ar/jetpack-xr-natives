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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_UTILS_BUFFER_FACTORY_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_UTILS_BUFFER_FACTORY_H_

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>

#include "absl/base/nullability.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"

namespace imp::split_engine {

// Represents a valid buffer of a given size produced by a BufferFactory.
// Buffer owns underlying memory and is responsible for freeing it.
//
// Not thread-safe.
class Buffer {
 public:
  virtual ~Buffer() = default;

  // Accessors to underlying memory.
  virtual const uint8_t* /*absl_nonnull*/  Data() const noexcept = 0;
  virtual uint8_t* /*absl_nonnull*/  Data() noexcept = 0;

  // Returns the size of the buffer in bytes.
  virtual size_t Size() const noexcept = 0;

  // Helper function to write data into the buffer.
  absl::Status Write(absl::Span<const uint8_t> data);

  // Returns amount of bytes that have been written to the buffer.
  size_t GetWrittenSize() const { return offset_bytes_; }

 private:
  size_t offset_bytes_ = 0;
};

// This factory abstracts away:
// - memory allocation details (heap, shared memory regions, etc.)
// - associated policies if any (e.g. memory allocation quotas)
//
// Thread safety is implementation specific.
class BufferFactory {
 public:
  virtual ~BufferFactory() = default;

  // Creates a buffer of the given size or returns an error explaining why it
  // failed.
  virtual absl::StatusOr</*absl_nonnull*/  std::unique_ptr<Buffer>> CreateBuffer(
      size_t size_in_bytes) noexcept = 0;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_UTILS_BUFFER_FACTORY_H_
