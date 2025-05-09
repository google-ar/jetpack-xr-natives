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

#include "core/common/buffer_access.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "absl/types/span.h"

namespace imp {

// static
BufferAccess BufferAccess::Clone(const uint8_t* data, size_t size) {
  auto storage = std::make_unique<uint8_t[]>(size);
  std::copy_n(data, size, storage.get());
  return BufferAccess{std::move(storage), size};
}

// static
BufferAccess BufferAccess::Wrap(const uint8_t* data, size_t size) {
  return BufferAccess{data, size};
}

// static
uint8_t* BufferAccess::Create(size_t size, BufferAccess* access) {
  auto storage = std::make_unique<uint8_t[]>(size);
  *access = BufferAccess{std::move(storage), size};
  return access->optional_storage_.get();
}

BufferAccess::BufferAccess(std::unique_ptr<uint8_t[]>&& storage, size_t size)
    : optional_storage_(std::move(storage)),
      view_(optional_storage_.get(), size) {}

BufferAccess::BufferAccess(std::unique_ptr<uint8_t[]>&& storage,
                           absl::Span<const uint8_t> view)
    : optional_storage_(std::move(storage)), view_(view) {}

BufferAccess::BufferAccess(const uint8_t* data, size_t size)
    : optional_storage_(), view_(data, size) {}

BufferAccess BufferAccess::ReleaseOwnership() {
  std::unique_ptr<uint8_t[]> storage = std::move(optional_storage_);
  return BufferAccess{std::move(storage), Size()};
}

std::unique_ptr<uint8_t[]> BufferAccess::ReleaseDataOwnership() {
  std::unique_ptr<uint8_t[]> storage = std::move(optional_storage_);
  return storage;
}

void BufferAccess::GainOwnershipByCopying() {
  if (optional_storage_) {
    // Return early, this BufferAccess already owns the memory.
    return;
  }
  // If the data is not owned by this BufferAccess, create owned copy.
  size_t size = view_.size();
  optional_storage_ = std::make_unique<uint8_t[]>(size);
  std::copy_n(view_.data(), size, optional_storage_.get());
  view_ = absl::MakeSpan(optional_storage_.get(), size);
}

}  // namespace imp
