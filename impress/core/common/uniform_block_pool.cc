// Copyright 2026 Google LLC
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

#include "core/common/uniform_block_pool.h"

#include <cstddef>

namespace imp {

constexpr std::size_t kMaxFreeBlocks = 4096;
constexpr std::size_t kInitialFreeBlocks = 128;

UniformBlockPool::UniformBlockPool(std::size_t block_bytes)
    : block_bytes_(block_bytes) {
  free_blocks_.reserve(kInitialFreeBlocks);
  for (int i = 0; i < kInitialFreeBlocks; ++i) {
    free_blocks_.push_back(::operator new(block_bytes_));
  }
}

UniformBlockPool::~UniformBlockPool() { Purge(); }

void* UniformBlockPool::Allocate() {
  allocate_count_++;
  if (!free_blocks_.empty()) {
    void* block = free_blocks_.back();
    free_blocks_.pop_back();
    return block;
  }
  return ::operator new(block_bytes_);
}

void UniformBlockPool::Deallocate(void* block) {
  if (block) {
    deallocate_count_++;
    if (free_blocks_.size() < kMaxFreeBlocks) {
      free_blocks_.push_back(block);
    } else {
      ::operator delete(block);
    }
  }
}

void UniformBlockPool::Purge() {
  for (void* block : free_blocks_) {
    ::operator delete(block);
  }
  free_blocks_.clear();
}

}  // namespace imp
