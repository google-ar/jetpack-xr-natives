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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_UNIFORM_BLOCK_POOL_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_UNIFORM_BLOCK_POOL_H_

#include <cstddef>
#include <vector>

namespace imp {

// A dynamically growing, LIFO pool for uniform size blocks of memory.
//
// Optimized for recycling uniform blocks of memory (where every allocation is
// exactly block_bytes in size). Avoids standard allocator splitting, metadata,
// and alignment overhead.
//
// Threading: This class is unsynchronized and intended for single-threaded or
// external-mutex synchronized usage.
//
// NOTE: This class doesn't support custom alignment.
class UniformBlockPool {
 public:
  explicit UniformBlockPool(std::size_t block_bytes);
  ~UniformBlockPool();

  UniformBlockPool(const UniformBlockPool&) = delete;
  UniformBlockPool& operator=(const UniformBlockPool&) = delete;

  // Provides a block of memory of size block_bytes.
  //
  // If there are no free blocks, a new block is allocated on the heap.
  // Otherwise, the last free block is returned.
  void* Allocate();

  // Returns a block of memory back to the pool.
  //
  // This doesn't actually free the memory, but instead adds it to a
  // LIFO list of free blocks to be reused.
  void Deallocate(void* block);

  // Returns the byte size of each uniform block.
  std::size_t GetBlockBytes() const { return block_bytes_; }

  // Frees recycled blocks entirely.
  void Purge();

  // Returns the total number of Allocate() calls.
  std::size_t AllocateCount() const { return allocate_count_; }

  // Returns the total number of Deallocate() calls.
  std::size_t DeallocateCount() const { return deallocate_count_; }

 private:
  std::size_t block_bytes_;
  std::vector<void*> free_blocks_;
  std::size_t allocate_count_ = 0;
  std::size_t deallocate_count_ = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_UNIFORM_BLOCK_POOL_H_
