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

#include "core/split_engine/flatbuffer_arena_allocator.h"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>

#include "zetasql/base/arena.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/span.h"
#include "flatbuffers/allocator.h"
#include "flatbuffers/base.h"

static constexpr absl::string_view kTag = "[FlatbufferArenaAllocator]: ";

namespace imp {

namespace {
class ArenaBasedFlatbufferAllocator : public flatbuffers::Allocator {
 public:
  ArenaBasedFlatbufferAllocator(ArenaAllocator& arena_allocator,
                                ArenaAllocator::ArenaHandle arena_handle)
      : arena_allocator_(arena_allocator), arena_handle_(arena_handle) {}

  uint8_t* allocate(size_t size) override {
    return arena_allocator_.AllocateArenaMemory(arena_handle_, size);
  }

  void deallocate(uint8_t* p, size_t) override {
    // deallocate() must be a no-op. This is because as FlatBufferBuilder
    // objects get destructed, they attempt to deallocate the memory backing the
    // flatbuffers that they have built. But we explicitly want to keep those
    // flatbuffers around until the arena is destroyed. Indeed, that's the whole
    // point of this class.
  }

 private:
  ArenaAllocator& arena_allocator_;
  const ArenaAllocator::ArenaHandle arena_handle_;
};

class SizePrefixedArenaBasedFlatbufferAllocator
    : public ArenaBasedFlatbufferAllocator {
  using SizeType = SizePrefixedArenaAllocator::SizeType;

 public:
  SizePrefixedArenaBasedFlatbufferAllocator(
      ArenaAllocator& arena_allocator, ArenaAllocator::ArenaHandle arena_handle)
      : ArenaBasedFlatbufferAllocator(arena_allocator, arena_handle) {}

  uint8_t* allocate(size_t size) override {
    return ArenaBasedFlatbufferAllocator::allocate(size + sizeof(SizeType)) +
           sizeof(SizeType);
  }
};

}  // namespace

ArenaAllocator::ArenaAndAllocFunc::ArenaAndAllocFunc(
    size_t block_size, MemoryOptions memory_options)
    : block_size_(block_size),
      first_block_head_(memory_options.first_block_alloc
                            ? memory_options.first_block_alloc(
                                  block_size, memory_options.user)
                            : nullptr),
      memory_options_(memory_options),
      in_use_(true) {
  arena_ = std::make_unique<zetasql_base::UnsafeArena>(
      reinterpret_cast<char*>(first_block_head_), block_size);
}

ArenaAllocator::ArenaAndAllocFunc::~ArenaAndAllocFunc() { Clear(); }

ArenaAllocator::ArenaAndAllocFunc::ArenaAndAllocFunc(
    ArenaAndAllocFunc&& other) {
  *this = std::move(other);
}

ArenaAllocator::ArenaAndAllocFunc& ArenaAllocator::ArenaAndAllocFunc::operator=(
    ArenaAndAllocFunc&& other) {
  arena_ = std::move(other.arena_);
  first_block_head_ = other.first_block_head_;
  memory_options_ = other.memory_options_;
  in_use_ = other.in_use_;
  block_size_ = other.block_size_;

  other.first_block_head_ = nullptr;
  other.memory_options_ = {};
  other.in_use_ = false;
  other.block_size_ = 0;

  return *this;
}

bool ArenaAllocator::ArenaAndAllocFunc::IsMatch(
    size_t block_size, const MemoryOptions& memory_options) {
  // We cannot rely on arena_->block_size() here, because the return value is
  // specific to the Malloc used.
  //
  // If TCMalloc is used (e.g. tests), it's value will be different than the
  // value we passed to the zetasql_base::UnsafeArena constructor: zetasql_base::UnsafeArena constructor
  // uses `nallocx(original_block_size)` function to initialize its private
  // block_size_ field (return value of arena_->block_size()), but return value
  // of `nallocx` will be bigger than `original_block_size` in case of TCMalloc
  // and that will break comparison below.
  //
  // So, we are using here our cached, unchanged value from GetBlockSize().
  //
  return (!in_use_ && arena_ && GetBlockSize() == block_size &&
          memory_options_ == memory_options);
}

void ArenaAllocator::ArenaAndAllocFunc::SetInUse() { in_use_ = true; }

void ArenaAllocator::ArenaAndAllocFunc::Reset() {
  arena_->Reset();
  in_use_ = false;
}

void ArenaAllocator::ArenaAndAllocFunc::Clear() {
  if (memory_options_.first_block_dealloc != nullptr) {
    memory_options_.first_block_dealloc(first_block_head_,
                                        memory_options_.user);
  }
  arena_.reset();
  first_block_head_ = nullptr;
  memory_options_ = {};
  in_use_ = false;
}

ArenaAllocator::ArenaHandle ArenaAllocator::CreateArena(size_t block_size) {
  return CreateArena(block_size, {});
}

ArenaAllocator::ArenaHandle ArenaAllocator::CreateArena(
    size_t block_size, MemoryOptions memory_options) {
  absl::MutexLock lock(arenas_mutex_);
  // First try to find an arena to reuse.
  for (int i = 0; i < arenas_.size(); ++i) {
    if (arenas_[i].IsMatch(block_size, memory_options)) {
      // The handle is the index into the vector.
      arenas_[i].SetInUse();
      return i;
    }
  }
  // If no empty arenas are found to reuse then allocate a new one.
  // Note that even when we don't reuse arenas, we still reuse slots in the
  // arenas_ vector.
  ArenaAndAllocFunc new_arena(block_size, memory_options);
  for (int i = 0; i < arenas_.size(); ++i) {
    if (!arenas_[i].Get()) {
      arenas_[i] = std::move(new_arena);
      return i;
    }
  }

  arenas_.emplace_back(std::move(new_arena));
  return arenas_.size() - 1;
}

void ArenaAllocator::DestroyArena(ArenaHandle arena_handle,
                                  bool allow_recycle) {
  absl::MutexLock lock(arenas_mutex_);
  

  ArenaAndAllocFunc& arena = arenas_[arena_handle];
  if (allow_recycle) {
    arena.Reset();
  } else {
    arena.Clear();
  }
}

size_t ArenaAllocator::GetArenaSize(ArenaHandle arena_handle) {
  absl::MutexLock lock(arenas_mutex_);
  

  return arenas_[arena_handle].Get()->status().bytes_allocated();
}

uint8_t* ArenaAllocator::GetArenaHead(ArenaHandle arena_handle) {
  absl::MutexLock lock(arenas_mutex_);
  
  

  return arenas_[arena_handle].GetArenaHead();
}

uint8_t* ArenaAllocator::AllocateArenaMemory(ArenaHandle arena_handle,
                                             size_t size) {
  absl::MutexLock lock(arenas_mutex_);
  
  ArenaAndAllocFunc& arena = arenas_[arena_handle];
  
  

  if (arena.GetGrowthStrategy() == GrowthStrategy::kDontGrowBeyondFirstBlock) {
    
  }

  return reinterpret_cast<uint8_t*>(arena.Get()->Alloc(size));
}

std::unique_ptr<flatbuffers::Allocator>
ArenaAllocator::CreateFlatbufferAllocator(ArenaHandle arena_handle) {
  return std::make_unique<ArenaBasedFlatbufferAllocator>(*this, arena_handle);
}

std::unique_ptr<flatbuffers::Allocator>
SizePrefixedArenaAllocator::CreateFlatbufferAllocator(
    ArenaHandle arena_handle) {
  return std::make_unique<SizePrefixedArenaBasedFlatbufferAllocator>(
      *this, arena_handle);
}

ArenaAllocator::ArenaHandle SizePrefixedArenaAllocator::CreateArena(
    size_t block_size, MemoryOptions memory_options) {
  return ArenaAllocator::CreateArena(
      block_size  // BeginMessageGroupSize + block_size + EndMessageGroupSize
          + sizeof(SizeType)  // BeginMessageGroup

          + sizeof(SizeType)  // At least one command if block_size is tailored.
                              // If not, block_size has vastly enough space for
                              // multiple commands and their sizes.

          + sizeof(SizeType),  // EndMessageGroup
      memory_options);
}

absl::Span<const uint8_t> SizePrefixedArenaAllocator::PrependSize(
    uint8_t* ptr, SizeType size) {
  SizeType* size_ptr = reinterpret_cast<SizeType*>(ptr - sizeof(SizeType));
  // This is the same idea as in
  // FlatbufferBuilder::FinishSizePrefixed, but without forcing the alignment.
  *size_ptr = flatbuffers::EndianScalar(size);
  return absl::MakeSpan(reinterpret_cast<uint8_t*>(size_ptr),
                        size + sizeof(SizeType));
}

}  // namespace imp
