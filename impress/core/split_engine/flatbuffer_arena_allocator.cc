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
#include "absl/types/span.h"
#include "flatbuffers/base.h"

static constexpr absl::string_view kTag = "[FlatbufferArenaAllocator]: ";

namespace imp {

FlatbufferArenaAllocator::ArenaAndAllocFunc::ArenaAndAllocFunc(
    size_t block_size, MemoryOptions memory_options)
    : first_block_head_(memory_options.first_block_alloc
                            ? memory_options.first_block_alloc(
                                  block_size, memory_options.user)
                            : nullptr),
      memory_options_(memory_options),
      in_use_(true) {
  arena_ = std::make_unique<zetasql_base::UnsafeArena>(static_cast<char*>(first_block_head_),
                                         block_size);
}

FlatbufferArenaAllocator::ArenaAndAllocFunc::~ArenaAndAllocFunc() { Clear(); }

FlatbufferArenaAllocator::ArenaAndAllocFunc::ArenaAndAllocFunc(
    ArenaAndAllocFunc&& other) {
  *this = std::move(other);
}

FlatbufferArenaAllocator::ArenaAndAllocFunc&
FlatbufferArenaAllocator::ArenaAndAllocFunc::operator=(
    ArenaAndAllocFunc&& other) {
  arena_ = std::move(other.arena_);
  first_block_head_ = other.first_block_head_;
  memory_options_ = other.memory_options_;
  in_use_ = other.in_use_;

  other.first_block_head_ = nullptr;
  other.memory_options_ = {};
  other.in_use_ = false;

  return *this;
}

bool FlatbufferArenaAllocator::ArenaAndAllocFunc::IsMatch(
    size_t block_size, const MemoryOptions& memory_options) {
  return (!in_use_ && arena_ && arena_->block_size() == block_size &&
          memory_options_ == memory_options);
}

void FlatbufferArenaAllocator::ArenaAndAllocFunc::SetInUse() { in_use_ = true; }

void FlatbufferArenaAllocator::ArenaAndAllocFunc::Reset() {
  arena_->Reset();
  in_use_ = false;
}

void FlatbufferArenaAllocator::ArenaAndAllocFunc::Clear() {
  if (memory_options_.first_block_dealloc != nullptr) {
    memory_options_.first_block_dealloc(first_block_head_,
                                        memory_options_.user);
  }
  arena_.reset();
  first_block_head_ = nullptr;
  memory_options_ = {};
  in_use_ = false;
}

FlatbufferArenaAllocator::ArenaHandle FlatbufferArenaAllocator::CreateArena(
    size_t block_size) {
  return CreateArena(block_size, {});
}

FlatbufferArenaAllocator::ArenaHandle FlatbufferArenaAllocator::CreateArena(
    size_t block_size, MemoryOptions memory_options) {
  // First try to find an arena to reuse.
  for (int i = 0; i < arenas_.size(); ++i) {
    if (i != active_arena_ && arenas_[i].IsMatch(block_size, memory_options)) {
      // The handle is the index into the vector.
      active_arena_ = i;
      arenas_[i].SetInUse();
      return active_arena_;
    }
  }
  // If no empty arenas are found to reuse then allocate a new one.
  // Note that even when we don't reuse arenas, we still reuse slots in the
  // arenas_ vector.
  ArenaAndAllocFunc new_arena(block_size, memory_options);
  for (int i = 0; i < arenas_.size(); ++i) {
    if (!arenas_[i].Get()) {
      arenas_[i] = std::move(new_arena);
      active_arena_ = i;
      return active_arena_;
    }
  }

  arenas_.emplace_back(std::move(new_arena));
  active_arena_ = arenas_.size() - 1;
  return active_arena_;
}

void FlatbufferArenaAllocator::DestroyArena(ArenaHandle arena_handle,
                                            bool allow_recycle) {
  

  ArenaAndAllocFunc& arena = arenas_[arena_handle];
  if (allow_recycle) {
    arena.Reset();
  } else {
    arena.Clear();
  }

  // Clients should never call DestroyArena on the active arena, but we can
  // simply defend against it anyway.
  if (active_arena_ == arena_handle) {
    
    IMP_LOG(imp::WARNING)
        << "[FlatbufferPoolAllocator] DestroyArena called on active arena: "
        << active_arena_;
    active_arena_ = -1;
  }
}

size_t FlatbufferArenaAllocator::GetArenaSize(ArenaHandle arena_handle) {
  

  return arenas_[arena_handle].Get()->status().bytes_allocated();
}

void* FlatbufferArenaAllocator::GetArenaHead(ArenaHandle arena_handle) {
  
  

  return arenas_[arena_handle].GetArenaHead();
}

uint8_t* FlatbufferArenaAllocator::allocate(size_t size) {
  
  

  if (arenas_[active_arena_].GetGrowthStrategy() ==
      GrowthStrategy::kDontGrowBeyondFirstBlock) {
    
  }

  return reinterpret_cast<uint8_t*>(arenas_[active_arena_].Get()->Alloc(size));
}

void FlatbufferArenaAllocator::deallocate(uint8_t* p, size_t size) {
  // deallocate() must be a no-op. This is because as FlatBufferBuilder objects
  // get destructed, they attempt to deallocate the memory backing the
  // flatbuffers that they have built. But we explicitly want to keep those
  // flatbuffers around until the arena is destroyed. Indeed, that's the whole
  // point of this class.
}

FlatbufferArenaAllocator::ArenaHandle
SizePrefixedFlatbufferArenaAllocator::CreateArena(
    size_t block_size, MemoryOptions memory_options) {
  return FlatbufferArenaAllocator::CreateArena(
      block_size  // BeginMessageGroupSize + block_size + EndMessageGroupSize
          + sizeof(SizeType)  // BeginMessageGroup

          + sizeof(SizeType)  // At least one command if block_size is tailored.
                              // If not, block_size has vastly enough space for
                              // multiple commands and their sizes.

          + sizeof(SizeType),  // EndMessageGroup
      memory_options);
}

uint8_t* SizePrefixedFlatbufferArenaAllocator::allocate(size_t size) {
  return FlatbufferArenaAllocator::allocate(size + sizeof(SizeType)) +
         sizeof(SizeType);
}

absl::Span<const uint8_t> SizePrefixedFlatbufferArenaAllocator::PrependSize(
    uint8_t* ptr, SizeType size) {
  SizeType* size_ptr = reinterpret_cast<SizeType*>(ptr - sizeof(SizeType));
  // This is the same idea as in
  // FlatbufferBuilder::FinishSizePrefixed, but without forcing the alignment.
  *size_ptr = flatbuffers::EndianScalar(size);
  return absl::MakeSpan(reinterpret_cast<uint8_t*>(size_ptr),
                        size + sizeof(SizeType));
}

}  // namespace imp
