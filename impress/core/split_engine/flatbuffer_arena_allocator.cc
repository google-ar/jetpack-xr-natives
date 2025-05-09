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
#include <memory>
#include <utility>

#include "zetasql/base/arena.h"
#include "core/common/log.h"
#include "filament/libs/utils/include/utils/compiler.h"
#include "core/common/platform_helpers.h"

namespace imp {

FlatbufferArenaAllocator::ArenaAndAllocFunc::ArenaAndAllocFunc(
    size_t block_size, LowLevelAllocFunc first_block_alloc,
    LowLevelDeallocFunc first_block_dealloc, void* user)
    : first_block_head_(first_block_alloc ? first_block_alloc(block_size, user)
                                          : nullptr),
      first_block_alloc_(first_block_alloc),
      first_block_dealloc_(first_block_dealloc),
      user_(user),
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
  first_block_alloc_ = other.first_block_alloc_;
  first_block_dealloc_ = other.first_block_dealloc_;
  user_ = other.user_;
  in_use_ = other.in_use_;

  other.first_block_head_ = nullptr;
  other.first_block_alloc_ = nullptr;
  other.first_block_dealloc_ = nullptr;
  other.user_ = nullptr;
  other.in_use_ = false;

  return *this;
}

bool FlatbufferArenaAllocator::ArenaAndAllocFunc::IsMatch(
    size_t block_size, LowLevelAllocFunc first_block_alloc,
    LowLevelDeallocFunc first_block_dealloc, void* user) {
  return (!in_use_ && arena_ && arena_->block_size() == block_size &&
          first_block_alloc_ == first_block_alloc &&
          first_block_dealloc_ == first_block_dealloc && user_ == user);
}

void FlatbufferArenaAllocator::ArenaAndAllocFunc::SetInUse() { in_use_ = true; }

void FlatbufferArenaAllocator::ArenaAndAllocFunc::Reset() {
  arena_->Reset();
  in_use_ = false;
}

void FlatbufferArenaAllocator::ArenaAndAllocFunc::Clear() {
  if (first_block_dealloc_ != nullptr) {
    first_block_dealloc_(first_block_head_, user_);
  }
  arena_.reset();
  first_block_head_ = nullptr;
  first_block_alloc_ = nullptr;
  first_block_dealloc_ = nullptr;
  user_ = nullptr;
  in_use_ = false;
}

FlatbufferArenaAllocator::ArenaHandle FlatbufferArenaAllocator::CreateArena(
    size_t block_size, LowLevelAllocFunc first_block_alloc,
    LowLevelDeallocFunc first_block_dealloc, void* user) {
  // First try to find an arena to reuse.
  for (int i = 0; i < arenas_.size(); ++i) {
    if (i != active_arena_ && arenas_[i].IsMatch(block_size, first_block_alloc,
                                                 first_block_dealloc, user)) {
      // The handle is the index into the vector.
      active_arena_ = i;
      arenas_[i].SetInUse();
      return active_arena_;
    }
  }
  // If no empty arenas are found to reuse then allocate a new one.
  // Note that even when we don't reuse arenas, we still reuse slots in the
  // arenas_ vector.
  ArenaAndAllocFunc new_arena(block_size, first_block_alloc,
                              first_block_dealloc, user);
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
  if (UTILS_UNLIKELY(arena_handle < 0 || arena_handle >= arenas_.size())) {
    IMP_LOG(imp::ERROR) << "[FlatbufferPoolAllocator] DestroyArena received invalid "
                  "arena handle "
               << arena_handle;
    return;
  }

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
  if (UTILS_UNLIKELY(arena_handle < 0 || arena_handle >= arenas_.size()) ||
      (arenas_[arena_handle].Get() == nullptr)) {
    IMP_LOG(imp::ERROR) << "[FlatbufferPoolAllocator] GetArenaSize received invalid "
                  "arena handle "
               << arena_handle;
    return 0;
  }
  return arenas_[arena_handle].Get()->status().bytes_allocated();
}

void* FlatbufferArenaAllocator::GetArenaHead(ArenaHandle arena_handle) {
  if (UTILS_UNLIKELY(arena_handle < 0 || arena_handle >= arenas_.size()) ||
      (arenas_[arena_handle].Get() == nullptr)) {
    IMP_LOG(imp::ERROR) << "[FlatbufferPoolAllocator] GetArenaHead received invalid "
                  "arena handle "
               << arena_handle;
    return 0;
  }
  return arenas_[arena_handle].GetArenaHead();
}

uint8_t* FlatbufferArenaAllocator::allocate(size_t size) {
  if (UTILS_UNLIKELY(active_arena_ < 0 || active_arena_ >= arenas_.size())) {
    return nullptr;
  }
  assert(arenas_[active_arena_].Get());
  return reinterpret_cast<uint8_t*>(arenas_[active_arena_].Get()->Alloc(size));
}

void FlatbufferArenaAllocator::deallocate(uint8_t* p, size_t size) {
  // deallocate() must be a no-op. This is because as FlatBufferBuilder objects
  // get destructed, they attempt to deallocate the memory backing the
  // flatbuffers that they have built. But we explicitly want to keep those
  // flatbuffers around until the arena is destroyed. Indeed, that's the whole
  // point of this class.
}

}  // namespace imp
