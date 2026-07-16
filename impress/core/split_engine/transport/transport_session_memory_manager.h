/*
 * Copyright 2026 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_TRANSPORT_SESSION_MEMORY_MANAGER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_TRANSPORT_SESSION_MEMORY_MANAGER_H_

#include <cstddef>
#include <cstdint>

namespace imp::split_engine {

// Abstracts how session memory blocks are managed from away from the
// `BasicTransport`.
class TransportSessionMemoryManager {
 public:
  using ArenaHandle = int32_t;

  // This arena handle always exists and is never returned by `CreateArena`.
  static constexpr ArenaHandle kHeapArenaHandle = -1;

  virtual ~TransportSessionMemoryManager() = default;

  // Creates a new arena with the given size.
  virtual ArenaHandle CreateArena(size_t session_max_size_bytes) = 0;

  // Returns the head of the arena.
  virtual uint8_t* GetArenaHead(ArenaHandle arena_handle) = 0;

  // Returns the size of the arena.
  virtual size_t GetArenaSize(ArenaHandle arena_handle) = 0;

  // Allocates memory from the arena.
  virtual uint8_t* AllocateArenaMemory(ArenaHandle arena_handle,
                                       size_t size_bytes) = 0;

  // Deallocates memory from the arena.
  virtual void DeallocateArenaMemory(ArenaHandle arena_handle,
                                     uint8_t* ptr) = 0;

  // Destroys the arena.
  //
  // If `arena_handle` was not returned by `CreateArena` (e.g.
  // `kHeapArenaHandle`), implementation shall CHECK-crash.
  virtual void DestroyArena(ArenaHandle arena_handle, bool recycle_memory) = 0;
};
}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_TRANSPORT_SESSION_MEMORY_MANAGER_H_
