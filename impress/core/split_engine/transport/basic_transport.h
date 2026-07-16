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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_BASIC_TRANSPORT_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_BASIC_TRANSPORT_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>

#include "absl/base/nullability.h"
#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/numeric/int128.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/span.h"
#include "core/async/background_scheduler.h"
#include "core/common/owned_ptr.h"
#include "core/split_engine/transport/concepts/concepts.h"
#include "core/split_engine/transport/concepts/underlying_transport.h"
#include "core/split_engine/transport/transport.h"
#include "core/split_engine/transport/transport_session_memory_manager.h"

namespace imp::split_engine {

// # Introduction
//
// BasicTransport is a partial implementation of Transport interface that takes
// care of session management. Communication methods are delegated to the
// `UnderlyingTransport`, but actual implementation is left to the derived
// classes.
//
// # Session management
//
// BasicTransport uses `TransportSessionMemoryManager` to manage session memory
// blocks.
//
// Call to `OpenSession` creates session memory block and returns `SessionID`.
// BasicTransport uses SessionID to encode ArenaHandle.
//
// Call to `CloseSession` marks session as closed and moves it to the list of
// undisposed sessions.
//
// Call to `DisposeSession` disposes the session memory block. Session memory
// can be deallocated or reused (`recycle_memory = true`) in later sessions.
//
// # Message memory management
//
// BasicTransport delegates message memory management to
// `TransportSessionMemoryManager`.
//
template <CONCEPT(UnderlyingTransport) UTransport>
class BasicTransport : public Transport {
 public:
  using BufferHead = uint8_t*;
  using ArenaHandle = typename TransportSessionMemoryManager::ArenaHandle;

  BasicTransport(
      std::unique_ptr<UTransport> underlying_transport,
      std::unique_ptr<TransportSessionMemoryManager> session_memory_manager,
      imp::BorrowedPtr<BackgroundScheduler> scheduler)
      : underlying_transport_(std::move(underlying_transport)),
        memory_manager_(std::move(session_memory_manager)),
        scheduler_(std::move(scheduler)) {
    sessions_.insert(Transport::kPermanentSessionID);
  }

  ~BasicTransport() override {
    scheduler_->Drain(absl::CancelledError("Transport is destroyed."));
  }

  absl::StatusOr<SessionID> OpenSession(
      size_t session_max_size_bytes) override {
    const ArenaHandle arena_handle =
        memory_manager_->CreateArena(session_max_size_bytes);

    const SessionID session_id = arena_handle;
    {
      absl::MutexLock lock(sessions_mutex_);
      sessions_.insert(session_id);
    }

    return session_id;
  }

  uint8_t* AllocateMessageMemory(SessionID session_id,
                                 size_t size_bytes) override {
    {
      absl::MutexLock lock(sessions_mutex_);
      
      if (!sessions_.contains(session_id)) {
        return nullptr;
      }
    }

    return memory_manager_->AllocateArenaMemory(GetArenaHandle(session_id),
                                                size_bytes);
  }

  void DeallocateMessageMemory(SessionID session_id, uint8_t* ptr) override {
    return memory_manager_->DeallocateArenaMemory(GetArenaHandle(session_id),
                                                  ptr);
  }

  absl::Status SendMessage(Transport::SessionID session_id,
                           absl::Span<const uint8_t> data,
                           MessageCallback callback) override {
    {
      absl::MutexLock lock(sessions_mutex_);
      if (!sessions_.contains(session_id)) {
        return absl::FailedPreconditionError("Session is not active");
      }
    }

    // Validate that data is from proper arena.
    const ArenaHandle arena_handle = GetArenaHandle(session_id);
    const uint8_t* /*absl_nullable*/  arena_head_ptr =
        memory_manager_->GetArenaHead(arena_handle);
    if (data.data() < arena_head_ptr) {
      return absl::FailedPreconditionError(
          "Message data does not belong to the arena: message pointer is "
          "before the arena head.");
    }

    // ArenaHead might be null in case of heap-based allocation and
    // `pointer - nullptr` is undefined behavior.
    //
    // Convert pointers to integers for proper arithmetic.
    const absl::uint128 offset = reinterpret_cast<uint64_t>(data.data()) -
                                 reinterpret_cast<uint64_t>(arena_head_ptr);
    const absl::uint128 arena_size =
        memory_manager_->GetArenaSize(arena_handle);
    if (offset > arena_size) {
      return absl::FailedPreconditionError(
          absl::StrCat("Message data does not belong to the arena, offset is "
                       "larger than the arena size: ",
                       offset, " > ", arena_size));
    }
    if (offset + data.size() > arena_size) {
      return absl::FailedPreconditionError(absl::StrCat(
          "Data is too large: ", offset + data.size(), " > ", arena_size));
    }

    return SendMessageImpl(session_id,
                           reinterpret_cast<BufferHead>(
                               memory_manager_->GetArenaHead(arena_handle)),
                           data, std::move(callback));
  }

  absl::Status CloseSession(SessionID session_id) override {
    if (session_id == Transport::kPermanentSessionID) {
      return absl::FailedPreconditionError(
          "Permanent session cannot be closed");
    }

    {
      absl::MutexLock lock(sessions_mutex_);
      
      if (!sessions_.contains(session_id)) {
        return absl::FailedPreconditionError("Session is not active");
      }

      sessions_.erase(session_id);
    }

    {
      absl::MutexLock lock(undisposed_sessions_mutex_);
      undisposed_sessions_.insert(session_id);
    }

    return absl::OkStatus();
  }

  absl::Status DisposeSession(SessionID session_id,
                              bool recycle_memory) override {
    if (session_id == Transport::kPermanentSessionID) {
      return absl::FailedPreconditionError(
          "Permanent session cannot be closed");
    }

    {
      absl::MutexLock lock(undisposed_sessions_mutex_);
      const auto it = undisposed_sessions_.find(session_id);
      if (it == undisposed_sessions_.end()) {
        return absl::FailedPreconditionError(
            "Session is either not closed or incorrect.");
      }
      undisposed_sessions_.erase(it);
    }

    memory_manager_->DestroyArena(GetArenaHandle(session_id), recycle_memory);
    return absl::OkStatus();
  }

 private:
  static constexpr ArenaHandle GetArenaHandle(Transport::SessionID session_id) {
    return session_id;
  }

 protected:
  const std::unique_ptr<UTransport> underlying_transport_;
  const std::unique_ptr<TransportSessionMemoryManager> memory_manager_;
  const imp::BorrowedPtr<BackgroundScheduler> scheduler_;

 private:
  virtual absl::Status SendMessageImpl(SessionID session_id,
                                       BufferHead buffer_head,
                                       absl::Span<const uint8_t> data,
                                       MessageCallback callback) = 0;

  mutable absl::Mutex sessions_mutex_;
  absl::flat_hash_set<SessionID> sessions_ ABSL_GUARDED_BY(sessions_mutex_);

  mutable absl::Mutex undisposed_sessions_mutex_;
  absl::flat_hash_set<SessionID> undisposed_sessions_
      ABSL_GUARDED_BY(undisposed_sessions_mutex_);
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_BASIC_TRANSPORT_H_
