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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_MESSAGE_SENDER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_MESSAGE_SENDER_H_

#include <cstddef>
#include <memory>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/background_scheduler.h"
#include "core/common/invocable.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/common/owned_ptr.h"
#include "core/split_engine/transport/flatbuffer_builder_holder.h"
#include "core/split_engine/transport/transport.h"

namespace imp::split_engine {

// MessageSender is the next layer on top of Transport. If Transport operates on
// spans of raw data, MessageSender operates on flatbuffers.
//
// Architecture / Layering:
//
//   [ Foreground Executor Thread ]
//                 | (DCHECK enforced)
//                 V
// ┌───────────────┴───────────────────────┐
// |             MessageSender             |
// ├───────────────────────────────────────┤
// | * Operates on Flatbuffers             |
// | * Content-agnostic                    |
// | * Creates FlatbufferBuilders using    |
// |   custom allocators backed by         |
// |   Transport's session memory blocks   |
// └───────────────┬───────────────────────┘
//                 |
//                 V
// ┌───────────────┴───────────────────────┐
// |               Transport               |
// ├───────────────────────────────────────┤
// | * Operates on raw data spans          |
// | * Provides Session Memory Blocks      |
// └───────────────────────────────────────┘
//
//
// MessageSender does not know about the content of the message.
//
// MessageSender provides convenient APIs to create flatbuffer builder with
// custom allocator that uses Transport's session memory block to allocate
// memory for messages.
//
// MessageSender should be accessed from the foreground executor thread only.
class MessageSender {
 public:
  using SessionID = Transport::SessionID;
  using FlatbufferBuilderHolder = FlatbufferBuilderHolder<MessageSender>;

  MessageSender(imp::OwnedOrBorrowedPtr<Transport> transport,
                imp::BorrowedPtr<BackgroundScheduler> scheduler);

  MessageSender(const MessageSender&) = delete;
  MessageSender& operator=(const MessageSender&) = delete;
  MessageSender(MessageSender&&) = default;
  MessageSender& operator=(MessageSender&&) = default;

  ~MessageSender();

  // Creates new session and allocates memory block of at least
  // `session_max_size_bytes` size.
  absl::StatusOr<SessionID> OpenSession(size_t session_max_size_bytes);

  // Creates a flatbuffer builder, backed by the session memory block.
  absl::StatusOr<imp::OwnedPtr<FlatbufferBuilderHolder>> CreateMessageBuilder(
      SessionID session_id, size_t initial_size);

  // Sends a message over the session.
  //
  // Call to `Transport::SendMessage` is scheduled on the background
  // scheduler.
  absl::Status SendMessage(SessionID session_id,
                           imp::OwnedPtr<FlatbufferBuilderHolder> fbb,
                           Transport::MessageCallback callback);

  using FlatbufferBuilderProducer =
      imp::Invocable<imp::OwnedPtr<FlatbufferBuilderHolder>()>;

  // Sends a message over the session.
  //
  // Execution of the `fbb_fn` and call to `Transport::SendMessage` is scheduled
  // on the background scheduler.
  absl::Status SendMessage(SessionID session_id,
                           FlatbufferBuilderProducer fbb_fn,
                           Transport::MessageCallback callback);

  // CloseSession forbids further messages to be sent over the session.
  absl::Status CloseSession(SessionID session_id);

  // Disposes the session memory block.
  //
  // recycle_memory:
  //     - If true, the underlying memory block is retained by the Transport
  //       and may be reused for subsequent `OpenSession` calls, avoiding
  //       the overhead of memory allocation.
  //     - If false, the underlying memory block is permanently deallocated
  //       and returned to the system.
  absl::Status DisposeSession(SessionID session_id, bool recycle_memory);

 private:
  imp::OwnedOrBorrowedPtr<Transport> transport_;

  imp::BorrowedPtr<BackgroundScheduler> scheduler_;

  // No synchronization is needed: this map is accessed from the foreground
  // executor only.
  class Allocator;
  absl::flat_hash_map<SessionID, std::unique_ptr<Allocator>>
      flatbuffer_allocators_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_MESSAGE_SENDER_H_
