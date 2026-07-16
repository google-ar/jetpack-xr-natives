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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_TRANSPORT_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_TRANSPORT_H_

#include <cstddef>
#include <cstdint>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "core/common/invocable.h"

namespace imp::split_engine {

// # Mental model of the transport.
//
// Transport represents a "physical" connection to the remote side.
// There is one "logical" channel that is established on top of that
// "physical" connection:
//   - Message channel is a two-way (message-response) session-based
//   communication channel.
//
// Messages are just some spans of data at this level of abstraction.
//
// Remote side does not initiate any communication on message channel on its
// own. It only responds to the messages sent by the caller.
//
// ## Message channel.
//
// ### Opening a session.
//
// Caller shall open a session (`OpenSession`) before sending a new message.
// Transport shall allocate session memory block - chunk of memory that shall be
// used to allocate memory for messages - during `OpenSession` call. Caller
// shall use `AllocateMessageMemory` to allocate memory for a message. If the
// memory is no longer needed (e.g. reallocation), caller shall call
// `DeallocateMessageMemory`.
//
// ### Sending a message.
//
// Caller shall use `SendMessage` to send a message. Caller is responsible to
// ensure that message data came from the session memory block allocated by the
// Transport.
//
// ### Closing a session.
//
// Caller shall use `CloseSession` to notify transport that no more messages
// will be sent over the session.
//
// ### Disposing a session.
//
// Caller shall use `DisposeSession` to notify transport that both sides are
// done with messages in the session. Transport shall dispose the session memory
// block during `DisposeSession` call.
//
class Transport {
 public:
  using SessionID = uint64_t;

  // Permanent session shall use heap as session memory block.
  static constexpr SessionID kPermanentSessionID = 0xFFFFFFFFFFFFFFFFull;

  Transport() = default;
  virtual ~Transport() = default;
  Transport(const Transport&) = delete;
  Transport& operator=(const Transport&) = delete;
  Transport(Transport&&) = delete;
  Transport& operator=(Transport&&) = delete;

  using MessageCallback = imp::Invocable<void(absl::Span<const uint8_t>)>;

  // Creates new session and allocates memory block of at least
  // `session_max_size_bytes` size.
  virtual absl::StatusOr<SessionID> OpenSession(
      size_t session_max_size_bytes) = 0;

  // Allocates a piece of memory for a message from the session memory block.
  virtual uint8_t* AllocateMessageMemory(SessionID session_id,
                                         size_t size_bytes) = 0;

  // Implementation may or may not return that piece of memory to the session
  // memory block. Implementation shall guarantee that session memory block will
  // be either deallocated or recycled when session is disposed.
  virtual void DeallocateMessageMemory(SessionID session_id, uint8_t* ptr) = 0;

  // [requirement] Messages shall be stored in memory from session memory block.
  //
  // Caller is responsible to ensure that `data` came from the session memory
  // block allocated by the Transport.
  virtual absl::Status SendMessage(SessionID session_id,
                                   absl::Span<const uint8_t> data,
                                   MessageCallback callback) = 0;

  // CloseSession forbids further messages to be sent over the session.
  virtual absl::Status CloseSession(SessionID session_id) = 0;

  // Disposes the session memory block.
  //
  // recycle_memory:
  //     - If true, the underlying memory block is retained by the Transport
  //       and may be reused for subsequent `OpenSession` calls, avoiding
  //       the overhead of memory allocation.
  //     - If false, the underlying memory block is permanently deallocated
  //       and returned to the system.
  virtual absl::Status DisposeSession(SessionID session_id,
                                      bool recycle_memory) = 0;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_TRANSPORT_H_
