/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_TRANSPORT_WITH_HOOKS_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_TRANSPORT_WITH_HOOKS_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>

#include "absl/base/nullability.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/split_engine/transport/transport.h"

namespace imp::split_engine {

// Adapter to allow using hooks to control Transport's behavior.
//
class TransportWithHooks : public Transport {
 public:
  class TransportHooks {
   public:
    virtual ~TransportHooks() = default;

    // Hook will be called before OpenSession call.
    //
    // Transport shall:
    //   - If hook returned session ID, Transport shall not call OpenSession.
    //   - If hook returned CancelledError, Transport shall call OpenSession.
    //   - If hook returned any other error, Transport shall not call
    //   OpenSession and return that error.
    //
    // Example: hook may return kPermanentSessionID to indicate that it wants to
    // use permanent session.
    virtual absl::StatusOr<SessionID> OnPreOpenSession(Transport& transport) {
      return absl::CancelledError("Proceed with OpenSession call.");
    }

    // Hook will be called after OpenSession call.
    //
    // Transport shall:
    //   - If hook returned OkStatus or CancelledError, Transport does nothing.
    //   - If hook returned any other error, Transport shall Close and Dispose
    //   the session and return that error.
    //
    // Example: hook may send extra message to the remote side right after the
    // session is opened.
    //
    virtual absl::Status OnPostOpenSession(Transport& transport,
                                           SessionID session_id) {
      return absl::OkStatus();
    }

    // Hook will be called before SendMessage call.
    //
    // Transport shall:
    //   - If hook returned MessageCallback, Transport shall call SendMessage
    //   with that callback.
    //   - If hook returned any other error, Transport shall not call
    //   SendMessage and return that error.
    //
    // Example: hook may adjust message acknowledgement process.
    virtual absl::StatusOr<MessageCallback> OnPreSendMessage(
        Transport& transport, SessionID session_id,
        absl::Span<const uint8_t> data, MessageCallback callback) {
      return std::move(callback);
    }

    // Hook will be called before CloseSession call.
    //
    // Transport shall:
    //   - If hook returned OkStatus or CancelledError, Transport shall proceed
    //   with CloseSession call.
    //   - If hook returned any other error, Transport shall not call
    //   CloseSession and return that error.
    //
    // Example: hook may send extra message to the remote side before closing
    // the session.
    virtual absl::Status OnPreCloseSession(Transport& transport,
                                           SessionID session_id) {
      return absl::OkStatus();
    }
  };

  // If all components use `TransportWithHooks` as the only `Transport`,
  // `TransportWithHooks` shall own the `Transport` it wraps.
  //
  // If some other component uses `TransportWithHooks` as the adapter to an
  // existing `Transport`, that component shall provide borrowed copy of the
  // `Transport`.
  TransportWithHooks(imp::OwnedOrBorrowedPtr<Transport> transport,
                     /*absl_nonnull*/  std::unique_ptr<TransportHooks> hooks);

  absl::StatusOr<SessionID> OpenSession(size_t session_max_size_bytes) override;

  absl::Status SendMessage(SessionID session_id, absl::Span<const uint8_t> data,
                           MessageCallback callback) override;

  absl::Status CloseSession(SessionID session_id) override;

  absl::Status DisposeSession(SessionID session_id,
                              bool recycle_memory) override;

  uint8_t* AllocateMessageMemory(SessionID session_id,
                                 size_t size_bytes) override;

  void DeallocateMessageMemory(SessionID session_id, uint8_t* ptr) override;

 private:
  const imp::OwnedOrBorrowedPtr<Transport> transport_;
  const /*absl_nonnull*/  std::unique_ptr<TransportHooks> hooks_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_TRANSPORT_WITH_HOOKS_H_
