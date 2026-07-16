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

#include "core/split_engine/transport/transport_with_hooks.h"

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

imp::split_engine::TransportWithHooks::TransportWithHooks(
    imp::OwnedOrBorrowedPtr<Transport> transport,
    /*absl_nonnull*/  std::unique_ptr<TransportHooks> hooks)
    : transport_(std::move(transport)), hooks_(std::move(hooks)) {}

absl::StatusOr<Transport::SessionID> TransportWithHooks::OpenSession(
    size_t session_max_size_bytes) {
  const auto pre_open_status = hooks_->OnPreOpenSession(*transport_);
  if (!absl::IsCancelled(pre_open_status.status())) {
    return pre_open_status;
  }

  const auto open_status = transport_->OpenSession(session_max_size_bytes);
  if (!open_status.ok()) {
    return open_status;
  }

  const SessionID session_id = open_status.value();
  const auto post_open_status =
      hooks_->OnPostOpenSession(*transport_, session_id);
  if (!post_open_status.ok()) {
    CloseSession(session_id).IgnoreError();
    DisposeSession(session_id, /*recycle_memory=*/false).IgnoreError();
    return post_open_status;
  }

  return session_id;
}

absl::Status TransportWithHooks::SendMessage(SessionID session_id,
                                             absl::Span<const uint8_t> data,
                                             MessageCallback callback) {
  auto new_callback = hooks_->OnPreSendMessage(*transport_, session_id, data,
                                               std::move(callback));
  if (!new_callback.ok()) {
    return new_callback.status();
  }

  return transport_->SendMessage(session_id, data, std::move(*new_callback));
}

absl::Status TransportWithHooks::CloseSession(SessionID session_id) {
  const auto pre_close_status =
      hooks_->OnPreCloseSession(*transport_, session_id);
  if (!absl::IsCancelled(pre_close_status) && !pre_close_status.ok()) {
    return pre_close_status;
  }
  return transport_->CloseSession(session_id);
}

absl::Status TransportWithHooks::DisposeSession(SessionID session_id,
                                                bool recycle_memory) {
  return transport_->DisposeSession(session_id, recycle_memory);
}

uint8_t* TransportWithHooks::AllocateMessageMemory(SessionID session_id,
                                                   size_t size_bytes) {
  return transport_->AllocateMessageMemory(session_id, size_bytes);
}

void TransportWithHooks::DeallocateMessageMemory(SessionID session_id,
                                                 uint8_t* ptr) {
  transport_->DeallocateMessageMemory(session_id, ptr);
}

}  // namespace imp::split_engine
