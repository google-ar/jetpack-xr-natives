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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_BASIC_TRANSPORT_REQUEST_ADAPTER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_BASIC_TRANSPORT_REQUEST_ADAPTER_H_

#include <cstddef>
#include <cstdint>
#include <utility>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "core/split_engine/transport/transport.h"

namespace imp::split_engine {

// RequestSenderTransportAdapter is a Transport adapter that uses
// Transport::SendMessage to send requests.
//
// Allocation is happening from the heap. kPermanentSessionID is helping with
// that.
//
class RequestSenderTransportAdapter : public Transport {
 public:
  explicit RequestSenderTransportAdapter(Transport& transport)
      : transport_(transport) {}

  absl::StatusOr<SessionID> OpenSession(
      size_t session_max_size_bytes) override {
    return Transport::kPermanentSessionID;
  }

  uint8_t* AllocateMessageMemory(SessionID session_id,
                                 size_t size_bytes) override {
    return transport_.AllocateMessageMemory(session_id, size_bytes);
  }

  void DeallocateMessageMemory(SessionID session_id, uint8_t* ptr) override {
    transport_.DeallocateMessageMemory(session_id, ptr);
  }

  absl::Status SendMessage(SessionID session_id, absl::Span<const uint8_t> data,
                           MessageCallback callback) override {
    
    return transport_.SendMessage(session_id, data, std::move(callback));
  }

  absl::Status CloseSession(SessionID session_id) override {
    
    return absl::OkStatus();
  }

  absl::Status DisposeSession(SessionID session_id,
                              bool recycle_memory) override {
    
    return absl::OkStatus();
  }

 private:
  Transport& transport_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_BASIC_TRANSPORT_REQUEST_ADAPTER_H_
