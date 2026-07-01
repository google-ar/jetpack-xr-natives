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

#include "core/split_engine/transport/basic_transport_request_stub.h"

#include <cstddef>
#include <cstdint>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "core/split_engine/transport/transport.h"

namespace imp::split_engine {

absl::StatusOr<Transport::SessionID> RequestTransportStub::OpenSession(
    size_t session_max_size_bytes) {
  return Transport::kPermanentSessionID;
}

uint8_t* RequestTransportStub::AllocateMessageMemory(SessionID session_id,
                                                     size_t size_bytes) {
  return new uint8_t[size_bytes];
}

void RequestTransportStub::DeallocateMessageMemory(SessionID session_id,
                                                   uint8_t* ptr) {
  
  delete[] ptr;
}

absl::Status RequestTransportStub::SendMessage(SessionID session_id,
                                               absl::Span<const uint8_t> data,
                                               MessageCallback callback) {
  return absl::OkStatus();
}

absl::Status RequestTransportStub::CloseSession(SessionID session_id) {
  return absl::OkStatus();
}

absl::Status RequestTransportStub::DisposeSession(SessionID session_id,
                                                  bool recycle_memory) {
  return absl::OkStatus();
}

}  // namespace imp::split_engine
