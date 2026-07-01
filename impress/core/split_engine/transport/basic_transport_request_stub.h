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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_BASIC_TRANSPORT_REQUEST_STUB_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_BASIC_TRANSPORT_REQUEST_STUB_H_

#include <cstddef>
#include <cstdint>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "core/split_engine/transport/transport.h"

namespace imp::split_engine {

class RequestTransportStub : public Transport {
 public:
  RequestTransportStub() = default;
  ~RequestTransportStub() override = default;

  absl::StatusOr<SessionID> OpenSession(size_t session_max_size_bytes) override;

  uint8_t* AllocateMessageMemory(SessionID session_id,
                                 size_t size_bytes) override;

  void DeallocateMessageMemory(SessionID session_id, uint8_t* ptr) override;

  absl::Status SendMessage(SessionID session_id, absl::Span<const uint8_t> data,
                           MessageCallback callback) override;

  absl::Status CloseSession(SessionID session_id) override;

  absl::Status DisposeSession(SessionID session_id,
                              bool recycle_memory) override;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_BASIC_TRANSPORT_REQUEST_STUB_H_
