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

#include "core/split_engine/transport/request_sender.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "flatbuffers/allocator.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/verifier.h"
#include "core/async/executor.h"
#include "core/common/invocable.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/common/owned_ptr.h"
#include "core/common/pass_key.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/transport/flatbuffer_builder_holder.h"
#include "core/split_engine/transport/transport.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::split_engine {

class Allocator : public flatbuffers::Allocator {
 public:
  Allocator(imp::BorrowedPtr<Transport> transport,
            Transport::SessionID session_id)
      : transport_(std::move(transport)), session_id_(session_id) {}

  uint8_t* allocate(size_t size) override {
    return transport_->AllocateMessageMemory(session_id_, size);
  }

  void deallocate(uint8_t* ptr, size_t size) override {
    transport_->DeallocateMessageMemory(session_id_, ptr);
  }

 private:
  imp::BorrowedPtr<Transport> transport_;
  const Transport::SessionID session_id_;
};

RequestSender::RequestSender(imp::OwnedOrBorrowedPtr<Transport> transport)
    : transport_(std::move(transport)) {
  
}

void RequestSender::StoreSessionID(uint64_t key,
                                   Transport::SessionID session_id) {
  if (session_id != Transport::kPermanentSessionID) {
    flatbuffer_builder_to_session_id_.emplace(key, session_id);
  }
}

Transport::SessionID RequestSender::ExtractSessionID(uint64_t key) {
  auto it = flatbuffer_builder_to_session_id_.find(key);
  if (it == flatbuffer_builder_to_session_id_.end()) {
    return Transport::kPermanentSessionID;
  }
  const Transport::SessionID session_id = it->second;
  flatbuffer_builder_to_session_id_.erase(it);
  return session_id;
}

absl::StatusOr<imp::OwnedPtr<FlatbufferBuilderHolder<RequestSender>>>
RequestSender::CreateRequestBuilder(size_t size) {
  

  MP_ASSIGN_OR_RETURN(const Transport::SessionID session_id,
                   transport_->OpenSession(size));

  // FlatbufferBuilder takes ownership of the allocator.
  auto fbb = std::make_unique<flatbuffers::FlatBufferBuilder>(
      size, new Allocator(transport_.Borrow(), session_id),
      /*own_allocator=*/true);

  const uint64_t key = reinterpret_cast<uint64_t>(fbb.get());
  StoreSessionID(key, session_id);

  return imp::OwnedPtr<FlatbufferBuilderHolder>(new FlatbufferBuilderHolder(
      PassKey<RequestSender>(), std::move(fbb), [this, key, session_id]() {
        

        // For the case when `SendRequest` is never called, but result of
        // `CreateRequestBuilder` is discarded, we need to clean things up by
        // hand.
        ExtractSessionID(key);

        
        
      }));
}

absl::Status RequestSender::SendRequest(
    imp::OwnedPtr<FlatbufferBuilderHolder> fbb,
    RequestSender::RequestCallback callback) {
  

  // Cleanup will be performed below, so cancel manual cleanup.
  fbb->CancelCleanup(PassKey<RequestSender>());

  const Transport::SessionID session_id =
      ExtractSessionID(reinterpret_cast<uint64_t>(&**fbb));

  const absl::Span<const uint8_t> request_span =
      absl::MakeConstSpan((*fbb)->GetBufferPointer(), (*fbb)->GetSize());
  const absl::Status send_status = transport_->SendMessage(
      session_id, request_span,
      [this, callback = std::move(callback), session_id,
       fbb = std::move(fbb)](absl::Span<const uint8_t> response_bytes) mutable {
        if (const auto dispose_status = this->transport_->DisposeSession(
                session_id, /*recycle_memory=*/false);
            !dispose_status.ok()) {
          IMP_LOG(imp::ERROR) << "Failed to dispose session: " << dispose_status;
        }

        flatbuffers::Verifier verifier(response_bytes.data(),
                                       response_bytes.size());
        if (!verifier.VerifyBuffer<android_xr::schemas::Response>()) {
          callback(
              absl::InternalError("SendRequest: response does not conform "
                                  "to Response schema."));
          return;
        }

        const android_xr::schemas::Response* response =
            flatbuffers::GetRoot<android_xr::schemas::Response>(
                response_bytes.data());

        if (response == nullptr) {
          callback(absl::InternalError(
              "SendRequest: failed to parse response from data."));
          return;
        }

        if (response->response_type() ==
            android_xr::schemas::ResponseTypes::ErrorResponse) {
          const android_xr::schemas::ErrorResponse* error_response =
              response->response_as<android_xr::schemas::ErrorResponse>();
          callback(ErrorCodeToStatus(error_response->error_code(),
                                     error_response->error_message()->str()));
          return;
        }

        callback(std::make_unique<Response>(std::vector<uint8_t>(
            response_bytes.begin(), response_bytes.end())));
      });

  

  return transport_->CloseSession(session_id);
}

}  // namespace imp::split_engine
