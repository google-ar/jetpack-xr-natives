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

#include "core/split_engine/transport/message_sender.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "flatbuffers/allocator.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/background_scheduler.h"
#include "core/async/executor.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/common/owned_ptr.h"
#include "core/common/pass_key.h"
#include "core/split_engine/transport/flatbuffer_builder_holder.h"
#include "core/split_engine/transport/transport.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::split_engine {

class MessageSender::Allocator : public flatbuffers::Allocator {
 public:
  Allocator(imp::BorrowedPtr<Transport> transport, SessionID session_id)
      : transport_(std::move(transport)), session_id_(session_id) {}

  uint8_t* allocate(size_t size) override {
    return transport_->AllocateMessageMemory(session_id_, size);
  }

  void deallocate(uint8_t* ptr, size_t size) override {
    transport_->DeallocateMessageMemory(session_id_, ptr);
  }

 private:
  imp::BorrowedPtr<Transport> transport_;
  const SessionID session_id_;
};

MessageSender::MessageSender(imp::OwnedOrBorrowedPtr<Transport> transport,
                             imp::BorrowedPtr<BackgroundScheduler> scheduler)
    : transport_(std::move(transport)), scheduler_(std::move(scheduler)) {}

absl::StatusOr<MessageSender::SessionID> MessageSender::OpenSession(
    size_t session_max_size_bytes) {
  

  MP_ASSIGN_OR_RETURN(const Transport::SessionID session_id,
                   transport_->OpenSession(session_max_size_bytes));

  flatbuffer_allocators_.emplace(
      session_id, std::make_unique<Allocator>(transport_.Borrow(), session_id));
  return session_id;
}

absl::StatusOr<imp::OwnedPtr<FlatbufferBuilderHolder<MessageSender>>>
MessageSender::CreateMessageBuilder(SessionID session_id, size_t initial_size) {
  

  auto it = flatbuffer_allocators_.find(session_id);
  if (it == flatbuffer_allocators_.end()) {
    return absl::InternalError("Failed to create flatbuffer builder");
  }

  return imp::OwnedPtr<FlatbufferBuilderHolder>(new FlatbufferBuilderHolder(
      PassKey<MessageSender>(),
      std::make_unique<flatbuffers::FlatBufferBuilder>(initial_size,
                                                       it->second.get())));
}

absl::Status MessageSender::SendMessage(
    SessionID session_id, imp::OwnedPtr<FlatbufferBuilderHolder> fbb,
    Transport::MessageCallback callback) {
  

  scheduler_->Schedule([this, session_id, fbb = std::move(fbb),
                        callback = std::move(callback)]() mutable {
    return transport_->SendMessage(
        session_id,
        absl::MakeConstSpan((*fbb)->GetBufferPointer(), (*fbb)->GetSize()),
        std::move(callback));
  });

  return absl::OkStatus();
}

absl::Status MessageSender::SendMessage(SessionID session_id,
                                        FlatbufferBuilderProducer fbb_fn,
                                        Transport::MessageCallback callback) {
  

  scheduler_->Schedule([this, session_id, fbb_fn = std::move(fbb_fn),
                        callback = std::move(callback)]() mutable {
    auto fbb = fbb_fn();
    return transport_->SendMessage(
        session_id,
        absl::MakeConstSpan((*fbb)->GetBufferPointer(), (*fbb)->GetSize()),
        std::move(callback));
  });

  return absl::OkStatus();
}

absl::Status MessageSender::CloseSession(SessionID session_id) {
  

  if (flatbuffer_allocators_.find(session_id) == flatbuffer_allocators_.end()) {
    return absl::FailedPreconditionError("Session is not open.");
  }

  scheduler_->Schedule(
      [this, session_id]() { return transport_->CloseSession(session_id); });
  return absl::OkStatus();
}

absl::Status MessageSender::DisposeSession(SessionID session_id,
                                           bool recycle_memory) {
  

  auto it = flatbuffer_allocators_.find(session_id);
  if (it == flatbuffer_allocators_.end()) {
    return absl::FailedPreconditionError("Session is not open.");
  }

  std::unique_ptr<Allocator> allocator =
      std::move(flatbuffer_allocators_.extract(it).mapped());

  scheduler_->Schedule([this, session_id, recycle_memory,
                        allocator = std::move(allocator)]() mutable {
    // Allocator shall be destroyed after the last message is sent to make sure
    // that last message's FlatbufferBuilder is destroyed _before_ the
    // allocator.
    allocator.reset();
    return transport_->DisposeSession(session_id, recycle_memory);
  });

  return absl::OkStatus();
}

MessageSender::~MessageSender() = default;

}  // namespace imp::split_engine
