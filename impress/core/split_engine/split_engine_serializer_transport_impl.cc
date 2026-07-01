// Copyright 2026 Google LLC
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

#include "core/split_engine/split_engine_serializer_transport_impl.h"

#include <cstddef>
#include <cstdint>
#include <utility>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/background_scheduler.h"
#include "core/common/invocable.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/common/owned_ptr.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_serializer_transport.h"
#include "core/split_engine/transport/message_group_sender.h"
#include "core/split_engine/transport/request_sender.h"
#include "core/split_engine/transport/shared_memory_region_allocator.h"
#include "core/split_engine/transport/transport.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace imp::split_engine {

SplitEngineSerializerTransportImpl::SplitEngineSerializerTransportImpl(
    imp::OwnedPtr<Transport> transport,
    imp::OwnedPtr<MessageGroupSender> message_group_sender,
    imp::OwnedPtr<RequestSender> request_sender,
    imp::OwnedPtr<SharedMemoryRegionAllocator> region_allocator,
    imp::OwnedPtr<BackgroundScheduler> scheduler)
    : scheduler_(std::move(scheduler)),
      region_allocator_(std::move(region_allocator)),
      transport_(std::move(transport)),
      message_group_sender_(std::move(message_group_sender)),
      request_sender_(std::move(request_sender)) {}

imp::OwnedOrBorrowedPtr<flatbuffers::FlatBufferBuilder>
SplitEngineSerializerTransportImpl::CreateBuilder(
    MessageGroupId message_group_id, size_t initial_size_bytes) {
  absl::StatusOr<imp::OwnedOrBorrowedPtr<flatbuffers::FlatBufferBuilder>>
      builder = message_group_sender_->CreateBuilder(message_group_id,
                                                     initial_size_bytes);
  
  return *std::move(builder);
}

absl::StatusOr<MessageGroupId>
SplitEngineSerializerTransportImpl::BeginFrameUpdate(
    size_t max_message_size_bytes) {
  return message_group_sender_->Start(
      MessageGroupSender::MessageGroupType::kFrameUpdate,
      max_message_size_bytes);
}

absl::StatusOr<MessageGroupId> SplitEngineSerializerTransportImpl::BeginOneShot(
    size_t max_message_size_bytes) {
  return message_group_sender_->Start(
      MessageGroupSender::MessageGroupType::kOneShot, max_message_size_bytes);
}

absl::Status SplitEngineSerializerTransportImpl::AddMessage(
    MessageGroupId message_group_id,
    imp::OwnedOrBorrowedPtr<flatbuffers::FlatBufferBuilder> builder,
    const flatbuffers::Offset<android_xr::schemas::Command>& offset) {
  return message_group_sender_->AddMessage(message_group_id, std::move(builder),
                                           offset);
}

absl::Status SplitEngineSerializerTransportImpl::AddMessage(
    MessageGroupId message_group_id,
    imp::OwnedOrBorrowedPtr<flatbuffers::FlatBufferBuilder> builder,
    OffsetProducer offset_fn) {
  return message_group_sender_->AddMessage(message_group_id, std::move(builder),
                                           std::move(offset_fn));
}

absl::Status SplitEngineSerializerTransportImpl::End(
    MessageGroupId message_group_id) {
  return message_group_sender_->Finish(message_group_id);
}

void SplitEngineSerializerTransportImpl::Schedule(
    imp::Invocable<absl::Status()> fn) {
  scheduler_->Schedule(std::move(fn));
}

absl::StatusOr<int32_t>
SplitEngineSerializerTransportImpl::GetActiveFrameUpdatesCount() const {
  return message_group_sender_->GetMessageGroupCount(
      MessageGroupSender::MessageGroupType::kFrameUpdate);
}

absl::StatusOr<int32_t>
SplitEngineSerializerTransportImpl::GetActiveOneShotsCount() const {
  return message_group_sender_->GetMessageGroupCount(
      MessageGroupSender::MessageGroupType::kOneShot);
}

}  // namespace imp::split_engine
