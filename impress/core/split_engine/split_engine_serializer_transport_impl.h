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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_TRANSPORT_IMPL_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_TRANSPORT_IMPL_H_

#include <cstddef>
#include <cstdint>

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

// Routes all messages through refactored Split Engine transport layer.
//
class SplitEngineSerializerTransportImpl
    : public SplitEngineSerializerTransport {
 public:
  SplitEngineSerializerTransportImpl(
      imp::OwnedPtr<Transport> transport,
      imp::OwnedPtr<MessageGroupSender> message_group_sender,
      imp::OwnedPtr<RequestSender> request_sender,
      imp::OwnedPtr<SharedMemoryRegionAllocator> region_allocator,
      imp::OwnedPtr<BackgroundScheduler> scheduler);

  ~SplitEngineSerializerTransportImpl() override = default;

  imp::OwnedOrBorrowedPtr<flatbuffers::FlatBufferBuilder> CreateBuilder(
      MessageGroupId message_group_id, size_t initial_size_bytes) override;

  absl::StatusOr<MessageGroupId> BeginFrameUpdate(
      size_t max_message_size_bytes) override;

  absl::StatusOr<MessageGroupId> BeginOneShot(
      size_t max_message_size_bytes) override;

  absl::Status AddMessage(
      MessageGroupId message_group_id,
      imp::OwnedOrBorrowedPtr<flatbuffers::FlatBufferBuilder> builder,
      const flatbuffers::Offset<android_xr::schemas::Command>& offset) override;

  absl::Status AddMessage(
      MessageGroupId message_group_id,
      imp::OwnedOrBorrowedPtr<flatbuffers::FlatBufferBuilder> builder,
      OffsetProducer offset_fn) override;

  absl::Status End(MessageGroupId message_group_id) override;

  void Schedule(imp::Invocable<absl::Status()> fn) override;

  absl::StatusOr<int32_t> GetActiveFrameUpdatesCount() const override;

  absl::StatusOr<int32_t> GetActiveOneShotsCount() const override;

 private:
  const imp::OwnedPtr<BackgroundScheduler> scheduler_;
  const imp::OwnedPtr<SharedMemoryRegionAllocator> region_allocator_;
  const imp::OwnedPtr<Transport> transport_;
  const imp::OwnedPtr<MessageGroupSender> message_group_sender_;
  const imp::OwnedPtr<RequestSender> request_sender_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_TRANSPORT_IMPL_H_
