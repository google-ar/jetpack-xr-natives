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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_TRANSPORT_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_TRANSPORT_H_

#include <cstddef>
#include <cstdint>

#include "absl/base/attributes.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/background_scheduler.h"
#include "core/common/invocable.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace imp::split_engine {

// Thin interface to allow for different transport implementations.
//
// Methods shall be accessed from the foreground executor only.
//
// TODO: (broken link) - use signed integers instead of unsigned ones to follow
// (broken link)
class SplitEngineSerializerTransport : public BackgroundScheduler {
 public:
  virtual ~SplitEngineSerializerTransport() = default;

  virtual absl::StatusOr<MessageGroupId> BeginFrameUpdate(
      size_t max_message_size_bytes) = 0;

  virtual absl::StatusOr<MessageGroupId> BeginOneShot(
      size_t max_message_size_bytes) = 0;

  virtual imp::OwnedOrBorrowedPtr<flatbuffers::FlatBufferBuilder> CreateBuilder(
      MessageGroupId message_group_id, size_t initial_size_bytes) = 0;

  // AddMessage accepts `builder` in unfinished state. It's up to the
  // implementation to finish the `builder` when needed.
  //
  // Accepts caller-prepared offset.
  // Suitable for simple cases when offset calculation is cheap and can be done
  // without blocking main thread.
  //
  virtual absl::Status AddMessage(
      MessageGroupId message_group_id,
      imp::OwnedOrBorrowedPtr<flatbuffers::FlatBufferBuilder> builder,
      const flatbuffers::Offset<android_xr::schemas::Command>& offset) = 0;

  // Delegates offset calculation to the underlying transport.
  // Use this method if offset calculation is expensive and shall be done on the
  // background executor.
  //
  using OffsetProducer =
      imp::Invocable<flatbuffers::Offset<android_xr::schemas::Command>(
          flatbuffers::FlatBufferBuilder&)>;
  virtual absl::Status AddMessage(
      MessageGroupId message_group_id,
      imp::OwnedOrBorrowedPtr<flatbuffers::FlatBufferBuilder> builder,
      OffsetProducer offset_fn) = 0;

  virtual absl::Status End(MessageGroupId message_group_id) = 0;

  virtual absl::StatusOr<int32_t> GetActiveFrameUpdatesCount() const = 0;

  // For unit tests only.
  virtual absl::StatusOr<int32_t> GetActiveOneShotsCount() const = 0;

  ABSL_DEPRECATED(
      "ClearReleasedMessageGroups() is deprecated.It will be removed as soon "
      "as we permanently switch to refactored Split Engine transport.")
  virtual void ClearReleasedMessageGroups() {}
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_TRANSPORT_H_
