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

#ifndef THIRD_PARTY_SPLIT_ENGINE_TRANSPORT_H_
#define THIRD_PARTY_SPLIT_ENGINE_TRANSPORT_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <variant>

#include "absl/base/attributes.h"
#include "absl/functional/any_invocable.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "split_engine/client/types.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace split_engine {

class OwnedOrBorrowedFlatBufferBuilder {
 public:
  explicit OwnedOrBorrowedFlatBufferBuilder(
      flatbuffers::FlatBufferBuilder* builder)
      : builder_(builder) {}

  explicit OwnedOrBorrowedFlatBufferBuilder(
      std::unique_ptr<flatbuffers::FlatBufferBuilder> builder)
      : builder_(std::move(builder)) {}

  flatbuffers::FlatBufferBuilder& operator*() const { return *Get(); }

  flatbuffers::FlatBufferBuilder* operator->() const { return Get(); }

 private:
  std::variant<flatbuffers::FlatBufferBuilder*,
               std::unique_ptr<flatbuffers::FlatBufferBuilder>>
      builder_;

  flatbuffers::FlatBufferBuilder* Get() const {
    return std::holds_alternative<flatbuffers::FlatBufferBuilder*>(builder_)
               ? std::get<flatbuffers::FlatBufferBuilder*>(builder_)
               : std::get<std::unique_ptr<flatbuffers::FlatBufferBuilder>>(
                     builder_)
                     .get();
  }
};

// Thin interface to allow for different transport implementations.
//
// Methods shall be accessed from the foreground executor only.
//
// TODO: (broken link) - use signed integers instead of unsigned ones to follow
// (broken link)

class Transport {
 public:
  virtual ~Transport() = default;

  virtual absl::StatusOr<MessageGroupId> BeginFrameUpdate(
      size_t max_message_size_bytes) = 0;

  virtual absl::StatusOr<MessageGroupId> BeginOneShot(
      size_t max_message_size_bytes) = 0;

  virtual OwnedOrBorrowedFlatBufferBuilder CreateBuilder(
      MessageGroupId message_group_id, size_t initial_size_bytes) = 0;

  virtual OwnedOrBorrowedFlatBufferBuilder CreateRequestBuilder(
      size_t initial_size_bytes) = 0;

  // AddMessage accepts `builder` in unfinished state. It's up to the
  // implementation to finish the `builder` when needed.
  //
  // Accepts caller-prepared offset.
  // Suitable for simple cases when offset calculation is cheap and can be done
  // without blocking main thread.
  //
  virtual absl::Status AddMessage(
      MessageGroupId message_group_id, OwnedOrBorrowedFlatBufferBuilder builder,
      const flatbuffers::Offset<android_xr::schemas::Command>& offset) = 0;

  // Delegates offset calculation to the underlying transport.
  // Use this method if offset calculation is expensive and shall be done on the
  // background executor.
  //
  using OffsetProducer =
      absl::AnyInvocable<flatbuffers::Offset<android_xr::schemas::Command>(
          flatbuffers::FlatBufferBuilder&)>;

  virtual absl::Status AddMessage(MessageGroupId message_group_id,
                                  OwnedOrBorrowedFlatBufferBuilder builder,
                                  OffsetProducer offset_fn) = 0;

  // Schedules work to be run sequentially on a background executor.
  virtual void Schedule(absl::AnyInvocable<absl::Status()> fn) = 0;

  virtual absl::Status End(MessageGroupId message_group_id) = 0;

  virtual absl::StatusOr<int32_t> GetActiveFrameUpdatesCount() const = 0;

  // For unit tests only.
  virtual absl::StatusOr<int32_t> GetActiveOneShotsCount() const = 0;

  ABSL_DEPRECATED(
      "ClearReleasedMessageGroups() is deprecated. It will be removed as soon "
      "as we permanently switch to refactored Split Engine transport.")
  virtual void ClearReleasedMessageGroups() = 0;

  virtual void SendRequest(
      OwnedOrBorrowedFlatBufferBuilder builder,
      const flatbuffers::Offset<android_xr::schemas::Request>& offset,
      absl::AnyInvocable<
          void(absl::StatusOr<android_xr::schemas::ResponseTypesUnion>)>
          response_handler) = 0;
};

}  // namespace split_engine

#endif  // THIRD_PARTY_SPLIT_ENGINE_TRANSPORT_H_
