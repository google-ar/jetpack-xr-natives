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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_TRANSPORT_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_TRANSPORT_H_

#include <cstddef>
#include <cstdint>
#include <functional>

#include "absl/base/attributes.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/background_scheduler.h"
#include "core/common/invocable.h"
#include "core/common/owned_ptr.h"
#include "core/common/pass_key.h"
#include "core/split_engine/android/split_engine_android_bridge.h"
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
  // The goal of `MessageBuilder` class is to prevent programming errors where
  // flatbuffer::FlatBufferBuilder is created manually (heap, stack) instead of
  // using `SplitEngineSerializerTransport::CreateBuilder`, because underlying
  // transport might need builder to use custom allocator.
  //
  // Implementations of `SplitEngineSerializerTransport` shall create their own
  // implementation-specific class derived from `MessageBuilder` to wrap
  // whatever underlying transport is using to hold its builders:
  // - underlying transport might be using raw flatbuffer::FlatBufferBuilder
  // - underlying transport might be using PassKey protected holder.
  //
  // This base class hides those implementation details, and it allows callers
  // to interact with `flatbuffers::FlatBufferBuilder` in the same way
  // regardless of the underlying transport. It intentionally avoids virtual
  // methods for builder access to prevent virtual dispatch overhead inside
  // tight loops.
  //
  // See `GetPassKey` comments for more details.
  class MessageBuilder {
   public:
    MessageBuilder(PassKey<SplitEngineSerializerTransport> pass_key,
                   flatbuffers::FlatBufferBuilder& builder)
        : builder_(builder) {}
    virtual ~MessageBuilder() = default;

    MessageBuilder(MessageBuilder&& other) = default;
    MessageBuilder& operator=(MessageBuilder&& other) = default;
    MessageBuilder(const MessageBuilder&) = delete;
    MessageBuilder& operator=(const MessageBuilder&) = delete;

    flatbuffers::FlatBufferBuilder& operator*() { return builder_; }
    flatbuffers::FlatBufferBuilder* operator->() { return &builder_; }

   private:
    flatbuffers::FlatBufferBuilder& builder_;
  };

  virtual ~SplitEngineSerializerTransport() = default;

  virtual absl::StatusOr<MessageGroupId> BeginFrameUpdate(
      size_t max_message_size_bytes) = 0;

  virtual absl::StatusOr<MessageGroupId> BeginOneShot(
      size_t max_message_size_bytes) = 0;

  virtual imp::OwnedPtr<MessageBuilder> CreateBuilder(
      MessageGroupId message_group_id, size_t initial_size_bytes) = 0;

  // AddMessage accepts `builder` in unfinished state. It's up to the
  // implementation to finish the `builder` when needed.
  //
  // Accepts caller-prepared offset.
  // Suitable for simple cases when offset calculation is cheap and can be done
  // without blocking main thread.
  //
  virtual absl::Status AddMessage(
      MessageGroupId message_group_id, imp::OwnedPtr<MessageBuilder> builder,
      const flatbuffers::Offset<android_xr::schemas::Command>& offset) = 0;

  // Delegates offset calculation to the underlying transport.
  // Use this method if offset calculation is expensive and shall be done on the
  // background executor.
  //
  using OffsetProducer =
      imp::Invocable<flatbuffers::Offset<android_xr::schemas::Command>(
          flatbuffers::FlatBufferBuilder&)>;
  virtual absl::Status AddMessage(MessageGroupId message_group_id,
                                  imp::OwnedPtr<MessageBuilder> builder,
                                  OffsetProducer offset_fn) = 0;

  virtual absl::Status End(MessageGroupId message_group_id) = 0;

  virtual absl::StatusOr<int32_t> GetActiveFrameUpdatesCount() const = 0;

  // For unit tests only.
  virtual absl::StatusOr<int32_t> GetActiveOneShotsCount() const = 0;

  // Backward compatibility methods to support legacy Split Engine transport
  // implementation.
  //
  // As soon as we permanently switch to refactored Split Engine transport and
  // new AIDL (sendMessage), these methods will be removed.
  ABSL_DEPRECATED(
      "GetBridge() is deprecated. It will be removed as soon as we permanently "
      "switch to "
      "refactored Split Engine transport.")
  virtual absl::StatusOr<
      std::reference_wrapper<imp::split_engine::SplitEngineAndroidBridge>>
  GetBridge() {
    return absl::UnimplementedError("GetBridge() is deprecated");
  }

  ABSL_DEPRECATED(
      "ClearReleasedMessageGroups() is deprecated.It will be removed as soon "
      "as we permanently switch to refactored Split Engine transport.")
  virtual void ClearReleasedMessageGroups() {}

 protected:
  // Allow derived classes to make classes derived from `MessageBuilder`.
  //
  // The goal is to make sure that an instance of a MessageBuilder passed into
  // `AddMessage` came from `CreateBuilder`. There will be only one instance of
  // a class derived from `SplitEngineSerializerTransport` at any given time,
  // and in turn the implementation of `AddMessage` can downcast
  // `MessageBuilder` safely to the implementation specific one.
  //
  PassKey<SplitEngineSerializerTransport> GetPassKey() {
    // The key returned from `GetPassKey` is "hot potato": derived classes can
    // only pass it as prvalue (a.k.a. temporary, unnamed value) to
    // MessageBuilder's constructor: C++17 copy elision will make sure that copy
    // constructor is never called.
    //
    return {};
  }
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_TRANSPORT_H_
