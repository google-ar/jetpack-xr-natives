/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_MESSAGE_GROUP_SENDER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_MESSAGE_GROUP_SENDER_H_

#include <cstddef>
#include <cstdint>
#include <variant>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/common/invocable.h"
#include "core/common/owned_ptr.h"
#include "core/common/pass_key.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/transport/flatbuffer_builder_holder.h"
#include "core/split_engine/transport/message_sender.h"
#include "core/split_engine/transport/transport.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace imp::split_engine {

class MessageGroupSender {
 public:
  // The intention behind this interface is to unify the way serializer composes
  // something called "message groups".
  //
  // Currently, existing "message group" is "discrete":
  //
  //  ┌ BeginMessageGroup
  //  ├ Message
  //  ├ Message
  //  ├ ...
  //  └ EndMessageGroup
  //
  // I.e., multiple messages are sent over time and they form "message group".
  //
  // There is a plan to introduce "self-contained message group":
  //
  //  + SelfContainedMessageGroup
  //  └-┬ Message
  //    ├ Message
  //    ├ ...
  //    └ Message
  //
  // I.e., a single message with collection of messages inside.
  //
  // These two approaches contradict in the way they use FlatbufferBuilders:
  //  1. Discrete: multiple flatbuffer builders can be active at the same time.
  //  2. Self-contained: only a single flatbuffer builder is active at a time.
  //
  // Serializer should not know about these details and should use this
  // interface blindly (pseudocode):
  //
  //   msg_sender.Start()
  //
  //   builder1 = msg_sender.CreateBuilder()
  //   builder2 = msg_sender.CreateBuilder()
  //
  //   message_offset1 = BuildMessage1(builder1)
  //   message_offset2 = BuildMessage2(builder2)
  //
  //   msg_sender.AddMessage(builder1, message_offset1)
  //   msg_sender.AddMessage(builder2, message_offset2)
  //
  //   msg_sender.Finish()
  //

  using FlatbufferBuilderHolder = FlatbufferBuilderHolder<MessageSender>;

  // This type has striking resemblance with imp::OwnedOrBorrowedPtr.
  // However, the purpose is different:
  // - Act as a holder of OwnedPtr or BorrowedPtr.
  // - Actions on BorrowedPtr / OwnedPtr are not exposed (i.e. no Borrow).
  // - Callers can access underlying FlatbufferBuilderHolder via operator*
  // - Callers can regain ownership of the stored instance via ExtractOwned() or
  // ExtractBorrowed().
  //
  // Usage of `OwnedOrBorrowedFlatbufferBuilderHolder` allows to hide
  // implementation detail (whether implementation uses single instance of
  // `FlatbufferBuilder` per "message group" or multiple ones) and in the same
  // time maintain ownership model: caller will not be able to hang on to the
  // instance of `FlatbufferBuilder` after `AddMessage` call.
  //
  class OwnedOrBorrowedFlatbufferBuilderHolder {
   public:
    explicit OwnedOrBorrowedFlatbufferBuilderHolder(
        imp::OwnedPtr<FlatbufferBuilderHolder> fbb);
    explicit OwnedOrBorrowedFlatbufferBuilderHolder(
        imp::BorrowedPtr<FlatbufferBuilderHolder> fbb);

    OwnedOrBorrowedFlatbufferBuilderHolder(
        const OwnedOrBorrowedFlatbufferBuilderHolder&) = delete;
    OwnedOrBorrowedFlatbufferBuilderHolder& operator=(
        const OwnedOrBorrowedFlatbufferBuilderHolder&) = delete;
    OwnedOrBorrowedFlatbufferBuilderHolder(
        OwnedOrBorrowedFlatbufferBuilderHolder&&) = default;
    OwnedOrBorrowedFlatbufferBuilderHolder& operator=(
        OwnedOrBorrowedFlatbufferBuilderHolder&&) = default;

    // Users of MessageGroupSender can use this to access underlying
    // FlatbufferBuilderHolder
    FlatbufferBuilderHolder& operator*() { return Get(); }

    // An implementation of MessageGroupSender can use one of these methods to
    // regain ownership of the stored instance.
    absl::StatusOr<imp::OwnedPtr<FlatbufferBuilderHolder>> ExtractOwned(
        PassKey<MessageGroupSender> passkey);
    absl::StatusOr<imp::BorrowedPtr<FlatbufferBuilderHolder>> ExtractBorrowed(
        PassKey<MessageGroupSender> passkey);

   private:
    FlatbufferBuilderHolder& Get();

    std::variant<std::monostate, imp::OwnedPtr<FlatbufferBuilderHolder>,
                 imp::BorrowedPtr<FlatbufferBuilderHolder>>
        fbb_;
  };

  enum class MessageGroupType {
    // Tailored for huge messages that are sent not on a regular basis.
    kOneShot,
    // Tailored for messages that are sent every frame.
    kFrameUpdate,
  };

  virtual ~MessageGroupSender() = default;

  // Starts a new message group, all messages are expected to fit into
  // `max_message_group_size_bytes` bytes.
  virtual absl::StatusOr<MessageGroupId> Start(
      MessageGroupType message_group_type,
      size_t max_message_group_size_bytes) = 0;

  // Creates a flatbuffer builder for a new message in the message group.
  // `initial_size_bytes` is the initial size of a buffer that is backing the
  // flatbuffer builder. If message will grow bigger, FlatbufferBuilder will
  // request more memory from the message group buffer of
  // `max_message_group_size_bytes` size.
  static constexpr size_t kInitialMessageSizeBytes = 1024;
  virtual absl::StatusOr<OwnedOrBorrowedFlatbufferBuilderHolder> CreateBuilder(
      MessageGroupId message_group_id,
      size_t initial_size_bytes = kInitialMessageSizeBytes) = 0;

  // Adds a message to the message group.
  // `offset` is pre-calculated by the caller.
  virtual absl::Status AddMessage(
      MessageGroupId message_group_id,
      OwnedOrBorrowedFlatbufferBuilderHolder fbb,
      const flatbuffers::Offset<android_xr::schemas::Command>& offset) = 0;

  using OffsetProducer =
      imp::Invocable<flatbuffers::Offset<android_xr::schemas::Command>(
          flatbuffers::FlatBufferBuilder&)>;

  // Adds a message to the message group.
  // Implementation decides _when_ to calculate the offset using provided
  // producer.
  virtual absl::Status AddMessage(MessageGroupId message_group_id,
                                  OwnedOrBorrowedFlatbufferBuilderHolder fbb,
                                  OffsetProducer offset_fn) = 0;

  // Finishes the message group, no more messages can be added to the message
  // group after this call.
  //
  // `onMessageGroupCompleted` will be called when the message group is
  // completed by the remote side.
  //
  virtual absl::Status Finish(
      MessageGroupId message_group_id,
      Transport::MessageCallback onMessageGroupCompleted =
          [](absl::Span<const uint8_t> response_bytes) {}) = 0;

  // Returns the number of message groups of the given type that were not
  // processed by the remote side yet.
  virtual absl::StatusOr<int32_t> GetMessageGroupCount(
      MessageGroupType message_group_type) = 0;

 protected:
  // Allows derived classes to interact with
  // OwnedOrBorrowedFlatbufferBuilderHolder methods.
  PassKey<MessageGroupSender> GetKey();
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_MESSAGE_GROUP_SENDER_H_
