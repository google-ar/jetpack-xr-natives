/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_BRIDGE_SENDER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_BRIDGE_SENDER_H_

#include <cstddef>
#include <functional>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/background_scheduler.h"
#include "core/common/owned_ptr.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

// An interface for SplitEngine to use to send serialized data to a renderer.
// TODO: (broken link) - remove SplitEngineBridgeSender interface.
class SplitEngineBridgeSender : public BackgroundScheduler {
 public:
  // Implementation of SplitEngineBridgeSender shall support multiple active
  // message groups in parallel.
  //
  // `BeginMessageGroup()` returns an ID that is used to identify which
  // message group to interact with during the following calls:
  // - `SendMessage()`
  // - `EndMessageGroup()`
  // - `CreateFlatBufferBuilder()`
  //
  // The type of message group to create.
  //
  // kOneShot message groups are used for sending larger, less frequent data
  // types, such as textures and meshes.
  //
  // kFrameUpdate message groups are used for sending data that is updated
  // every frame.
  enum class MessageType {
    kOneShot,
    kFrameUpdate,
  };

  virtual ~SplitEngineBridgeSender() = default;

  // TODO - food for thought:
  // 1. There's CreateFlatBufferBuilder() that creates a new FlatBufferBuilder
  //    that shall be used to build a message.
  // 2. There's SendMessage() that accepts _a_ FlatBufferBuilder that may not
  //    have come from CreateFlatBufferBuilder().
  //
  // Implementation will CHECK-crash now if the underlying buffer is not part of
  // the shared memory. There should be typesafe way to enforce this without
  // crashing.

  virtual absl::Status SendMessage(
      MessageGroupId group_id,
      imp::OwnedPtr<flatbuffers::FlatBufferBuilder> message) = 0;

  // Begin a new message group which will be contained within buffer of
  // specified size. The SplitEngineBridgeSender will handle adding overhead
  // for begin and end message group messages.
  virtual absl::StatusOr<MessageGroupId> BeginMessageGroup(
      size_t size_byte, MessageType message_type) = 0;

  // Ends the current message group and mark it eligible for release once
  // all messages in the group have been processed. A new message group will
  // need to be started before any more messages can be sent.
  virtual absl::Status EndMessageGroup(MessageGroupId group_id) = 0;

  // Returns the total number of message groups that remote side has not
  // released yet. 'Total' means that the result is not restricted to the
  // particular sender, but rather the total number of message groups that
  // were sent by every sender and have not been released yet by the remote
  // side.
  virtual absl::StatusOr<size_t> GetActiveMessageGroupCount() const = 0;

  // Clears any message groups that have been released via ReleaseMessageGroup.
  // This should be called once per frame to ensure that any message groups
  // marked for release are actually released. This is a no-op if there are no
  // released message groups.
  virtual void ClearReleasedMessageGroups() = 0;

  // Creates a new flatbuffer builder that is backed by a buffer of the
  // specified size. It is the responsibility of the caller to ensure that the
  // the current message group has sufficient space to contain the builder.
  //
  // The memory backing the FlatBufferBuilder is owned by the
  // SplitEngineBridgeSender, and so the returned FlatBufferBuilder is only
  // valid for the lifetime of the SplitEngineBridgeSender.
  //
  // Returned FlatBufferBuilder will be shared between serialization tasks and
  // SendMessage task. Serialization tasks should use BorrowedPtr, SendMessage
  // task should take the ownership of imp::OwnedPtr.
  virtual imp::OwnedPtr<flatbuffers::FlatBufferBuilder> CreateFlatBufferBuilder(
      MessageGroupId group_id, size_t size_bytes) = 0;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_BRIDGE_SENDER_H_
