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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_MESSAGE_GROUP_SENDER_DISCRETE_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_MESSAGE_GROUP_SENDER_DISCRETE_H_

#include <cstddef>
#include <cstdint>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/buffer.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/transport/message_group_id_generator.h"
#include "core/split_engine/transport/message_group_sender.h"
#include "core/split_engine/transport/message_sender.h"
#include "core/split_engine/transport/transport.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace imp::split_engine {

// Implementation of MesssageGroupSender that sends discrete message groups.
//
//  ┌ BeginMessageGroup
//  ├ Message
//  ├ Message
//  ├ ...
//  └ EndMessageGroup
//
class DiscreteMessageGroupSender : public MessageGroupSender {
 public:
  DiscreteMessageGroupSender(
      OwnedOrBorrowedPtr<MessageSender> sender,
      MessageGroupIdGenerator& message_group_id_generator);

  absl::StatusOr<MessageGroupId> Start(
      MessageGroupType message_group_type,
      size_t max_message_group_size_bytes) override;

  absl::StatusOr<OwnedOrBorrowedFlatbufferBuilderHolder> CreateBuilder(
      MessageGroupId message_group_id, size_t initial_size_bytes) override;

  absl::Status AddMessage(
      MessageGroupId message_group_id,
      OwnedOrBorrowedFlatbufferBuilderHolder fbb,
      const flatbuffers::Offset<android_xr::schemas::Command>& offset) override;

  absl::Status AddMessage(MessageGroupId message_group_id,
                          OwnedOrBorrowedFlatbufferBuilderHolder fbb,
                          OffsetProducer offset_fn) override;

  absl::Status Finish(
      MessageGroupId message_group_id,
      Transport::MessageCallback onMessageGroupCompleted) override;

  absl::StatusOr<int32_t> GetMessageGroupCount(
      MessageGroupType message_group_type) override;

 protected:
  const absl::flat_hash_set<MessageGroupId>& GetFrameUpdates() const {
    return frame_updates_;
  }
  const absl::flat_hash_set<MessageGroupId>& GetOneShotUpdates() const {
    return one_shot_updates_;
  }

 private:
  OwnedOrBorrowedPtr<MessageSender> sender_;
  MessageGroupIdGenerator& message_group_id_generator_;

  struct MessageGroupInfo {
    const Transport::SessionID session_id;
    const MessageGroupType type;
  };

  // The following members are only accessed on the foreground executor thread,
  // so no additional synchronization is needed.

  // Started, but not finished by DiscreteMessageGroupSender: entries are added
  // in `Start` and removed in `Finish`.
  absl::flat_hash_map<MessageGroupId, MessageGroupInfo> message_group_info_;

  // Started, but not confirmed by Renderer: entries are added in `Start` and
  // removed in the callback of EndMessageGroup.
  absl::flat_hash_set<MessageGroupId> frame_updates_;
  absl::flat_hash_set<MessageGroupId> one_shot_updates_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_MESSAGE_GROUP_SENDER_DISCRETE_H_
