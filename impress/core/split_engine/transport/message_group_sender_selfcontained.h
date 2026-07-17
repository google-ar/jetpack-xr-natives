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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_MESSAGE_GROUP_SENDER_SELFCONTAINED_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_MESSAGE_GROUP_SENDER_SELFCONTAINED_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/buffer.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/transport/message_group_sender.h"
#include "core/split_engine/transport/message_sender.h"
#include "core/split_engine/transport/transport.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace imp::split_engine {

// Boilerplate implementation of a sender that sends self-contained message
// groups:
//
//  + SelfContainedMessageGroup
//  └-┬ Message
//    ├ Message
//    ├ ...
//    └ Message
class SelfContainedMessageGroupSender : public MessageGroupSender {
 public:
  explicit SelfContainedMessageGroupSender(
      OwnedOrBorrowedPtr<MessageSender> sender);

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

 private:
  OwnedOrBorrowedPtr<MessageSender> sender_;
  std::vector<flatbuffers::Offset<android_xr::schemas::Command>> commands_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_MESSAGE_GROUP_SENDER_SELFCONTAINED_H_
