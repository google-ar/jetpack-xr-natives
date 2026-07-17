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

#include "core/split_engine/transport/message_group_sender_selfcontained.h"

#include <cstddef>
#include <cstdint>
#include <utility>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/buffer.h"
#include "core/async/executor.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/transport/message_group_sender.h"
#include "core/split_engine/transport/message_sender.h"
#include "core/split_engine/transport/transport.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace imp::split_engine {

SelfContainedMessageGroupSender::SelfContainedMessageGroupSender(
    OwnedOrBorrowedPtr<MessageSender> sender)
    : sender_(std::move(sender)) {}

absl::StatusOr<MessageGroupId> SelfContainedMessageGroupSender::Start(
    MessageGroupType message_group_type, size_t max_message_group_size_bytes) {
  

  return absl::UnimplementedError("Not implemented.");
}

absl::StatusOr<MessageGroupSender::OwnedOrBorrowedFlatbufferBuilderHolder>
SelfContainedMessageGroupSender::CreateBuilder(MessageGroupId message_group_id,
                                               size_t initial_size_bytes) {
  return absl::UnimplementedError("Not implemented.");
}

absl::Status SelfContainedMessageGroupSender::AddMessage(
    MessageGroupId message_group_id, OwnedOrBorrowedFlatbufferBuilderHolder fbb,
    const flatbuffers::Offset<android_xr::schemas::Command>& offset) {
  

  return absl::UnimplementedError("Not implemented.");
}

absl::Status SelfContainedMessageGroupSender::AddMessage(
    MessageGroupId message_group_id, OwnedOrBorrowedFlatbufferBuilderHolder fbb,
    OffsetProducer offset_fn) {
  

  return absl::UnimplementedError("Not implemented.");
}

absl::Status SelfContainedMessageGroupSender::Finish(
    MessageGroupId message_group_id,
    Transport::MessageCallback onMessageGroupCompleted) {
  

  return absl::UnimplementedError("Not implemented.");
}

absl::StatusOr<int32_t> SelfContainedMessageGroupSender::GetMessageGroupCount(
    MessageGroupType message_group_type) {
  return absl::UnimplementedError("Not implemented.");
}

}  // namespace imp::split_engine
