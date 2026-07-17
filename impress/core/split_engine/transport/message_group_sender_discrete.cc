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

#include "core/split_engine/transport/message_group_sender_discrete.h"

#include <cstddef>
#include <cstdint>
#include <utility>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "flatbuffers/buffer.h"
#include "core/async/executor.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/common/owned_ptr.h"
#include "core/split_engine/flatbuffer_size_calculator.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/transport/flatbuffer_builder_holder.h"
#include "core/split_engine/transport/message_group_id_generator.h"
#include "core/split_engine/transport/message_group_sender.h"
#include "core/split_engine/transport/message_sender.h"
#include "core/split_engine/transport/transport.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::split_engine {

namespace {
size_t GetBeginMessageSize() {
  static const size_t kBeginMessageSize = FlatbufferSizeCalculator()
                                              .AddBeginMessageGroup()
                                              .AddMessageGroup()
                                              .Finish()
                                              .AddScratchSpace()
                                              .ComputeSize();
  return kBeginMessageSize;
}

size_t GetEndMessageSize() {
  static const size_t kEndMessageSize = FlatbufferSizeCalculator()
                                            .AddEndMessageGroup()
                                            .AddMessageGroup()
                                            .Finish()
                                            .AddScratchSpace()
                                            .ComputeSize();
  return kEndMessageSize;
}

}  // namespace

DiscreteMessageGroupSender::DiscreteMessageGroupSender(
    OwnedOrBorrowedPtr<MessageSender> sender,
    MessageGroupIdGenerator& message_group_id_generator)
    : sender_(std::move(sender)),
      message_group_id_generator_(message_group_id_generator) {};

absl::StatusOr<MessageGroupId> DiscreteMessageGroupSender::Start(
    MessageGroupType message_group_type, size_t max_message_group_size_bytes) {
  

  MP_ASSIGN_OR_RETURN(
      const Transport::SessionID session_id,
      sender_->OpenSession(GetBeginMessageSize() +
                           max_message_group_size_bytes + GetEndMessageSize()));

  MP_ASSIGN_OR_RETURN(
      imp::OwnedPtr<FlatbufferBuilderHolder> fbb,
      sender_->CreateMessageBuilder(session_id, GetBeginMessageSize()));

  const MessageGroupId message_group_id =
      message_group_id_generator_.Generate(session_id);

  message_group_info_.emplace(message_group_id,
                              MessageGroupInfo{session_id, message_group_type});

  switch (message_group_type) {
    case MessageGroupType::kFrameUpdate:
      frame_updates_.insert(message_group_id);
      break;
    case MessageGroupType::kOneShot:
      one_shot_updates_.insert(message_group_id);
      break;
    default:
      IMP_LOG(imp::FATAL) << "Unknown message group type: "
                 << static_cast<int>(message_group_type);
      break;
  }

  (*fbb)->Finish(android_xr::schemas::CreateMessageGroupOperation(
      **fbb, message_group_id,
      android_xr::schemas::MessageGroupOperationTypes::BeginMessageGroup,
      android_xr::schemas::CreateBeginMessageGroup(**fbb).Union()));

  MP_RETURN_IF_ERROR(
      sender_->SendMessage(session_id, std::move(fbb),
                           [](absl::Span<const uint8_t> response_bytes) {}));

  return message_group_id;
}

absl::StatusOr<MessageGroupSender::OwnedOrBorrowedFlatbufferBuilderHolder>
DiscreteMessageGroupSender::CreateBuilder(MessageGroupId message_group_id,
                                          size_t initial_size_bytes) {
  

  auto group_info_it = message_group_info_.find(message_group_id);
  if (group_info_it == message_group_info_.end()) {
    return absl::FailedPreconditionError("Message group is not started.");
  }
  const Transport::SessionID session_id = group_info_it->second.session_id;

  MP_ASSIGN_OR_RETURN(
      imp::OwnedPtr<FlatbufferBuilderHolder> fbb,
      sender_->CreateMessageBuilder(session_id, initial_size_bytes));

  return OwnedOrBorrowedFlatbufferBuilderHolder(std::move(fbb));
}

absl::Status DiscreteMessageGroupSender::AddMessage(
    MessageGroupId message_group_id, OwnedOrBorrowedFlatbufferBuilderHolder fbb,
    const flatbuffers::Offset<android_xr::schemas::Command>& offset) {
  

  auto group_info_it = message_group_info_.find(message_group_id);
  if (group_info_it == message_group_info_.end()) {
    return absl::FailedPreconditionError("Message group is not started.");
  }
  const Transport::SessionID session_id = group_info_it->second.session_id;

  (*fbb)->Finish(offset);

  absl::StatusOr<imp::OwnedPtr<FlatbufferBuilderHolder>> owned_fbb =
      fbb.ExtractOwned(GetKey());
  

  return sender_->SendMessage(session_id, std::move(*owned_fbb),
                              [](absl::Span<const uint8_t> response_bytes) {});
}

absl::Status DiscreteMessageGroupSender::AddMessage(
    MessageGroupId message_group_id, OwnedOrBorrowedFlatbufferBuilderHolder fbb,
    OffsetProducer offset_fn) {
  

  auto group_info_it = message_group_info_.find(message_group_id);
  if (group_info_it == message_group_info_.end()) {
    return absl::FailedPreconditionError("Message group is not started.");
  }
  const Transport::SessionID session_id = group_info_it->second.session_id;

  absl::StatusOr<imp::OwnedPtr<FlatbufferBuilderHolder>> owned_fbb =
      fbb.ExtractOwned(GetKey());
  

  return sender_->SendMessage(
      session_id,
      [fbb = std::move(*owned_fbb),
       offset_fn = std::move(offset_fn)]() mutable {
        (*fbb)->Finish(offset_fn(**fbb));
        return std::move(fbb);
      },
      [](absl::Span<const uint8_t> response_bytes) {});
}

absl::Status DiscreteMessageGroupSender::Finish(
    MessageGroupId message_group_id,
    Transport::MessageCallback onMessageGroupCompleted) {
  

  auto group_info_it = message_group_info_.find(message_group_id);
  if (group_info_it == message_group_info_.end()) {
    return absl::FailedPreconditionError("Message group is not started.");
  }
  const Transport::SessionID session_id = group_info_it->second.session_id;
  const MessageGroupType message_group_type = group_info_it->second.type;
  const bool recycle_memory =
      message_group_type == MessageGroupType::kFrameUpdate;

  message_group_info_.erase(group_info_it);

  MP_ASSIGN_OR_RETURN(
      imp::OwnedPtr<FlatbufferBuilderHolder> fbb,
      sender_->CreateMessageBuilder(session_id, GetEndMessageSize()));

  (*fbb)->Finish(android_xr::schemas::CreateMessageGroupOperation(
      **fbb, message_group_id,
      android_xr::schemas::MessageGroupOperationTypes::EndMessageGroup,
      android_xr::schemas::CreateEndMessageGroup(**fbb).Union()));

  MP_RETURN_IF_ERROR(sender_->SendMessage(
      session_id, std::move(fbb),
      [this, recycle_memory, user_callback = std::move(onMessageGroupCompleted),
       session_id, message_group_id,
       message_group_type](absl::Span<const uint8_t> response_bytes) {
        

        

        user_callback(response_bytes);

        switch (message_group_type) {
          case MessageGroupType::kFrameUpdate:
            frame_updates_.erase(message_group_id);
            break;
          case MessageGroupType::kOneShot:
            one_shot_updates_.erase(message_group_id);
            break;
          default:
            IMP_LOG(imp::FATAL) << "Unknown message group type: "
                       << static_cast<int>(message_group_type);
            break;
        }
      }));

  return sender_->CloseSession(session_id);
}

absl::StatusOr<int32_t> DiscreteMessageGroupSender::GetMessageGroupCount(
    MessageGroupType message_group_type) {
  
  switch (message_group_type) {
    case MessageGroupType::kFrameUpdate:
      return frame_updates_.size();
    case MessageGroupType::kOneShot:
      return one_shot_updates_.size();
    default:
      return absl::FailedPreconditionError(
          absl::StrCat("Unknown message group type: ",
                       static_cast<int>(message_group_type)));
  }
}

}  // namespace imp::split_engine
