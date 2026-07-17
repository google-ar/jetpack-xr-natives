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

#include "core/split_engine/transport/message_group_sender_discrete_legacy.h"

#include <cstdint>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "core/async/executor.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/split_engine/message_group_monitor.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/transport/message_group_id_generator.h"
#include "core/split_engine/transport/message_group_sender_discrete.h"
#include "core/split_engine/transport/message_sender.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::split_engine {
DiscreteMessageGroupSenderLegacy::DiscreteMessageGroupSenderLegacy(
    OwnedOrBorrowedPtr<MessageSender> sender,
    MessageGroupIdGenerator& message_group_id_generator, ClientId client_id)
    : DiscreteMessageGroupSender(std::move(sender), message_group_id_generator),
      client_id_(client_id) {}

absl::StatusOr<int32_t> DiscreteMessageGroupSenderLegacy::GetMessageGroupCount(
    MessageGroupType message_group_type) {
  
  if (message_group_type != MessageGroupType::kFrameUpdate &&
      message_group_type != MessageGroupType::kOneShot) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Unknown message group type: ", static_cast<int>(message_group_type)));
  }

  const absl::flat_hash_set<MessageGroupId>& message_groups =
      message_group_type == MessageGroupType::kFrameUpdate
          ? GetFrameUpdates()
          : GetOneShotUpdates();

  int32_t message_group_count = 0;

  MP_RETURN_IF_ERROR(MessageGroupMonitor::WithActiveMessageGroups(
      client_id_,
      [&message_groups, &message_group_count](
          const absl::flat_hash_set<MessageGroupId>& active_message_groups) {
        for (const MessageGroupId message_group_id : message_groups) {
          if (active_message_groups.contains(message_group_id)) {
            message_group_count++;
          }
        }
      }));

  return message_group_count;
}

}  // namespace imp::split_engine
