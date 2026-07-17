/*
 * Copyright 2026 Google LLC
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

#include "core/split_engine/transport/basic_transport_shmem_hooks.h"

#include <cstdint>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/span.h"
#include "third_party/gloop/util/status/status_macros.h"
#include "core/async/executor.h"
#include "core/split_engine/message_group_monitor.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/transport/transport.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"

namespace imp::split_engine {

BasicSharedMemoryTransportHooks::BasicSharedMemoryTransportHooks(
    BaseView& view, ClientId client_id)
    : Updater(view), client_id_(client_id) {}

MessageGroupId BasicSharedMemoryTransportHooks::Generate(
    Transport::SessionID session_id) {
  

  MessageGroupId result;
  {
    absl::MutexLock lock(message_group_id_mutex_);
    result = next_message_group_id_++;

    // Cache message group information.
    auto [_, inserted] =
        session_id_to_message_group_id_.emplace(session_id, result);
    

    message_group_data_.emplace(result, MessageGroupData{session_id, {}});
  }

  // We cannot determine on our own yet if message group was processed by the
  // remote side, so we'll use MessageGroupMonitor for that.
  //
  // The Monitor maintains the proper state: it will know when the group was
  // processed by the remote side. We'll sync up with it during the next
  // `Update` call.
  //
  // Let the Monitor know that new message group was created.
  const absl::Status status =
      MessageGroupMonitor::EnqueueMessageGroup(client_id_, result);
  if (!status.ok()) {
    IMP_LOG(imp::FATAL) << "Failed to enqueue message group: " << status;
  }

  return result;
}

absl::StatusOr<Transport::MessageCallback>
BasicSharedMemoryTransportHooks::OnPreSendMessage(
    Transport& transport, SessionID session_id, absl::Span<const uint8_t> data,
    Transport::MessageCallback callback) {
  MP_ASSIGN_OR_RETURN(const MessageGroupId message_group_id,
                   GetMessageGroupId(session_id));

  absl::MutexLock lock(message_group_id_mutex_);
  auto it = message_group_data_.find(message_group_id);
  if (it == message_group_data_.end()) {
    return absl::NotFoundError("Message group not found");
  }
  it->second.callbacks.emplace_back(std::move(callback));

  return [](absl::Span<const uint8_t> response_bytes) {};
}

void BasicSharedMemoryTransportHooks::Update(const FrameTime& frame_time) {
  // MessageGroupMonitor maintains the list of messages that were not
  // processed by the remote side yet. The following code will remove all
  // message groups that were processed by the remote side from the internal
  // cache.
  //
  // Important note: `Update` will not be called if the View will decide to
  // skip the frame.
  const absl::Status status = MessageGroupMonitor::WithActiveMessageGroups(
      client_id_,
      [this](const absl::flat_hash_set<MessageGroupId>& active_message_groups) {
        // We should iterate over all message groups known to us and check if
        // they are in the `active_message_groups`:
        //  - if not, it means that the group was processed by the remote
        //    side and we should remove it from the internal cache.
        //  - if yes, it means that the group is still being processed by the
        //    remote side, no action required.
        absl::MutexLock lock(message_group_id_mutex_);
        for (auto it = message_group_data_.begin(),
                  end = message_group_data_.end();
             it != end;) {
          auto it_copy = it++;
          const MessageGroupId message_group_id = it_copy->first;
          if (active_message_groups.contains(message_group_id)) {
            // Do nothing: the message group is still active.
            continue;
          }

          // The message group was processed by the remote side.
          const SessionID session_id = it_copy->second.session_id;
          const MessageGroupData& message_group_data = it_copy->second;

          // Execute all callbacks.
          const absl::Span<const uint8_t> callback_data = absl::MakeConstSpan(
              reinterpret_cast<const uint8_t*>(&message_group_id),
              sizeof(MessageGroupId));
          for (const auto& callback : message_group_data.callbacks) {
            callback(callback_data);
          }

          // Remove the message group from the internal cache.
          message_group_data_.erase(it_copy);
          session_id_to_message_group_id_.erase(session_id);
        }
      });

  
}

absl::StatusOr<MessageGroupId>
BasicSharedMemoryTransportHooks::GetMessageGroupId(SessionID session_id) {
  absl::MutexLock lock(message_group_id_mutex_);
  auto it = session_id_to_message_group_id_.find(session_id);
  if (it != session_id_to_message_group_id_.end()) {
    return it->second;
  } else {
    return absl::NotFoundError("SessionID not found");
  }
}

}  // namespace imp::split_engine
