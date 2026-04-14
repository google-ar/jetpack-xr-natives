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

#include "core/split_engine/message_group_monitor.h"

#include <functional>

#include "absl/base/no_destructor.h"
#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

namespace {

// This is used to track the active message groups for each client. It is
// thread-safe and static to ensure it is not tied to any Impress objects such
// as View or Registry since callbacks may happen on Binder threads in a race
// condition with the Impress app going away on bridge shutdown.
struct MessageGroupTrackingData {
  absl::Mutex mutex;
  absl::flat_hash_map<ClientId, absl::flat_hash_set<MessageGroupId>> active_ids
      ABSL_GUARDED_BY(mutex);
};

MessageGroupTrackingData& GetActiveMessageGroups() {
  static absl::NoDestructor<MessageGroupTrackingData> active_message_groups;
  return *active_message_groups;
}

}  // namespace

absl::Status MessageGroupMonitor::ConnectClient(ClientId client_id) {
  MessageGroupTrackingData& message_group_tracking_data =
      GetActiveMessageGroups();
  absl::MutexLock lock(message_group_tracking_data.mutex);
  if (message_group_tracking_data.active_ids.contains(client_id)) {
    return absl::AlreadyExistsError("Client already exists");
  }
  message_group_tracking_data.active_ids.emplace(
      client_id, absl::flat_hash_set<MessageGroupId>());
  return absl::OkStatus();
}

absl::Status MessageGroupMonitor::DisconnectClient(ClientId client_id) {
  MessageGroupTrackingData& message_group_tracking_data =
      GetActiveMessageGroups();
  absl::MutexLock lock(message_group_tracking_data.mutex);
  if (!message_group_tracking_data.active_ids.contains(client_id)) {
    return absl::NotFoundError("Client not found");
  }
  message_group_tracking_data.active_ids.erase(client_id);
  return absl::OkStatus();
}

absl::Status MessageGroupMonitor::EnqueueMessageGroup(
    ClientId client_id, MessageGroupId message_group_id) {
  MessageGroupTrackingData& message_group_tracking_data =
      GetActiveMessageGroups();
  absl::MutexLock lock(message_group_tracking_data.mutex);
  auto it = message_group_tracking_data.active_ids.find(client_id);
  if (it == message_group_tracking_data.active_ids.end()) {
    return absl::NotFoundError("Client not found");
  }
  // Ensure the message group is not already in the set.
  if (!it->second.insert(message_group_id).second) {
    return absl::AlreadyExistsError("Message group already exists");
  }
  return absl::OkStatus();
}

absl::Status MessageGroupMonitor::ReleaseMessageGroup(
    ClientId client_id, MessageGroupId message_group_id) {
  MessageGroupTrackingData& message_group_tracking_data =
      GetActiveMessageGroups();
  absl::MutexLock lock(message_group_tracking_data.mutex);
  auto it = message_group_tracking_data.active_ids.find(client_id);
  if (it == message_group_tracking_data.active_ids.end()) {
    return absl::NotFoundError("Client not found");
  }
  if (!it->second.contains(message_group_id)) {
    return absl::NotFoundError("Message group not found");
  }
  it->second.erase(message_group_id);
  return absl::OkStatus();
}

absl::Status MessageGroupMonitor::WithActiveMessageGroups(
    ClientId client_id,
    std::function<void(const absl::flat_hash_set<MessageGroupId>&)> fn) {
  if (!fn) {
    return absl::InvalidArgumentError("Callback not provided");
  }

  MessageGroupTrackingData& message_group_tracking_data =
      GetActiveMessageGroups();
  absl::MutexLock lock(message_group_tracking_data.mutex);
  auto it = message_group_tracking_data.active_ids.find(client_id);
  if (it == message_group_tracking_data.active_ids.end()) {
    return absl::NotFoundError("Client not found");
  }
  // Call the callback with the set of active message groups.
  fn(it->second);
  return absl::OkStatus();
}

}  // namespace imp::split_engine
