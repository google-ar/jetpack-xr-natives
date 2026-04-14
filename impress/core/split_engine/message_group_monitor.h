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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MESSAGE_GROUP_MONITOR_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MESSAGE_GROUP_MONITOR_H_

#include <functional>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

// A monitor for tracking active message groups across the bridge.
// All methods are thread-safe.
class MessageGroupMonitor {
 public:
  // Creates a new, empty set of active message groups for the client.
  static absl::Status ConnectClient(ClientId client_id);

  // Removes the active message groups for the client.
  static absl::Status DisconnectClient(ClientId client_id);

  // Marks the given message group as processing for the given client.
  // This message group will not be released until the client signals via
  // ReleaseMessageGroup.
  static absl::Status EnqueueMessageGroup(ClientId client_id,
                                          MessageGroupId message_group_id);

  // Releases the given message group for the given client, meaning it can be
  // reclaimed for reuse by the client.
  static absl::Status ReleaseMessageGroup(ClientId client_id,
                                          MessageGroupId message_group_id);

  // Executes the given function with the set of active message groups for the
  // given client. Can return an error if the client is not found.
  static absl::Status WithActiveMessageGroups(
      ClientId client_id,
      std::function<void(const absl::flat_hash_set<MessageGroupId>&)> fn);
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MESSAGE_GROUP_MONITOR_H_
