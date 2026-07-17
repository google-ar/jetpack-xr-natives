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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_BASIC_TRANSPORT_SHMEM_HOOKS_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_BASIC_TRANSPORT_SHMEM_HOOKS_H_

#include <cstdint>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/span.h"
#include "core/ncsb/update_phase.h"
#include "core/ncsb/update_system.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/transport/message_group_id_generator.h"
#include "core/split_engine/transport/transport.h"
#include "core/split_engine/transport/transport_with_hooks.h"
#include "core/view/utils/frame_time.h"

namespace imp::split_engine {

// Hooks to adapt transport to the legacy AIDL interface that uses callback-less
// `ProcessRegion` API.
class BasicSharedMemoryTransportHooks
    : public TransportWithHooks::TransportHooks,
      public UpdateSystem::Updater<BasicSharedMemoryTransportHooks>,
      public MessageGroupIdGenerator {
 public:
  using SessionID = Transport::SessionID;

  BasicSharedMemoryTransportHooks(BaseView& view, ClientId client_id);

  // MessageGroupIdGenerator implementation lives here because `Update` needs
  // knowledge about message groups
  MessageGroupId Generate(Transport::SessionID session_id) override;

  // Saves the callback to be called when the message group is completed by the
  // remote side.
  absl::StatusOr<Transport::MessageCallback> OnPreSendMessage(
      Transport& transport, SessionID session_id,
      absl::Span<const uint8_t> data,
      Transport::MessageCallback callback) override;

  // UpdateSystem::Updater<BasicTransport> setup.
  //
  // BasicTransport shall be called before the SplitEngineSerializerImpl.
  // kPostDefault is executed right before kEnd.
  // BasicTransport shall subscribe to kPostDefault to do not introduce
  // dependencies on SplitEngineSerializerImpl.
  static constexpr UpdatePhase kUpdatePhase = UpdatePhase::kPostDefault;

  // Relies on MessageGroupMonitor to figure out released message groups and
  // process all stored callbacks.
  void Update(const FrameTime& frame_time) override;

 private:
  const ClientId client_id_;

  absl::Mutex message_group_id_mutex_;
  MessageGroupId next_message_group_id_
      ABSL_GUARDED_BY(message_group_id_mutex_) = 0;
  struct MessageGroupData {
    const SessionID session_id;
    std::vector<Transport::MessageCallback> callbacks;
  };

  absl::flat_hash_map<SessionID, MessageGroupId> session_id_to_message_group_id_
      ABSL_GUARDED_BY(message_group_id_mutex_);
  absl::flat_hash_map<MessageGroupId, MessageGroupData> message_group_data_
      ABSL_GUARDED_BY(message_group_id_mutex_);

  absl::StatusOr<MessageGroupId> GetMessageGroupId(SessionID session_id);
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_BASIC_TRANSPORT_SHMEM_HOOKS_H_
