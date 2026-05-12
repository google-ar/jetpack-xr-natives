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

#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_sender.h"

#include <cstddef>
#include <utility>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/executor.h"
#include "core/common/owned_ptr.h"
#include "core/split_engine/android/bridge_buffer.h"
#include "core/split_engine/android/buffer_handle_factory.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_sender_base.h"
#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_client.h"
#include "core/split_engine/flatbuffer_arena_allocator.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::split_engine {

SplitEngineMMDesktopBridgeSender::SplitEngineMMDesktopBridgeSender(
    SplitEngineMMDesktopBridgeClient& client)
    : client_(client) {}

MessageGroupId SplitEngineMMDesktopBridgeSender::GenerateMessageGroupId() {
  return client_.GenerateMessageGroupId();
}

ClientId SplitEngineMMDesktopBridgeSender::GetClientId() const {
  return client_.GetClientId();
}

ArenaAllocator& SplitEngineMMDesktopBridgeSender::GetArenaAllocator() {
  return arena_allocator_;
}

BufferHandleFactory&
SplitEngineMMDesktopBridgeSender::GetBufferHandleFactory() {
  return buffer_handle_factory_;
}

absl::Status SplitEngineMMDesktopBridgeSender::SendMessage(
    MessageGroupId group_id,
    imp::OwnedPtr<flatbuffers::FlatBufferBuilder> message) {
  

  Schedule([this, group_id, message = std::move(message)]() mutable {
    const BridgeBuffer& bridge_buffer = GetBridgeBuffer(group_id);
    const absl::StatusOr<size_t> message_group_size_bytes =
        GetMessageGroupSizeBytes(group_id);
    if (!message_group_size_bytes.ok()) {
      IMP_LOG(imp::FATAL) << "Failed to get message group size bytes: "
                 << message_group_size_bytes.status();
    }

    if (!bridge_buffer.IsValidBlock(message->GetBufferPointer(),
                                    message->GetSize())) {
      IMP_LOG(imp::FATAL) << "Message is not in the active bridge buffer.";
    }

    const absl::Status status = client_.SendMessage(
        group_id, *message_group_size_bytes,
        arena_allocator_.PrependSize(message->GetBufferPointer(),
                                     message->GetSize()));

    if (!status.ok()) {
      IMP_LOG(imp::FATAL) << "Failed to send message: " << status;
    }

    return status;
  });

  return absl::OkStatus();
}

absl::Status SplitEngineMMDesktopBridgeSender::EndMessageGroup(
    MessageGroupId group_id) {
  MP_RETURN_IF_ERROR(
      SplitEngineSharedMemoryBridgeSenderBase::EndMessageGroup(group_id));

  Schedule([this, group_id]() {
    const absl::Status status = client_.EndMessageGroup(group_id);
    if (!status.ok()) {
      IMP_LOG(imp::FATAL) << "Failed to end message group: " << status;
    }
    return status;
  });

  return absl::OkStatus();
}

SplitEngineMMDesktopBridgeSender::~SplitEngineMMDesktopBridgeSender() {
  DrainScheduler();
}

}  // namespace imp::split_engine
