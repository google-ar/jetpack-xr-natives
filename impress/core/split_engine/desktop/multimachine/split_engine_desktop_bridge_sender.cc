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

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "flatbuffers/flatbuffer_builder.h"
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
    MessageGroupId group_id, const flatbuffers::FlatBufferBuilder& message) {
  const BridgeBuffer& bridge_buffer = GetBridgeBuffer(group_id);
  if (!bridge_buffer.IsValidBlock(message.GetBufferPointer(),
                                  message.GetSize())) {
    return absl::InternalError("Message is not in the active bridge buffer.");
  }

  MP_ASSIGN_OR_RETURN(const size_t message_group_size_bytes,
                   GetMessageGroupSizeBytes(group_id));

  return client_.SendMessage(
      group_id, message_group_size_bytes,
      arena_allocator_.PrependSize(message.GetBufferPointer(),
                                   message.GetSize()));
}

absl::Status SplitEngineMMDesktopBridgeSender::EndMessageGroup(
    MessageGroupId group_id) {
  MP_RETURN_IF_ERROR(
      SplitEngineSharedMemoryBridgeSenderBase::EndMessageGroup(group_id));

  return client_.EndMessageGroup(group_id);
}

}  // namespace imp::split_engine
