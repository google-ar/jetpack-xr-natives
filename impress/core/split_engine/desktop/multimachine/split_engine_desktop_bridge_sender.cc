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

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/split_engine/android/buffer_handle_factory.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_sender_base.h"
#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_client.h"
#include "core/split_engine/flatbuffer_arena_allocator.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

SplitEngineMMDesktopBridgeSender::SplitEngineMMDesktopBridgeSender(
    SplitEngineMMDesktopBridgeClient& client, bool recycle_buffers)
    : SplitEngineSharedMemoryBridgeSenderBase(recycle_buffers),
      client_(client) {}

MessageGroupId SplitEngineMMDesktopBridgeSender::GenerateMessageGroupId() {
  return client_.GenerateMessageGroupId();
}

ClientId SplitEngineMMDesktopBridgeSender::GetClientId() const {
  return client_.GetClientId();
}

FlatbufferArenaAllocator& SplitEngineMMDesktopBridgeSender::GetAllocator() {
  return arena_allocator_;
}

BufferHandleFactory&
SplitEngineMMDesktopBridgeSender::GetBufferHandleFactory() {
  return buffer_handle_factory_;
}

void SplitEngineMMDesktopBridgeSender::SendMessage(
    const flatbuffers::FlatBufferBuilder& message) {
  
  
  

  if (const absl::Status status = client_.SendMessage(
          *GetActiveMessageGroupId(), *GetActiveMessageGroupSizeBytes(),
          arena_allocator_.PrependSize(message.GetBufferPointer(),
                                       message.GetSize()));
      !status.ok()) {
    IMP_LOG(imp::FATAL) << "Failed to send message: " << status.ToString();
  }
}

void SplitEngineMMDesktopBridgeSender::EndMessageGroup() {
  
  const MessageGroupId message_group_id = *GetActiveMessageGroupId();
  SplitEngineSharedMemoryBridgeSenderBase::EndMessageGroup();
  
}

}  // namespace imp::split_engine
