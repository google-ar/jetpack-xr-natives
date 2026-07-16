// Copyright 2024 Google LLC
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

#include "core/split_engine/android/split_engine_shared_memory_bridge_sender.h"

#include <cassert>
#include <cstdint>
#include <memory>
#include <utility>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/executor.h"
#include "core/common/owned_ptr.h"
#include "core/split_engine/android/bridge_buffer.h"
#include "core/split_engine/android/buffer_handle_factory.h"
#include "core/split_engine/android/buffer_handle_factory_shmem.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"
#include "core/split_engine/flatbuffer_arena_allocator.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

SplitEngineSharedMemoryBridgeSender::SplitEngineSharedMemoryBridgeSender(
    SplitEngineSharedMemoryBridgeClient& bridge)
    : bridge_(bridge),
      buffer_handle_factory_(
          std::make_unique<SharedMemoryBufferHandleFactory>(bridge)) {}

absl::Status SplitEngineSharedMemoryBridgeSender::SendMessage(
    MessageGroupId group_id,
    imp::OwnedPtr<flatbuffers::FlatBufferBuilder> builder) {
  

  Schedule(
      [this, group_id, builder = std::move(builder)]() mutable -> absl::Status {
        const BridgeBuffer& bridge_buffer = GetBridgeBuffer(group_id);
        if (!bridge_buffer.IsValidBlock(builder->GetBufferPointer(),
                                        builder->GetSize())) {
          IMP_LOG(imp::FATAL) << "Message is not in the bridge buffer.";
        }

        const int offset =
            builder->GetBufferPointer() - bridge_buffer.DataAs<uint8_t>();

        const absl::Status status =
            bridge_.ProcessRegion(bridge_buffer.GetHandle(), offset,
                                  static_cast<int>(builder->GetSize()));
        if (!status.ok()) {
          IMP_LOG(imp::FATAL) << "Failed to process region: " << status;
        }

        return status;
      });

  return absl::OkStatus();
}

MessageGroupId SplitEngineSharedMemoryBridgeSender::GenerateMessageGroupId() {
  return bridge_.GenerateMessageGroupId();
}
ClientId SplitEngineSharedMemoryBridgeSender::GetClientId() const {
  return bridge_.GetClientId();
}
BufferHandleFactory&
SplitEngineSharedMemoryBridgeSender::GetBufferHandleFactory() {
  return *buffer_handle_factory_;
}

ArenaAllocator& SplitEngineSharedMemoryBridgeSender::GetArenaAllocator() {
  return arena_allocator_;
}

SplitEngineSharedMemoryBridgeSender::~SplitEngineSharedMemoryBridgeSender() {
  // Need to make sure that all pending tasks that may be referring to members
  // of this class are executed before destruction.
  DrainScheduler();
}

}  // namespace imp::split_engine
