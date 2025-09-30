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
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/split_engine/android/bridge_buffer.h"
#include "core/split_engine/android/buffer_handle_factory.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_sender_base.h"
#include "core/split_engine/flatbuffer_arena_allocator.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

namespace {

class SplitEngineSharedMemoryBridgeBufferHandleFactory
    : public BufferHandleFactory {
 public:
  SplitEngineSharedMemoryBridgeBufferHandleFactory(
      SplitEngineSharedMemoryBridgeClient& client)
      : client_(client) {}

  std::unique_ptr<BufferHandle> Create(int fd,
                                       size_t buffer_size_bytes) override {
    absl::StatusOr<std::unique_ptr<BufferHandle>> handle =
        client_.RegisterBuffer(fd, buffer_size_bytes);
    if (!handle.ok()) {
      IMP_LOG(imp::FATAL) << "Failed to register bridge buffer: " << handle.status();
    }

    return *std::move(handle);
  }

 private:
  SplitEngineSharedMemoryBridgeClient& client_;
};

}  // namespace

SplitEngineSharedMemoryBridgeSender::SplitEngineSharedMemoryBridgeSender(
    SplitEngineSharedMemoryBridgeClient& bridge, bool recycle_buffers)
    : SplitEngineSharedMemoryBridgeSenderBase(recycle_buffers),
      bridge_(bridge),
      buffer_handle_factory_(
          std::make_unique<SplitEngineSharedMemoryBridgeBufferHandleFactory>(
              bridge)) {}

void SplitEngineSharedMemoryBridgeSender::SendMessage(
    const flatbuffers::FlatBufferBuilder& builder) {
  
  const int offset =
      builder.GetBufferPointer() - GetActiveBridgeBuffer().DataAs<uint8_t>();

  const absl::Status process_result =
      bridge_.ProcessRegion(GetActiveBridgeBuffer().GetHandle(), offset,
                            static_cast<int>(builder.GetSize()));
  if (!process_result.ok()) {
    IMP_LOG(imp::FATAL) << "Failed to queue buffer to rendering bridge: "
               << process_result.message();
  }
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

FlatbufferArenaAllocator& SplitEngineSharedMemoryBridgeSender::GetAllocator() {
  return arena_allocator_;
}

}  // namespace imp::split_engine
