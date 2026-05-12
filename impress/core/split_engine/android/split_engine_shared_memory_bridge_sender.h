/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_SHARED_MEMORY_BRIDGE_SENDER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_SHARED_MEMORY_BRIDGE_SENDER_H_

#include <memory>

#include "absl/status/status.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/common/owned_ptr.h"
#include "core/split_engine/android/buffer_handle_factory.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_sender_base.h"
#include "core/split_engine/flatbuffer_arena_allocator.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

/**
 * An implementation of the SplitEngineBridgeSender which communicates using
 * shared memory, via the ISplitEngineSharedMemoryBridge AIDL interface.
 **/
class SplitEngineSharedMemoryBridgeSender
    : public SplitEngineSharedMemoryBridgeSenderBase {
 public:
  SplitEngineSharedMemoryBridgeSender(
      SplitEngineSharedMemoryBridgeClient& bridge);

  ~SplitEngineSharedMemoryBridgeSender() override;

  absl::Status SendMessage(
      MessageGroupId group_id,
      imp::OwnedPtr<flatbuffers::FlatBufferBuilder> builder) override;

 protected:
  MessageGroupId GenerateMessageGroupId() override;
  ClientId GetClientId() const override;
  BufferHandleFactory& GetBufferHandleFactory() override;
  ArenaAllocator& GetArenaAllocator() override;

 private:
  SplitEngineSharedMemoryBridgeClient& bridge_;
  std::unique_ptr<BufferHandleFactory> buffer_handle_factory_;
  ArenaAllocator arena_allocator_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_SHARED_MEMORY_BRIDGE_SENDER_H_
