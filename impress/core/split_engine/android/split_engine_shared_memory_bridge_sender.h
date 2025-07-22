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

#include <android/binder_auto_utils.h>
#include <android/binder_ibinder.h>
#include <sys/mman.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/common/robin_map.h"
#include "core/split_engine/android/bridge_buffer.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"
#include "core/split_engine/flatbuffer_arena_allocator.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_bridge_sender.h"

namespace imp::split_engine {

/**
 * An implementation of the SplitEngineBridgeSender which communicates using
 * shared memory, via the ISplitEngineSharedMemoryBridge AIDL interface.
 **/
class SplitEngineSharedMemoryBridgeSender
    : public imp::split_engine::SplitEngineBridgeSender {
 public:
  SplitEngineSharedMemoryBridgeSender(
      SplitEngineSharedMemoryBridgeClient& bridge, bool recycle_buffers);

  SplitEngineSharedMemoryBridgeSender(
      const SplitEngineSharedMemoryBridgeSender&) = delete;
  SplitEngineSharedMemoryBridgeSender(SplitEngineSharedMemoryBridgeSender&&) =
      delete;

  SplitEngineSharedMemoryBridgeSender& operator=(
      const SplitEngineSharedMemoryBridgeSender&) = delete;
  SplitEngineSharedMemoryBridgeSender& operator=(
      SplitEngineSharedMemoryBridgeSender&&) = delete;

  virtual void SendMessage(const flatbuffers::FlatBufferBuilder& builder);

  void BeginMessageGroup(size_t size_bytes) override;
  void EndMessageGroup() override;

  bool IsMessageGroupActive() const override;

  std::unique_ptr<flatbuffers::FlatBufferBuilder> CreateFlatBufferBuilder(
      size_t size_bytes) override;

  void ClearReleasedMessageGroups() override;

  void* CreateSharedMemoryBuffer(size_t size_in_bytes);
  void DestroySharedMemoryBuffer(void*);

 private:
  SplitEngineSharedMemoryBridgeClient& bridge_;
  const ndk::SpAIBinder bridge_handle_;
  bool recycle_buffers_;
  std::optional<MessageGroupId> active_message_group_id_;
  imp::RobinMap<void*, std::unique_ptr<BridgeBuffer>> bridge_buffers_;
  BridgeBuffer* active_bridge_buffer_;
  FlatbufferArenaAllocator arena_allocator_;
  absl::flat_hash_map<MessageGroupId, FlatbufferArenaAllocator::ArenaHandle>
      arena_handles_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_SHARED_MEMORY_BRIDGE_SENDER_H_
