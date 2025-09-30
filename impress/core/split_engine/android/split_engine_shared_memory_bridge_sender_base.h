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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_SHARED_MEMORY_BRIDGE_SENDER_BASE_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_SHARED_MEMORY_BRIDGE_SENDER_BASE_H_

#include <cstddef>
#include <memory>
#include <optional>

#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/common/robin_map.h"
#include "core/split_engine/android/bridge_buffer.h"
#include "core/split_engine/android/buffer_handle_factory.h"
#include "core/split_engine/flatbuffer_arena_allocator.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_bridge_sender.h"

namespace imp::split_engine {

// Has no dependency on the actual implementation of the underlying transport.
// TODO: (broken link) - extract memory management and rename the class to
// "BasicSplitEngineBridgeSender"
class SplitEngineSharedMemoryBridgeSenderBase
    : public imp::split_engine::SplitEngineBridgeSender {
 public:
  SplitEngineSharedMemoryBridgeSenderBase(bool recycle_buffers);

  SplitEngineSharedMemoryBridgeSenderBase(
      const SplitEngineSharedMemoryBridgeSenderBase&) = delete;
  SplitEngineSharedMemoryBridgeSenderBase(
      SplitEngineSharedMemoryBridgeSenderBase&&) = delete;

  SplitEngineSharedMemoryBridgeSenderBase& operator=(
      const SplitEngineSharedMemoryBridgeSenderBase&) = delete;
  SplitEngineSharedMemoryBridgeSenderBase& operator=(
      SplitEngineSharedMemoryBridgeSenderBase&&) = delete;

  void* CreateSharedMemoryBuffer(size_t size_in_bytes);
  void DestroySharedMemoryBuffer(void*);

  void BeginMessageGroup(size_t size_bytes) override;
  void EndMessageGroup() override;

  bool IsMessageGroupActive() const override;

  std::unique_ptr<flatbuffers::FlatBufferBuilder> CreateFlatBufferBuilder(
      size_t size_bytes) override;

  absl::StatusOr<size_t> GetActiveMessageGroupCount() const override;

  void ClearReleasedMessageGroups() override;

 protected:
  virtual MessageGroupId GenerateMessageGroupId() = 0;
  virtual ClientId GetClientId() const = 0;
  virtual BufferHandleFactory& GetBufferHandleFactory() = 0;
  virtual FlatbufferArenaAllocator& GetAllocator() = 0;

  const BridgeBuffer& GetActiveBridgeBuffer() const;
  std::optional<MessageGroupId> GetActiveMessageGroupId() const;
  std::optional<size_t> GetActiveMessageGroupSizeBytes() const;

 private:
  const bool recycle_buffers_;
  std::optional<MessageGroupId> active_message_group_id_ = std::nullopt;
  std::optional<size_t> active_message_group_size_bytes_ = std::nullopt;
  imp::RobinMap<void*, std::unique_ptr<BridgeBuffer>> bridge_buffers_;
  BridgeBuffer* active_bridge_buffer_;
  absl::flat_hash_map<MessageGroupId, FlatbufferArenaAllocator::ArenaHandle>
      arena_handles_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_SHARED_MEMORY_BRIDGE_SENDER_BASE_H_
