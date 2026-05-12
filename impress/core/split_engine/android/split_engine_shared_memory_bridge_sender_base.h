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

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "flatbuffers/allocator.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/future.h"
#include "core/common/invocable.h"
#include "core/common/owned_ptr.h"
#include "core/common/robin_map.h"
#include "core/split_engine/android/bridge_buffer.h"
#include "core/split_engine/android/buffer_handle_factory.h"
#include "core/split_engine/flatbuffer_arena_allocator.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_bridge_sender.h"

namespace imp::split_engine {

// Has no dependency on the actual implementation of the underlying transport.
// TODO: (broken link) - remove this class.
class SplitEngineSharedMemoryBridgeSenderBase
    : public imp::split_engine::SplitEngineBridgeSender {
 public:
  using MessageType = SplitEngineBridgeSender::MessageType;

  SplitEngineSharedMemoryBridgeSenderBase();
  ~SplitEngineSharedMemoryBridgeSenderBase() override;

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

  absl::StatusOr<MessageGroupId> BeginMessageGroup(
      size_t size_bytes, MessageType message_type) override;
  absl::Status EndMessageGroup(MessageGroupId group_id) override;

  imp::OwnedPtr<flatbuffers::FlatBufferBuilder> CreateFlatBufferBuilder(
      MessageGroupId group_id, size_t size_bytes) override;

  absl::StatusOr<size_t> GetActiveMessageGroupCount() const override;

  void ClearReleasedMessageGroups() override;

  void Schedule(imp::Invocable<absl::Status()> fn) override;

 protected:
  virtual MessageGroupId GenerateMessageGroupId() = 0;
  virtual ClientId GetClientId() const = 0;
  virtual BufferHandleFactory& GetBufferHandleFactory() = 0;
  virtual ArenaAllocator& GetArenaAllocator() = 0;
  flatbuffers::Allocator& GetFlatbuffersAllocator(MessageGroupId group_id);

  const BridgeBuffer& GetBridgeBuffer(MessageGroupId group_id);
  absl::StatusOr<size_t> GetMessageGroupSizeBytes(
      MessageGroupId group_id) const;

  // Drains the scheduler and sets the given status as the result of the last
  // Future.
  //
  // The method is blocking.
  void DrainScheduler(absl::Status status = absl::OkStatus());

 private:
  absl::Mutex bridge_buffers_mutex_;
  imp::RobinMap<const void*, std::unique_ptr<BridgeBuffer>> bridge_buffers_
      ABSL_GUARDED_BY(bridge_buffers_mutex_);

  absl::Mutex arena_handles_mutex_;
  absl::flat_hash_map<MessageGroupId, ArenaAllocator::ArenaHandle>
      arena_handles_ ABSL_GUARDED_BY(arena_handles_mutex_);
  absl::flat_hash_map<MessageGroupId, MessageType> message_group_types_
      ABSL_GUARDED_BY(arena_handles_mutex_);

  mutable absl::Mutex message_sizes_mutex_;
  absl::flat_hash_map<MessageGroupId, size_t> message_group_id_to_size_bytes_
      ABSL_GUARDED_BY(message_sizes_mutex_);

  // Represents a chain of operations that are scheduled to be executed on
  // the background thread.
  //
  // Accessed via `Schedule` method only and only on the foreground thread.
  Future<absl::Status> pending_ops_ = Future<absl::Status>(absl::OkStatus());

  // This is used to determine when it's time to use kScheduleAlways mode to
  // prevent infinite callstacks. See `Schedule` method for more details.
  size_t total_ops_count_ = 0;

  // Obtained results from `future_benchmark_test.cc` shows that 64 is sweet
  // spot both for XR device and Desktop.
  static constexpr size_t kScheduleAlwaysEveryN = 64;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_SHARED_MEMORY_BRIDGE_SENDER_BASE_H_
