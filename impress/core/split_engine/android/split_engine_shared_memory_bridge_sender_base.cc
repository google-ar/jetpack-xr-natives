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

#include "core/split_engine/android/split_engine_shared_memory_bridge_sender_base.h"

#include <cassert>
#include <cstddef>
#include <cstring>
#include <memory>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "absl/synchronization/notification.h"
#include "flatbuffers/allocator.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/common/invocable.h"
#include "core/common/owned_ptr.h"
#include "core/split_engine/android/bridge_buffer.h"
#include "core/split_engine/flatbuffer_arena_allocator.h"
#include "core/split_engine/flatbuffer_size_calculator.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_bridge_sender.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::split_engine {
namespace {

void* AllocateSharedMemoryBuffer(size_t size_in_bytes, void* user) {
  return reinterpret_cast<SplitEngineSharedMemoryBridgeSenderBase*>(user)
      ->CreateSharedMemoryBuffer(size_in_bytes);
}

void DeallocateSharedMemoryBuffer(void* ptr, void* user) {
  reinterpret_cast<SplitEngineSharedMemoryBridgeSenderBase*>(user)
      ->DestroySharedMemoryBuffer(ptr);
}

size_t GetBeginMessageSize() {
  static const size_t kBeginMessageSize = FlatbufferSizeCalculator()
                                              .AddBeginMessageGroup()
                                              .AddMessageGroup()
                                              .Finish()
                                              .AddScratchSpace()
                                              .ComputeSize();
  return kBeginMessageSize;
}

size_t GetEndMessageSize() {
  static const size_t kEndMessageSize = FlatbufferSizeCalculator()
                                            .AddEndMessageGroup()
                                            .AddMessageGroup()
                                            .Finish()
                                            .AddScratchSpace()
                                            .ComputeSize();
  return kEndMessageSize;
}

}  // namespace

SplitEngineSharedMemoryBridgeSenderBase::
    SplitEngineSharedMemoryBridgeSenderBase() {}

void* SplitEngineSharedMemoryBridgeSenderBase::CreateSharedMemoryBuffer(
    size_t size_in_bytes) {
  auto bridge_buffer = std::make_unique<BridgeBuffer>(GetBufferHandleFactory(),
                                                      size_in_bytes, *this);
  void* buffer_head = bridge_buffer->Data();
  {
    absl::MutexLock lock(bridge_buffers_mutex_);
    bridge_buffers_.emplace(buffer_head, std::move(bridge_buffer));
  }
  return buffer_head;
}

void SplitEngineSharedMemoryBridgeSenderBase::DestroySharedMemoryBuffer(
    void* head) {
  // Note that the map that we're erasing from holds unique_ptrs, so this
  // erase() doesn't just remove it from the map but also destroys the
  // BridgeBuffer object.
  absl::MutexLock lock(bridge_buffers_mutex_);
  bridge_buffers_.erase(head);
}

absl::StatusOr<MessageGroupId>
SplitEngineSharedMemoryBridgeSenderBase::BeginMessageGroup(
    size_t size_bytes, MessageType message_type) {
  // Step 1: Create a memory arena for the new group.
  const ArenaAllocator::ArenaHandle arena_handle =
      GetArenaAllocator().CreateArena(
          GetBeginMessageSize() + size_bytes + GetEndMessageSize(),
          {AllocateSharedMemoryBuffer, DeallocateSharedMemoryBuffer, this});

  const MessageGroupId group_id = GenerateMessageGroupId();

  // TODO: (broken link) - Logic of ClearReleasedMessageGroups assumes that if
  // message group is part of arena_handles_ map but not part of active message
  // groups (the one updated by EnqueueMessageGroup), then it is released.
  //
  // From thread safety perspective, it is better to enqueue the message group
  // first and only then add it to the arena handles map.
  //
  // Otherwise, if BeginMessageGroup and ClearReleasedMessageGroups will ever be
  // called from different threads, Message Group will be released before it was
  // even sent to the remote side.
  MP_RETURN_IF_ERROR(
      SplitEngineBridgeSender::EnqueueMessageGroup(GetClientId(), group_id));
  {
    absl::MutexLock lock(arena_handles_mutex_);
    if (!arena_handles_.emplace(group_id, arena_handle).second) {
      return absl::AlreadyExistsError(
          "Message group ID already exists (arena_handles_)");
    };
    if (!message_group_types_.emplace(group_id, message_type).second) {
      return absl::AlreadyExistsError(
          "Message group ID already exists (message_group_types_)");
    };
  }

  {
    absl::MutexLock lock(message_sizes_mutex_);
    if (!message_group_id_to_size_bytes_.emplace(group_id, size_bytes).second) {
      return absl::AlreadyExistsError(
          "Message group ID already exists (message_group_id_to_size_bytes_)");
    }
  }

  {
    const void* arena_head = GetArenaAllocator().GetArenaHead(arena_handle);
    absl::MutexLock lock(bridge_buffers_mutex_);
    if (bridge_buffers_.find(arena_head) == bridge_buffers_.end()) {
      return absl::InternalError("Failed to create shared memory buffer");
    }
  }

  imp::OwnedPtr<flatbuffers::FlatBufferBuilder> fbb =
      CreateFlatBufferBuilder(group_id, GetBeginMessageSize());

  flatbuffers::Offset<android_xr::schemas::MessageGroupOperation> operation =
      android_xr::schemas::CreateMessageGroupOperation(
          *fbb, group_id,
          android_xr::schemas::MessageGroupOperationTypes::BeginMessageGroup,
          android_xr::schemas::CreateBeginMessageGroup(*fbb).Union());
  fbb->Finish(operation);

  MP_RETURN_IF_ERROR(SendMessage(group_id, std::move(fbb)));

  return group_id;
}

imp::OwnedPtr<flatbuffers::FlatBufferBuilder>
SplitEngineSharedMemoryBridgeSenderBase::CreateFlatBufferBuilder(
    MessageGroupId message_group_id, size_t size_bytes) {
  return imp::MakeOwned<flatbuffers::FlatBufferBuilder>(
      size_bytes, &GetFlatbuffersAllocator(message_group_id));
}

flatbuffers::Allocator&
SplitEngineSharedMemoryBridgeSenderBase::GetFlatbuffersAllocator(
    MessageGroupId group_id) {
  ArenaAllocator::ArenaHandle arena_handle;
  {
    absl::MutexLock lock(arena_handles_mutex_);
    const auto arena_handle_it = arena_handles_.find(group_id);
    
    arena_handle = arena_handle_it->second;
  }
  return GetArenaAllocator().GetFlatbufferAllocator(arena_handle);
}

absl::Status SplitEngineSharedMemoryBridgeSenderBase::EndMessageGroup(
    MessageGroupId group_id) {
  

  imp::OwnedPtr<flatbuffers::FlatBufferBuilder> fbb =
      CreateFlatBufferBuilder(group_id, GetEndMessageSize());

  flatbuffers::Offset<android_xr::schemas::MessageGroupOperation> operation =
      android_xr::schemas::CreateMessageGroupOperation(
          *fbb, group_id,
          android_xr::schemas::MessageGroupOperationTypes::EndMessageGroup,
          android_xr::schemas::CreateEndMessageGroup(*fbb).Union());
  fbb->Finish(operation);

  MP_RETURN_IF_ERROR(SendMessage(group_id, std::move(fbb)));

  Schedule([this, group_id]() {
    absl::MutexLock lock(message_sizes_mutex_);
    message_group_id_to_size_bytes_.erase(group_id);
    return absl::OkStatus();
  });

  return absl::OkStatus();
}

void SplitEngineSharedMemoryBridgeSenderBase::ClearReleasedMessageGroups() {
  

  // Get the set of active message groups from the bridge.
  absl::Status status = SplitEngineBridgeSender::WithActiveMessageGroups(
      GetClientId(),
      [this](const absl::flat_hash_set<MessageGroupId>& active_message_groups) {
        absl::MutexLock lock(arena_handles_mutex_);
        // Iterate through all the active arenas and check if they are released.
        for (auto it = arena_handles_.begin(), end = arena_handles_.end();
             it != end;) {
          // Note: this is the advised pattern for erasing an item from a map
          // while iterating through it based on the documentation of
          // flat_hash_map.
          auto it_copy = it++;
          const MessageGroupId message_group_id = it_copy->first;
          const ArenaAllocator::ArenaHandle arena_handle = it_copy->second;
          if (active_message_groups.contains(message_group_id)) {
            // The message group is still active.
            continue;
          }
          const auto message_group_type_it =
              message_group_types_.find(message_group_id);
          
          const bool recycle =
              message_group_type_it->second == MessageType::kFrameUpdate;
          GetArenaAllocator().DestroyArena(arena_handle, recycle);
          arena_handles_.erase(it_copy);
          message_group_types_.erase(message_group_type_it);
        }
      });
  if (!status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to clear released message groups: "
               << status.ToString();
  }
}

absl::StatusOr<size_t>
SplitEngineSharedMemoryBridgeSenderBase::GetActiveMessageGroupCount() const {
  size_t in_flight_frame_count = 0;
  MP_RETURN_IF_ERROR(SplitEngineBridgeSender::WithActiveMessageGroups(
      GetClientId(),
      [&in_flight_frame_count](
          const absl::flat_hash_set<MessageGroupId>& active_message_groups) {
        in_flight_frame_count = active_message_groups.size();
      }));

  return in_flight_frame_count;
}

const BridgeBuffer& SplitEngineSharedMemoryBridgeSenderBase::GetBridgeBuffer(
    MessageGroupId group_id) {
  ArenaAllocator::ArenaHandle arena_handle;
  {
    absl::MutexLock lock(arena_handles_mutex_);
    const auto arena_handle_it = arena_handles_.find(group_id);
    
    arena_handle = arena_handle_it->second;
  }
  const void* arena_head = GetArenaAllocator().GetArenaHead(arena_handle);

  absl::MutexLock lock(bridge_buffers_mutex_);
  auto it = bridge_buffers_.find(arena_head);
  
  return *it->second;
}

absl::StatusOr<size_t>
SplitEngineSharedMemoryBridgeSenderBase::GetMessageGroupSizeBytes(
    MessageGroupId group_id) const {
  absl::MutexLock lock(message_sizes_mutex_);
  const auto size_it = message_group_id_to_size_bytes_.find(group_id);
  if (size_it == message_group_id_to_size_bytes_.end()) {
    return absl::NotFoundError("Message group ID not found");
  }
  return size_it->second;
}

// TODO: (broken link) - consider extracting scheduler implementation to a
// separate class.
void SplitEngineSharedMemoryBridgeSenderBase::Schedule(
    imp::Invocable<absl::Status()> fn) {
  

  // Once future becomes ready, it stops referencing its parents, so there's no
  // need to manage the chain manually. I.e., it's not possible to have a chain
  // that have 100 completed futures and keep on growing.
  //
  // What actually can happen is that we may have a very long chain of pending
  // futures (think creating and destroying a hundred of textures per frame).
  // Since child futures are called recursively by default, that increases the
  // risk of stack overflow (call stacks with 1000+ frames were observed in
  // stress tests).
  //
  // `future_benchmark_test.cc` shows that using kScheduleAlways slow things
  // down noticeably. Based on obtained results, it's better to inject
  // kScheduleAlways every 64 futures to maintain performance that is similar to
  // default behavior, and keep the size of the call stack reasonable.
  //
  // TODO: (broken link) - re-visit this logic later to make sure it still shows
  // good performance.
  total_ops_count_++;
  const FutureExecutorMode executor_mode =
      (total_ops_count_ % kScheduleAlwaysEveryN == 0)
          ? FutureExecutorMode::kScheduleAlways
          : FutureExecutorMode::kScheduleIfNotOnExecutorThread;

  pending_ops_ = pending_ops_.Then(
      std::move(fn), {
                         .executor = Executor::BackgroundExecutor(),
                         .executor_mode = executor_mode,
                     });
}

void SplitEngineSharedMemoryBridgeSenderBase::DrainScheduler(
    absl::Status status) {
  
  absl::Notification notification;
  Schedule([&notification, status = std::move(status)]() {
    notification.Notify();
    return status;
  });
  notification.WaitForNotification();
}

SplitEngineSharedMemoryBridgeSenderBase::
    ~SplitEngineSharedMemoryBridgeSenderBase() {
  // Destruction of BridgeBuffers (if any will be left by this time) will submit
  // bunch of tasks to unmap the memory, so we need to wait for them to
  // complete.
  {
    absl::MutexLock lock(bridge_buffers_mutex_);
    bridge_buffers_.clear();
  }

  // Once the bridge is destroyed, the scheduler will stay in this error state
  // to prevent any new futures getting executed (highly unlikely, because
  // scheduler will no longer be alive after exit from this destructor).
  DrainScheduler(absl::CancelledError("Scheduler is destroyed."));
}

}  // namespace imp::split_engine
