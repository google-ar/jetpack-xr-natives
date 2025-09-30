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
#include <optional>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
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
    SplitEngineSharedMemoryBridgeSenderBase(bool recycle_buffers)
    : recycle_buffers_(recycle_buffers) {}

void* SplitEngineSharedMemoryBridgeSenderBase::CreateSharedMemoryBuffer(
    size_t size_in_bytes) {
  auto bridge_buffer =
      std::make_unique<BridgeBuffer>(GetBufferHandleFactory(), size_in_bytes);
  void* buffer_head = bridge_buffer->Data();
  bridge_buffers_.emplace(buffer_head, std::move(bridge_buffer));
  return buffer_head;
}

void SplitEngineSharedMemoryBridgeSenderBase::DestroySharedMemoryBuffer(
    void* head) {
  // Note that the map that we're erasing from holds unique_ptrs, so this
  // erase() doesn't just remove it from the map but also destroys the
  // BridgeBuffer object.
  bridge_buffers_.erase(head);
}

void SplitEngineSharedMemoryBridgeSenderBase::BeginMessageGroup(
    size_t size_bytes) {
  // Step 1: Create a memory arena for the new group.
  FlatbufferArenaAllocator::ArenaHandle arena_handle =
      GetAllocator().CreateArena(
          GetBeginMessageSize() + size_bytes + GetEndMessageSize(),
          {AllocateSharedMemoryBuffer, DeallocateSharedMemoryBuffer, this});

  // Step 2: Remember which BridgeBuffer object this ArenaHandle is associated
  // with.
  auto bridge_buffer_backing_this_arena =
      bridge_buffers_.find(GetAllocator().GetArenaHead(arena_handle));
  assert(bridge_buffer_backing_this_arena != bridge_buffers_.end());
  active_bridge_buffer_ = bridge_buffer_backing_this_arena->second.get();

  // Step 3: Send a `BeginMessageGroup` message with the arena handle.
  std::unique_ptr<flatbuffers::FlatBufferBuilder> fbb =
      CreateFlatBufferBuilder(GetBeginMessageSize());

  MessageGroupId message_group_id = GenerateMessageGroupId();
  arena_handles_.emplace(message_group_id, arena_handle);

  if (absl::Status enqueue_result =
          SplitEngineBridgeSender::EnqueueMessageGroup(GetClientId(),
                                                       message_group_id);
      !enqueue_result.ok()) {
    IMP_LOG(imp::FATAL) << "EnqueueMessageGroupTransaction failed: "
               << enqueue_result.ToString();
  }

  flatbuffers::Offset<android_xr::schemas::MessageGroupOperation> operation =
      android_xr::schemas::CreateMessageGroupOperation(
          *fbb, message_group_id,
          android_xr::schemas::MessageGroupOperationTypes::BeginMessageGroup,
          android_xr::schemas::CreateBeginMessageGroup(*fbb).Union());
  fbb->Finish(operation);

  active_message_group_id_ = message_group_id;
  active_message_group_size_bytes_ = size_bytes;

  SendMessage(*fbb);
}

std::unique_ptr<flatbuffers::FlatBufferBuilder>
SplitEngineSharedMemoryBridgeSenderBase::CreateFlatBufferBuilder(
    size_t size_bytes) {
  return std::make_unique<flatbuffers::FlatBufferBuilder>(size_bytes,
                                                          &GetAllocator());
}

void SplitEngineSharedMemoryBridgeSenderBase::EndMessageGroup() {
  
  const MessageGroupId message_group_id = *active_message_group_id_;

  // Verify that the active arena is the one we expect.
  auto arena_handle = GetAllocator().GetActiveArena();
  

  std::unique_ptr<flatbuffers::FlatBufferBuilder> fbb =
      CreateFlatBufferBuilder(GetBeginMessageSize());

  flatbuffers::Offset<android_xr::schemas::MessageGroupOperation> operation =
      android_xr::schemas::CreateMessageGroupOperation(
          *fbb, message_group_id,
          android_xr::schemas::MessageGroupOperationTypes::EndMessageGroup,
          android_xr::schemas::CreateEndMessageGroup(*fbb).Union());
  fbb->Finish(operation);

  GetAllocator().CloseActiveArena();

  SendMessage(*fbb);

  active_message_group_id_ = std::nullopt;
  active_message_group_size_bytes_ = std::nullopt;
}

bool SplitEngineSharedMemoryBridgeSenderBase::IsMessageGroupActive() const {
  return active_message_group_id_.has_value();
}

void SplitEngineSharedMemoryBridgeSenderBase::ClearReleasedMessageGroups() {
  // Get the set of active message groups from the bridge.
  absl::Status status = SplitEngineBridgeSender::WithActiveMessageGroups(
      GetClientId(),
      [this](const absl::flat_hash_set<MessageGroupId>& active_message_groups) {
        // Iterate through all the active arenas and check if they are released.
        for (auto it = arena_handles_.begin(), end = arena_handles_.end();
             it != end;) {
          // Note: this is the advised pattern for erasing an item from a map
          // while iterating through it based on the documentation of
          // flat_hash_map.
          auto it_copy = it++;
          if (active_message_groups.contains(it_copy->first)) {
            // The message group is still active.
            continue;
          }
          GetAllocator().DestroyArena(it_copy->second, recycle_buffers_);
          arena_handles_.erase(it_copy);
        }
      });
  if (!status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to clear released message groups: "
               << status.ToString();
  }
}

const BridgeBuffer&
SplitEngineSharedMemoryBridgeSenderBase::GetActiveBridgeBuffer() const {
  
  return *active_bridge_buffer_;
}
std::optional<MessageGroupId>
SplitEngineSharedMemoryBridgeSenderBase::GetActiveMessageGroupId() const {
  return active_message_group_id_;
}
std::optional<size_t>
SplitEngineSharedMemoryBridgeSenderBase::GetActiveMessageGroupSizeBytes()
    const {
  return active_message_group_size_bytes_;
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

}  // namespace imp::split_engine
