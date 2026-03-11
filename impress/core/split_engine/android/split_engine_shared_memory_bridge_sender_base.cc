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
#include "flatbuffers/allocator.h"
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
    SplitEngineSharedMemoryBridgeSenderBase() {}

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

absl::StatusOr<MessageGroupId>
SplitEngineSharedMemoryBridgeSenderBase::BeginMessageGroup(
    size_t size_bytes, MessageType message_type) {
  // Step 1: Create a memory arena for the new group.
  const ArenaAllocator::ArenaHandle arena_handle =
      GetArenaAllocator().CreateArena(
          GetBeginMessageSize() + size_bytes + GetEndMessageSize(),
          {AllocateSharedMemoryBuffer, DeallocateSharedMemoryBuffer, this});

  const MessageGroupId group_id = GenerateMessageGroupId();
  arena_handles_.emplace(group_id, arena_handle);
  message_group_id_to_size_bytes_.emplace(group_id, size_bytes);
  message_group_types_.emplace(group_id, message_type);

  if (bridge_buffers_.find(GetArenaAllocator().GetArenaHead(arena_handle)) ==
      bridge_buffers_.end()) {
    return absl::InternalError(
        "Cannot find bridge buffer corresponding to the arena handle.");
  }

  // Step 2: Send a `BeginMessageGroup` message with the arena handle.
  std::unique_ptr<flatbuffers::FlatBufferBuilder> fbb =
      CreateFlatBufferBuilder(group_id, GetBeginMessageSize());

  MP_RETURN_IF_ERROR(
      SplitEngineBridgeSender::EnqueueMessageGroup(GetClientId(), group_id));

  flatbuffers::Offset<android_xr::schemas::MessageGroupOperation> operation =
      android_xr::schemas::CreateMessageGroupOperation(
          *fbb, group_id,
          android_xr::schemas::MessageGroupOperationTypes::BeginMessageGroup,
          android_xr::schemas::CreateBeginMessageGroup(*fbb).Union());
  fbb->Finish(operation);

  MP_RETURN_IF_ERROR(SendMessage(group_id, *fbb));

  return group_id;
}

std::unique_ptr<flatbuffers::FlatBufferBuilder>
SplitEngineSharedMemoryBridgeSenderBase::CreateFlatBufferBuilder(
    MessageGroupId message_group_id, size_t size_bytes) {
  return std::make_unique<flatbuffers::FlatBufferBuilder>(
      size_bytes, &GetFlatbuffersAllocator(message_group_id));
}

flatbuffers::Allocator&
SplitEngineSharedMemoryBridgeSenderBase::GetFlatbuffersAllocator(
    MessageGroupId group_id) {
  const auto arena_handle_it = arena_handles_.find(group_id);
  
  return GetArenaAllocator().GetFlatbufferAllocator(arena_handle_it->second);
}

absl::Status SplitEngineSharedMemoryBridgeSenderBase::EndMessageGroup(
    MessageGroupId group_id) {
  std::unique_ptr<flatbuffers::FlatBufferBuilder> fbb =
      CreateFlatBufferBuilder(group_id, GetEndMessageSize());

  flatbuffers::Offset<android_xr::schemas::MessageGroupOperation> operation =
      android_xr::schemas::CreateMessageGroupOperation(
          *fbb, group_id,
          android_xr::schemas::MessageGroupOperationTypes::EndMessageGroup,
          android_xr::schemas::CreateEndMessageGroup(*fbb).Union());
  fbb->Finish(operation);

  MP_RETURN_IF_ERROR(SendMessage(group_id, *fbb));

  message_group_id_to_size_bytes_.erase(group_id);
  return absl::OkStatus();
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
          message_group_types_.erase(message_group_type_it);
          arena_handles_.erase(it_copy);
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
  const auto arena_handle_it = arena_handles_.find(group_id);
  
  const ArenaAllocator::ArenaHandle arena_handle = arena_handle_it->second;
  const void* arena_head = GetArenaAllocator().GetArenaHead(arena_handle);
  auto it = bridge_buffers_.find(arena_head);
  
  return *it->second;
}

absl::StatusOr<size_t>
SplitEngineSharedMemoryBridgeSenderBase::GetMessageGroupSizeBytes(
    MessageGroupId group_id) const {
  const auto size_bytes_it = message_group_id_to_size_bytes_.find(group_id);
  if (size_bytes_it == message_group_id_to_size_bytes_.end()) {
    return absl::NotFoundError("Message group not found");
  }
  return size_bytes_it->second;
}

}  // namespace imp::split_engine
