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

#include "absl/functional/bind_front.h"
#include "core/common/log.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/common/platform_helpers.h"
#include "core/split_engine/android/bridge_buffer.h"
#include "core/split_engine/android/message_group_id_mapper.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"
#include "core/split_engine/flatbuffer_arena_allocator.h"
#include "core/split_engine/flatbuffer_size_calculator.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace imp::split_engine {

namespace {

void* AllocateSharedMemoryBuffer(size_t size_in_bytes, void* user) {
  return reinterpret_cast<SplitEngineSharedMemoryBridgeSender*>(user)
      ->CreateSharedMemoryBuffer(size_in_bytes);
}

void DeallocateSharedMemoryBuffer(void* ptr, void* user) {
  reinterpret_cast<SplitEngineSharedMemoryBridgeSender*>(user)
      ->DestroySharedMemoryBuffer(ptr);
}

template <typename T>
void CreateMessageGroup(flatbuffers::FlatBufferBuilder& fbb,
                        MessageGroupId message_group_id,
                        flatbuffers::Offset<T> command_offset) {
  flatbuffers::Offset<android_xr::schemas::MessageGroup> message_group =
      android_xr::schemas::CreateMessageGroup(
          fbb, message_group_id,
          android_xr::schemas::MessageGroupTypesTraits<T>::enum_value,
          command_offset.Union());
  fbb.Finish(message_group);
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
};

}  // namespace

SplitEngineSharedMemoryBridgeSender::SplitEngineSharedMemoryBridgeSender(
    SplitEngineSharedMemoryBridgeClient& bridge,
    std::unique_ptr<MessageGroupIdMapper> message_group_id_mapper,
    bool recycle_buffers)
    : bridge_(bridge),
      message_group_id_mapper_(std::move(message_group_id_mapper)),
      recycle_buffers_(recycle_buffers),
      message_group_active_(false) {
  bridge_.RegisterReverseBridgeMessageHandler(absl::bind_front(
      &SplitEngineSharedMemoryBridgeSender::OnMessageGroupComplete, this));
}

void* SplitEngineSharedMemoryBridgeSender::CreateSharedMemoryBuffer(
    size_t size_in_bytes) {
  auto bridge_buffer = std::make_unique<BridgeBuffer>(bridge_, size_in_bytes);
  void* buffer_head = bridge_buffer->Data();
  bridge_buffers_.emplace(buffer_head, std::move(bridge_buffer));
  return buffer_head;
}

void SplitEngineSharedMemoryBridgeSender::DestroySharedMemoryBuffer(
    void* head) {
  // Note that the map that we're erasing from holds unique_ptrs, so this
  // erase() doesn't just remove it from the map but also destroys the
  // BridgeBuffer object.
  bridge_buffers_.erase(head);
}

void SplitEngineSharedMemoryBridgeSender::SendMessage(
    const flatbuffers::FlatBufferBuilder& builder) {
  assert(builder.GetBufferPointer() > active_bridge_buffer_->Data());
  int offset =
      builder.GetBufferPointer() - (uint8_t*)active_bridge_buffer_->Data();

  SplitEngineSharedMemoryBridgeClient::Result process_result =
      bridge_.ProcessRegion(active_bridge_buffer_->Handle(), offset,
                            builder.GetSize());
  if (!process_result.is_ok()) {
    IMP_LOG(imp::FATAL) << "Failed to queue buffer to rendering bridge: "
               << process_result.message();
  }
}

void SplitEngineSharedMemoryBridgeSender::BeginMessageGroup(size_t size_bytes) {
  // Step 1: Create a memory arena for the new group.
  FlatbufferArenaAllocator::ArenaHandle arena_handle =
      arena_allocator_.CreateArena(
          GetBeginMessageSize() + size_bytes + GetEndMessageSize(),
          AllocateSharedMemoryBuffer, DeallocateSharedMemoryBuffer, this);

  // Step 2: Remember which BridgeBuffer object this ArenaHandle is associated
  // with.
  auto bridge_buffer_backing_this_arena =
      bridge_buffers_.find(arena_allocator_.GetArenaHead(arena_handle));
  assert(bridge_buffer_backing_this_arena != bridge_buffers_.end());
  active_bridge_buffer_ = bridge_buffer_backing_this_arena->second.get();

  // Step 3: Send a `BeginMessageGroup` message with the arena handle.
  flatbuffers::FlatBufferBuilder fbb(GetBeginMessageSize(), &arena_allocator_);

  MessageGroupId msg_group_id =
      message_group_id_mapper_->GetMessageGroupId(arena_handle);

  CreateMessageGroup(fbb, msg_group_id,
                     android_xr::schemas::CreateBeginMessageGroup(fbb));
  SendMessage(fbb);

  message_group_active_ = true;
}

std::unique_ptr<flatbuffers::FlatBufferBuilder>
SplitEngineSharedMemoryBridgeSender::CreateFlatBufferBuilder(
    size_t size_bytes) {
  return std::make_unique<flatbuffers::FlatBufferBuilder>(size_bytes,
                                                          &arena_allocator_);
}

void SplitEngineSharedMemoryBridgeSender::EndMessageGroup() {
  message_group_active_ = false;

  auto arena_handle = arena_allocator_.GetActiveArena();
  flatbuffers::FlatBufferBuilder fbb(GetBeginMessageSize(), &arena_allocator_);

  MessageGroupId msg_group_id =
      message_group_id_mapper_->GetMessageGroupId(arena_handle);
  CreateMessageGroup(fbb, msg_group_id,
                     android_xr::schemas::CreateEndMessageGroup(fbb));
  arena_allocator_.CloseActiveArena();
  SendMessage(fbb);
}

bool SplitEngineSharedMemoryBridgeSender::IsMessageGroupActive() const {
  return message_group_active_;
}

void SplitEngineSharedMemoryBridgeSender::OnMessageGroupComplete(
    MessageGroupId group_id) {
  // All messages are delivered to all senders, and filtered for messages of
  // interest.
  if (!message_group_id_mapper_->IsMessageGroupIdValid(group_id)) {
    // Not a message for this sender's messages. Ignore.
    return;
  }
  auto arena_handle = message_group_id_mapper_->GetArenaHandle(group_id);
  arena_allocator_.DestroyArena(arena_handle, recycle_buffers_);
}

}  // namespace imp::split_engine
