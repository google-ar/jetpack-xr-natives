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

#include "core/split_engine/split_engine_test_bridge.h"

#include <asm-generic/mman-common.h>
#include <jni.h>
#include <linux/mman.h>
#include <sys/mman.h>

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iterator>
#include <memory>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "filament/libs/utils/include/utils/ashmem.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/common/invocable.h"
#include "core/common/owned_ptr.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"
#include "core/split_engine/flatbuffer_arena_allocator.h"
#include "core/split_engine/flatbuffer_size_calculator.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_test_bridge_serializer.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace imp::split_engine {

TestSplitEngineAndroidBridge::TestSplitEngineAndroidBridge(
    SplitEngineSharedMemoryBridgeClient& bridge_client,
    SplitEngineTestBridgeSerializer& split_engine_test_bridge_serializer)
    : bridge_client_(bridge_client),
      split_engine_test_bridge_serializer_(
          split_engine_test_bridge_serializer) {}

jobject TestSplitEngineAndroidBridge::CreateExternalTextureSurface(
    const std::vector<TextureId>& texture_ids) {
  return *bridge_client_.CreateExternalTextureSurface(texture_ids);
}

bool TestSplitEngineAndroidBridge::SetExternalTextureSurfaceSize(
    TextureId texture_id, int32_t width, int32_t height) {
  if (absl::Status status = bridge_client_.SetExternalTextureSurfaceSize(
          texture_id, width, height);
      status.ok()) {
    return true;
  }
  return false;
}

bool TestSplitEngineAndroidBridge::SendCommand(
    const std::vector<uint8_t>& data) {
  return split_engine_test_bridge_serializer_
      .SendCommand(bridge_client_.GetClientId(), data)
      .ok();
}

bool TestSplitEngineAndroidBridge::SendRequest(
    const std::vector<uint8_t>& data,
    std::function<void(const std::vector<uint8_t>&)> callback) {
  return split_engine_test_bridge_serializer_
      .SendRequest(bridge_client_.GetClientId(), data, callback)
      .ok();
}

MessageGroupId TestSplitEngineAndroidBridge::GenerateMessageGroupId() {
  return bridge_client_.GenerateMessageGroupId();
}

TestSplitEngineBridgeBuffer::TestSplitEngineBridgeBuffer(
    size_t buffer_size_bytes)
    : size_in_bytes_(buffer_size_bytes) {
  shared_memory_region_fd_ =
      // Using cross-platform Filament API to create the shared memory region
      utils::ashmem_create_region("RenderingBridgeBuffer", size_in_bytes_);
  if (shared_memory_region_fd_ == 0) {
    IMP_LOG(imp::FATAL) << "Failed to allocate render bridge buffer";
  }
  mmapped_ptr_ = ::mmap(nullptr, size_in_bytes_, PROT_READ | PROT_WRITE,
                        MAP_SHARED, shared_memory_region_fd_, 0);
  if (mmapped_ptr_ == MAP_FAILED) {
    IMP_LOG(imp::FATAL) << "Failed to mmap RenderingBridgeAssetBuffer";
  }
}

TestSplitEngineBridgeBuffer::TestSplitEngineBridgeBuffer(
    TestSplitEngineBridgeBuffer&& other)
    : shared_memory_region_fd_(std::move(other.shared_memory_region_fd_)),
      mmapped_ptr_(other.mmapped_ptr_),
      size_in_bytes_(other.size_in_bytes_) {
  other.shared_memory_region_fd_ = 0;
  other.mmapped_ptr_ = nullptr;
  other.size_in_bytes_ = 0;
}

TestSplitEngineBridgeBuffer::~TestSplitEngineBridgeBuffer() {
  if (mmapped_ptr_ != nullptr) {
    ::munmap(mmapped_ptr_, size_in_bytes_);
    mmapped_ptr_ = nullptr;
  }
  close(shared_memory_region_fd_);
}

TestSplitEngineBridgeSender::TestSplitEngineBridgeSender(
    TestSplitEngineAndroidBridge& test_bridge)
    : test_bridge_(test_bridge) {}

void* AllocateSharedMemoryBuffer(size_t size_in_bytes, void* user) {
  return reinterpret_cast<TestSplitEngineBridgeSender*>(user)
      ->CreateSharedMemoryBuffer(size_in_bytes);
}

void DeallocateSharedMemoryBuffer(void* ptr, void* user) {
  reinterpret_cast<TestSplitEngineBridgeSender*>(user)
      ->DestroySharedMemoryBuffer(ptr);
}

void* TestSplitEngineBridgeSender::CreateSharedMemoryBuffer(
    size_t size_in_bytes) {
  auto bridge_buffer =
      std::make_unique<TestSplitEngineBridgeBuffer>(size_in_bytes);
  void* buffer_head = bridge_buffer->Data();
  bridge_buffers_.emplace(buffer_head, std::move(bridge_buffer));
  return buffer_head;
}

void TestSplitEngineBridgeSender::DestroySharedMemoryBuffer(void* head) {
  // Note that the map that we're erasing from holds unique_ptrs, so this
  // erase() doesn't just remove it from the map but also destroys the
  // BridgeBuffer object.
  bridge_buffers_.erase(head);
}
namespace {
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

absl::StatusOr<MessageGroupId> TestSplitEngineBridgeSender::BeginMessageGroup(
    size_t size_bytes, MessageType message_type) {
  // Step 1: Create a memory arena for the new group.
  const ArenaAllocator::ArenaHandle arena_handle = arena_allocator_.CreateArena(
      GetBeginMessageSize() + size_bytes + GetEndMessageSize(),
      {AllocateSharedMemoryBuffer, DeallocateSharedMemoryBuffer, this});

  

  // Step 2: Send a `BeginMessageGroup` message with the arena handle.
  flatbuffers::FlatBufferBuilder fbb(
      GetBeginMessageSize(),
      &arena_allocator_.GetFlatbufferAllocator(arena_handle));

  const MessageGroupId group_id = test_bridge_.GenerateMessageGroupId();
  
  

  flatbuffers::Offset<android_xr::schemas::MessageGroupOperation>
      message_group = android_xr::schemas::CreateMessageGroupOperation(
          fbb, group_id,
          android_xr::schemas::MessageGroupOperationTypes::BeginMessageGroup,
          android_xr::schemas::CreateBeginMessageGroup(fbb).Union());
  fbb.Finish(message_group);

  // Test Bridge forwards everything to Renderer::HandleMessage, so we don't
  // need to actually send the BeginMessageGroup message, but we do need to
  // build the message to utilize the memory.
  //
  // SendMessage(fbb);

  return group_id;
}

imp::OwnedPtr<flatbuffers::FlatBufferBuilder>
TestSplitEngineBridgeSender::CreateFlatBufferBuilder(MessageGroupId group_id,
                                                     size_t size_bytes) {
  auto it = arena_handles_.find(group_id);
  
  return imp::MakeOwned<flatbuffers::FlatBufferBuilder>(
      size_bytes, &arena_allocator_.GetFlatbufferAllocator(it->second));
}

absl::Status TestSplitEngineBridgeSender::EndMessageGroup(
    MessageGroupId group_id) {
  auto it = arena_handles_.find(group_id);
  
  const ArenaAllocator::ArenaHandle arena_handle = it->second;
  flatbuffers::FlatBufferBuilder fbb(
      GetBeginMessageSize(),
      &arena_allocator_.GetFlatbufferAllocator(arena_handle));

  const flatbuffers::Offset<android_xr::schemas::MessageGroupOperation>
      message_group = android_xr::schemas::CreateMessageGroupOperation(
          fbb, group_id,
          android_xr::schemas::MessageGroupOperationTypes::EndMessageGroup,
          android_xr::schemas::CreateEndMessageGroup(fbb).Union());
  fbb.Finish(message_group);

  // Test Bridge forwards everything to Renderer::HandleMessage, so we don't
  // need to actually send the EndMessageGroup message, but we do need to
  // build the message to utilize the memory.
  //
  // SendMessage(fbb);

  // It's okay to destroy the arena here, since ::SendMessage copies the data
  // out of the arena.
  arena_allocator_.DestroyArena(
      arena_handle,
      /* allow_recycle= */
      GetMessageGroupType(group_id) == MessageType::kFrameUpdate);
  arena_handles_.erase(it);

  return absl::OkStatus();
}

const TestSplitEngineBridgeBuffer& TestSplitEngineBridgeSender::GetBridgeBuffer(
    MessageGroupId group_id) {
  auto arena_it = arena_handles_.find(group_id);
  
  const void* arena_head = arena_allocator_.GetArenaHead(arena_it->second);
  auto bridge_it = bridge_buffers_.find(arena_head);
  
  return *bridge_it->second;
}

absl::Status TestSplitEngineBridgeSender::SendMessage(
    MessageGroupId group_id,
    imp::OwnedPtr<flatbuffers::FlatBufferBuilder> fbb) {
  if (!GetBridgeBuffer(group_id).IsValidBlock(fbb->GetBufferPointer(),
                                              fbb->GetSize())) {
    return absl::InternalError("Message is not in the active bridge buffer.");
  }

  std::vector<uint8_t> command_data;
  std::copy(fbb->GetBufferPointer(), &fbb->GetBufferPointer()[fbb->GetSize()],
            std::back_inserter(command_data));

  if (!test_bridge_.SendCommand(command_data)) {
    return absl::InternalError("Failed to send command.");
  }
  return absl::OkStatus();
}

void TestSplitEngineBridgeSender::ClearReleasedMessageGroups() {
  // Do nothing since data is copied out of the arenas.
}

absl::StatusOr<size_t> TestSplitEngineBridgeSender::GetActiveMessageGroupCount()
    const {
  return 0;
}

TestSplitEngineBridgeSender::MessageType
TestSplitEngineBridgeSender::GetMessageGroupType(
    MessageGroupId message_group_id) {
  auto it = message_group_types_.find(message_group_id);
  
  return it->second;
}

void TestSplitEngineBridgeSender::Schedule(imp::Invocable<absl::Status()> fn) {
  
}

}  // namespace imp::split_engine
