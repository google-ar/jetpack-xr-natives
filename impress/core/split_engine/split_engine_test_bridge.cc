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
#include <optional>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "filament/libs/utils/include/utils/ashmem.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
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
    TestSplitEngineAndroidBridge& test_bridge, bool recycle_buffers)
    : test_bridge_(test_bridge), recycle_buffers_(recycle_buffers) {}

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

void TestSplitEngineBridgeSender::BeginMessageGroup(size_t size_bytes) {
  // Step 1: Create a memory arena for the new group.
  FlatbufferArenaAllocator::ArenaHandle arena_handle =
      arena_allocator_.CreateArena(
          GetBeginMessageSize() + size_bytes + GetEndMessageSize(),
          {AllocateSharedMemoryBuffer, DeallocateSharedMemoryBuffer, this});

  // Step 2: Remember which BridgeBuffer object this ArenaHandle is associated
  // with.
  auto bridge_buffer_backing_this_arena =
      bridge_buffers_.find(arena_allocator_.GetArenaHead(arena_handle));
  assert(bridge_buffer_backing_this_arena != bridge_buffers_.end());
  active_bridge_buffer_ = bridge_buffer_backing_this_arena->second.get();

  // Step 3: Send a `BeginMessageGroup` message with the arena handle.
  flatbuffers::FlatBufferBuilder fbb(GetBeginMessageSize(), &arena_allocator_);

  MessageGroupId message_group_id = test_bridge_.GenerateMessageGroupId();

  flatbuffers::Offset<android_xr::schemas::MessageGroupOperation>
      message_group = android_xr::schemas::CreateMessageGroupOperation(
          fbb, message_group_id,
          android_xr::schemas::MessageGroupOperationTypes::BeginMessageGroup,
          android_xr::schemas::CreateBeginMessageGroup(fbb).Union());
  fbb.Finish(message_group);

  // Test Bridge forwards everything to Renderer::HandleMessage, so we don't
  // need to actually send the BeginMessageGroup message, but we do need to
  // build the message to utilize the memory.
  //
  // SendMessage(fbb);

  active_message_group_id_ = message_group_id;
}

std::unique_ptr<flatbuffers::FlatBufferBuilder>
TestSplitEngineBridgeSender::CreateFlatBufferBuilder(size_t size_bytes) {
  return std::make_unique<flatbuffers::FlatBufferBuilder>(size_bytes,
                                                          &arena_allocator_);
}

void TestSplitEngineBridgeSender::EndMessageGroup() {
  
  MessageGroupId message_group_id = *active_message_group_id_;
  active_message_group_id_ = std::nullopt;

  auto arena_handle = arena_allocator_.GetActiveArena();
  flatbuffers::FlatBufferBuilder fbb(GetBeginMessageSize(), &arena_allocator_);

  flatbuffers::Offset<android_xr::schemas::MessageGroupOperation>
      message_group = android_xr::schemas::CreateMessageGroupOperation(
          fbb, message_group_id,
          android_xr::schemas::MessageGroupOperationTypes::EndMessageGroup,
          android_xr::schemas::CreateEndMessageGroup(fbb).Union());
  fbb.Finish(message_group);

  arena_allocator_.CloseActiveArena();
  // Test Bridge forwards everything to Renderer::HandleMessage, so we don't
  // need to actually send the EndMessageGroup message, but we do need to
  // build the message to utilize the memory.
  //
  // SendMessage(fbb);

  // It's okay to destroy the arena here, since ::SendMessage copies the data
  // out of the arena.
  arena_allocator_.DestroyArena(arena_handle, recycle_buffers_);
}

void TestSplitEngineBridgeSender::SendMessage(
    const flatbuffers::FlatBufferBuilder& fbb) {
  

  std::vector<uint8_t> command_data;
  std::copy(fbb.GetBufferPointer(), &fbb.GetBufferPointer()[fbb.GetSize()],
            std::back_inserter(command_data));

  test_bridge_.SendCommand(command_data);
}

void TestSplitEngineBridgeSender::ClearReleasedMessageGroups() {
  // Do nothing since data is copied out of the arenas.
}

absl::StatusOr<size_t> TestSplitEngineBridgeSender::GetActiveMessageGroupCount()
    const {
  return 0;
}

}  // namespace imp::split_engine
