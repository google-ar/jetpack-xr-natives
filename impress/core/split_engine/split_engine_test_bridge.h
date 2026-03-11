/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_TEST_BRIDGE_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_TEST_BRIDGE_H_

#include <jni.h>
#include <sys/types.h>

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/split_engine/android/split_engine_android_bridge.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"
#include "core/split_engine/flatbuffer_arena_allocator.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_bridge_sender.h"
#include "core/split_engine/split_engine_test_bridge_serializer.h"

namespace imp::split_engine {

// A SplitEngineAndroidBridge for use in unit tests. This bridge uses a mock
// SplitEngineSharedMemoryBridgeClient and a SplitEngineTestBridgeSerializer in
// order to pass commands directly to the renderer view and save them to a file.
// This allows for doing backwards-compatibility testing of the Split Engine
// schema.
// Note: this bridge does not use the shared memory code of the real bridge.
//
// Generally speaking, users should use SplitEngineTestFixture instead of
// directly using this class.
class TestSplitEngineAndroidBridge : public SplitEngineAndroidBridge {
 public:
  TestSplitEngineAndroidBridge(
      SplitEngineSharedMemoryBridgeClient& bridge_client,
      SplitEngineTestBridgeSerializer& split_engine_test_bridge_serializer);

  jobject CreateExternalTextureSurface(
      const std::vector<TextureId>& texture_ids) override;

  bool SetExternalTextureSurfaceSize(TextureId texture_id, int32_t width,
                                     int32_t height) override;

  bool SendCommand(const std::vector<uint8_t>& data);

  bool SendRequest(
      const std::vector<uint8_t>& data,
      std::function<void(const std::vector<uint8_t>&)> callback) override;

  MessageGroupId GenerateMessageGroupId();

 private:
  SplitEngineSharedMemoryBridgeClient& bridge_client_;
  SplitEngineTestBridgeSerializer& split_engine_test_bridge_serializer_;
};

// Manages a shared memory bridge buffer.
class TestSplitEngineBridgeBuffer {
 public:
  TestSplitEngineBridgeBuffer(size_t buffer_size_bytes);
  TestSplitEngineBridgeBuffer(TestSplitEngineBridgeBuffer&& other);
  TestSplitEngineBridgeBuffer(const TestSplitEngineBridgeBuffer&) = delete;
  TestSplitEngineBridgeBuffer& operator=(const TestSplitEngineBridgeBuffer&) =
      delete;

  ~TestSplitEngineBridgeBuffer();

  void* Data() { return mmapped_ptr_; }
  const void* Data() const { return mmapped_ptr_; }
  bool IsValidBlock(const uint8_t* data, size_t data_size_in_bytes) const {
    return data >= Data() &&
           (data + data_size_in_bytes) <=
               (static_cast<const uint8_t*>(Data()) + size_in_bytes_);
  }

 private:
  int shared_memory_region_fd_ = 0;
  void* mmapped_ptr_ = nullptr;
  size_t size_in_bytes_ = 0;
};

// A SplitEngineBridgeSender for use in unit tests. This sender pulls the test
// bridge from the registry and sends the messages using the test bridge's
// SendCommand method.
//
// It uses shared memory for writing to be as close to the real world as
// possible.
class TestSplitEngineBridgeSender : public SplitEngineBridgeSender {
 public:
  using MessageType = SplitEngineBridgeSender::MessageType;

  TestSplitEngineBridgeSender(TestSplitEngineAndroidBridge& bridge);

  absl::StatusOr<MessageGroupId> BeginMessageGroup(
      size_t size_bytes, MessageType message_type) override;

  absl::Status EndMessageGroup(MessageGroupId group_id) override;

  std::unique_ptr<flatbuffers::FlatBufferBuilder> CreateFlatBufferBuilder(
      MessageGroupId group_id, size_t size_bytes) override;

  absl::Status SendMessage(MessageGroupId group_id,
                           const flatbuffers::FlatBufferBuilder& fbb) override;

  void ClearReleasedMessageGroups() override;

  absl::StatusOr<size_t> GetActiveMessageGroupCount() const override;

  void* CreateSharedMemoryBuffer(size_t size_in_bytes);
  void DestroySharedMemoryBuffer(void*);

 private:
  const TestSplitEngineBridgeBuffer& GetBridgeBuffer(MessageGroupId group_id);

  TestSplitEngineAndroidBridge& test_bridge_;

  absl::flat_hash_map<const void*, std::unique_ptr<TestSplitEngineBridgeBuffer>>
      bridge_buffers_;
  ArenaAllocator arena_allocator_;

  absl::flat_hash_map<MessageGroupId, ArenaAllocator::ArenaHandle>
      arena_handles_;

  absl::flat_hash_map<MessageGroupId, MessageType> message_group_types_;

  MessageType GetMessageGroupType(MessageGroupId message_group_id);
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_TEST_BRIDGE_H_
