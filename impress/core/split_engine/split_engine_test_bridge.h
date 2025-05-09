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

#include "flatbuffers/flatbuffer_builder.h"
#include "core/split_engine/android/split_engine_android_bridge.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client_mock.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_bridge_sender.h"
#include "core/split_engine/split_engine_test_bridge_serializer.h"
#include "core/view/base_view.h"

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
      BridgeId bridge_id,
      SplitEngineTestBridgeSerializer& split_engine_test_bridge_serializer);

  jobject CreateExternalTextureSurface(
      const std::vector<TextureId>& texture_ids) override;

  bool SetExternalTextureSurfaceSize(TextureId texture_id, int32_t width,
                                     int32_t height) override;

  bool SendCommand(const std::vector<uint8_t>& data);

  bool SendRequest(
      const std::vector<uint8_t>& data,
      std::function<void(const std::vector<uint8_t>&)> callback) override;

  SplitEngineSharedMemoryBridgeClient& GetSplitEngineSharedMemoryBridgeClient()
      override;

 private:
  BridgeId bridge_id_;
  MockSplitEngineSharedMemoryBridgeClient bridge_client_;
  SplitEngineTestBridgeSerializer& split_engine_test_bridge_serializer_;
};

// A SplitEngineBridgeSender for use in unit tests. This sender pulls the test
// bridge from the registry and sends the messages using the test bridge's
// SendCommand method.
class TestSplitEngineSender : public SplitEngineBridgeSender {
 public:
  TestSplitEngineSender(BaseView& view);
  void BeginMessageGroup(size_t size_bytes) override;
  void EndMessageGroup() override;
  bool IsMessageGroupActive() const override;

  std::unique_ptr<flatbuffers::FlatBufferBuilder> CreateFlatBufferBuilder(
      size_t size_bytes) override;

  void SendMessage(const flatbuffers::FlatBufferBuilder& fbb) override;
  void SetEnabled(bool enabled);

 private:
  BaseView& view_;
  bool message_group_active_ = false;
  bool enabled_ = true;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_TEST_BRIDGE_H_
