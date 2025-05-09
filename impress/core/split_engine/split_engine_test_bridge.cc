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

#include <jni.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iterator>
#include <memory>
#include <vector>

#include "core/common/log.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/common/registry.h"
#include "core/split_engine/android/split_engine_android_bridge.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_test_bridge_serializer.h"
#include "core/view/base_view.h"

namespace imp::split_engine {

TestSplitEngineAndroidBridge::TestSplitEngineAndroidBridge(
    BridgeId bridge_id,
    SplitEngineTestBridgeSerializer& split_engine_test_bridge_serializer)
    : bridge_id_(bridge_id),
      split_engine_test_bridge_serializer_(
          split_engine_test_bridge_serializer) {}

jobject TestSplitEngineAndroidBridge::CreateExternalTextureSurface(
    const std::vector<TextureId>& texture_ids) {
  return bridge_client_.CreateExternalTextureSurface(texture_ids);
}

bool TestSplitEngineAndroidBridge::SetExternalTextureSurfaceSize(
    TextureId texture_id, int32_t width, int32_t height) {
  if (SplitEngineSharedMemoryBridgeClient::Result result =
          bridge_client_.SetExternalTextureSurfaceSize(texture_id, width,
                                                       height);
      result.is_ok()) {
    return true;
  }
  return false;
}

bool TestSplitEngineAndroidBridge::SendCommand(
    const std::vector<uint8_t>& data) {
  return split_engine_test_bridge_serializer_.SendCommand(bridge_id_, data)
      .ok();
}

bool TestSplitEngineAndroidBridge::SendRequest(
    const std::vector<uint8_t>& data,
    std::function<void(const std::vector<uint8_t>&)> callback) {
  return split_engine_test_bridge_serializer_
      .SendRequest(bridge_id_, data, callback)
      .ok();
}

SplitEngineSharedMemoryBridgeClient&
TestSplitEngineAndroidBridge::GetSplitEngineSharedMemoryBridgeClient() {
  IMP_LOG(imp::FATAL) << "Not implemented - the test bridge doesn't use shared memory.";
  return bridge_client_;
}

TestSplitEngineSender::TestSplitEngineSender(BaseView& view) : view_(view) {}

void TestSplitEngineSender::BeginMessageGroup(size_t size_bytes) {
  message_group_active_ = true;
};

void TestSplitEngineSender::EndMessageGroup() {
  message_group_active_ = false;
};
bool TestSplitEngineSender::IsMessageGroupActive() const {
  return message_group_active_;
}

std::unique_ptr<flatbuffers::FlatBufferBuilder>
TestSplitEngineSender::CreateFlatBufferBuilder(size_t size_bytes) {
  return std::make_unique<flatbuffers::FlatBufferBuilder>();
}

void TestSplitEngineSender::SendMessage(
    const flatbuffers::FlatBufferBuilder& fbb) {
  SplitEngineAndroidBridge& bridge =
      view_.GetRegistry().Get<SplitEngineAndroidBridge>()->get();
  TestSplitEngineAndroidBridge& test_bridge =
      static_cast<TestSplitEngineAndroidBridge&>(bridge);
  std::vector<uint8_t> command_data;
  std::copy(fbb.GetBufferPointer(), &fbb.GetBufferPointer()[fbb.GetSize()],
            std::back_inserter(command_data));
  test_bridge.SendCommand(command_data);
}

void TestSplitEngineSender::SetEnabled(bool enabled) { enabled_ = enabled; }

}  // namespace imp::split_engine
