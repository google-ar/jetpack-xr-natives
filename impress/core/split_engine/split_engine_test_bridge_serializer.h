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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_TEST_BRIDGE_SERIALIZER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_TEST_BRIDGE_SERIALIZER_H_

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/verifier.h"
#include "core/async/executor.h"
#include "core/common/invocable.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_service_impl.h"
#include "core/split_engine/schema_test_data/bridge_sequence_generated.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_renderer.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_schema_version.h"

namespace imp::split_engine {

// A SplitEngineSharedMemoryBridgeServiceImpl for use in unit tests.
// This class is needed to override the HandleMessage method so that the test
// can call directly into the renderer view.
class TestSplitEngineSharedMemoryBridgeServiceImpl
    : public SplitEngineSharedMemoryBridgeServiceImpl {
 public:
  TestSplitEngineSharedMemoryBridgeServiceImpl(BaseView& view,
                                               Executor* executor);

  // Note: the change from protected to public is so the test can call directly.
  absl::Status HandleMessage(
      BridgeId bridge_id, flatbuffers::Verifier& verifier,
      const uint8_t* message,
      SplitEngineRenderer::OnFinishedCallback on_finished) override;
};

// A helper class for SplitEngineTestFixture to send flatbuffer schema data to
// the renderer view and save them to a file. This allows for doing
// backwards-compatibility testing of the Split Engine schema.
// The RunAndCaptureTask method must be called with a TestT with a GetParam()
// method that returns the schema version via TEST_P.
//
// Generally speaking, users should use SplitEngineTestFixture instead of
// directly using this class.
class SplitEngineTestBridgeSerializer {
 public:
  SplitEngineTestBridgeSerializer(BaseView& renderer_view,
                                  Executor& renderer_executor)
      : view_(renderer_view),
        bridge_service_(renderer_view, &renderer_executor) {}

  // Sends a command to the renderer view.
  // If save is true, the command will be saved to the current snapshot.
  absl::Status SendCommand(BridgeId bridge_id, const std::vector<uint8_t>& data,
                           bool save = true);

  // Sends a request to the renderer view.
  // If save is true, the request will be saved to the current snapshot.
  absl::Status SendRequest(
      BridgeId bridge_id, const std::vector<uint8_t>& data,
      std::function<void(const std::vector<uint8_t>&)> callback,
      bool save = true);

  // Runs the given task on the serializer view and captures the messages if the
  // schema version is current.
  // If the schema version is not current, the task will be skipped and the
  // snapshot will be played back from file instead.
  // Note: this method must be called from a TestT with a GetParam() method that
  // returns the schema version via TEST_P.
  // Note: After calling RunAndCaptureTask, the test must call TakeSnapshot to
  // save the messages to a file or playback the snapshot.
  template <typename TestT>
  void RunAndCaptureTask(const TestT& test, Invocable<void()> task);
  // Saves the current snapshot to a file or plays it back if running against an
  // old schema version.
  void TakeSnapshot();

 private:
  void SaveMessage(BridgeId bridge_id, android_xr::schemas::MessageType type,
                   const std::vector<uint8_t>& message);
  void CreateSnapshot();
  void PlaybackSnapshot();
  std::string GetDataFilename();
  std::string GetSnapshotDirWrite();
  std::string GetSnapshotDirRead();

  BaseView& view_;
  TestSplitEngineSharedMemoryBridgeServiceImpl bridge_service_;

  // A snapshot of all split engine messages that were sent to the renderer view
  // during a single test segment. This is used to replay the messages in the
  // renderer view for backwards-compatibility testing.
  struct Snapshot {
    std::string schema_version;
    std::vector<flatbuffers::Offset<android_xr::schemas::Message>>
        message_sequence;
    std::unique_ptr<flatbuffers::FlatBufferBuilder> fbb;
  };
  std::optional<Snapshot> snapshot_;
  int32_t snapshot_index_ = 0;
};

template <typename TestT>
void SplitEngineTestBridgeSerializer::RunAndCaptureTask(
    const TestT& test, Invocable<void()> task) {
  snapshot_.emplace();
  snapshot_->schema_version = test.GetParam();
  if (snapshot_->schema_version ==
      android_xr::kSplitEngineSchemaVersionCurrent) {
    snapshot_->fbb = std::make_unique<flatbuffers::FlatBufferBuilder>();
    task();
  }  // else do nothing since the snapshot will be played back.
}

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_TEST_BRIDGE_SERIALIZER_H_
