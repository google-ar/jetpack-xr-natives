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

#include "core/split_engine/split_engine_test_bridge_serializer.h"

#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/notification.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/verifier.h"
#include "core/async/executor.h"
#include "core/common/file_helpers.h"
#include "core/common/invocable.h"
#include "core/resources/resource_manager.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_service_impl.h"
#include "core/split_engine/schema_test_data/bridge_sequence_generated.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_renderer.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "testing/base_executor_test_helper.h"
#include "split_engine/schemas/split_engine_schema_version.h"

namespace imp::split_engine {

namespace {
constexpr absl::string_view kSchemaDataTmpEnvVar = "SCHEMA_DATA_OUT_DIR";
constexpr absl::string_view kSchemaDataDefaultEnvVar =
    "TEST_UNDECLARED_OUTPUTS_DIR";
constexpr absl::string_view kSchemaTestDataDirectory =
    "third_party/impress/core/split_engine/schema_test_data";
}  // namespace

TestSplitEngineSharedMemoryBridgeServiceImpl::
    TestSplitEngineSharedMemoryBridgeServiceImpl(BaseView& view,
                                                 Executor* executor)
    : SplitEngineSharedMemoryBridgeServiceImpl(view, executor) {}

absl::Status TestSplitEngineSharedMemoryBridgeServiceImpl::HandleCommand(
    BridgeId bridge_id, flatbuffers::Verifier& verifier, const uint8_t* message,
    SplitEngineRenderer::OnFinishedCallback on_finished) {
  return SplitEngineSharedMemoryBridgeServiceImpl::HandleCommand(
      bridge_id, verifier, message, std::move(on_finished));
}

absl::Status SplitEngineTestBridgeSerializer::SendCommand(
    BridgeId bridge_id, const std::vector<uint8_t>& data, bool save) {
  if (!snapshot_.has_value()) {
    IMP_LOG(imp::FATAL) << "Call to SendCommand outside of RunAndCaptureTask.";
  }

  if (save) {
    // First, store the message in the current sequence.
    SaveMessage(bridge_id, android_xr::schemas::MessageType::Command, data);
  }

  // Next, pass the message to the bridge service to be processed.
  std::unique_ptr<std::vector<uint8_t>> data_storage =
      std::make_unique<std::vector<uint8_t>>(data);
  std::vector<uint8_t>* data_storage_ptr = data_storage.get();
  Invocable<void()> on_finished = [data = std::move(data_storage)]() mutable {
    data.reset();
  };
  flatbuffers::Verifier verifier(data_storage_ptr->data(),
                                 data_storage_ptr->size());
  return bridge_service_.HandleCommand(
      bridge_id, verifier, data_storage_ptr->data(), std::move(on_finished));
}

absl::Status SplitEngineTestBridgeSerializer::SendRequest(
    BridgeId bridge_id, const std::vector<uint8_t>& data,
    std::function<void(const std::vector<uint8_t>&)> callback, bool save) {
  if (!snapshot_.has_value()) {
    IMP_LOG(imp::FATAL) << "Call to SendCommand outside of RunAndCaptureTask.";
  }
  if (save) {
    // First, store the message in the current sequence.
    SaveMessage(bridge_id, android_xr::schemas::MessageType::Request, data);
  }
  // Next, pass the message to the bridge service to be processed.
  return bridge_service_.SendRequest(bridge_id, data, callback);
}

void SplitEngineTestBridgeSerializer::SaveMessage(
    BridgeId bridge_id, android_xr::schemas::MessageType type,
    const std::vector<uint8_t>& message) {
  snapshot_->message_sequence.push_back(android_xr::schemas::CreateMessage(
      *snapshot_->fbb, bridge_id, type,
      snapshot_->fbb->CreateVector(message.data(), message.size())));
}

std::string SplitEngineTestBridgeSerializer::GetDataFilename() {
  // Returns a file of the form "<test_suite>_<test_name>_<snapshot_index>.bin".
  const ::testing::TestInfo* const test_info =
      ::testing::UnitTest::GetInstance()->current_test_info();
  const std::string test_name_with_index(test_info->name());
  const std::string test_name =
      test_name_with_index.substr(0, test_name_with_index.rfind('/'));
  const std::string test_suite_with_index(test_info->test_suite_name());
  const std::string test_suite =
      test_suite_with_index.substr(test_suite_with_index.rfind('/') + 1);
  return absl::StrFormat("%s_%s_%s.bin", test_suite, test_name,
                         absl::StrCat(snapshot_index_));
}

std::string SplitEngineTestBridgeSerializer::GetSnapshotDirWrite() {
  // The schema_test_data/generate_schema_test_data.sh script sets this env
  // var to output to /tmp so it can be copied to the google3 schema_test_data
  // directory. This is to streamline local generation of the schema test
  // data.
  char* schema_data_out_dir = getenv(kSchemaDataTmpEnvVar.data());
  if (!schema_data_out_dir) {
    // If the env var is not set, use the TEST_UNDECLARED_OUTPUTS_DIR which
    // is set by the test framework. This will ensure that the schema test
    // data is output is uploaded to sponge. It can be found in "artifacts".
    schema_data_out_dir = getenv(kSchemaDataDefaultEnvVar.data());
  }

  return absl::StrFormat("%s/%s", schema_data_out_dir,
                         snapshot_->schema_version);
}

std::string SplitEngineTestBridgeSerializer::GetSnapshotDirRead() {
  return absl::StrFormat("file://%s/%s/%s/%s", ::testing::SrcDir(), "google3",
                         kSchemaTestDataDirectory, snapshot_->schema_version);
}

void SplitEngineTestBridgeSerializer::TakeSnapshot() {
  if (snapshot_->schema_version ==
      android_xr::kSplitEngineSchemaVersionCurrent) {
    CreateSnapshot();
  } else {
    PlaybackSnapshot();
  }
  snapshot_ = std::nullopt;
  snapshot_index_++;
}

void SplitEngineTestBridgeSerializer::CreateSnapshot() {
  snapshot_->fbb->Finish(android_xr::schemas::CreateBridgeSequence(
      *snapshot_->fbb,
      snapshot_->fbb->CreateVector(snapshot_->message_sequence.data(),
                                   snapshot_->message_sequence.size())));
  absl::Status result =
      SaveBinary(absl::StrCat(GetSnapshotDirWrite(), "_", GetDataFilename()),
                 snapshot_->fbb->GetBufferPointer(), snapshot_->fbb->GetSize());
  if (!result.ok()) {
    IMP_LOG(imp::FATAL) << "Failed to save message: " << result.ToString();
  }
}

void SplitEngineTestBridgeSerializer::PlaybackSnapshot() {
  std::string snapshot_filename =
      absl::StrCat(GetSnapshotDirRead(), "_", GetDataFilename());
  absl::StatusOr<resources::Resource> resource =
      testing::BaseExecutorTestHelper::MoveFuture(
          view_.GetAssetManager().LoadResource(snapshot_filename));
  if (!resource.ok()) {
    IMP_LOG(imp::FATAL) << "Failed to load " << snapshot_filename;
  }
  flatbuffers::Verifier verifier(
      reinterpret_cast<const uint8_t*>(resource->GetData().Data()),
      resource->GetData().Size());
  if (!verifier.VerifyBuffer<android_xr::schemas::BridgeSequence>()) {
    IMP_LOG(imp::FATAL) << "Failed to verify buffer";
  }

  const android_xr::schemas::BridgeSequence* sequence =
      flatbuffers::GetRoot<android_xr::schemas::BridgeSequence>(
          resource->GetData().Data());

  for (const android_xr::schemas::Message* message : *sequence->messages()) {
    std::vector<uint8_t> data(message->data()->begin(), message->data()->end());
    if (message->type() == android_xr::schemas::MessageType::Command) {
      absl::Status result = SendCommand(message->bridge_id(), data, false);
      if (!result.ok()) {
        IMP_LOG(imp::FATAL) << "Failed to send command: " << result.ToString();
      }
    } else if (message->type() == android_xr::schemas::MessageType::Request) {
      absl::Notification notification;
      absl::Status result = SendRequest(
          message->bridge_id(), data,
          [&notification](const std::vector<uint8_t>&) {
            notification.Notify();
          },
          false);
      if (!result.ok()) {
        IMP_LOG(imp::FATAL) << "Failed to send request: " << result.ToString();
      }
      // Must block until the request completes, otherwise the test will play
      // messages that may reference objects like materials that take time to
      // request.
      auto now = absl::Now();
      while (!notification.HasBeenNotified()) {
        Executor::ForegroundExecutor()->Pump(true);
        if (now - absl::Now() > absl::Seconds(60)) {
          IMP_LOG(imp::FATAL) << "PlaybackSnapshot timeout waiting for request.";
        }
      }
    }
  }
}

}  // namespace imp::split_engine
