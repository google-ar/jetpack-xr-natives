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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_SPLIT_ENGINE_DESKTOP_BRIDGE_CLIENT_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_SPLIT_ENGINE_DESKTOP_BRIDGE_CLIENT_H_

#include <jni.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>  // NOLINT: Need to use threads available in bazel.
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "third_party/grpc/include/grpcpp/support/client_callback.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"
#include "core/split_engine/desktop/split_engine_desktop_bridge.grpc.pb.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

// Implementation of SplitEngineBridgeClient for Single Machine Desktop Split
// Engine infrastructure.
class SplitEngineDesktopBridgeClient
    : public SplitEngineSharedMemoryBridgeClient {
 public:
  static absl::StatusOr<std::unique_ptr<SplitEngineDesktopBridgeClient>> Create(
      std::unique_ptr<SplitEngineDesktopBridge::Stub> stub);

  ~SplitEngineDesktopBridgeClient() override;

  absl::StatusOr<std::unique_ptr<BufferHandle>> RegisterBuffer(
      int fd, size_t buffer_size_bytes) override;

  absl::Status ProcessRegion(const BufferHandle& buffer_handle,
                             int offset_bytes,
                             int region_length_bytes) override;

  absl::StatusOr<jobject> CreateExternalTextureSurface(
      const std::vector<TextureId>& in_texture_ids) override;

  absl::Status SetExternalTextureSurfaceSize(TextureId in_texture_id,
                                             int32_t width,
                                             int32_t height) override;

  absl::Status SendRequest(
      const std::vector<uint8_t>& data,
      std::function<void(const std::vector<uint8_t>&)> callback) override;

  ClientId GetClientId() const override { return client_id_; }

  MessageGroupId GenerateMessageGroupId() override {
    return ++next_message_group_id_;
  }

 private:
  explicit SplitEngineDesktopBridgeClient(
      std::unique_ptr<SplitEngineDesktopBridge::Stub> stub);
  absl::Status Initialize();

  const ClientId client_id_;
  MessageGroupId next_message_group_id_ = 0;

  std::unique_ptr<SplitEngineDesktopBridge::Stub> stub_;
  BridgeId bridge_id_;
  std::unique_ptr<grpc::ClientReadReactor<MessageGroupCompletionResponse>>
      message_group_completion_reactor_;
  std::thread heartbeat_thread_;

  enum class HeartbeatState {
    kRun,
    kStop,
  };
  absl::Mutex heartbeat_state_mutex_;
  HeartbeatState heartbeat_state_ ABSL_GUARDED_BY(heartbeat_state_mutex_) =
      HeartbeatState::kRun;
  void Heartbeat(absl::Duration heartbeat_interval);

  std::string uds_path_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_SPLIT_ENGINE_DESKTOP_BRIDGE_CLIENT_H_
