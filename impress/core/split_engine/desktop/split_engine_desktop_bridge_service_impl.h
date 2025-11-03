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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_SPLIT_ENGINE_DESKTOP_BRIDGE_IMPL_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_SPLIT_ENGINE_DESKTOP_BRIDGE_IMPL_H_

#include <memory>
#include <string>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "third_party/grpc/include/grpcpp/server_context.h"
#include "third_party/grpc/include/grpcpp/support/server_callback.h"
#include "core/async/executor.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_service_impl.h"
#include "core/split_engine/desktop/split_engine_desktop_bridge.grpc.pb.h"
#include "core/split_engine/desktop/utils/split_engine_desktop_bridge_heartbeat.h"
#include "core/split_engine/desktop/utils/split_engine_desktop_bridge_utils.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/view/base_view.h"

namespace imp::split_engine {

// Implementation of gRPC service for Single Machine Desktop Split Engine
// infrastructure.
class SplitEngineDesktopBridgeServiceImpl final
    : public SplitEngineDesktopBridge::CallbackService {
 public:
  SplitEngineDesktopBridgeServiceImpl(BaseView& view, Executor& executor);
  SplitEngineDesktopBridgeServiceImpl(
      const SplitEngineDesktopBridgeServiceImpl&) = delete;
  SplitEngineDesktopBridgeServiceImpl& operator=(
      const SplitEngineDesktopBridgeServiceImpl&) = delete;

  grpc::ServerUnaryReactor* InitializeBridge(
      grpc::CallbackServerContext* context,
      const InitializeBridgeRequest* request,
      InitializeBridgeResponse* response) override;

  grpc::ServerUnaryReactor* Heartbeat(grpc::CallbackServerContext* context,
                                      const HeartbeatRequest* request,
                                      google::rpc::Status* response) override;

  grpc::ServerUnaryReactor* RegisterBuffer(
      grpc::CallbackServerContext* context,
      const RegisterBufferRequest* request,
      RegisterBufferResponse* response) override;

  grpc::ServerUnaryReactor* ProcessRegion(
      grpc::CallbackServerContext* context, const ProcessRegionRequest* request,
      google::rpc::Status* response) override;

  grpc::ServerUnaryReactor* SendRequest(grpc::CallbackServerContext* context,
                                        const SendRequestRequest* request,
                                        SendRequestResponse* response) override;

  grpc::ServerWriteReactor<MessageGroupCompletionResponse>*
  ReadMessageGroupCompletions(
      grpc::CallbackServerContext* context,
      const MessageGroupCompletionRequest* request) override;

 private:
  // Service will tell client to send heartbeat every kHeartbeatInterval.
  static constexpr absl::Duration kHeartbeatInterval = absl::Milliseconds(300);
  // Service will check when was the last heartbeat for each bridge every
  // kPulseCheckInterval. If the last heartbeat is older than kTTL, the bridge
  // will be considered dead and will be removed from the service.
  static constexpr absl::Duration kPulseCheckInterval = absl::Seconds(1);
  static constexpr absl::Duration kTTL = absl::Seconds(1);

  BaseView& view_;
  Executor& foreground_executor_;
  SplitEngineSharedMemoryBridgeServiceImpl impl_;
  HeartbeatMonitor heartbeat_monitor_;

  absl::Mutex bridge_id_mutex_;
  BridgeId new_bridge_id ABSL_GUARDED_BY(bridge_id_mutex_) = 1;

  // TODO: (broken link) - Depending on the outcome of the task, bridge_id
  // generation might need protection from brute-forcing
  BridgeId GenerateBridgeId() {
    absl::MutexLock lock(bridge_id_mutex_);
    return new_bridge_id++;
  }

  absl::Mutex buffer_id_mutex_;
  BridgeId new_buffer_id ABSL_GUARDED_BY(buffer_id_mutex_) = 1;

  BridgeId GetNewBufferId() {
    absl::MutexLock lock(buffer_id_mutex_);
    return new_buffer_id++;
  }

  struct BridgeData {
    grpc::ServerWriteReactor<MessageGroupCompletionResponse>* reactor;
  };

  absl::Mutex bridge_data_mutex_;
  absl::flat_hash_map<BridgeId, BridgeData> bridge_data_
      ABSL_GUARDED_BY(bridge_data_mutex_);

  std::string uds_path_;
  std::unique_ptr<FileDescriptorReceiver> fd_receiver_;
  absl::Mutex fd_metadata_mutex_;
  absl::flat_hash_map<BridgeId,
                      absl::flat_hash_map<int, FileDescriptorMetadata>>
      fd_metadata_map_ ABSL_GUARDED_BY(fd_metadata_mutex_);
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_SPLIT_ENGINE_DESKTOP_BRIDGE_IMPL_H_
