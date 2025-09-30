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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_MULTIMACHINE_SPLIT_ENGINE_DESKTOP_BRIDGE_SERVICE_IMPL_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_MULTIMACHINE_SPLIT_ENGINE_DESKTOP_BRIDGE_SERVICE_IMPL_H_

#include <memory>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "third_party/grpc/include/grpcpp/server_context.h"
#include "third_party/grpc/include/grpcpp/support/server_callback.h"
#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge.grpc.pb.h"
#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_service_impl_facade.h"
#include "core/split_engine/desktop/utils/split_engine_desktop_bridge_heartbeat.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

// The service shall support the following two use cases:
// 1. Multi-machine desktop only use-case
//
// Same story as single machine use case: service receives data and forwards it
// to the renderer.
//
// 2. Multi-machine desktop and mobile/AXR use-case.
//
// Service is running on the device and acts as a proxy. It receives data from
// the host and pushes it through the Binder to the renderer.
//
class SplitEngineMMDesktopBridgeService final
    : public SplitEngineMMDesktopBridge::CallbackService {
 public:
  SplitEngineMMDesktopBridgeService(
      std::unique_ptr<SplitEngineMMDesktopBridgeServiceImplFacade> impl);
  SplitEngineMMDesktopBridgeService(const SplitEngineMMDesktopBridgeService&) =
      delete;
  SplitEngineMMDesktopBridgeService& operator=(
      const SplitEngineMMDesktopBridgeService&) = delete;

  grpc::ServerUnaryReactor* Connect(grpc::CallbackServerContext* context,
                                    const ConnectRequest* request,
                                    ConnectResponse* response) override;

  grpc::ServerUnaryReactor* Heartbeat(grpc::CallbackServerContext* context,
                                      const HeartbeatRequest* request,
                                      google::rpc::Status* response) override;

  grpc::ServerUnaryReactor* SendRequest(grpc::CallbackServerContext* context,
                                        const SendRequestRequest* request,
                                        SendRequestResponse* response) override;

  grpc::ServerReadReactor<SendMessageGroupRequest>* SendMessageGroup(
      grpc::CallbackServerContext* context,
      SendMessageGroupResponse* response) override;

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

  std::unique_ptr<SplitEngineMMDesktopBridgeServiceImplFacade> impl_;
  HeartbeatMonitor heartbeat_monitor_;

  absl::Mutex bridge_id_mutex_;
  BridgeId new_bridge_id ABSL_GUARDED_BY(bridge_id_mutex_) = 1;

  // TODO: (broken link) - depending on the outcome of the task, bridge_id
  // generation may be updated
  BridgeId GetNewBridgeId();

  struct BridgeData {
    grpc::ServerWriteReactor<MessageGroupCompletionResponse>* reactor = nullptr;
  };

  absl::Mutex bridge_data_mutex_;
  absl::flat_hash_map<BridgeId, BridgeData> bridge_data_
      ABSL_GUARDED_BY(bridge_data_mutex_);

  void DestroyBridge(BridgeId bridge_id);
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_MULTIMACHINE_SPLIT_ENGINE_DESKTOP_BRIDGE_SERVICE_IMPL_H_
