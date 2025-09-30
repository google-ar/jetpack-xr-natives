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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_UTILS_SPLIT_ENGINE_DESKTOP_BRIDGE_HEARTBEAT_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_UTILS_SPLIT_ENGINE_DESKTOP_BRIDGE_HEARTBEAT_H_

#include <concepts>
#include <functional>
#include <thread>  // NOLINT: Need to use threads available in bazel.

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

// Utility class to monitor heartbeats from a set of bridges.
class HeartbeatMonitor {
 public:
  // `check_interval`: how often to check reported heartbeats.
  //
  // `ttl`: if heartbeat is not received after this amount of time, the
  // connection with the bridge is presumed to be broken and
  // `on_bridge_disconnected` is called.
  //
  // `on_bridge_disconnected`: called when connection with a bridge is presumed
  // to be broken.
  HeartbeatMonitor(absl::Duration check_interval, absl::Duration ttl,
                   std::function<void(BridgeId)>&& on_bridge_disconnected);
  ~HeartbeatMonitor();

  // Register a bridge to be monitored.
  absl::Status RegisterBridge(BridgeId bridge_id);

  // Called by a service to indicate that heartbeat was received for the bridge.
  absl::Status Heartbeat(BridgeId bridge_id);

  // Stops the monitoring thread.
  void TerminateMonitoring();

 private:
  const absl::Duration check_interval_;
  const absl::Duration ttl_;
  const std::function<void(BridgeId)> on_bridge_disconnected_;

  absl::Mutex heartbeat_mutex_;
  bool monitor_should_stop_ = false;
  absl::flat_hash_map<BridgeId, absl::Time> bridge_heartbeat_map_
      ABSL_GUARDED_BY(heartbeat_mutex_);
  std::thread monitor_thread_;

  void Monitor();
};
}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_UTILS_SPLIT_ENGINE_DESKTOP_BRIDGE_HEARTBEAT_H_
