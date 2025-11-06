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

#include "core/split_engine/desktop/utils/split_engine_desktop_bridge_heartbeat.h"

#include <functional>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

HeartbeatMonitor::HeartbeatMonitor(
    absl::Duration check_interval, absl::Duration ttl,
    std::function<void(BridgeId)>&& on_bridge_disconnected)
    : check_interval_(check_interval),
      ttl_(ttl),
      on_bridge_disconnected_(std::move(on_bridge_disconnected)),
      monitor_thread_(&HeartbeatMonitor::Monitor, this) {}

HeartbeatMonitor::~HeartbeatMonitor() { TerminateMonitoring(); }

absl::Status HeartbeatMonitor::RegisterBridge(BridgeId bridge_id) {
  if (monitor_should_stop_) {
    return absl::FailedPreconditionError("Heartbeat monitor was terminated.");
  }
  absl::MutexLock lock(&heartbeat_mutex_);
  if (bridge_heartbeat_map_.contains(bridge_id)) {
    return absl::AlreadyExistsError(
        absl::StrCat("Bridge ", bridge_id, " already registered."));
  }
  bridge_heartbeat_map_[bridge_id] = absl::Now();
  return absl::OkStatus();
}

absl::Status HeartbeatMonitor::Heartbeat(BridgeId bridge_id) {
  if (monitor_should_stop_) {
    return absl::FailedPreconditionError("Heartbeat monitor was terminated.");
  }
  absl::MutexLock lock(&heartbeat_mutex_);
  if (!bridge_heartbeat_map_.contains(bridge_id)) {
    return absl::FailedPreconditionError(
        absl::StrCat("Bridge ", bridge_id, " not registered."));
  }
  bridge_heartbeat_map_[bridge_id] = absl::Now();
  return absl::OkStatus();
}

void HeartbeatMonitor::TerminateMonitoring() {
  if (monitor_should_stop_) {
    return;
  }

  {
    absl::MutexLock lock(&heartbeat_mutex_);
    monitor_should_stop_ = true;
  }

  if (monitor_thread_.joinable()) {
    monitor_thread_.join();
  }
}

void HeartbeatMonitor::Monitor() {
  absl::MutexLock lock(&heartbeat_mutex_);
  while (!heartbeat_mutex_.AwaitWithTimeout(
      absl::Condition(&monitor_should_stop_), check_interval_)) {
    const absl::Time now = absl::Now();
    for (auto it = bridge_heartbeat_map_.begin(),
              end = bridge_heartbeat_map_.end();
         it != end;) {
      auto copy_it = it++;
      const auto remaining_time = ttl_ - (now - copy_it->second);
      IMP_LOG(imp::ERROR) << "Bridge #" << copy_it->first
                  << "; time to live:" << remaining_time;
      if (remaining_time <= absl::ZeroDuration()) {
        const BridgeId bridge_id = copy_it->first;
        IMP_LOG(imp::WARNING) << "Bridge " << bridge_id << " got disconnected.";
        bridge_heartbeat_map_.erase(copy_it);
        on_bridge_disconnected_(bridge_id);
      }
    }
  }
  IMP_LOG(imp::ERROR) << "Heartbeat monitor thread stopped.";
}

}  // namespace imp::split_engine
