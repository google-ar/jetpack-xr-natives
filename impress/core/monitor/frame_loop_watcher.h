/*
 * Copyright 2024 Google LLC
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
#ifndef THIRD_PARTY_IMPRESS_CORE_MONITOR_FRAME_LOOP_WATCHER_H_
#define THIRD_PARTY_IMPRESS_CORE_MONITOR_FRAME_LOOP_WATCHER_H_

#include "core/monitor/duration_measurement.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/view/base_view.h"

namespace imp {

// Tracks timing for creation of impress frames.
//
// Output:
//   Populates the kFramePresented measurement in the monitor.

//
class FrameLoopWatcher {
 public:
  FrameLoopWatcher(BaseView& view);

 private:
  BaseView& view_;
  Monitor& monitor_;
  Dispatcher::ScopedConnection post_render_connection_;
  // Tracks FPS only counting frames to be presented.
  DurationMeasurement frame_presented_interval_;
  void OnViewPostRender();
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MONITOR_FRAME_LOOP_WATCHER_H_
