// Copyright 2024 Google LLC
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

#include "core/monitor/frame_loop_watcher.h"

#include "core/monitor/monitor_helpers.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/view/base_view.h"
#include "core/view/view_events.h"

namespace imp {

FrameLoopWatcher::FrameLoopWatcher(BaseView& view)
    : view_(view),
      monitor_(*view.GetMonitor()),
      frame_presented_interval_(&monitor_, kFramePresented) {
  post_render_connection_ = view.GetDispatcher().Connect(
      [this](const ViewPostRenderEvent&) { OnViewPostRender(); });
}

void FrameLoopWatcher::OnViewPostRender() {
  // This frame was actually presented, if a frame was in progress, end it and
  // start the next one.
  if (frame_presented_interval_.IsInProgress()) {
    frame_presented_interval_.EndSample();
  }

  frame_presented_interval_.BeginSample();
}

}  // namespace imp
