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

#include "core/view/view_events.h"

#include <functional>

#include "filament/filament/include/filament/Renderer.h"

namespace imp {

ViewPreFrameUpdateEvent::ViewPreFrameUpdateEvent(
    std::function<void(absl::optional<absl::Duration>)> skip_frame_fn)
    : skip_frame_fn_(skip_frame_fn) {}

void ViewPreFrameUpdateEvent::SkipFrame(
    absl::optional<absl::Duration> time_until_retry) const {
  if (skip_frame_fn_) {
    skip_frame_fn_(time_until_retry);
  }
}

ViewPreRenderEvent::ViewPreRenderEvent(filament::Renderer* filament_renderer)
    : filament_renderer_(filament_renderer) {}

filament::Renderer* ViewPreRenderEvent::GetRenderer() const {
  return filament_renderer_;
}

}  // namespace imp
