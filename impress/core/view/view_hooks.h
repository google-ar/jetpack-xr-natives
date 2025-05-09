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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_VIEW_HOOKS_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_VIEW_HOOKS_H_

#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "filament/filament/include/filament/Renderer.h"
#include "core/common/invocable.h"
#include "core/math/vec.h"
#include "core/window/filament_host.h"
#include "core/window/window_rotation.h"

namespace imp {

struct ViewHooks {
  virtual ~ViewHooks() {}

  // Called by ViewHost when the host is created.
  virtual void OnHostCreated(window::FilamentHost* host) = 0;

  // Called by ViewHost::ViewState when the host is setting up. This happens
  // right after the filament engine is created.
  // If dev mode is enabled, additional_setup_function contains additional
  // functionality needed to set up the Impress Editor.
  // If dev mode is not enabled, additional_setup_function will be an empty
  // Invocable.
  virtual void OnHostSetup(
      uint2 dimensions,
      imp::Invocable<void(bool)> additional_setup_function) = 0;

  // Called by ViewHost::ViewState when the host is cleaning up.
  virtual void OnHostCleanup() = 0;

  // Called by ViewHost::ViewState when the host is rotating.
  virtual void OnHostSetDisplayRotation(window::WindowRotation rotation) = 0;

  // Called by ViewHost::ViewState when the host is resizing.
  virtual void OnHostResize(uint2 dimensions, uint4 margins,
                            float2 subpixel_ratio) = 0;

  // Called by ViewHost::ViewState each frame before updating.
  // last_vsync_time: platform timestamp for the previous updated frame.
  // next_vsync_time: target vsync timestamp the current frame will render. On
  // iOS is the target timestamp for the frame to be displayed, while on Android
  // it's the timestamp of the current frame in progress.
  // out_flags: out parameter to notify caller that frames should be skipped.
  // out_time_until_retry: out parameter used to communicate retry times, for
  //   e.g. when ARCore's next frame is not ready yet.
  // Note: frames may be skipped.
  // TODO Clean this up after decoupling the PreUpdate logic from
  // IsolatedPreRender
  virtual void OnHostPreUpdate(
      window::FilamentHost* host, absl::Duration last_vsync_time,
      absl::Duration next_vsync_time,
      window::FilamentHost::UpdateStageFlags* out_flags,
      absl::optional<absl::Duration>* out_time_until_retry) = 0;

  virtual void OnHostUpdate(
      window::FilamentHost* host, absl::Duration last_vsync_time,
      absl::Duration next_vsync_time,
      const window::FilamentHost::UpdateStageFlags& update_flags) = 0;

  virtual void OnHostPostUpdate() = 0;

  // Called by ViewHost::ViewState each frame before rendering.
  virtual void OnHostPreRender(window::FilamentHost* host) = 0;

  virtual void OnHostMultiPassRender() = 0;

  virtual void OnHostPostRender() = 0;

  // Called by ViewHost::ViewState to allow secondary views to render.
  virtual void OnHostSecondaryViewRender() = 0;

  // Called by ViewHost::ViewState after each frame.
  virtual void OnHostPostFrame() = 0;

  virtual void OnHostOffscreenRender(filament::Renderer* filament_renderer) = 0;

  // This function is called as the view becomes visible.
  virtual void Resume() = 0;

  // This function is called as the view becomes hidden.
  virtual void Pause() = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_VIEW_HOOKS_H_
