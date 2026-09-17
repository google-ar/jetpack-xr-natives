/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_BASE_REMOTE_EDITOR_RENDERER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_BASE_REMOTE_EDITOR_RENDERER_H_

#include "absl/base/thread_annotations.h"
#include "absl/synchronization/mutex.h"
#include "filament/filament/include/filament/Renderer.h"
#include "core/common/rememberer.h"

namespace imp::editor {

// Holds the active target native window pointer and its protective mutex lock.
struct ActiveWindowInfo {
  mutable absl::Mutex lock;
  void* active_window_ptr_ ABSL_GUARDED_BY(lock) = nullptr;
  mutable int in_flight_readbacks ABSL_GUARDED_BY(lock) = 0;
  mutable absl::CondVar readbacks_done;

  // Atomically clears the window pointer to skip future blits and waits for any
  // in-flight Filament GPU readback callbacks to finish before returning.
  // This acts as a teardown barrier to prevent use-after-free on disconnect.
  void RevokeWindowAndWait() {
    absl::MutexLock lock(this->lock);
    active_window_ptr_ = nullptr;
    while (in_flight_readbacks > 0) {
      readbacks_done.Wait(&this->lock);
    }
  }
};

// Abstract base class for rendering the Impress remote editor UI to a native
// window.
class BaseRemoteEditorRenderer : public Rememberer {
 public:
  virtual ~BaseRemoteEditorRenderer() = default;

  // Sets the native rendering target window and resized texture dimensions.
  virtual void SetRenderTargetWindow(void* native_window, int width,
                                     int height) = 0;

  // Clears the target window and releases rendering resources.
  virtual void ClearRenderTargetWindow() = 0;

  // Returns true if the renderer has an active target window or texture
  // surface configured for streaming.
  virtual bool IsReady() const = 0;

  // Asynchronously reads GPU pixels from render_target into a CPU buffer at
  // throttled intervals. Default implementation is a no-op for renderers that
  // do not require offscreen readback.
  virtual void ReadPixelsIfNeeded(filament::Renderer* renderer,
                                  filament::RenderTarget* render_target) {}
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_BASE_REMOTE_EDITOR_RENDERER_H_
