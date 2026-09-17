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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_XR_RENDERER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_XR_RENDERER_H_

#include <cstdint>
#include <memory>
#include <vector>

#include "absl/time/time.h"
#include "filament/filament/include/filament/Renderer.h"
#include "core/config.h"
#include "core/editor/remote_editor/base_remote_editor_renderer.h"
#include "core/ncsb/update_system.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"

namespace imp::editor {

// Renders the Impress editor UI to a specified native window for OpenXR.
//
// This class manages an offscreen texture that receives the editor UI
// output. It then initiates asynchronous PBO pixel readback to blit
// this texture onto a provided native window to facilitate video streaming
// without swap chain multiplexing driver contention.
class RemoteEditorXrRenderer
    : public BaseRemoteEditorRenderer,
      public UpdateSystem::Updater<RemoteEditorXrRenderer> {
 public:
  explicit RemoteEditorXrRenderer(BaseView& view,
                                  const ActiveWindowInfo& active_window_info);

  RemoteEditorXrRenderer(const RemoteEditorXrRenderer&) = delete;
  RemoteEditorXrRenderer& operator=(const RemoteEditorXrRenderer&) = delete;

  ~RemoteEditorXrRenderer() override;

#if IMP_RUNTIME(DEV)
  // UI streaming must continue updating time tracking and sending frames to the
  // remote client even when the editor pauses or edits the scene.
  bool ShouldRunInEditMode() const override { return true; }
#endif

  void Update(const FrameTime& frame_time) override;

  void SetRenderTargetWindow(void* native_window, int width,
                             int height) override;

  void ClearRenderTargetWindow() override;

  bool IsReady() const override;

  // Asynchronously reads GPU pixels from render_target into a CPU buffer at
  // throttled intervals. On completion, posts the RGBA data to the target
  // remote native window (e.g., feeding MediaCodec).
  void ReadPixelsIfNeeded(filament::Renderer* renderer,
                          filament::RenderTarget* render_target) override;

 private:
  BaseView& view_;
  const ActiveWindowInfo& active_window_info_;
  std::vector<uint8_t> readback_buffer_;

  absl::Duration time_since_last_update_ = absl::ZeroDuration();
  void* remote_window_ = nullptr;

  // --- Impress managed resources ---
  OwnedTexturePtr texture_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_XR_RENDERER_H_
