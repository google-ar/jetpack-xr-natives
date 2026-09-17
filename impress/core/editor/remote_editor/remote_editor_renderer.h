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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_RENDERER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_RENDERER_H_

#include "absl/time/time.h"
#include "filament/filament/include/filament/SwapChain.h"
#include "core/editor/remote_editor/base_remote_editor_renderer.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/update_system.h"
#include "core/view/base_view.h"

namespace imp::window {
class ImGuiRenderer;
}

namespace imp::editor {

// Renders the Impress editor UI to a specified native window.
//
// This class is an internal implementation detail of RemoteEditor and
// should not be used directly.
//
// This class manages rendering of the editor UI view directly onto the
// remote swap chain target window.
class RemoteEditorRenderer : public BaseRemoteEditorRenderer {
 public:
  explicit RemoteEditorRenderer(BaseView& view);

  RemoteEditorRenderer(const RemoteEditorRenderer&) = delete;
  RemoteEditorRenderer& operator=(const RemoteEditorRenderer&) = delete;

  ~RemoteEditorRenderer();

  // Sets the remote window to draw the UI onto. Reallocates the internal
  // offscreen texture if the dimensions change. A valid, non-null
  // remote_window must be provided.
  void SetRenderTargetWindow(void* native_window, int width,
                             int height) override;

  // Clears the remote window and releases rendering resources.
  void ClearRenderTargetWindow() override;

  bool IsReady() const override;

 private:
  // Renders UI streaming, capped at 30fps via kRenderTargetUpdateDelayMs to
  // minimize network bandwidth requirements.
  //
  // Note: This must execute during the secondary view rendering phase
  // (via ViewSecondaryRenderEvent) after ImGui has completed its PreRender
  // step for the current frame. Executing during standard update phases before
  // ImGui updates can cause stale scissor rectangles to be clipped against
  // newly resized viewports, triggering fatal assertion crashes in Filament.
  void OnSecondaryRender();

  // The main view this renderer is attached to.
  BaseView& view_;

  // --- Rendering state ---
  // Tracks the elapsed time since the last UI frame was drawn and streamed.
  absl::Duration time_since_last_update_ = absl::ZeroDuration();

  // A counter used for invalidation and lifetime management of asynchronous
  // operations and the rendering loop.
  int window_generation_ = 0;

  // The native window target we render onto.
  void* remote_window_ = nullptr;
  int width_ = 0;
  int height_ = 0;

  window::ImGuiRenderer* imgui_renderer_ = nullptr;

  // --- Filament raw resources ---
  // TODO: Refactor to avoid using Filament directly.
  filament::SwapChain* remote_swap_chain_ = nullptr;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_RENDERER_H_
