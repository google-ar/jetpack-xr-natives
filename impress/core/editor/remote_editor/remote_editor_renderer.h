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
#include "filament/filament/include/filament/Camera.h"
#include "filament/filament/include/filament/Scene.h"
#include "filament/filament/include/filament/SwapChain.h"
#include "filament/filament/include/filament/View.h"
#include "core/common/rememberer.h"
#include "core/materials/material.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/update_system.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"

namespace imp::editor {

// Renders the Impress editor UI to a specified native window.
//
// This class is an internal implementation detail of RemoteEditorServer and
// should not be used directly.
//
// This class manages an offscreen texture that receives the editor UI
// output. It then draws this texture onto a provided native window to
// facilitate video streaming of the UI.
class RemoteEditorRenderer : public UpdateSystem::Updater<RemoteEditorRenderer>,
                             public Rememberer {
 public:
  explicit RemoteEditorRenderer(BaseView& view);

  // RemoteEditorRenderer is not copyable or movable because it manages unique
  // resources (OwnedTexturePtr, OwnedMaterialPtr) and holds references to
  // view_ and owner_.
  RemoteEditorRenderer(const RemoteEditorRenderer&) = delete;
  RemoteEditorRenderer& operator=(const RemoteEditorRenderer&) = delete;

  ~RemoteEditorRenderer() override;

  // Updates UI streaming, capped at 30fps via kRenderTargetUpdateDelayMs to
  // minimize network bandwidth requirements.
  void Update(const FrameTime& frame_time) override;

  // Sets the remote window to draw the UI onto. Reallocates the internal
  // offscreen texture if the dimensions change. A valid, non-null
  // remote_window must be provided.
  void SetRenderTargetWindow(void* native_window, int width, int height);

  // Clears the remote window and releases rendering resources.
  void ClearRenderTargetWindow();

 private:
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

  // --- Impress managed resources ---
  // The offscreen texture representing the rendered UI.
  OwnedTexturePtr texture_;

  // Renders the offscreen UI texture.
  OwnedMaterialPtr ui_material_;

  // Node that displays the UI quad in the remote scene.
  NodeHandle ui_quad_node_;

  // --- Filament raw resources ---
  // TODO: Refactor to avoid using Filament directly.
  filament::SwapChain* remote_swap_chain_ = nullptr;
  filament::View* remote_view_ = nullptr;
  filament::Scene* ui_scene_ = nullptr;
  filament::Camera* ui_camera_ = nullptr;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_RENDERER_H_
