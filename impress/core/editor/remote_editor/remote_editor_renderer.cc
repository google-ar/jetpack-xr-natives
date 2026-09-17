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

#include "core/editor/remote_editor/remote_editor_renderer.h"

#include <cstdint>

#include "core/common/log.h"
#include "absl/time/time.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/SwapChain.h"
#include "filament/filament/include/filament/View.h"
#include "filament/filament/include/filament/Viewport.h"
#include "core/editor/remote_editor/remote_editor_info.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"
#include "core/view/view_events.h"
#include "core/window/filament_host.h"
#include "core/window/filament_view.h"
#include "core/window/imgui_renderer.h"

namespace imp::editor {

RemoteEditorRenderer::RemoteEditorRenderer(BaseView& view) : view_(view) {
  window::FilamentHost::DevModeExtension* extension =
      view_.GetHost()->TryGetExtension();
  if (extension) {
    imgui_renderer_ = extension->GetImGuiRenderer();
  }
  view_.GetDispatcher().Connect(
      [this](const ViewSecondaryRenderEvent& /*event*/) {
        OnSecondaryRender();
      },
      this);
}

RemoteEditorRenderer::~RemoteEditorRenderer() { ClearRenderTargetWindow(); }

void RemoteEditorRenderer::ClearRenderTargetWindow() {
  window_generation_++;
  filament::Engine* engine = view_.GetHost()->GetEngine();

  const bool use_ogl_external_texture =
      (view_.GetSplitEngineSerializer() != nullptr);
  if (use_ogl_external_texture) {
    if (imgui_renderer_) {
      imgui_renderer_->SetRenderTargetWindow(nullptr);
    }
  }

  if (remote_swap_chain_) {
    engine->destroy(
        static_cast<const filament::SwapChain*>(remote_swap_chain_));
    remote_swap_chain_ = nullptr;
  }
  remote_window_ = nullptr;
  width_ = 0;
  height_ = 0;
}

void RemoteEditorRenderer::OnSecondaryRender() {
  time_since_last_update_ += view_.GetFrameTime().GetDeltaTime();
  if (time_since_last_update_ <
      absl::Milliseconds(RemoteEditorInfo::RemoteEditorStreamingConfig::
                             kDefaultUiStreamingUpdateDelayMs)) {
    return;
  }

  if (!remote_swap_chain_) return;

  filament::Renderer* renderer = view_.GetHost()->GetRenderer();
  window::FilamentHost::DevModeExtension* extension =
      view_.GetHost()->TryGetExtension();
  window::detail::FilamentView* ui_view =
      extension ? extension->GetUiView() : nullptr;

  if (renderer && ui_view && remote_swap_chain_) {
    filament::View* ui_view_filament = ui_view->Get();
    const filament::Viewport original_viewport =
        ui_view_filament->getViewport();

    // Dynamically override viewport to match remote size and avoid
    // out-of-bounds crash in GLES driver
    ui_view_filament->setViewport(
        {0, 0, static_cast<uint32_t>(width_), static_cast<uint32_t>(height_)});

    if (renderer->beginFrame(remote_swap_chain_)) {
      renderer->render(ui_view_filament);
      renderer->endFrame();
    }

    // Restore the viewport back
    ui_view_filament->setViewport(original_viewport);
  }

  time_since_last_update_ %=
      absl::Milliseconds(RemoteEditorInfo::RemoteEditorStreamingConfig::
                             kDefaultUiStreamingUpdateDelayMs);
}

void RemoteEditorRenderer::SetRenderTargetWindow(void* native_window, int width,
                                                 int height) {
  if (!native_window) {
    IMP_LOG(imp::ERROR) << "remote_window must be non-null.";
    return;
  }
  if (width <= 0 || height <= 0) {
    IMP_LOG(imp::ERROR) << "Ignoring invalid dimensions with valid window.";
    return;
  }

  if (remote_window_ == native_window && width_ == width && height_ == height) {
    return;
  }

  const bool use_ogl_external_texture =
      (view_.GetSplitEngineSerializer() != nullptr);

  if (use_ogl_external_texture) {
    if (imgui_renderer_) {
      imgui_renderer_->SetRenderTargetWindow(native_window);
      imgui_renderer_->SetRenderTargetDisplaySize(width, height, 1.0f, 1.0f,
                                                  false);
    }
    remote_window_ = native_window;
    width_ = width;
    height_ = height;
    // When the streaming target window coordinates/dimensions are bound by
    // MediaCodec on Android phone/desktop, explicitly notify the dev mode
    // extension so it can synchronize project viewports prior to desktop
    // layout instantiation via SetDisplayMode.
    if (window::FilamentHost::DevModeExtension* extension =
            view_.GetHost()->TryGetExtension()) {
      extension->UpdateCameraAndViewport(
          {static_cast<uint32_t>(width), static_cast<uint32_t>(height)},
          {1.0f, 1.0f});
    }
    return;
  }

  ClearRenderTargetWindow();

  remote_window_ = native_window;
  width_ = width;
  height_ = height;

  filament::Engine* engine = view_.GetHost()->GetEngine();
  remote_swap_chain_ = engine->createSwapChain(native_window);

  if (imgui_renderer_) {
    imgui_renderer_->SetRenderTargetDisplaySize(width, height, 1.0f, 1.0f,
                                                false);
  }
  // When the streaming target window coordinates/dimensions are bound by
  // MediaCodec on Android phone/desktop, explicitly notify the dev mode
  // extension so it can synchronize project viewports prior to desktop
  // layout instantiation via SetDisplayMode.
  if (window::FilamentHost::DevModeExtension* extension =
          view_.GetHost()->TryGetExtension()) {
    extension->UpdateCameraAndViewport(
        {static_cast<uint32_t>(width), static_cast<uint32_t>(height)},
        {1.0f, 1.0f});
  }
}

bool RemoteEditorRenderer::IsReady() const {
  return remote_window_ != nullptr && width_ > 0 && height_ > 0;
}

}  // namespace imp::editor
