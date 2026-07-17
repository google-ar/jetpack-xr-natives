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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_ANDROID_WEB_UI_REMOTE_EDITOR_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_ANDROID_WEB_UI_REMOTE_EDITOR_H_

#include <memory>

#include "absl/status/status.h"
#include "core/async/executor.h"
#include "core/editor/editor_info.h"
#include "core/editor/remote_editor/remote_editor_info.h"
#include "core/editor/remote_editor/android_remote_editor_wrapper.h"
#include "core/editor/remote_editor/remote_editor_renderer.h"
#include "core/editor/remote_editor/web_ui_remote_editor.h"
#include "core/view/base_view.h"

namespace imp::editor {

// Android implementation of the WebUiRemoteEditor interface.
class AndroidWebUiRemoteEditor : public WebUiRemoteEditor,
                                 public AndroidRemoteEditorWrapper::Callback {
 public:
  explicit AndroidWebUiRemoteEditor(BaseView& view);
  ~AndroidWebUiRemoteEditor() override;

 protected:
  void OnWebUiStart() override;
  void OnWebUiStop() override;

 private:
  // Called via JNI to notify that a remote client has connected or
  // disconnected.
  void NotifyClientConnected();
  void NotifyClientDisconnected();

  // Sets the native rendering window and resizes the
  // internal offscreen texture. Called during the initial setup of the
  // rendering pipeline, and when the remote web UI requests a size change.
  // If called from a background thread, this method returns OkStatus and
  // asynchronously attempts setting the render target window on the
  // foreground thread.
  absl::Status SetRenderTargetWindow(void* native_window, int width,
                                     int height) override;

  // Signals that the native rendering window should be released so it can be
  // destroyed or reused by the system. This method marshals the call to the
  // foreground thread.
  void ClearRenderTargetWindow() override;

  // Internal helper to release the native window. This method does the actual
  // work of releasing the window and must be called on the foreground thread.
  void ReleaseNativeWindow();

  // Updates the internal renderer state based on the current configuration
  // (running state, mode, and native window availability). This will enable or
  // disable UI streaming as needed.
  void UpdateRemoteRenderingState();

  // --- Core dependencies ---
  // Executor for scheduling asynchronous tasks, if applicable.
  Executor* executor_ = nullptr;

  // Wrapper for managing the lifecycle of the underlying remote editor
  // connection.
  std::unique_ptr<AndroidRemoteEditorWrapper> android_remote_editor_wrapper_;

  // --- Rendering state ---
  // Renders the remote editor UI for streaming.
  std::unique_ptr<editor::RemoteEditorRenderer> remote_editor_renderer_;

  // The native window where the UI is rendered.
  void* native_window_ = nullptr;

  // Dimensions of the rendering target.
  int width_ = 0;
  int height_ = 0;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_ANDROID_WEB_UI_REMOTE_EDITOR_H_
