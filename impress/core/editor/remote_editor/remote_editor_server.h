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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_SERVER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_SERVER_H_

#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/async/executor.h"
#include "core/common/rememberer.h"
#include "core/editor/remote_editor/remote_editor_config.h"
#include "core/editor/remote_editor/remote_editor_renderer.h"
#include "core/editor/remote_editor/remote_editor_server_wrapper.h"
#include "core/view/base_view.h"

namespace imp {

// Registry service that starts HTTP and WebSocket servers to allow connecting a
// remote editor client (browser-based).
//
// It serves web files from the application assets (assuming an index.html
// file is present). The WebSocket server communicates with the remote
// editor client, relaying messages back and forth between the client and
// the Impress scripting API.
//
// In kUiStreaming mode, it will also stream the editor UI to the
// remote client.
class RemoteEditorServer
    : public Rememberer,
      public RemoteEditorServerWrapper::RenderTargetWindowCallback {
 public:
  using Mode = RemoteEditorMode;
  using Config = RemoteEditorConfig;

  // Creates and starts the Remote Editor Server.
  // This will also find and enable the view's local Editor component (if
  // present) so that the editor UI is rendered offscreen and can be streamed to
  // the remote client.
  static absl::StatusOr<std::unique_ptr<RemoteEditorServer>> Create(
      BaseView& view, const Config& config);

  ~RemoteEditorServer();

  // Starts the remote editor server using the current configuration.
  void Start();

  // Starts the remote editor server with the given configuration.
  // If the server is already running with different settings, it will be
  // stopped and restarted with the new configuration.
  void Start(const Config& config);

  // Stops the remote editor server.
  void Stop();

  // Returns true if the server is currently running.
  bool IsRunning() const { return running_; }

  // Gets the current configuration of the remote editor.
  const Config& GetConfig() const { return config_; }

 private:
  explicit RemoteEditorServer(BaseView& view, const Config& config);

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
  // The main view this server is streaming from.
  BaseView& view_;

  // Executor for scheduling asynchronous tasks, if applicable.
  Executor* executor_ = nullptr;

  // --- Server configuration & state ---
  Config config_;

  // Indicates whether the underlying servers have been started.
  bool running_ = false;

  // Wrapper for managing the lifecycle of the underlying remote server
  // connection.
  std::unique_ptr<RemoteEditorServerWrapper> remote_editor_server_wrapper_;

  // --- Rendering state ---
  // Renders the remote editor UI for streaming.
  std::unique_ptr<editor::RemoteEditorRenderer> remote_editor_renderer_;

  // The native window where the UI is rendered.
  void* native_window_ = nullptr;

  // Dimensions of the rendering target.
  int width_ = 0;
  int height_ = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_SERVER_H_
