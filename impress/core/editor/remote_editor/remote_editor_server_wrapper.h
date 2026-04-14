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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_SERVER_WRAPPER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_SERVER_WRAPPER_H_

#include "absl/status/status.h"
#include "core/editor/remote_editor/remote_editor_config.h"

namespace imp {

// Abstract wrapper used by the RemoteEditorServer class.
// This allows for platform-specific implementations of the server.
class RemoteEditorServerWrapper {
 public:
  // Interface for handling render target window callbacks.
  class RenderTargetWindowCallback {
   public:
    virtual ~RenderTargetWindowCallback() = default;

    // Sets the render target window.
    virtual absl::Status SetRenderTargetWindow(void* native_window, int width,
                                               int height) = 0;

    // Clears the render target window.
    virtual void ClearRenderTargetWindow() = 0;
  };

  virtual ~RemoteEditorServerWrapper() = default;

  // Initializes the networking stack (HTTP and WebSockets). The implementation
  // must bind to the specified ports and store the provided callbacks to be
  // triggered asynchronously when native window lifecycle events occur.
  virtual void StartServer(
      const RemoteEditorConfig& config,
      RenderTargetWindowCallback& render_target_window_callback) = 0;

  // Gracefully terminates the HTTP and WebSocket connections and cleans up all
  // allocated networking resources.
  virtual void StopServer() = 0;

  // Performs platform-specific cleanup of the provided native window handle,
  // releasing its memory back to the OS.
  virtual void ReleaseNativeWindow(void* native_window) = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_SERVER_WRAPPER_H_
