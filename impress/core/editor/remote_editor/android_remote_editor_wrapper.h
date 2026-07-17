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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_ANDROID_REMOTE_EDITOR_WRAPPER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_ANDROID_REMOTE_EDITOR_WRAPPER_H_

#include <memory>

#include "absl/status/status.h"
#include "core/common/jni_helpers.h"
#include "core/view/base_view.h"

namespace imp {

class AndroidRemoteEditorScriptApiBridgeWrapper;

// Android JNI wrapper for the RemoteEditor. This class acts as a bidirectional
// bridge, integrating with the Java RemoteEditor implementation through JNI.
//
// C++ to Java: It provides C++ methods (like Start, Stop) that use cached
// JNI method IDs to invoke the corresponding methods on the Java object.
//
// Java to C++: It exposes methods (like SetRenderTargetWindow) that are
// called from Java via JNI entry points (in the _jni.cc files). These methods
// then route the events back into the core C++ engine via the Callback
// interface.
class AndroidRemoteEditorWrapper : public JavaWrapper {
 public:
  // Interface for AndroidRemoteEditorWrapper to make callbacks for client
  // connection events and render target changes.
  class Callback {
   public:
    virtual ~Callback() = default;

    // Sets the render target window.
    virtual absl::Status SetRenderTargetWindow(void* native_window, int width,
                                               int height) = 0;

    // Clears the render target window.
    virtual void ClearRenderTargetWindow() = 0;

    // Called for each client that connects.
    virtual void NotifyClientConnected() = 0;

    // Called for each client that disconnects.
    virtual void NotifyClientDisconnected() = 0;
  };

  explicit AndroidRemoteEditorWrapper(BaseView& view);
  ~AndroidRemoteEditorWrapper() override;

  // Parameters for starting the remote editor.
  struct Params {
    int http_port;
    int script_api_bridge_port;
    int ui_streaming_port;
  };

  // Initializes the networking stack (HTTP and WebSockets). The implementation
  // must bind to the specified ports and store the provided callbacks to be
  // triggered asynchronously when native window lifecycle events occur.
  void Start(const Params& params, Callback& callback);

  // Gracefully terminates the HTTP and WebSocket connections and cleans up all
  // allocated networking resources.
  void Stop();

  // Performs platform-specific cleanup of the provided native window handle,
  // releasing its memory back to the OS.
  void ReleaseNativeWindow(void* native_window);

  // Invoked via JNI from the Java layer to set the render target window.
  // Triggers the `SetRenderTargetWindow` callback.
  absl::Status SetRenderTargetWindow(void* native_window, int width,
                                     int height);

  // Invoked via JNI from the Java layer to clear the render target window.
  // Triggers the `ClearRenderTargetWindow` callback.
  void ClearRenderTargetWindow();

  // Invoked via JNI from the Java layer when a client connects. Triggers the
  // `NotifyClientConnected` callback.
  void NotifyClientConnected();

  // Invoked via JNI from the Java layer when a client disconnects. Triggers the
  // `NotifyClientDisconnected` callback.
  void NotifyClientDisconnected();

 private:
  // --- JNI components ---
  // Cached JNI method IDs for starting and stopping the remote editor.
  JniHandle start_method_;
  JniHandle stop_method_;
  JniHandle release_method_;
  JniHandle get_script_api_bridge_method_;

  // Callback invoked by the Java side when the native window
  // is created, changed, or destroyed.
  Callback* callback_ = nullptr;

  std::unique_ptr<AndroidRemoteEditorScriptApiBridgeWrapper>
      script_api_bridge_wrapper_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_ANDROID_REMOTE_EDITOR_WRAPPER_H_
