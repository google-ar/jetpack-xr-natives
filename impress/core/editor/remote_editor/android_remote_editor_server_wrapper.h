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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_ANDROID_REMOTE_EDITOR_SERVER_WRAPPER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_ANDROID_REMOTE_EDITOR_SERVER_WRAPPER_H_

#include <memory>

#include "absl/status/status.h"
#include "core/common/jni_helpers.h"
#include "core/editor/remote_editor/android_remote_editor_script_api_bridge_wrapper.h"
#include "core/editor/remote_editor/remote_editor_config.h"
#include "core/editor/remote_editor/remote_editor_server_wrapper.h"
#include "core/view/base_view.h"

namespace imp {
namespace android {

// Android implementation of the RemoteEditorServerWrapper. This class
// integrates with the Java RemoteEditorServer implementation through JNI.
class AndroidRemoteEditorServerWrapper : public RemoteEditorServerWrapper,
                                         public JavaWrapper {
 public:
  explicit AndroidRemoteEditorServerWrapper(BaseView& view);
  ~AndroidRemoteEditorServerWrapper() override;

  void StartServer(
      const RemoteEditorConfig& config,
      RenderTargetWindowCallback& render_target_window_callback) override;

  void StopServer() override;

  void ReleaseNativeWindow(void* native_window) override;

  // Invoked by the Java side through JNI to set the render target window.
  // Validates and triggers the bound set_native_window callback.
  absl::Status SetRenderTargetWindow(void* native_window, int width,
                                     int height);

  // Invoked by the Java side through JNI to clear the render target window.
  // Triggers the bound clear_native_window callback.
  void ClearRenderTargetWindow();

 private:
  // --- JNI components ---
  // Cached JNI method IDs for starting and stopping the server.
  JniHandle start_server_method_;
  JniHandle stop_server_method_;
  JniHandle release_method_;
  JniHandle get_script_api_bridge_method_;

  // Callback invoked by the Java side when the native window
  // is created, changed, or destroyed.
  RenderTargetWindowCallback* render_target_window_callback_ = nullptr;

  std::unique_ptr<AndroidRemoteEditorScriptApiBridgeWrapper>
      script_api_bridge_wrapper_;
};

}  // namespace android
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_ANDROID_REMOTE_EDITOR_SERVER_WRAPPER_H_
