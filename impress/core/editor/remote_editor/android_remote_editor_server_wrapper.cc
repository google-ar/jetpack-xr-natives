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

#include "core/editor/remote_editor/android_remote_editor_server_wrapper.h"

#include <android/native_window.h>

#include "absl/status/status.h"
#include "core/async/executor.h"
#include "core/common/jni_helpers.h"

namespace imp {
namespace android {

namespace {
template <class T>
jlong ToJava(T* p) {
  return imp::JniAllowlist<
      T, imp::BaseView, imp::Executor, AndroidRemoteEditorServerWrapper,
      AndroidRemoteEditorScriptApiBridgeWrapper>::ToJava(p);
}
}  // namespace

AndroidRemoteEditorServerWrapper::AndroidRemoteEditorServerWrapper(
    BaseView& view)
    : JavaWrapper(view.GetContext().GetJniEnv(),
                  "com/google/ar/imp/core/editor/"
                  "RemoteEditorServer",
                  "(Landroid/content/Context;JJ)V",
                  view.GetContext().GetActivityContext(), ToJava(&view),
                  ToJava(Executor::ForegroundExecutor())) {
  // LINT.IfChange(remote_editor_server)
  start_server_method_ = GetMethodHandle("startServer", "(JJIIIZ)V");
  stop_server_method_ = GetMethodHandle("stopServer", "()V");
  release_method_ = GetMethodHandle("release", "()V");
  get_script_api_bridge_method_ = GetMethodHandle(
      "getScriptApiBridge",
      "()Lcom/google/ar/imp/core/editor/RemoteEditorScriptApiBridge;");
  // LINT.ThenChange(//depot/google3/third_party/impress/java/com/google/ar/imp/core/editor/RemoteEditorServer.java:remote_editor_server)
}

AndroidRemoteEditorServerWrapper::~AndroidRemoteEditorServerWrapper() {
  CallVoidMethod(release_method_);
}

void AndroidRemoteEditorServerWrapper::StartServer(
    const RemoteEditorConfig& config,
    RenderTargetWindowCallback& render_target_window_callback) {
  render_target_window_callback_ = &render_target_window_callback;

  // Create the ScriptApiBridge wrapper once here
  JNIEnv* env = Env();
  JniUniquePtr<jobject> server_ref =
      CallObjectMethod(get_script_api_bridge_method_);

  if (server_ref) {
    script_api_bridge_wrapper_ =
        std::make_unique<AndroidRemoteEditorScriptApiBridgeWrapper>(
            env, std::move(server_ref));
  }

  bool ui_streaming = (config.mode == RemoteEditorMode::kUiStreaming);

  CallVoidMethod(start_server_method_, ToJava(this),
                 ToJava(script_api_bridge_wrapper_.get()), config.http_port,
                 config.script_api_bridge_port, config.ui_streaming_port,
                 ui_streaming);
}

void AndroidRemoteEditorServerWrapper::StopServer() {
  CallVoidMethod(stop_server_method_);
}

void AndroidRemoteEditorServerWrapper::ReleaseNativeWindow(
    void* native_window) {
  if (native_window) {
    ANativeWindow_release(static_cast<ANativeWindow*>(native_window));
  }
}

absl::Status AndroidRemoteEditorServerWrapper::SetRenderTargetWindow(
    void* native_window, int width, int height) {
  return render_target_window_callback_
             ? render_target_window_callback_->SetRenderTargetWindow(
                   native_window, width, height)
             : absl::OkStatus();
}

void AndroidRemoteEditorServerWrapper::ClearRenderTargetWindow() {
  if (render_target_window_callback_) {
    render_target_window_callback_->ClearRenderTargetWindow();
  }
}

}  // namespace android
}  // namespace imp
