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

#include "core/editor/remote_editor/android_remote_editor_wrapper.h"

#include <android/native_window.h>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "core/async/executor.h"
#include "core/common/jni_helpers.h"
#include "core/editor/remote_editor/android_remote_editor_script_api_bridge_wrapper.h"

namespace imp {

namespace {
template <class T>
jlong ToJava(T* p) {
  return JniAllowlist<T, BaseView, Executor, AndroidRemoteEditorWrapper,
                      AndroidRemoteEditorScriptApiBridgeWrapper>::ToJava(p);
}
}  // namespace

AndroidRemoteEditorWrapper::AndroidRemoteEditorWrapper(BaseView& view)
    : JavaWrapper(view.GetContext().GetJniEnv(),
                  "com/google/ar/imp/core/editor/RemoteEditor",
                  "(Landroid/content/Context;JJ)V",
                  view.GetContext().GetActivityContext(), ToJava(&view),
                  ToJava(Executor::ForegroundExecutor())) {
  // LINT.IfChange(remote_editor)
  start_method_ = GetMethodHandle("start", "(JJIII)V");
  stop_method_ = GetMethodHandle("stop", "()V");
  release_method_ = GetMethodHandle("release", "()V");
  get_script_api_bridge_method_ = GetMethodHandle(
      "getScriptApiBridge",
      "()Lcom/google/ar/imp/core/editor/RemoteEditorScriptApiBridge;");
  // LINT.ThenChange(//depot/google3/third_party/impress/java/com/google/ar/imp/core/editor/RemoteEditor.java:remote_editor)
}

AndroidRemoteEditorWrapper::~AndroidRemoteEditorWrapper() {
  CallVoidMethod(release_method_);
}

void AndroidRemoteEditorWrapper::Start(const Params& params,
                                       Callback& callback) {
  callback_ = &callback;
  // Create the ScriptApiBridge wrapper once here
  JNIEnv* env = Env();
  JniUniquePtr<jobject> script_api_bridge_ref =
      CallObjectMethod(get_script_api_bridge_method_);

  if (script_api_bridge_ref) {
    script_api_bridge_wrapper_ =
        std::make_unique<AndroidRemoteEditorScriptApiBridgeWrapper>(
            env, std::move(script_api_bridge_ref), callback_);
  }

  CallVoidMethod(start_method_, ToJava(this),
                 ToJava(script_api_bridge_wrapper_.get()), params.http_port,
                 params.script_api_bridge_port, params.ui_streaming_port);
}

void AndroidRemoteEditorWrapper::Stop() { CallVoidMethod(stop_method_); }

void AndroidRemoteEditorWrapper::ReleaseNativeWindow(void* native_window) {
  if (native_window) {
    ANativeWindow_release(static_cast<ANativeWindow*>(native_window));
  }
}

absl::Status AndroidRemoteEditorWrapper::SetRenderTargetWindow(
    void* native_window, int width, int height) {
  return callback_
             ? callback_->SetRenderTargetWindow(native_window, width, height)
             : absl::OkStatus();
}

void AndroidRemoteEditorWrapper::ClearRenderTargetWindow() {
  if (callback_) {
    callback_->ClearRenderTargetWindow();
  }
}

void AndroidRemoteEditorWrapper::NotifyClientConnected() {
  if (callback_) {
    callback_->NotifyClientConnected();
  }
}

void AndroidRemoteEditorWrapper::NotifyClientDisconnected() {
  if (callback_) {
    callback_->NotifyClientDisconnected();
  }
}

}  // namespace imp
