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

#include "core/config.h"

#if IMP_PLATFORM(ANDROID)

#include <android/native_window_jni.h>

#include "core/common/log.h"
#include "core/common/jni_helpers.h"
#include "core/editor/remote_editor/android_remote_editor_wrapper.h"

namespace {
template <class T>
T* FromJava(jlong n) {
  return imp::JniAllowlist<T, imp::AndroidRemoteEditorWrapper>::FromJava(n);
}
}  // namespace

#define JNI_VIDEOSTREAMER_METHOD(return_type, method_name) \
  IMP_JNI return_type JNICALL                              \
      Java_com_google_ar_imp_core_editor_RemoteEditorVideoStreamer_##method_name  // NOLINT

extern "C" {

// Sets the output native window for the remote editor UI.
// LINT.IfChange(nativeSetEditorUiRenderSurface)
JNI_VIDEOSTREAMER_METHOD(jboolean, nativeSetEditorUiRenderSurface)
(JNIEnv* env, jobject /*obj*/, jlong nativeRemoteEditorWrapperPtr,
 jobject surface, jint width, jint height) {
  // If wrapper pointer is null, it may have been destroyed during shutdown.
  // Return true to avoid JNI exceptions during graceful shutdown.
  if (!nativeRemoteEditorWrapperPtr) {
    return true;
  }
  auto* wrapper =
      FromJava<imp::AndroidRemoteEditorWrapper>(nativeRemoteEditorWrapperPtr);
  if (!wrapper) {
    return false;
  }

  if (!surface) {
    IMP_LOG(imp::ERROR) << "Surface is null in nativeSetEditorUiRenderSurface. Call "
                  "nativeReleaseEditorUiRenderSurface to clear.";
    return false;
  }

  ANativeWindow* nativeWindow = ANativeWindow_fromSurface(env, surface);
  if (!nativeWindow) {
    IMP_LOG(imp::ERROR) << "Failed to get ANativeWindow from surface";
    return false;
  }

  absl::Status status =
      wrapper->SetRenderTargetWindow(nativeWindow, width, height);
  if (!status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to set editor UI render target window: " << status;
    ANativeWindow_release(nativeWindow);
    return false;
  }
  return true;
}
// LINT.ThenChange(//depot/google3/third_party/impress/java/com/google/ar/imp/core/editor/RemoteEditorVideoStreamer.java:nativeSetEditorUiRenderSurface)

// Releases the output native window for the remote editor UI.
// LINT.IfChange(nativeReleaseEditorUiRenderSurface)
JNI_VIDEOSTREAMER_METHOD(jboolean, nativeReleaseEditorUiRenderSurface)
(JNIEnv* env, jobject /*obj*/, jlong nativeRemoteEditorWrapperPtr) {
  // If wrapper pointer is null, it may have been destroyed during shutdown.
  // Return true to avoid JNI exceptions during graceful shutdown.
  if (!nativeRemoteEditorWrapperPtr) {
    return true;
  }
  auto* wrapper =
      FromJava<imp::AndroidRemoteEditorWrapper>(nativeRemoteEditorWrapperPtr);
  if (!wrapper) {
    return false;
  }
  wrapper->ClearRenderTargetWindow();
  return true;
}
// LINT.ThenChange(//depot/google3/third_party/impress/java/com/google/ar/imp/core/editor/RemoteEditorVideoStreamer.java:nativeReleaseEditorUiRenderSurface)

// Notifies the remote editor that a client has connected.
// LINT.IfChange(nativeOnClientConnected)
JNI_VIDEOSTREAMER_METHOD(jboolean, nativeOnClientConnected)
(JNIEnv* env, jobject /*obj*/, jlong nativeRemoteEditorWrapperPtr) {
  // If wrapper pointer is null, it may have been destroyed during shutdown.
  // Return true to avoid JNI exceptions during graceful shutdown.
  if (!nativeRemoteEditorWrapperPtr) {
    return true;
  }
  auto* wrapper =
      FromJava<imp::AndroidRemoteEditorWrapper>(nativeRemoteEditorWrapperPtr);
  if (!wrapper) {
    return false;
  }
  wrapper->NotifyClientConnected();
  return true;
}
// LINT.ThenChange(//depot/google3/third_party/impress/java/com/google/ar/imp/core/editor/RemoteEditorVideoStreamer.java:nativeOnClientConnected)

// Notifies the remote editor that a client has disconnected.
// LINT.IfChange(nativeOnClientDisconnected)
JNI_VIDEOSTREAMER_METHOD(jboolean, nativeOnClientDisconnected)
(JNIEnv* env, jobject /*obj*/, jlong nativeRemoteEditorWrapperPtr) {
  // If wrapper pointer is null, it may have been destroyed during shutdown.
  // Return true to avoid JNI exceptions during graceful shutdown.
  if (!nativeRemoteEditorWrapperPtr) {
    return true;
  }
  auto* wrapper =
      FromJava<imp::AndroidRemoteEditorWrapper>(nativeRemoteEditorWrapperPtr);
  if (!wrapper) {
    return false;
  }
  wrapper->NotifyClientDisconnected();
  return true;
}
// LINT.ThenChange(//depot/google3/third_party/impress/java/com/google/ar/imp/core/editor/RemoteEditorVideoStreamer.java:nativeOnClientDisconnected)

}  // extern "C"

#endif  // IMP_PLATFORM(ANDROID)
