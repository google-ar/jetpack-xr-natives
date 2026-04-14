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
#include "core/editor/remote_editor/android_remote_editor_server_wrapper.h"

namespace {
template <class T>
T* FromJava(jlong n) {
  return imp::JniAllowlist<
      T, imp::android::AndroidRemoteEditorServerWrapper>::FromJava(n);
}
}  // namespace

#define JNI_VIDEOSTREAMER_METHOD(return_type, method_name) \
  IMP_JNI return_type JNICALL                              \
      Java_com_google_ar_imp_core_editor_RemoteEditorVideoStreamer_##method_name  // NOLINT

extern "C" {

// Sets the output native window for the remote editor UI.
// LINT.IfChange(nativeSetEditorUiRenderSurface)
JNI_VIDEOSTREAMER_METHOD(void, nativeSetEditorUiRenderSurface)
(JNIEnv* env, jclass /*clazz*/, jlong nativeServerWrapperPtr, jobject surface,
 jint width, jint height) {
  if (!nativeServerWrapperPtr) {
    IMP_LOG(imp::ERROR) << "Wrapper pointer is null in nativeSetEditorUiRenderSurface";
    return;
  }
  auto* wrapper = FromJava<imp::android::AndroidRemoteEditorServerWrapper>(
      nativeServerWrapperPtr);

  if (!surface) {
    IMP_LOG(imp::ERROR) << "Surface is null in nativeSetEditorUiRenderSurface. Call "
                  "nativeReleaseEditorUiRenderSurface to clear.";
    return;
  }

  ANativeWindow* nativeWindow = ANativeWindow_fromSurface(env, surface);
  if (!nativeWindow) {
    IMP_LOG(imp::ERROR) << "Failed to get ANativeWindow from surface";
    return;
  }

  absl::Status status =
      wrapper->SetRenderTargetWindow(nativeWindow, width, height);
  if (!status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to set editor UI render target window: " << status;
    ANativeWindow_release(nativeWindow);
  }
}
// LINT.ThenChange(//depot/google3/third_party/impress/java/com/google/ar/imp/core/editor/RemoteEditorVideoStreamer.java:nativeSetEditorUiRenderSurface)

// Releases the output native window for the remote editor UI.
// LINT.IfChange(nativeReleaseEditorUiRenderSurface)
JNI_VIDEOSTREAMER_METHOD(void, nativeReleaseEditorUiRenderSurface)
(JNIEnv* env, jclass /*clazz*/, jlong nativeServerWrapperPtr) {
  if (!nativeServerWrapperPtr) {
    IMP_LOG(imp::ERROR)
        << "Wrapper pointer is null in nativeReleaseEditorUiRenderSurface";
    return;
  }
  auto* wrapper = FromJava<imp::android::AndroidRemoteEditorServerWrapper>(
      nativeServerWrapperPtr);
  wrapper->ClearRenderTargetWindow();
}
// LINT.ThenChange(//depot/google3/third_party/impress/java/com/google/ar/imp/core/editor/RemoteEditorVideoStreamer.java:nativeReleaseEditorUiRenderSurface)

}  // extern "C"

#endif  // IMP_PLATFORM(ANDROID)
