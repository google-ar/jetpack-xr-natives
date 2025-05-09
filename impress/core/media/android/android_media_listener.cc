// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/media/android/android_media_listener.h"

namespace imp::media {

#define JNI_METHOD(return_type, class_name, method_name) \
  IMP_JNI return_type JNICALL                            \
      Java_com_google_ar_imp_core_media_##class_name##_##method_name

extern "C" {
// LINT.IfChange(OnCompletionJni)
JNI_METHOD(void, OnCompletionListener, nOnCompletion)
(JNIEnv* env, jclass /*clazz*/, jlong listener_handle) {
  return FromJava<AndroidMediaListener>(listener_handle)->OnPlaybackComplete();
}
// LINT.ThenChange(
//     //depot/google3/third_party/impress/java/com/google/ar/imp/core/media/OnCompletionListener.java:OnCompletionJni
// )

// LINT.IfChange(OnSeekCompleteJni)
JNI_METHOD(void, OnSeekCompleteListener, nSeekComplete)
(JNIEnv* env, jclass /*clazz*/, jlong listener_handle) {
  return FromJava<AndroidMediaListener>(listener_handle)->OnSeekComplete();
}
// LINT.ThenChange(
//     //depot/google3/third_party/impress/java/com/google/ar/imp/core/media/OnSeekCompleteListener.java:OnSeekCompleteJni
// )

// LINT.IfChange(OnPreparedJni)
JNI_METHOD(void, OnPreparedListener, nOnPrepared)
(JNIEnv* env, jclass /*clazz*/, jlong listener_handle) {
  return FromJava<AndroidMediaListener>(listener_handle)->OnReady();
}
// LINT.ThenChange(
//     //depot/google3/third_party/impress/java/com/google/ar/imp/core/media/OnCompletionListener.java:OnPreparedJni
// )

// LINT.IfChange(OnErrorJni)
JNI_METHOD(bool, OnErrorListener, nOnError)
(JNIEnv* env, jclass /*clazz*/, jlong listener_handle, jint what, jint extra) {
  return FromJava<AndroidMediaListener>(listener_handle)->OnError(what, extra);
}
// LINT.ThenChange(
//     //depot/google3/third_party/impress/java/com/google/ar/imp/core/media/OnCompletionListener.java:OnErrorJni
// )

// LINT.IfChange(OnInfoJni)
JNI_METHOD(bool, OnInfoListener, nOnInfo)
(JNIEnv* env, jclass /*clazz*/, jlong listener_handle, jint what, jint extra) {
  return FromJava<AndroidMediaListener>(listener_handle)->OnInfo(what, extra);
}
// LINT.ThenChange(
//     //depot/google3/third_party/impress/java/com/google/ar/imp/core/media/OnCompletionListener.java:OnInfoJni
// )

}  // extern "C"
}  // namespace imp::media
