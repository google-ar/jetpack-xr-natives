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

#include "core/media/android/android_exoplayer_listener.h"

#include <jni.h>

#include "core/common/jni_helpers.h"

namespace imp::media {

#define JNI_METHOD(return_type, class_name, method_name) \
  IMP_JNI return_type JNICALL                            \
      Java_com_google_ar_imp_core_media_##class_name##_##method_name

extern "C" {
// LINT.IfChange(ExoPlayerListener)
JNI_METHOD(void, ImpExoPlayerListener, nOnReady)
(JNIEnv* env, jclass /*clazz*/, jlong listener_handle) {
  return FromJava<AndroidExoPlayerListener>(listener_handle)->OnReady();
}

JNI_METHOD(void, ImpExoPlayerListener, nOnPlaybackComplete)
(JNIEnv* env, jclass /*clazz*/, jlong listener_handle) {
  return FromJava<AndroidExoPlayerListener>(listener_handle)
      ->OnPlaybackComplete();
}

JNI_METHOD(void, ImpExoPlayerListener, nOnSeekComplete)
(JNIEnv* env, jclass /*clazz*/, jlong listener_handle) {
  return FromJava<AndroidExoPlayerListener>(listener_handle)->OnSeekComplete();
}

JNI_METHOD(void, ImpExoPlayerListener, nOnBuffering)
(JNIEnv* env, jclass /*clazz*/, jlong listener_handle, jint buffering_state) {
  return FromJava<AndroidExoPlayerListener>(listener_handle)
      ->OnBuffering(buffering_state);
}
// LINT.ThenChange(
//     //depot/google3/third_party/impress/java/com/google/ar/imp/core/media/ImpExoPlayerListener.java:ExoPlayerListener
// )
}  // extern "C"
}  // namespace imp::media
