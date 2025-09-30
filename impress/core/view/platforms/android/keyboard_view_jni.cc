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

#include "core/view/platforms/android/android_key_helper.h"
#if defined(__ANDROID__)
#include <android/native_window_jni.h>
#endif  // defined(__ANDROID__)

#include "core/common/jni_helpers.h"
#include "core/view/framework/view.h"

#define JNI_METHOD(return_type, method_name) \
  IMP_JNI return_type JNICALL                \
      Java_com_google_ar_imp_core_input_KeyboardView_##method_name

namespace {

using ::imp::JniAllowlist;
using ::imp::View;

template <class T>
inline T* FromJava(jlong n) {
  return JniAllowlist<T, View>::FromJava(n);
}

}  // namespace


extern "C" {

JNI_METHOD(void, nProcessKeyboardEvent)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle, jint charCode,
 jint keyCode, jint action, jint modifiers) {
  auto* view = FromJava<View>(view_host_handle);

  imp::ProcessKeyboardEvent(view, charCode, keyCode, action, modifiers);
  }

}  // extern "C"
