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

#include <jni.h>

#include "core/common/jni_helpers.h"
#include "core/scripting/message_helpers.h"
#include "core/scripting/proto/bridge.proto.imp.h"
#include "core/scripting/web/android/web_view.h"

#define JNI_METHOD(return_type, method_name) \
  IMP_JNI return_type JNICALL                \
      Java_com_google_ar_imp_core_web_ImpWebViewBridge_##method_name

namespace {

using ::imp::JniAllowlist;
using ::imp::scripting::AndroidWebView;
using ::imp::scripting::MessageToNative;
using ::imp::scripting::ParseFromArray;

template <class T>
inline jlong ToJava(T* p) {
  return JniAllowlist<T, AndroidWebView>::ToJava(p);
}

template <class T>
inline T* FromJava(jlong n) {
  return JniAllowlist<T, AndroidWebView>::FromJava(n);
}

}  // namespace

extern "C" {

JNI_METHOD(void, nPostMessage)
(JNIEnv* env, jobject instance, jlong handle, jbyteArray array) {
  jbyte* bufferPtr = env->GetByteArrayElements(array, nullptr);
  jsize length = env->GetArrayLength(array);
  MessageToNative message = MessageToNative();
  if (ParseFromArray(bufferPtr, length, &message)) {
    FromJava<AndroidWebView>(handle)->HandleMessage(message);
  }
}

}  // extern "C"
