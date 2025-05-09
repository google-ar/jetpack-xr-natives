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

#if defined(__ANDROID__)
#include <android/native_window_jni.h>
#endif  // defined(__ANDROID__)

#include "core/common/context.h"
#include "core/common/jni_helpers.h"
#include "core/scripting/testing/scripting_test.h"
#define JNI_METHOD(return_type, method_name) \
  IMP_JNI return_type JNICALL                \
      Java_com_google_ar_imp_core_web_testing_ScriptingTest_##method_name

namespace {
using ::imp::Context;
using ::imp::JniAllowlist;
using ::imp::scripting::ScriptingTest;

template <class T>
inline jlong ToJava(T* p) {
  return JniAllowlist<T, ScriptingTest, Context>::ToJava(p);
}

template <class T>
inline T* FromJava(jlong n) {
  return JniAllowlist<T, ScriptingTest, Context>::FromJava(n);
}

}  // namespace

extern "C" {

JNI_METHOD(void, nInitTestView)
(JNIEnv* env, jclass /*clazz*/, jlong handle, jlong context_handle) {
  auto context = std::unique_ptr<Context>(FromJava<Context>(context_handle));
  FromJava<ScriptingTest>(handle)->InitTestView(std::move(context));
}

JNI_METHOD(void, nTearDownView)
(JNIEnv* env, jclass /*clazz*/, jlong handle) {
  FromJava<ScriptingTest>(handle)->TearDownView();
}

}  // extern "C"
