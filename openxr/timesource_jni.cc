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

#include "openxr/openxr_manager.h"

static jlong NativeGetXrTimeNow(JNIEnv* env) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  return xr_manager.GetXrTimeNow();
}

extern "C" {
JNIEXPORT jlong JNICALL
Java_androidx_xr_openxr_OpenXrTimeSource_nativeGetXrTimeNow(JNIEnv* env,
                                                            jobject /*clazz*/) {
  return NativeGetXrTimeNow(env);
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_runtime_openxr_OpenXrTimeSource_nativeGetXrTimeNow(
    JNIEnv* env, jobject /*clazz*/) {
  return NativeGetXrTimeNow(env);
}
}  // extern "C"
