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

#include "common/pointer_util.h"
#include "openxr/openxr_manager.h"

static jlong NativeGetPointer(JNIEnv* env) {
  return androidx::xr::common::PointerToJLong(
      &androidx::xr::openxr::OpenXrManager::GetOpenXrManager());
}

static jboolean NativeInit(JNIEnv* env, jobject activity) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  return xr_manager.Init(env, activity);
}

static void NativeDeInit(JNIEnv* env) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  xr_manager.DeInit(/*stop_polling_thread=*/true);
}

static jboolean NativePause(JNIEnv* env) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  return xr_manager.PauseSession();
}

extern "C" {
JNIEXPORT jlong JNICALL Java_androidx_xr_openxr_OpenXrManager_nativeGetPointer(
    JNIEnv* env, jclass /*clazz*/) {
  return NativeGetPointer(env);
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_runtime_openxr_OpenXrManager_nativeGetPointer(
    JNIEnv* env, jclass /*clazz*/) {
  return NativeGetPointer(env);
}

JNIEXPORT jboolean JNICALL Java_androidx_xr_openxr_OpenXrManager_nativeInit(
    JNIEnv* env, jclass /*clazz*/, jobject activity) {
  return NativeInit(env, activity);
}

JNIEXPORT jboolean JNICALL
Java_androidx_xr_runtime_openxr_OpenXrManager_nativeInit(JNIEnv* env,
                                                         jclass /*clazz*/,
                                                         jobject activity) {
  return NativeInit(env, activity);
}

JNIEXPORT void JNICALL Java_androidx_xr_openxr_OpenXrManager_nativeDeInit(
    JNIEnv* env, jclass /*clazz*/) {
  return NativeDeInit(env);
}

JNIEXPORT void JNICALL
Java_androidx_xr_runtime_openxr_OpenXrManager_nativeDeInit(JNIEnv* env,
                                                           jclass /*clazz*/) {
  return NativeDeInit(env);
}

JNIEXPORT jboolean JNICALL Java_androidx_xr_openxr_OpenXrManager_nativePause(
    JNIEnv* env, jclass /*clazz*/) {
  return NativePause(env);
}

JNIEXPORT jboolean JNICALL
Java_androidx_xr_runtime_openxr_OpenXrManager_nativePause(JNIEnv* env,
                                                          jclass /*clazz*/) {
  return NativePause(env);
}
}  // extern "C"
