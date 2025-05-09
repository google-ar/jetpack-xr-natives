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

#ifdef __ANDROID__
#include <android/binder_ibinder.h>
#include <android/binder_ibinder_jni.h>
#endif  // __ANDROID__
#include <jni.h>
#include <openxr/openxr.h>

#include <cstdint>

#include "absl/log/log.h"
#include "openxr/jobject_converter.h"
#include "openxr/jobject_creator.h"
#include "openxr/openxr_manager.h"

static jobject NativeGetAnchorState(JNIEnv* env, jlong native_ptr,
                                    jlong monotonic_time_ns) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  XrSpace anchor_space = androidx::xr::openxr::ConvertToXrSpace(native_ptr);

  XrSpaceLocation anchor_location = {.type = XR_TYPE_SPACE_LOCATION,
                                     .next = nullptr};
  if (!xr_manager.GetAnchorLocationData(anchor_space,
                                        static_cast<int64_t>(monotonic_time_ns),
                                        &anchor_location)) {
    return nullptr;
  }
  return androidx::xr::openxr::CreateJavaAnchorState(env, anchor_location);
}

static jobject NativeGetAnchorToken(JNIEnv* env, jlong native_ptr) {
#ifdef __ANDROID__
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  XrSpace anchor_space = androidx::xr::openxr::ConvertToXrSpace(native_ptr);
  AIBinder* anchor_token = nullptr;
  if (!xr_manager.ExportAnchor(anchor_space, &anchor_token)) {
    xrDestroySpace(anchor_space);
    return nullptr;
  }

  if (__builtin_available(android __ANDROID_API_S__, *)) {
    return AIBinder_toJavaBinder(env, anchor_token);
  } else {
    LOG(ERROR) << "Failed to fetch anchor token due to the reason: "
                  "Android Version is not S or above.";
    return nullptr;
  }

#endif  // __ANDROID__
  LOG(ERROR) << "Failed to fetch anchor token unexpectedly. Precompile flags "
                "should prevent us from getting in this state.";
  return nullptr;
}

static jbyteArray NativePersistAnchor(JNIEnv* env, jlong native_ptr) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  XrSpace anchor_space = androidx::xr::openxr::ConvertToXrSpace(native_ptr);
  XrUuidEXT xr_uuid;
  if (!xr_manager.PersistAnchor(anchor_space, &xr_uuid)) {
    return nullptr;
  }
  jbyteArray result = env->NewByteArray(XR_UUID_SIZE);
  if (result != nullptr) {
    env->SetByteArrayRegion(result, 0, XR_UUID_SIZE, (jbyte*)xr_uuid.data);
  }
  return result;
}

static jobject NativeGetPersistenceState(JNIEnv* env, jobject uuid) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  XrUuidEXT xr_uuid = androidx::xr::openxr::ConvertToXrUuid(env, uuid);
  XrAnchorPersistStateANDROID state = XR_ANCHOR_PERSIST_STATE_MAX_ENUM_ANDROID;
  if (!xr_manager.GetAnchorPersistState(xr_uuid, &state)) {
    LOG(ERROR) << "Failed to get anchor persist state.";
    return nullptr;
  }
  return androidx::xr::openxr::CreateJavaAnchorPersistenceState(env, state);
}

static bool NativeDestroyAnchor(JNIEnv* env, jlong native_ptr,
                                jlong monotonic_time_ns) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  XrSpace xr_space = androidx::xr::openxr::ConvertToXrSpace(native_ptr);
  return xr_manager.DestroyAnchor(xr_space);
}

extern "C" {
JNIEXPORT bool JNICALL Java_androidx_xr_openxr_OpenXrAnchor_nativeDestroyAnchor(
    JNIEnv* env, jclass /*clazz*/, jlong native_ptr, jlong monotonic_time_ns) {
  return NativeDestroyAnchor(env, native_ptr, monotonic_time_ns);
}
JNIEXPORT bool JNICALL
Java_androidx_xr_runtime_openxr_OpenXrAnchor_nativeDestroyAnchor(
    JNIEnv* env, jclass /*clazz*/, jlong native_ptr, jlong monotonic_time_ns) {
  return NativeDestroyAnchor(env, native_ptr, monotonic_time_ns);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_openxr_OpenXrAnchor_nativeGetAnchorState(
    JNIEnv* env, jclass /*clazz*/, jlong native_ptr, jlong monotonic_time_ns) {
  return NativeGetAnchorState(env, native_ptr, monotonic_time_ns);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_runtime_openxr_OpenXrAnchor_nativeGetAnchorState(
    JNIEnv* env, jclass /*clazz*/, jlong native_ptr, jlong monotonic_time_ns) {
  return NativeGetAnchorState(env, native_ptr, monotonic_time_ns);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_openxr_OpenXrAnchor_nativeGetAnchorToken(JNIEnv* env,
                                                          jclass /*clazz*/,
                                                          jlong native_ptr) {
  return NativeGetAnchorToken(env, native_ptr);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_runtime_openxr_OpenXrAnchor_nativeGetAnchorToken(
    JNIEnv* env, jclass /*clazz*/, jlong native_ptr) {
  return NativeGetAnchorToken(env, native_ptr);
}

JNIEXPORT jbyteArray JNICALL
Java_androidx_xr_openxr_OpenXrAnchor_nativePersistAnchor(JNIEnv* env,
                                                         jclass /*clazz*/,
                                                         jlong native_ptr) {
  return NativePersistAnchor(env, native_ptr);
}

JNIEXPORT jbyteArray JNICALL
Java_androidx_xr_runtime_openxr_OpenXrAnchor_nativePersistAnchor(
    JNIEnv* env, jclass /*clazz*/, jlong native_ptr) {
  return NativePersistAnchor(env, native_ptr);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_openxr_OpenXrAnchor_nativeGetPersistenceState(JNIEnv* env,
                                                               jclass /*clazz*/,
                                                               jobject uuid) {
  return NativeGetPersistenceState(env, uuid);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_runtime_openxr_OpenXrAnchor_nativeGetPersistenceState(
    JNIEnv* env, jclass /*clazz*/, jobject uuid) {
  return NativeGetPersistenceState(env, uuid);
}
}  // extern "C"
