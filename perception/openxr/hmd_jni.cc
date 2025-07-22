// Copyright 2025 Google LLC
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
#include <openxr/openxr.h>

#include <cstdint>
#include <vector>

#include "openxr/jobject_creator.h"
#include "openxr/openxr_manager.h"

static jobject NativeGetHeadPose(JNIEnv* env, jlong monotonic_time_ns) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();

  XrPosef pose;
  if (!xr_manager.GetHeadPose(static_cast<int64_t>(monotonic_time_ns), &pose)) {
    return nullptr;
  }

  return androidx::xr::openxr::CreateJavaPose(env, pose);
}

static jobjectArray NativeGetViewCameras(JNIEnv* env,
                                         jboolean is_head_tracking_enabled,
                                         jlong monotonic_time_ns) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();

  std::vector<XrView> xr_views;
  xr_views.resize(2, XrView{.type = XR_TYPE_VIEW, .next = nullptr});
  if (!xr_manager.GetStereoViews(static_cast<int64_t>(monotonic_time_ns),
                                 is_head_tracking_enabled, &xr_views)) {
    return nullptr;
  }

  return androidx::xr::openxr::CreateJavaViewCameraStates(
      env, /*view_count=*/2, xr_views.data());
}

extern "C" {
JNIEXPORT jobject JNICALL
Java_androidx_xr_openxr_OpenXrDevice_nativeGetHeadPose(
    JNIEnv* env, jclass /*clazz*/, jlong monotonic_time_ns) {
  return NativeGetHeadPose(env, monotonic_time_ns);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_runtime_openxr_OpenXrDevice_nativeGetHeadPose(
    JNIEnv* env, jclass /*clazz*/, jlong monotonic_time_ns) {
  return NativeGetHeadPose(env, monotonic_time_ns);
}

JNIEXPORT jobjectArray JNICALL
Java_androidx_xr_openxr_OpenXrPerceptionManager_nativeGetViewCameras(
    JNIEnv* env, jclass /*clazz*/, jboolean is_head_tracking_enabled,
    jlong monotonic_time_ns) {
  return NativeGetViewCameras(env, is_head_tracking_enabled, monotonic_time_ns);
}

JNIEXPORT jobjectArray JNICALL
Java_androidx_xr_runtime_openxr_OpenXrPerceptionManager_nativeGetViewCameras(
    JNIEnv* env, jclass /*clazz*/, jboolean is_head_tracking_enabled,
    jlong monotonic_time_ns) {
  return NativeGetViewCameras(env, is_head_tracking_enabled, monotonic_time_ns);
}

}  // extern "C"
