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
#include <openxr/openxr.h>

#include <cstdint>

#include "openxr/jobject_converter.h"
#include "openxr/jobject_creator.h"
#include "openxr/openxr_manager.h"

static jobject NativeGetPlaneState(JNIEnv* env, jlong plane_id,
                                   jlong monotonic_time_ns) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  uint32_t vertex_count = 0;
  XrTrackablePlaneANDROID plane = {
      .type = XR_TYPE_TRACKABLE_PLANE_ANDROID,
      .trackingState = XR_TRACKING_STATE_PAUSED_ANDROID,
      .vertexCapacityInput = 0,
      .vertexCountOutput = &vertex_count,
      .vertices = nullptr,
  };
  if (!xr_manager.GetPlaneState(
          static_cast<XrTrackableANDROID>(plane_id),
          XrReferenceSpaceType::XR_REFERENCE_SPACE_TYPE_UNBOUNDED_ANDROID,
          static_cast<int64_t>(monotonic_time_ns), plane)) {
    return nullptr;
  }

  return androidx::xr::openxr::CreateJavaPlaneState(env, plane);
}

static jlong NativeCreateAnchorForPlane(JNIEnv* env, jlong plane_id,
                                        jobject pose, jlong monotonic_time_ns) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();

  XrPosef xr_pose = androidx::xr::openxr::ConvertToXrPosef(env, pose);
  XrSpace anchor;
  xr_manager.CreateAnchorForPlane(plane_id,
                                  /*plane=*/nullptr,
                                  static_cast<int64_t>(monotonic_time_ns),
                                  xr_pose, &anchor);

  return androidx::xr::openxr::CreateJavaAnchorHandle(anchor);
}

extern "C" {
JNIEXPORT jobject JNICALL
Java_androidx_xr_openxr_OpenXrPlane_nativeGetPlaneState(
    JNIEnv* env, jclass /*clazz*/, jlong plane_id, jlong monotonic_time_ns) {
  return NativeGetPlaneState(env, plane_id, monotonic_time_ns);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_runtime_openxr_OpenXrPlane_nativeGetPlaneState(
    JNIEnv* env, jclass /*clazz*/, jlong plane_id, jlong monotonic_time_ns) {
  return NativeGetPlaneState(env, plane_id, monotonic_time_ns);
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_openxr_OpenXrPlane_nativeCreateAnchorForPlane(
    JNIEnv* env, jclass /*clazz*/, jlong plane_id, jobject pose,
    jlong monotonic_time_ns) {
  return NativeCreateAnchorForPlane(env, plane_id, pose, monotonic_time_ns);
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_runtime_openxr_OpenXrPlane_nativeCreateAnchorForPlane(
    JNIEnv* env, jclass /*clazz*/, jlong plane_id, jobject pose,
    jlong monotonic_time_ns) {
  return NativeCreateAnchorForPlane(env, plane_id, pose, monotonic_time_ns);
}
}  // extern "C"
