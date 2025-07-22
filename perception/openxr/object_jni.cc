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

#include "openxr/jobject_converter.h"
#include "openxr/jobject_creator.h"
#include "openxr/openxr_manager.h"

inline jlong CreateJavaLongFromCreateAnchorResult(
    androidx::xr::openxr::OpenXrManager::CreateAnchorResult result,
    XrSpace xr_space) {
  if (result !=
      androidx::xr::openxr::OpenXrManager::CreateAnchorResult::kSuccess) {
    return static_cast<jlong>(result);
  }
  return androidx::xr::openxr::CreateJavaAnchorHandle(xr_space);
}

static jobject NativeGetAugmentedObjectState(JNIEnv* env, jlong object_id,
                                             jlong monotonic_time_ns) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  XrTrackableObjectANDROID object;
  if (!xr_manager.GetTrackableObjectState(
          static_cast<XrTrackableANDROID>(object_id),
          XrReferenceSpaceType::XR_REFERENCE_SPACE_TYPE_UNBOUNDED_ANDROID,
          static_cast<int64_t>(monotonic_time_ns), object)) {
    return nullptr;
  }

  return androidx::xr::openxr::CreateJavaAugmentedObjectState(env, object);
}

static jlong NativeCreateAnchorForObject(JNIEnv* env, jlong object_id,
                                        jobject pose,
                                        jlong monotonic_time_ns) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();

  XrPosef xr_pose = androidx::xr::openxr::ConvertToXrPosef(env, pose);
  XrSpace anchor;
  androidx::xr::openxr::OpenXrManager::CreateAnchorResult result =
      xr_manager.CreateAnchorForObject(object_id,
                                       /*object=*/nullptr,
                                       static_cast<int64_t>(monotonic_time_ns),
                                       xr_pose, &anchor);
  return CreateJavaLongFromCreateAnchorResult(result, anchor);
}

extern "C" {
JNIEXPORT jobject JNICALL
Java_androidx_xr_openxr_OpenXrAugmentedObject_nativeGetAugmentedObjectState(
    JNIEnv* env, jclass /*clazz*/, jlong object_id, jlong monotonic_time_ns) {
  return NativeGetAugmentedObjectState(env, object_id, monotonic_time_ns);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_runtime_openxr_OpenXrAugmentedObject_nativeGetAugmentedObjectState(
    JNIEnv* env, jclass /*clazz*/, jlong object_id, jlong monotonic_time_ns) {
  return NativeGetAugmentedObjectState(env, object_id, monotonic_time_ns);
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_openxr_OpenXrAugmentedObject_nativeCreateAnchorForObject(
    JNIEnv* env, jclass /*clazz*/, jlong object_id, jobject pose,
    jlong monotonic_time_ns) {
  return NativeCreateAnchorForObject(env, object_id, pose, monotonic_time_ns);
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_runtime_openxr_OpenXrAugmentedObject_nativeCreateAnchorForObject(
    JNIEnv* env, jclass /*clazz*/, jlong object_id, jobject pose,
    jlong monotonic_time_ns) {
  return NativeCreateAnchorForObject(env, object_id, pose, monotonic_time_ns);
}

}  // extern "C"
