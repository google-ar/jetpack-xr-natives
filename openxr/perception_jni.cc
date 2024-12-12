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
#include <vector>

#include "openxr/jobject_converter.h"
#include "openxr/jobject_creator.h"
#include "openxr/openxr_manager.h"
#include "common/namespace_util.h"

static jlong NativeCreateAnchor(JNIEnv* env, jobject pose,
                                jlong monotonic_time_ns) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  XrPosef xr_pose = androidx::xr::openxr::ConvertToXrPosef(env, pose);
  XrSpace anchor;
  if (!xr_manager.CreateAnchor(static_cast<int64_t>(monotonic_time_ns), xr_pose,
                               &anchor)) {
    return 0;
  }
  return androidx::xr::openxr::CreateJavaAnchorHandle(anchor);
}

static jlongArray NativeGetPlanes(JNIEnv* env) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  std::vector<XrTrackableANDROID> trackables = xr_manager.GetPlanes();
  jlongArray trackables_array = env->NewLongArray(trackables.size());
  jlong* array_elements =
      env->GetLongArrayElements(trackables_array, /*isCopy=*/JNI_FALSE);
  for (int i = 0; i < trackables.size(); i++) {
    array_elements[i] = static_cast<jlong>(trackables[i]);
  }
  env->ReleaseLongArrayElements(trackables_array, array_elements, /*mode=*/0);
  return trackables_array;
}

static jint NativeGetPlaneType(JNIEnv* env, jlong plane_id,
                               jlong monotonic_time_ns) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  uint32_t vertex_count = 0;
  XrTrackablePlaneANDROID plane = {
      .type = XR_TYPE_TRACKABLE_PLANE_ANDROID,
      .vertexCapacityInput = 0,
      .vertexCountOutput = &vertex_count,
      .vertices = nullptr,
  };
  if (!xr_manager.GetPlaneState(
          static_cast<XrTrackableANDROID>(plane_id),
          XrReferenceSpaceType::XR_REFERENCE_SPACE_TYPE_UNBOUNDED_ANDROID,
          static_cast<int64_t>(monotonic_time_ns), plane)) {
    return -1;
  }

  return static_cast<uint32_t>(plane.planeType);
}

static jobjectArray NativeHitTest(JNIEnv* env, jint max_results,
                                  jfloat origin_x, jfloat origin_y,
                                  jfloat origin_z, jfloat direction_x,
                                  jfloat direction_y, jfloat direction_z,
                                  jlong monotonic_time_ns) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();

  XrRaycastInfoANDROID raycast_info = {
      .type = XR_TYPE_RAYCAST_INFO_ANDROID,
      .next = nullptr,
      .maxResults = static_cast<uint32_t>(max_results),
      .origin = XrVector3f{.x = origin_x, .y = origin_y, .z = origin_z},
      .trajectory =
          XrVector3f{.x = direction_x, .y = direction_y, .z = direction_z},
      .time = static_cast<int64_t>(monotonic_time_ns),
  };
  std::vector<XrRaycastHitResultANDROID> hit_results_container(max_results);
  XrRaycastHitResultsANDROID out_hit_results = {
      .type = XR_TYPE_RAYCAST_HIT_RESULTS_ANDROID,
      .resultsCapacityInput = static_cast<uint32_t>(max_results),
      .results = hit_results_container.data(),
  };

  // Perform the hit test.
  if (!xr_manager.HitTest(&raycast_info, &out_hit_results)) {
    return nullptr;
  }

  jclass hit_data_class = androidx::xr::common::GetJxrClass(env,
                            androidx::xr::common::PACKAGE_OPENXR, "HitData");

  jobjectArray hit_result_array = env->NewObjectArray(
      out_hit_results.resultsCountOutput, hit_data_class, nullptr);
  for (int i = 0; i < out_hit_results.resultsCountOutput; i++) {
    jobject hit_result = androidx::xr::openxr::CreateJavaHitData(
        env, out_hit_results.results[i]);
    env->SetObjectArrayElement(hit_result_array, i, hit_result);
  }
  return hit_result_array;
}

static jobjectArray NativeGetPersistedAnchorUuids(JNIEnv* env) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  std::vector<XrUuidEXT> uuids = xr_manager.GetPersistedAnchorUuids();
  jobjectArray uuids_array =
      env->NewObjectArray(uuids.size(), env->FindClass("[B"), nullptr);
  for (int i = 0; i < uuids.size(); i++) {
    jbyteArray result = env->NewByteArray(XR_UUID_SIZE);
    if (result != nullptr) {
      env->SetByteArrayRegion(result, 0, XR_UUID_SIZE, (jbyte*)uuids[i].data);
    }
    env->SetObjectArrayElement(uuids_array, i, result);
  }
  return uuids_array;
}

static jlong NativeLoadAnchor(JNIEnv* env, jobject uuid) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  XrUuidEXT xr_uuid = androidx::xr::openxr::ConvertToXrUuid(env, uuid);
  XrSpace anchor;
  if (!xr_manager.LocatePersistedAnchorSpace(xr_uuid, &anchor)) {
    return 0;
  }
  return androidx::xr::openxr::CreateJavaAnchorHandle(anchor);
}

static jboolean NativeUnpersistAnchor(JNIEnv* env, jobject uuid) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  XrUuidEXT xr_uuid = androidx::xr::openxr::ConvertToXrUuid(env, uuid);
  return xr_manager.UnpersistAnchor(xr_uuid);
}

extern "C" {
JNIEXPORT jlong JNICALL
Java_androidx_xr_openxr_OpenXrPerceptionManager_nativeCreateAnchor(
    JNIEnv* env, jclass /*clazz*/, jobject pose, jlong monotonic_time_ns) {
  return NativeCreateAnchor(env, pose, monotonic_time_ns);
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_runtime_openxr_OpenXrPerceptionManager_nativeCreateAnchor(
    JNIEnv* env, jclass /*clazz*/, jobject pose, jlong monotonic_time_ns) {
  return NativeCreateAnchor(env, pose, monotonic_time_ns);
}

JNIEXPORT jlongArray JNICALL
Java_androidx_xr_openxr_OpenXrPerceptionManager_nativeGetPlanes(
    JNIEnv* env, jclass /*clazz*/) {
  return NativeGetPlanes(env);
}

JNIEXPORT jlongArray JNICALL
Java_androidx_xr_runtime_openxr_OpenXrPerceptionManager_nativeGetPlanes(
    JNIEnv* env, jclass /*clazz*/) {
  return NativeGetPlanes(env);
}

JNIEXPORT jint JNICALL
Java_androidx_xr_openxr_OpenXrPerceptionManager_nativeGetPlaneType(
    JNIEnv* env, jclass /*clazz*/, jlong plane_id, jlong monotonic_time_ns) {
  return NativeGetPlaneType(env, plane_id, monotonic_time_ns);
}

JNIEXPORT jint JNICALL
Java_androidx_xr_runtime_openxr_OpenXrPerceptionManager_nativeGetPlaneType(
    JNIEnv* env, jclass /*clazz*/, jlong plane_id, jlong monotonic_time_ns) {
  return NativeGetPlaneType(env, plane_id, monotonic_time_ns);
}

JNIEXPORT jobjectArray JNICALL
Java_androidx_xr_openxr_OpenXrPerceptionManager_nativeHitTest(
    JNIEnv* env, jclass /*clazz*/, jint max_results, jfloat origin_x,
    jfloat origin_y, jfloat origin_z, jfloat direction_x, jfloat direction_y,
    jfloat direction_z, jlong monotonic_time_ns) {
  return NativeHitTest(env, max_results, origin_x, origin_y, origin_z,
                       direction_x, direction_y, direction_z,
                       monotonic_time_ns);
}

JNIEXPORT jobjectArray JNICALL
Java_androidx_xr_runtime_openxr_OpenXrPerceptionManager_nativeHitTest(
    JNIEnv* env, jclass /*clazz*/, jint max_results, jfloat origin_x,
    jfloat origin_y, jfloat origin_z, jfloat direction_x, jfloat direction_y,
    jfloat direction_z, jlong monotonic_time_ns) {
  return NativeHitTest(env, max_results, origin_x, origin_y, origin_z,
                       direction_x, direction_y, direction_z,
                       monotonic_time_ns);
}

JNIEXPORT jobjectArray JNICALL
Java_androidx_xr_openxr_OpenXrPerceptionManager_nativeGetPersistedAnchorUuids(
    JNIEnv* env, jclass /*clazz*/) {
  return NativeGetPersistedAnchorUuids(env);
}

JNIEXPORT jobjectArray JNICALL
Java_androidx_xr_runtime_openxr_OpenXrPerceptionManager_nativeGetPersistedAnchorUuids(
    JNIEnv* env, jclass /*clazz*/) {
  return NativeGetPersistedAnchorUuids(env);
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_openxr_OpenXrPerceptionManager_nativeLoadAnchor(
    JNIEnv* env, jclass /*clazz*/, jobject uuid) {
  return NativeLoadAnchor(env, uuid);
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_runtime_openxr_OpenXrPerceptionManager_nativeLoadAnchor(
    JNIEnv* env, jclass /*clazz*/, jobject uuid) {
  return NativeLoadAnchor(env, uuid);
}

JNIEXPORT jboolean JNICALL
Java_androidx_xr_openxr_OpenXrPerceptionManager_nativeUnpersistAnchor(
    JNIEnv* env, jclass /*clazz*/, jobject uuid) {
  return NativeUnpersistAnchor(env, uuid);
}

JNIEXPORT jboolean JNICALL
Java_androidx_xr_runtime_openxr_OpenXrPerceptionManager_nativeUnpersistAnchor(
    JNIEnv* env, jclass /*clazz*/, jobject uuid) {
  return NativeUnpersistAnchor(env, uuid);
}
}  // extern "C"
