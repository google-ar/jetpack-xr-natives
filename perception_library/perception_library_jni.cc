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
#include <jni.h>
#endif  // __ANDROID__

#include <openxr/openxr.h>

#include <cstdint>
#include <vector>

#include "absl/base/casts.h"
#include "absl/log/log.h"
#include "openxr/openxr_manager.h"
#include "perception_library/jni_utils.h"

static jstring StringFromJNI(JNIEnv* env) {
  std::string connection_string = "Connected to JNI";
  return env->NewStringUTF(connection_string.c_str());
}

static jboolean CreateOpenXrSession(JNIEnv* env, jobject activity,
                                    jint reference_space_type) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();

  bool result = xr_manager.Init(
      env, activity, static_cast<XrReferenceSpaceType>(reference_space_type));

  if (env->ExceptionCheck()) {
    env->ExceptionDescribe();
    env->ExceptionClear();
    return false;
  }
  return result;
}

static jobject GetAnchor(JNIEnv* env, jfloat min_width, jfloat min_height,
                         jint type, jint label) {
#ifdef __ANDROID__
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  AIBinder* anchor_token = nullptr;

  androidx::xr::openxr::OpenXrManager::PlaneConstraints plane_constraints = {
      .min_width = min_width,
      .min_height = min_height,
      .type = static_cast<XrPlaneTypeANDROID>(type),
      .label = static_cast<XrPlaneLabelANDROID>(label),
  };
  XrSpace anchor_id = XR_NULL_HANDLE;
  if (!xr_manager.CreateSemanticAnchor(plane_constraints, &anchor_token,
                                       &anchor_id)) {
    return 0;
  }

  if (__builtin_available(android __ANDROID_API_S__, *)) {
    jobject binder = AIBinder_toJavaBinder(env, anchor_token);
    jobject anchor_data =
        vr::realitycore::CreateAnchorDataForAnchor(env, binder, anchor_id);
    if (env->ExceptionCheck()) {
      env->ExceptionDescribe();
      env->ExceptionClear();
      return 0;
    }
    return anchor_data;
  }
#endif  // __ANDROID__
  LOG(ERROR) << "Anchor creation failed unexpectedly. Precompile flags should "
                "prevent us from getting in this state.";
  return nullptr;
}

static jboolean DetachAnchor(JNIEnv* env, jlong anchor_id) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();

  if (!xr_manager.DestroyAnchor(absl::bit_cast<XrSpace>(anchor_id))) {
    return false;
  }

  if (env->ExceptionCheck()) {
    env->ExceptionDescribe();
    env->ExceptionClear();
    return false;
  }
  return true;
}

static jbyteArray PersistAnchor(JNIEnv* env, jlong anchor_id) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  XrUuidEXT xr_uuid;
  if (!xr_manager.PersistAnchor(absl::bit_cast<XrSpace>(anchor_id), &xr_uuid)) {
    return nullptr;
  }
  jbyteArray result = env->NewByteArray(XR_UUID_SIZE);
  if (result != nullptr) {
    env->SetByteArrayRegion(result, 0, XR_UUID_SIZE, (jbyte*)xr_uuid.data);
  }
  return result;
}

static jobject GetCurrentHeadPose(JNIEnv* env) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();

  XrPosef xr_pose;
  if (!xr_manager.GetHeadPose(xr_manager.GetXrTimeNow(), &xr_pose)) {
    return nullptr;
  }
  jobject pose = vr::realitycore::CreateJavaPose(env, xr_pose);

  if (env->ExceptionCheck()) {
    env->ExceptionDescribe();
    env->ExceptionClear();
    return nullptr;
  }
  return pose;
}

static jobject GetCurrentStereoViews(JNIEnv* env) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  std::vector<XrView> xr_views;
  xr_views.resize(2, XrView{.type = XR_TYPE_VIEW, .next = nullptr});
  if (!xr_manager.GetStereoViews(xr_manager.GetXrTimeNow(), &xr_views)) {
    return nullptr;
  }
  jobject left_view =
      vr::realitycore::CreateJavaViewProjection(env, xr_views[0]);
  jobject right_view =
      vr::realitycore::CreateJavaViewProjection(env, xr_views[1]);

  jclass pair_class = env->FindClass("android/util/Pair");
  jmethodID pair_constructor = env->GetMethodID(
      pair_class, "<init>", "(Ljava/lang/Object;Ljava/lang/Object;)V");
  jobject view_pair =
      env->NewObject(pair_class, pair_constructor, left_view, right_view);

  if (env->ExceptionCheck()) {
    env->ExceptionDescribe();
    env->ExceptionClear();
    return nullptr;
  }
  return view_pair;
}

static jobject GetPlanes(JNIEnv* env) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();

  std::vector<XrTrackableANDROID> planes = xr_manager.GetPlanes();
  jobject plane_list =
      vr::realitycore::ConvertVectorToLongArrayList(env, planes);
  if (env->ExceptionCheck()) {
    env->ExceptionDescribe();
    env->ExceptionClear();
    return nullptr;
  }
  return plane_list;
}

static jobject GetPlaneData(JNIEnv* env, jlong plane_id,
                            jint reference_space_type,
                            jlong monotonic_time_ns) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  XrTime xr_time = xr_manager.GetXrTimeFromNanoseconds(monotonic_time_ns);

  // We are ignoring the vertex count for now but creating it here as it
  // cannot be null in the XrTrackablePlaneANDROID.
  uint32_t vertex_count = 0;
  XrTrackablePlaneANDROID plane_data = {
      .type = XR_TYPE_TRACKABLE_PLANE_ANDROID,
      .vertexCapacityInput = 0,
      .vertexCountOutput = &vertex_count,
      .vertices = nullptr,
  };
  if (!xr_manager.GetPlaneState(
          static_cast<XrTrackableANDROID>(plane_id),
          static_cast<XrReferenceSpaceType>(reference_space_type), xr_time,
          plane_data)) {
    return nullptr;
  }
  jobject plane_data_java = vr::realitycore::CreatePlaneData(env, plane_data);
  if (env->ExceptionCheck()) {
    env->ExceptionDescribe();
    env->ExceptionClear();
    return nullptr;
  }
  return plane_data_java;
}

static jobject CreateAnchorOnPlane(JNIEnv* env, jlong plane_id, jobject pose,
                                   jlong monotonic_time_ns) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();

  XrTime xr_time = xr_manager.GetXrTimeFromNanoseconds(monotonic_time_ns);
  XrSpace anchor_space = XR_NULL_HANDLE;
  XrPosef xr_pose = vr::realitycore::CreateXrPose(env, pose);
  if (!xr_manager.CreateAnchorForPlane(
          static_cast<XrTrackableANDROID>(plane_id), nullptr, xr_time, xr_pose,
          &anchor_space)) {
    return nullptr;
  }
#ifdef __ANDROID__
  AIBinder* anchor_token = nullptr;

  if (!xr_manager.ExportAnchor(anchor_space, &anchor_token)) {
    xrDestroySpace(anchor_space);
    return 0;
  }

  if (__builtin_available(android __ANDROID_API_S__, *)) {
    jobject binder = AIBinder_toJavaBinder(env, anchor_token);
    jobject anchor_data =
        vr::realitycore::CreateAnchorDataForAnchor(env, binder, anchor_space);
    if (env->ExceptionCheck()) {
      env->ExceptionDescribe();
      env->ExceptionClear();
      return 0;
    }
    return anchor_data;
  }
#endif  // __ANDROID__
  LOG(ERROR) << "Anchor creation failed unexpectedly. Precompile flags should "
                "prevent us from getting in this state.";
  return nullptr;
}

static jobject GetPersistState(JNIEnv* env, jlong high_bits, jlong low_bits) {
  const XrUuidEXT xr_uuid =
      vr::realitycore::ConvertLongsToUuid(high_bits, low_bits);
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  XrAnchorPersistStateANDROID state = XR_ANCHOR_PERSIST_STATE_MAX_ENUM_ANDROID;
  if (!xr_manager.GetAnchorPersistState(xr_uuid, &state)) {
    LOG(ERROR) << "Failed to get anchor persist state.";
    return nullptr;
  }
  return vr::realitycore::ConvertXrAnchorPersistStateANDROIDToJavaPersistState(
      env, state);
}

static jboolean UnpersistAnchor(JNIEnv* env, jlong high_bits, jlong low_bits) {
  const XrUuidEXT xr_uuid =
      vr::realitycore::ConvertLongsToUuid(high_bits, low_bits);
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  return xr_manager.UnpersistAnchor(xr_uuid);
}

static jobject CreatePersistedAnchor(JNIEnv* env, jlong highBits,
                                     jlong lowBits) {
  const XrUuidEXT xr_uuid =
      vr::realitycore::ConvertLongsToUuid(highBits, lowBits);
  XrSpace anchor_id = XR_NULL_HANDLE;
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  if (!xr_manager.LocatePersistedAnchorSpace(xr_uuid, &anchor_id)) {
    return nullptr;
  }
#ifdef __ANDROID__
  AIBinder* anchor_token = nullptr;
  if (!xr_manager.ExportAnchor(anchor_id, &anchor_token)) {
    xrDestroySpace(anchor_id);
    return nullptr;
  }

  if (__builtin_available(android __ANDROID_API_S__, *)) {
    jobject binder = AIBinder_toJavaBinder(env, anchor_token);
    jobject anchor_data =
        vr::realitycore::CreateAnchorDataForAnchor(env, binder, anchor_id);
    if (env->ExceptionCheck()) {
      env->ExceptionDescribe();
      env->ExceptionClear();
      return nullptr;
    }
    return anchor_data;
  }
#endif  // __ANDROID__
  LOG(ERROR) << "Anchor creation failed unexpectedly. Precompile flags should "
                "prevent us from getting in this state.";
  return nullptr;
}

static jlong GetNativeSession(JNIEnv* env) {
  XrSession session =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager().GetXrSession();
  return *reinterpret_cast<jlong*>(&session);
}

static jlong GetNativeInstance(JNIEnv* env) {
  XrInstance instance =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager().GetXrInstance();
  return *reinterpret_cast<jlong*>(&instance);
}

extern "C" {
JNIEXPORT jstring JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_PerceptionLibrary_stringFromJNI(
    JNIEnv* env, jobject /* this */) {
  return StringFromJNI(env);
}

JNIEXPORT jstring JNICALL
Java_androidx_xr_scenecore_impl_perception_PerceptionLibrary_stringFromJNI(
    JNIEnv* env, jobject /* this */) {
  return StringFromJNI(env);
}

JNIEXPORT jboolean JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Session_createOpenXrSession(
    JNIEnv* env, jobject /* this */, jobject activity,
    jint reference_space_type) {
  return CreateOpenXrSession(env, activity, reference_space_type);
}

JNIEXPORT jboolean JNICALL
Java_androidx_xr_scenecore_impl_perception_Session_createOpenXrSession(
    JNIEnv* env, jobject /* this */, jobject activity,
    jint reference_space_type) {
  return CreateOpenXrSession(env, activity, reference_space_type);
}

JNIEXPORT jobject JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Session_getAnchor(
    JNIEnv* env, jobject /* this */, jfloat min_width, jfloat min_height,
    jint type, jint label) {
  return GetAnchor(env, min_width, min_height, type, label);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_scenecore_impl_perception_Session_getAnchor(
    JNIEnv* env, jobject /* this */, jfloat min_width, jfloat min_height,
    jint type, jint label) {
  return GetAnchor(env, min_width, min_height, type, label);
}

JNIEXPORT jboolean JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Anchor_detachAnchor(
    JNIEnv* env, jobject /* this */, jlong anchor_id) {
  return DetachAnchor(env, anchor_id);
}

JNIEXPORT jboolean JNICALL
Java_androidx_xr_scenecore_impl_perception_Anchor_detachAnchor(
    JNIEnv* env, jobject /* this */, jlong anchor_id) {
  return DetachAnchor(env, anchor_id);
}

JNIEXPORT jbyteArray JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Anchor_persistAnchor(
    JNIEnv* env, jobject /* this */, jlong anchor_id) {
  return PersistAnchor(env, anchor_id);
}

JNIEXPORT jbyteArray JNICALL
Java_androidx_xr_scenecore_impl_perception_Anchor_persistAnchor(
    JNIEnv* env, jobject /* this */, jlong anchor_id) {
  return PersistAnchor(env, anchor_id);
}

JNIEXPORT jobject JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Session_getCurrentHeadPose(
    JNIEnv* env, jobject /* this */) {
  return GetCurrentHeadPose(env);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_scenecore_impl_perception_Session_getCurrentHeadPose(
    JNIEnv* env, jobject /* this */) {
  return GetCurrentHeadPose(env);
}

JNIEXPORT jobject JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Session_getCurrentStereoViews(
    JNIEnv* env, jobject /* this */) {
  return GetCurrentStereoViews(env);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_scenecore_impl_perception_Session_getCurrentStereoViews(
    JNIEnv* env, jobject /* this */) {
  return GetCurrentStereoViews(env);
}

JNIEXPORT jobject JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Session_getPlanes(
    JNIEnv* env, jobject /* this */) {
  return GetPlanes(env);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_scenecore_impl_perception_Session_getPlanes(
    JNIEnv* env, jobject /* this */) {
  return GetPlanes(env);
}

JNIEXPORT jobject JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Plane_getPlaneData(
    JNIEnv* env, jobject /* this */, jlong plane_id, jint reference_space_type,
    jlong monotonic_time_ns) {
  return GetPlaneData(env, plane_id, reference_space_type, monotonic_time_ns);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_scenecore_impl_perception_Plane_getPlaneData(
    JNIEnv* env, jobject /* this */, jlong plane_id, jint reference_space_type,
    jlong monotonic_time_ns) {
  return GetPlaneData(env, plane_id, reference_space_type, monotonic_time_ns);
}

JNIEXPORT jobject JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Plane_createAnchorOnPlane(
    JNIEnv* env, jobject /* this */, jlong plane_id, jobject pose,
    jlong monotonic_time_ns) {
  return CreateAnchorOnPlane(env, plane_id, pose, monotonic_time_ns);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_scenecore_impl_perception_Plane_createAnchorOnPlane(
    JNIEnv* env, jobject /* this */, jlong plane_id, jobject pose,
    jlong monotonic_time_ns) {
  return CreateAnchorOnPlane(env, plane_id, pose, monotonic_time_ns);
}

JNIEXPORT jobject JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Anchor_getPersistState(
    JNIEnv* env, jobject /* this */, jlong high_bits, jlong low_bits) {
  return GetPersistState(env, high_bits, low_bits);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_scenecore_impl_perception_Anchor_getPersistState(
    JNIEnv* env, jobject /* this */, jlong high_bits, jlong low_bits) {
  return GetPersistState(env, high_bits, low_bits);
}

JNIEXPORT jboolean JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Session_unpersistAnchor(
    JNIEnv* env, jobject /* this */, jlong high_bits, jlong low_bits) {
  return UnpersistAnchor(env, high_bits, low_bits);
}

JNIEXPORT jboolean JNICALL
Java_androidx_xr_scenecore_impl_perception_Session_unpersistAnchor(
    JNIEnv* env, jobject /* this */, jlong high_bits, jlong low_bits) {
  return UnpersistAnchor(env, high_bits, low_bits);
}

JNIEXPORT jobject JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Session_createPersistedAnchor(
    JNIEnv* env, jobject /* this */, jlong highBits, jlong lowBits) {
  return CreatePersistedAnchor(env, highBits, lowBits);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_scenecore_impl_perception_Session_createPersistedAnchor(
    JNIEnv* env, jobject /* this */, jlong highBits, jlong lowBits) {
  return CreatePersistedAnchor(env, highBits, lowBits);
}

JNIEXPORT jlong JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Session_getNativeSession(
    JNIEnv* env, jobject /* this */) {
  return GetNativeSession(env);
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_scenecore_impl_perception_Session_getNativeSession(
    JNIEnv* env, jobject /* this */) {
  return GetNativeSession(env);
}

JNIEXPORT jlong JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Session_getNativeInstance(
    JNIEnv* env, jobject /* this */) {
  return GetNativeInstance(env);
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_scenecore_impl_perception_Session_getNativeInstance(
    JNIEnv* env, jobject /* this */) {
  return GetNativeInstance(env);
}
}  // extern "C"
