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
#include <string>
#include <utility>
#include <vector>

#include "absl/base/casts.h"
#include "common/namespace_util.h"
#include "perception_library/jni_utils.h"
#include "perception_library/test_utils/fake_perception_library_state.h"

static void NativeReset(JNIEnv* env) {
  vr::realitycore::FakePerceptionLibraryState::GetFakePerceptionLibraryState()
      .initialize_success_ = JNI_FALSE;
  vr::realitycore::FakePerceptionLibraryState::GetFakePerceptionLibraryState()
      .anchor_binder_ = nullptr;
  vr::realitycore::FakePerceptionLibraryState::GetFakePerceptionLibraryState()
      .anchor_id_ = XR_NULL_HANDLE;
  vr::realitycore::FakePerceptionLibraryState::GetFakePerceptionLibraryState()
      .anchor_uuid_bytes_.clear();
  vr::realitycore::FakePerceptionLibraryState::GetFakePerceptionLibraryState()
      .current_head_pose_ = nullptr;
  vr::realitycore::FakePerceptionLibraryState::GetFakePerceptionLibraryState()
      .left_view_ = nullptr;
  vr::realitycore::FakePerceptionLibraryState::GetFakePerceptionLibraryState()
      .right_view_ = nullptr;
  vr::realitycore::FakePerceptionLibraryState::GetFakePerceptionLibraryState()
      .planes_.clear();
}

static void NativeSetCreateSessionResult(JNIEnv* env, jboolean success) {
  vr::realitycore::FakePerceptionLibraryState::GetFakePerceptionLibraryState()
      .initialize_success_ = (success != JNI_FALSE);
}

static jint NativeGetOpenXrSessionReferenceSpaceType(JNIEnv* env) {
  return vr::realitycore::FakePerceptionLibraryState::
      GetFakePerceptionLibraryState()
          .reference_space_type_;
}

static void NativeSetGetAnchorResult(JNIEnv* env, jobject binder,
                                     jlong anchor_id) {
  if (binder == nullptr) {
    vr::realitycore::FakePerceptionLibraryState::GetFakePerceptionLibraryState()
        .anchor_binder_ = nullptr;
  }
  vr::realitycore::FakePerceptionLibraryState::GetFakePerceptionLibraryState()
      .anchor_binder_ = env->NewGlobalRef(binder);
  vr::realitycore::FakePerceptionLibraryState::GetFakePerceptionLibraryState()
      .anchor_id_ = absl::bit_cast<XrSpace>(anchor_id);
}

static void NativeSetAnchorUuidBytes(JNIEnv* env,
                                     jbyteArray anchor_uuid_bytes) {
  auto& uuid = vr::realitycore::FakePerceptionLibraryState::
                   GetFakePerceptionLibraryState()
                       .anchor_uuid_bytes_;
  if (anchor_uuid_bytes == nullptr) {
    return;
  }

  jsize length = env->GetArrayLength(anchor_uuid_bytes);
  uuid.reserve(length);
  uuid.clear();
  jbyte* data = env->GetByteArrayElements(anchor_uuid_bytes, nullptr);
  for (jsize i = 0; i < length; i++) {
    uuid.push_back(data[i]);
  }
  env->ReleaseByteArrayElements(anchor_uuid_bytes, data, 0);
}

static void NativeSetAnchorPersistState(JNIEnv* env, jint persist_state) {
  XrAnchorPersistStateANDROID& state =
      vr::realitycore::FakePerceptionLibraryState::
          GetFakePerceptionLibraryState()
              .persist_state_;
  state = static_cast<XrAnchorPersistStateANDROID>(persist_state);
}

static void NativeSetGetCurrentHeadPoseResult(JNIEnv* env, jfloat x, jfloat y,
                                              jfloat z, jfloat qx, jfloat qy,
                                              jfloat qz, jfloat qw) {
  jclass pose_class =
      GetJxrClass(env, ::androidx::xr::common::PACKAGE_PERCEPTION, "Pose");
  jmethodID pose_constructor =
      env->GetMethodID(pose_class, "<init>", "(FFFFFFF)V");

  jobject pose =
      env->NewObject(pose_class, pose_constructor, x, y, z, qx, qy, qz, qw);
  vr::realitycore::FakePerceptionLibraryState::GetFakePerceptionLibraryState()
      .current_head_pose_ = env->NewGlobalRef(pose);
}

static void NativeSetLeftView(JNIEnv* env, jfloat x, jfloat y, jfloat z,
                              jfloat qx, jfloat qy, jfloat qz, jfloat qw,
                              float angleLeft, float angleRight, float angleUp,
                              float angleDown) {
  XrView left_view = {
      .type = XR_TYPE_VIEW,
      .next = nullptr,
      .pose =
          {
              .orientation = {.x = qx, .y = qy, .z = qz, .w = qw},
              .position = {x, y, z},
          },
      .fov = {angleLeft, angleRight, angleUp, angleDown}};

  jobject view = vr::realitycore::CreateJavaViewProjection(env, left_view);
  vr::realitycore::FakePerceptionLibraryState::GetFakePerceptionLibraryState()
      .left_view_ = env->NewGlobalRef(view);
}

static void NativeSetRightView(JNIEnv* env, jfloat x, jfloat y, jfloat z,
                               jfloat qx, jfloat qy, jfloat qz, jfloat qw,
                               float angleLeft, float angleRight, float angleUp,
                               float angleDown) {
  XrView right_view = {
      .type = XR_TYPE_VIEW,
      .next = nullptr,
      .pose =
          {
              .orientation = {.x = qx, .y = qy, .z = qz, .w = qw},
              .position = {x, y, z},
          },
      .fov = {angleLeft, angleRight, angleUp, angleDown}};

  jobject view = vr::realitycore::CreateJavaViewProjection(env, right_view);
  vr::realitycore::FakePerceptionLibraryState::GetFakePerceptionLibraryState()
      .right_view_ = env->NewGlobalRef(view);
}

static void NativeAddPlane(JNIEnv* env, jlong plane, jobject center_pose,
                           jfloat extent_width, jfloat extent_height, jint type,
                           jint label) {
  uint32_t vertex_count = 0;
  XrTrackablePlaneANDROID plane_data = {
      .type = XR_TYPE_TRACKABLE_PLANE_ANDROID,
      .centerPose = vr::realitycore::CreateXrPose(env, center_pose),
      .extents = {.width = extent_width, .height = extent_height},
      .planeType = static_cast<XrPlaneTypeANDROID>(type),
      .planeLabel = static_cast<XrPlaneLabelANDROID>(label),
      .vertexCapacityInput = 0,
      .vertexCountOutput = &vertex_count,
      .vertices = nullptr,
  };
  vr::realitycore::FakePerceptionLibraryState::GetFakePerceptionLibraryState()
      .planes_[static_cast<XrTrackableANDROID>(plane)] = std::move(plane_data);
}

static jstring NativeStringFromJNI(JNIEnv* env) {
  std::string connection_string = "Connected to JNI for test";
  return env->NewStringUTF(connection_string.c_str());
}

static jboolean NativeCreateOpenXrSession(JNIEnv* env, jobject activity,
                                          int reference_space_type) {
  vr::realitycore::FakePerceptionLibraryState::GetFakePerceptionLibraryState()
      .reference_space_type_ =
      static_cast<XrReferenceSpaceType>(reference_space_type);
  return vr::realitycore::FakePerceptionLibraryState::
      GetFakePerceptionLibraryState()
          .initialize_success_;
}

static jobject NativeGetAnchor(JNIEnv* env, jfloat min_width, jfloat min_height,
                               jint type, jint label) {
  if (vr::realitycore::FakePerceptionLibraryState::
          GetFakePerceptionLibraryState()
              .anchor_binder_ == nullptr) {
    return nullptr;
  }
  return vr::realitycore::CreateAnchorDataForAnchor(
      env,
      vr::realitycore::FakePerceptionLibraryState::
          GetFakePerceptionLibraryState()
              .anchor_binder_,
      vr::realitycore::FakePerceptionLibraryState::
          GetFakePerceptionLibraryState()
              .anchor_id_);
}

static jboolean NativeDetachAnchor(JNIEnv* env, jlong anchor_id) {
  XrSpace xr_space_id = absl::bit_cast<XrSpace>(anchor_id);
  return xr_space_id == vr::realitycore::FakePerceptionLibraryState::
                            GetFakePerceptionLibraryState()
                                .anchor_id_;
}

static jbyteArray NativePersistAnchor(JNIEnv* env, jlong anchor_id) {
  XrSpace xr_space_id = absl::bit_cast<XrSpace>(anchor_id);
  if (xr_space_id != vr::realitycore::FakePerceptionLibraryState::
                         GetFakePerceptionLibraryState()
                             .anchor_id_) {
    return nullptr;
  }

  const auto& uuid = vr::realitycore::FakePerceptionLibraryState::
                         GetFakePerceptionLibraryState()
                             .anchor_uuid_bytes_;

  if (uuid.empty()) {
    return nullptr;
  }

  jbyteArray anchor_uuid_bytes = env->NewByteArray(uuid.size());
  env->SetByteArrayRegion(anchor_uuid_bytes, 0, uuid.size(),
                          reinterpret_cast<const jbyte*>(uuid.data()));
  return anchor_uuid_bytes;
}

/**
 * Returns whether the UUID{high_bits, low_bits} is as same as the UUID
 * predefined in FakePerceptionLibraryState.
 */
bool CompareWithUuidInFakePerceptionLibraryState(jlong high_bits,
                                                 jlong low_bits) {
  const XrUuidEXT xr_uuid =
      vr::realitycore::ConvertLongsToUuid(high_bits, low_bits);
  for (int i = 0; i < XR_UUID_SIZE; ++i) {
    if (xr_uuid.data[i] != vr::realitycore::FakePerceptionLibraryState::
                               GetFakePerceptionLibraryState()
                                   .anchor_uuid_bytes_[i]) {
      return false;
    }
  }
  return true;
}

static jobject NativeGetPersistState(JNIEnv* env, jlong high_bits,
                                     jlong low_bits) {
  if (!CompareWithUuidInFakePerceptionLibraryState(high_bits, low_bits)) {
    return nullptr;
  }
  XrAnchorPersistStateANDROID state =
      vr::realitycore::FakePerceptionLibraryState::
          GetFakePerceptionLibraryState()
              .persist_state_;
  return vr::realitycore::ConvertXrAnchorPersistStateANDROIDToJavaPersistState(
      env, state);
}

static jboolean NativeUnpersistAnchor(JNIEnv* env, jlong high_bits,
                                      jlong low_bits) {
  return CompareWithUuidInFakePerceptionLibraryState(high_bits, low_bits);
}

static jobject NativeCreatePersistedAnchor(JNIEnv* env, jlong high_bits,
                                           jlong low_bits) {
  if (!CompareWithUuidInFakePerceptionLibraryState(high_bits, low_bits)) {
    return nullptr;
  }
  return vr::realitycore::CreateAnchorDataForAnchor(
      env,
      vr::realitycore::FakePerceptionLibraryState::
          GetFakePerceptionLibraryState()
              .anchor_binder_,
      vr::realitycore::FakePerceptionLibraryState::
          GetFakePerceptionLibraryState()
              .anchor_id_);
}

static jobject NativeGetCurrentHeadPose(JNIEnv* env) {
  if (vr::realitycore::FakePerceptionLibraryState::
          GetFakePerceptionLibraryState()
              .current_head_pose_ == nullptr) {
    return nullptr;
  }
  return vr::realitycore::FakePerceptionLibraryState::
      GetFakePerceptionLibraryState()
          .current_head_pose_;
}

static jobject NativeGetCurrentStereoViews(JNIEnv* env) {
  if (vr::realitycore::FakePerceptionLibraryState::
              GetFakePerceptionLibraryState()
                  .left_view_ == nullptr ||
      vr::realitycore::FakePerceptionLibraryState::
              GetFakePerceptionLibraryState()
                  .right_view_ == nullptr) {
    return nullptr;
  }
  jclass pair_class = env->FindClass("android/util/Pair");
  jmethodID pair_constructor = env->GetMethodID(
      pair_class, "<init>", "(Ljava/lang/Object;Ljava/lang/Object;)V");
  return env->NewObject(pair_class, pair_constructor,
                        vr::realitycore::FakePerceptionLibraryState::
                            GetFakePerceptionLibraryState()
                                .left_view_,
                        vr::realitycore::FakePerceptionLibraryState::
                            GetFakePerceptionLibraryState()
                                .right_view_);
}

static jobject NativeGetPlanes(JNIEnv* env) {
  std::vector<XrTrackableANDROID> planes;
  for (const auto& [plane, plane_data] :
       vr::realitycore::FakePerceptionLibraryState::
           GetFakePerceptionLibraryState()
               .planes_) {
    planes.push_back(plane);
  }
  jobject plane_list =
      vr::realitycore::ConvertVectorToLongArrayList(env, planes);
  if (env->ExceptionCheck()) {
    env->ExceptionDescribe();
    env->ExceptionClear();
    return nullptr;
  };
  return plane_list;
}

static jobject NativeGetPlaneData(JNIEnv* env, jlong plane_id,
                                  jint reference_space_type,
                                  jlong monotonic_time_ms) {
  const auto& plane_data =
      vr::realitycore::FakePerceptionLibraryState::
          GetFakePerceptionLibraryState()
              .planes_.find(static_cast<XrTrackableANDROID>(plane_id));
  if (plane_data == vr::realitycore::FakePerceptionLibraryState::
                        GetFakePerceptionLibraryState()
                            .planes_.end()) {
    return nullptr;
  }
  return vr::realitycore::CreatePlaneData(env, plane_data->second);
}

static jobject NativeCreateAnchorOnPlane(JNIEnv* env, jlong plane_id,
                                         jobject pose,
                                         jlong monotonic_time_ns) {
  const auto& plane_data =
      vr::realitycore::FakePerceptionLibraryState::
          GetFakePerceptionLibraryState()
              .planes_.find(static_cast<XrTrackableANDROID>(plane_id));
  if (plane_data == vr::realitycore::FakePerceptionLibraryState::
                        GetFakePerceptionLibraryState()
                            .planes_.end()) {
    return nullptr;
  }
  if (vr::realitycore::FakePerceptionLibraryState::
          GetFakePerceptionLibraryState()
              .anchor_binder_ == nullptr) {
    return nullptr;
  }
  return vr::realitycore::CreateAnchorDataForAnchor(
      env,
      vr::realitycore::FakePerceptionLibraryState::
          GetFakePerceptionLibraryState()
              .anchor_binder_,
      vr::realitycore::FakePerceptionLibraryState::
          GetFakePerceptionLibraryState()
              .anchor_id_);
}

extern "C" {

JNIEXPORT void JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_PerceptionLibraryTestHelper_reset(
    JNIEnv* env, jobject /* this */) {
  NativeReset(env);
}

JNIEXPORT void JNICALL
Java_androidx_xr_scenecore_impl_perception_PerceptionLibraryTestHelper_reset(
    JNIEnv* env, jobject /* this */) {
  NativeReset(env);
}

JNIEXPORT void JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_PerceptionLibraryTestHelper_setCreateSessionResult(
    JNIEnv* env, jobject /* this */, jboolean success) {
  NativeSetCreateSessionResult(env, success);
}

JNIEXPORT void JNICALL
Java_androidx_xr_scenecore_impl_perception_PerceptionLibraryTestHelper_setCreateSessionResult(
    JNIEnv* env, jobject /* this */, jboolean success) {
  NativeSetCreateSessionResult(env, success);
}

JNIEXPORT jint JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_PerceptionLibraryTestHelper_getOpenXrSessionReferenceSpaceType(
    JNIEnv* env, jobject /* this */) {
  return NativeGetOpenXrSessionReferenceSpaceType(env);
}

JNIEXPORT jint JNICALL
Java_androidx_xr_scenecore_impl_perception_PerceptionLibraryTestHelper_getOpenXrSessionReferenceSpaceType(
    JNIEnv* env, jobject /* this */) {
  return NativeGetOpenXrSessionReferenceSpaceType(env);
}

JNIEXPORT void JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_PerceptionLibraryTestHelper_setGetAnchorResult(
    JNIEnv* env, jobject /* this */, jobject binder, jlong anchor_id) {
  NativeSetGetAnchorResult(env, binder, anchor_id);
}

JNIEXPORT void JNICALL
Java_androidx_xr_scenecore_impl_perception_PerceptionLibraryTestHelper_setGetAnchorResult(
    JNIEnv* env, jobject /* this */, jobject binder, jlong anchor_id) {
  NativeSetGetAnchorResult(env, binder, anchor_id);
}

JNIEXPORT void JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_PerceptionLibraryTestHelper_setAnchorUuidBytes(
    JNIEnv* env, jobject /* this */, jbyteArray anchor_uuid_bytes) {
  NativeSetAnchorUuidBytes(env, anchor_uuid_bytes);
}

JNIEXPORT void JNICALL
Java_androidx_xr_scenecore_impl_perception_PerceptionLibraryTestHelper_setAnchorUuidBytes(
    JNIEnv* env, jobject /* this */, jbyteArray anchor_uuid_bytes) {
  NativeSetAnchorUuidBytes(env, anchor_uuid_bytes);
}

JNIEXPORT void JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_PerceptionLibraryTestHelper_setAnchorPersistState(
    JNIEnv* env, jobject /* this */, jint persist_state) {
  NativeSetAnchorPersistState(env, persist_state);
}

JNIEXPORT void JNICALL
Java_androidx_xr_scenecore_impl_perception_PerceptionLibraryTestHelper_setAnchorPersistState(
    JNIEnv* env, jobject /* this */, jint persist_state) {
  NativeSetAnchorPersistState(env, persist_state);
}

JNIEXPORT void JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_PerceptionLibraryTestHelper_setGetCurrentHeadPoseResult(
    JNIEnv* env, jobject /* this */, jfloat x, jfloat y, jfloat z, jfloat qx,
    jfloat qy, jfloat qz, jfloat qw) {
  NativeSetGetCurrentHeadPoseResult(env, x, y, z, qx, qy, qz, qw);
}

JNIEXPORT void JNICALL
Java_androidx_xr_scenecore_impl_perception_PerceptionLibraryTestHelper_setGetCurrentHeadPoseResult(
    JNIEnv* env, jobject /* this */, jfloat x, jfloat y, jfloat z, jfloat qx,
    jfloat qy, jfloat qz, jfloat qw) {
  NativeSetGetCurrentHeadPoseResult(env, x, y, z, qx, qy, qz, qw);
}

JNIEXPORT void JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_PerceptionLibraryTestHelper_setLeftView(
    JNIEnv* env, jobject /* this */, jfloat x, jfloat y, jfloat z, jfloat qx,
    jfloat qy, jfloat qz, jfloat qw, float angleLeft, float angleRight,
    float angleUp, float angleDown) {
  NativeSetLeftView(env, x, y, z, qx, qy, qz, qw, angleLeft, angleRight,
                    angleUp, angleDown);
}

JNIEXPORT void JNICALL
Java_androidx_xr_scenecore_impl_perception_PerceptionLibraryTestHelper_setLeftView(
    JNIEnv* env, jobject /* this */, jfloat x, jfloat y, jfloat z, jfloat qx,
    jfloat qy, jfloat qz, jfloat qw, float angleLeft, float angleRight,
    float angleUp, float angleDown) {
  NativeSetLeftView(env, x, y, z, qx, qy, qz, qw, angleLeft, angleRight,
                    angleUp, angleDown);
}

JNIEXPORT void JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_PerceptionLibraryTestHelper_setRightView(
    JNIEnv* env, jobject /* this */, jfloat x, jfloat y, jfloat z, jfloat qx,
    jfloat qy, jfloat qz, jfloat qw, float angleLeft, float angleRight,
    float angleUp, float angleDown) {
  NativeSetRightView(env, x, y, z, qx, qy, qz, qw, angleLeft, angleRight,
                     angleUp, angleDown);
}

JNIEXPORT void JNICALL
Java_androidx_xr_scenecore_impl_perception_PerceptionLibraryTestHelper_setRightView(
    JNIEnv* env, jobject /* this */, jfloat x, jfloat y, jfloat z, jfloat qx,
    jfloat qy, jfloat qz, jfloat qw, float angleLeft, float angleRight,
    float angleUp, float angleDown) {
  NativeSetRightView(env, x, y, z, qx, qy, qz, qw, angleLeft, angleRight,
                     angleUp, angleDown);
}

JNIEXPORT void JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_PerceptionLibraryTestHelper_addPlane(
    JNIEnv* env, jobject /* this */, jlong plane, jobject center_pose,
    jfloat extent_width, jfloat extent_height, jint type, jint label) {
  NativeAddPlane(env, plane, center_pose, extent_width, extent_height, type,
                 label);
}

JNIEXPORT void JNICALL
Java_androidx_xr_scenecore_impl_perception_PerceptionLibraryTestHelper_addPlane(
    JNIEnv* env, jobject /* this */, jlong plane, jobject center_pose,
    jfloat extent_width, jfloat extent_height, jint type, jint label) {
  NativeAddPlane(env, plane, center_pose, extent_width, extent_height, type,
                 label);
}

JNIEXPORT jstring JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_PerceptionLibrary_stringFromJNI(
    JNIEnv* env, jobject /* this */) {
  return NativeStringFromJNI(env);
}

JNIEXPORT jstring JNICALL
Java_androidx_xr_scenecore_impl_perception_PerceptionLibrary_stringFromJNI(
    JNIEnv* env, jobject /* this */) {
  return NativeStringFromJNI(env);
}

JNIEXPORT jboolean JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Session_createOpenXrSession(
    JNIEnv* env, jobject /* this */, jobject activity,
    int reference_space_type) {
  return NativeCreateOpenXrSession(env, activity, reference_space_type);
}

JNIEXPORT jboolean JNICALL
Java_androidx_xr_scenecore_impl_perception_Session_createOpenXrSession(
    JNIEnv* env, jobject /* this */, jobject activity,
    int reference_space_type) {
  return NativeCreateOpenXrSession(env, activity, reference_space_type);
}

JNIEXPORT jobject JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Session_getAnchor(
    JNIEnv* env, jobject /* this */, jfloat min_width, jfloat min_height,
    jint type, jint label) {
  return NativeGetAnchor(env, min_width, min_height, type, label);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_scenecore_impl_perception_Session_getAnchor(
    JNIEnv* env, jobject /* this */, jfloat min_width, jfloat min_height,
    jint type, jint label) {
  return NativeGetAnchor(env, min_width, min_height, type, label);
}

JNIEXPORT jboolean JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Anchor_detachAnchor(
    JNIEnv* env, jobject /* this */, jlong anchor_id) {
  return NativeDetachAnchor(env, anchor_id);
}

JNIEXPORT jboolean JNICALL
Java_androidx_xr_scenecore_impl_perception_Anchor_detachAnchor(
    JNIEnv* env, jobject /* this */, jlong anchor_id) {
  return NativeDetachAnchor(env, anchor_id);
}

JNIEXPORT jbyteArray JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Anchor_persistAnchor(
    JNIEnv* env, jobject /* this */, jlong anchor_id) {
  return NativePersistAnchor(env, anchor_id);
}

JNIEXPORT jbyteArray JNICALL
Java_androidx_xr_scenecore_impl_perception_Anchor_persistAnchor(
    JNIEnv* env, jobject /* this */, jlong anchor_id) {
  return NativePersistAnchor(env, anchor_id);
}

JNIEXPORT jobject JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Anchor_getPersistState(
    JNIEnv* env, jobject /* this */, jlong high_bits, jlong low_bits) {
  return NativeGetPersistState(env, high_bits, low_bits);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_scenecore_impl_perception_Anchor_getPersistState(
    JNIEnv* env, jobject /* this */, jlong high_bits, jlong low_bits) {
  return NativeGetPersistState(env, high_bits, low_bits);
}

JNIEXPORT jboolean JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Session_unpersistAnchor(
    JNIEnv* env, jobject /* this */, jlong high_bits, jlong low_bits) {
  return NativeUnpersistAnchor(env, high_bits, low_bits);
}

JNIEXPORT jboolean JNICALL
Java_androidx_xr_scenecore_impl_perception_Session_unpersistAnchor(
    JNIEnv* env, jobject /* this */, jlong high_bits, jlong low_bits) {
  return NativeUnpersistAnchor(env, high_bits, low_bits);
}

JNIEXPORT jobject JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Session_createPersistedAnchor(
    JNIEnv* env, jobject /* this */, jlong high_bits, jlong low_bits) {
  return NativeCreatePersistedAnchor(env, high_bits, low_bits);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_scenecore_impl_perception_Session_createPersistedAnchor(
    JNIEnv* env, jobject /* this */, jlong high_bits, jlong low_bits) {
  return NativeCreatePersistedAnchor(env, high_bits, low_bits);
}

JNIEXPORT jobject JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Session_getCurrentHeadPose(
    JNIEnv* env, jobject /* this */) {
  return NativeGetCurrentHeadPose(env);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_scenecore_impl_perception_Session_getCurrentHeadPose(
    JNIEnv* env, jobject /* this */) {
  return NativeGetCurrentHeadPose(env);
}

JNIEXPORT jobject JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Session_getCurrentStereoViews(
    JNIEnv* env, jobject /* this */) {
  return NativeGetCurrentStereoViews(env);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_scenecore_impl_perception_Session_getCurrentStereoViews(
    JNIEnv* env, jobject /* this */) {
  return NativeGetCurrentStereoViews(env);
}

JNIEXPORT jobject JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Session_getPlanes(
    JNIEnv* env, jobject /* this */) {
  return NativeGetPlanes(env);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_scenecore_impl_perception_Session_getPlanes(
    JNIEnv* env, jobject /* this */) {
  return NativeGetPlanes(env);
}

JNIEXPORT jobject JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Plane_getPlaneData(
    JNIEnv* env, jobject /* this */, jlong plane_id, jint reference_space_type,
    jlong monotonic_time_ms) {
  return NativeGetPlaneData(env, plane_id, reference_space_type,
                            monotonic_time_ms);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_scenecore_impl_perception_Plane_getPlaneData(
    JNIEnv* env, jobject /* this */, jlong plane_id, jint reference_space_type,
    jlong monotonic_time_ms) {
  return NativeGetPlaneData(env, plane_id, reference_space_type,
                            monotonic_time_ms);
}

JNIEXPORT jobject JNICALL
Java_com_google_vr_realitycore_runtime_androidxr_perception_Plane_createAnchorOnPlane(
    JNIEnv* env, jobject /* this */, jlong plane_id, jobject pose,
    jlong monotonic_time_ns) {
  return NativeCreateAnchorOnPlane(env, plane_id, pose, monotonic_time_ns);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_scenecore_impl_perception_Plane_createAnchorOnPlane(
    JNIEnv* env, jobject /* this */, jlong plane_id, jobject pose,
    jlong monotonic_time_ns) {
  return NativeCreateAnchorOnPlane(env, plane_id, pose, monotonic_time_ns);
}

}  // extern "C"
