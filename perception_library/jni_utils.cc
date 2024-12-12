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

#include "perception_library/jni_utils.h"

#include <jni.h>
#include <openxr/openxr.h>

#include <cstdint>
#include <string>
#include <vector>

#include "absl/strings/str_format.h"
#include "common/namespace_util.h"

namespace vr::realitycore {
using ::androidx::xr::common::GetJxrClass;
using ::androidx::xr::common::GetJxrFullClassName;
using ::androidx::xr::common::PACKAGE_PERCEPTION;

// Returns a Java object of type `Pose` from an `XrPosef`.
jobject CreateJavaPose(JNIEnv* env, const XrPosef& xr_pose) {
  jclass pose_class = GetJxrClass(env, PACKAGE_PERCEPTION, "Pose");
  jmethodID pose_constructor =
      env->GetMethodID(pose_class, "<init>", "(FFFFFFF)V");

  return env->NewObject(pose_class, pose_constructor, xr_pose.position.x,
                        xr_pose.position.y, xr_pose.position.z,
                        xr_pose.orientation.x, xr_pose.orientation.y,
                        xr_pose.orientation.z, xr_pose.orientation.w);
}

XrPosef CreateXrPose(JNIEnv* env, const jobject& java_pose) {
  jclass pose_class = GetJxrClass(env, PACKAGE_PERCEPTION, "Pose");
  jfieldID pose_tx_field = env->GetFieldID(pose_class, "tx", "F");
  jfieldID pose_ty_field = env->GetFieldID(pose_class, "ty", "F");
  jfieldID pose_tz_field = env->GetFieldID(pose_class, "tz", "F");
  jfieldID pose_qx_field = env->GetFieldID(pose_class, "qx", "F");
  jfieldID pose_qy_field = env->GetFieldID(pose_class, "qy", "F");
  jfieldID pose_qz_field = env->GetFieldID(pose_class, "qz", "F");
  jfieldID pose_qw_field = env->GetFieldID(pose_class, "qw", "F");
  return {
      .orientation{.x = env->GetFloatField(java_pose, pose_qx_field),
                   .y = env->GetFloatField(java_pose, pose_qy_field),
                   .z = env->GetFloatField(java_pose, pose_qz_field),
                   .w = env->GetFloatField(java_pose, pose_qw_field)},
      .position{
          .x = env->GetFloatField(java_pose, pose_tx_field),
          .y = env->GetFloatField(java_pose, pose_ty_field),
          .z = env->GetFloatField(java_pose, pose_tz_field),
      },
  };
}

// Returns a Java object of type `Fov` from an `XrFovf`.
jobject CreateJavaFov(JNIEnv* env, const XrFovf& fov) {
  jclass fov_class = GetJxrClass(env, PACKAGE_PERCEPTION, "Fov");
  jmethodID fov_constructor = env->GetMethodID(fov_class, "<init>", "(FFFF)V");

  return env->NewObject(fov_class, fov_constructor, fov.angleLeft,
                        fov.angleRight, fov.angleUp, fov.angleDown);
}

// Returns a Java object of type `ViewProjection` from an `XrView`.
jobject CreateJavaViewProjection(JNIEnv* env, const XrView& view) {
  jclass viewprojection_class =
      GetJxrClass(env, PACKAGE_PERCEPTION, "ViewProjection");
  jmethodID viewprojection_constructor = env->GetMethodID(
      viewprojection_class, "<init>",
      absl::StrFormat("(L%s;L%s;)V",
                      GetJxrFullClassName(env, PACKAGE_PERCEPTION, "Pose"),
                      GetJxrFullClassName(env, PACKAGE_PERCEPTION, "Fov"))
          .c_str());

  jobject pose = CreateJavaPose(env, view.pose);
  jobject fov = CreateJavaFov(env, view.fov);
  return env->NewObject(viewprojection_class, viewprojection_constructor, pose,
                        fov);
}

jobject ConvertVectorToLongArrayList(
    JNIEnv* env, const std::vector<XrTrackableANDROID>& trackable_vector) {
  jclass list_class = env->FindClass("java/util/ArrayList");
  jmethodID list_constructor = env->GetMethodID(list_class, "<init>", "(I)V");
  jobject plane_list =
      env->NewObject(list_class, list_constructor, trackable_vector.size());
  jmethodID list_add_method_id =
      env->GetMethodID(list_class, "add", "(Ljava/lang/Object;)Z");

  jclass long_class = env->FindClass("java/lang/Long");
  jmethodID long_constructor = env->GetMethodID(long_class, "<init>", "(J)V");

  for (const auto& trackable : trackable_vector) {
    jobject long_element =
        env->NewObject(long_class, long_constructor, trackable);
    env->CallBooleanMethod(plane_list, list_add_method_id, long_element);
  }
  return plane_list;
}

jobject CreatePlaneData(JNIEnv* env, const XrTrackablePlaneANDROID& plane) {
  jclass plane_data_class =
      GetJxrClass(env, PACKAGE_PERCEPTION, "Plane$PlaneData");
  jmethodID plane_data_constructor = env->GetMethodID(
      plane_data_class, "<init>",
      absl::StrFormat("(L%s;FFII)V",
                      GetJxrFullClassName(env, PACKAGE_PERCEPTION, "Pose"))
          .c_str());
  jobject plane_data =
      env->NewObject(plane_data_class, plane_data_constructor,
                     CreateJavaPose(env, plane.centerPose), plane.extents.width,
                     plane.extents.height, plane.planeType, plane.planeLabel);
  return plane_data;
}

jobject CreateAnchorDataForAnchor(JNIEnv* env, jobject anchor_token,
                                  XrSpace anchor_id) {
  jobject anchor_data;
  jclass anchor_data_class =
      GetJxrClass(env, PACKAGE_PERCEPTION, "Anchor$AnchorData");
  jmethodID anchor_data_constructor =
      env->GetMethodID(anchor_data_class, "<init>", "()V");
  anchor_data = env->NewObject(anchor_data_class, anchor_data_constructor);
  jfieldID anchor_token_field =
      env->GetFieldID(anchor_data_class, "anchorToken", "Landroid/os/IBinder;");
  env->SetObjectField(anchor_data, anchor_token_field,
                      env->NewGlobalRef(anchor_token));

  // Doing a reinterperet cast from the XrSpace which could be an arbitrary
  // pointer. Check the size to make sure it is appropriate.
  static_assert(sizeof(XrSpace) <= sizeof(uint64_t));
  jfieldID anchor_id_field =
      env->GetFieldID(anchor_data_class, "anchorId", "J");
  env->SetLongField(anchor_data, anchor_id_field,
                    reinterpret_cast<uint64_t>(anchor_id));

  return anchor_data;
}

XrUuidEXT ConvertLongsToUuid(jlong high_bits, jlong low_bits) {
  XrUuidEXT xr_uuid;
  const int bytes_per_long = XR_UUID_SIZE / 2;
  for (int i = bytes_per_long - 1; i >= 0; --i) {
    xr_uuid.data[i] = static_cast<uint8_t>(high_bits & 0xff);
    high_bits = high_bits >> 8;
  }
  for (int i = XR_UUID_SIZE - 1; i >= bytes_per_long; --i) {
    xr_uuid.data[i] = static_cast<uint8_t>(low_bits & 0xff);
    low_bits = low_bits >> 8;
  }
  return xr_uuid;
}

jobject ConvertXrAnchorPersistStateANDROIDToJavaPersistState(
    JNIEnv* env, XrAnchorPersistStateANDROID persist_state) {
  jclass state_enum_class =
      GetJxrClass(env, PACKAGE_PERCEPTION, "Anchor$PersistState");
  jfieldID state_field_id;
  std::string persist_state_field_signature = absl::StrFormat(
      "L%s;",
      GetJxrFullClassName(env, PACKAGE_PERCEPTION, "Anchor$PersistState"));
  switch (persist_state) {
    case XR_ANCHOR_PERSIST_STATE_PERSIST_NOT_REQUESTED_ANDROID:
      state_field_id =
          env->GetStaticFieldID(state_enum_class, "PERSIST_NOT_REQUESTED",
                                persist_state_field_signature.c_str());
      break;
    case XR_ANCHOR_PERSIST_STATE_PERSIST_PENDING_ANDROID:
      state_field_id =
          env->GetStaticFieldID(state_enum_class, "PERSIST_PENDING",
                                persist_state_field_signature.c_str());
      break;
    case XR_ANCHOR_PERSIST_STATE_PERSISTED_ANDROID:
      state_field_id = env->GetStaticFieldID(
          state_enum_class, "PERSISTED", persist_state_field_signature.c_str());
      break;
    default:
      state_field_id = env->GetStaticFieldID(
          state_enum_class, "NOT_VALID", persist_state_field_signature.c_str());
      break;
  }
  return env->GetStaticObjectField(state_enum_class, state_field_id);
}

}  // namespace vr::realitycore
