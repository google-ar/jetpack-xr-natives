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

#include "openxr/jobject_creator.h"

#include <jni.h>
#include <openxr/openxr.h>

#include <cstdint>

#include "absl/strings/str_format.h"
#include "common/namespace_util.h"

namespace androidx::xr::openxr {
using ::androidx::xr::common::GetJxrClass;
using ::androidx::xr::common::GetJxrFullClassName;

using ::androidx::xr::common::PACKAGE_CORE;
using ::androidx::xr::common::PACKAGE_MATH;
using ::androidx::xr::common::PACKAGE_OPENXR;
using ::androidx::xr::common::PACKAGE_RUNTIME;

constexpr XrSpaceLocationFlags kPoseValidFlags =
    XR_SPACE_LOCATION_POSITION_VALID_BIT |
    XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;

jobject CreateJavaTrackingState(
    JNIEnv* env, const XrTrackingStateANDROID& xr_tracking_state) {
  jclass plane_state_ext_cls =
      GetJxrClass(env, PACKAGE_OPENXR, "PlaneStateKt");
  jclass tracking_state_enum = GetJxrClass(env, PACKAGE_CORE, "TrackingState");
  jfieldID tracking_state_static_fid = env->GetStaticFieldID(
      tracking_state_enum, "Companion",
      absl::StrFormat("L%s;", GetJxrFullClassName(env, PACKAGE_CORE,
                                                  "TrackingState$Companion"))
          .c_str());
  jmethodID fromOpenXrTrackingState = env->GetStaticMethodID(
      plane_state_ext_cls, "fromOpenXrTrackingState",
      absl::StrFormat(
          "(L%s;I)L%s;",
          GetJxrFullClassName(env, PACKAGE_CORE, "TrackingState$Companion"),
          GetJxrFullClassName(env, PACKAGE_CORE, "TrackingState"))
          .c_str());
  jobject tracking_state_static_obj =
      env->GetStaticObjectField(tracking_state_enum, tracking_state_static_fid);
  return env->CallStaticObjectMethod(
      plane_state_ext_cls, fromOpenXrTrackingState,
      tracking_state_static_obj, static_cast<uint32_t>(xr_tracking_state));
}

jobject CreateJavaTrackingState(
    JNIEnv* env, const XrFaceTrackingStateANDROID& xr_face_tracking_state) {
  jclass plane_state_ext_cls = GetJxrClass(env, PACKAGE_OPENXR, "FaceStateKt");
  jclass tracking_state_enum = GetJxrClass(env, PACKAGE_CORE, "TrackingState");
  jfieldID tracking_state_static_fid = env->GetStaticFieldID(
      tracking_state_enum, "Companion",
      absl::StrFormat("L%s;", GetJxrFullClassName(env, PACKAGE_CORE,
                                                  "TrackingState$Companion"))
          .c_str());
  jmethodID fromOpenXrFaceTrackingState = env->GetStaticMethodID(
      plane_state_ext_cls, "fromOpenXrFaceTrackingState",
      absl::StrFormat(
          "(L%s;I)L%s;",
          GetJxrFullClassName(env, PACKAGE_CORE, "TrackingState$Companion"),
          GetJxrFullClassName(env, PACKAGE_CORE, "TrackingState"))
          .c_str());
  jobject tracking_state_static_obj =
      env->GetStaticObjectField(tracking_state_enum, tracking_state_static_fid);
  return env->CallStaticObjectMethod(
      plane_state_ext_cls, fromOpenXrFaceTrackingState,
      tracking_state_static_obj, static_cast<uint32_t>(xr_face_tracking_state));
}

jobject CreateJavaTrackingState(JNIEnv* env,
                                const XrSpaceLocationFlags& location_flags) {
  jclass tracking_state_ext_cls =
      GetJxrClass(env, PACKAGE_OPENXR, "AnchorStateKt");
  jclass tracking_state_enum = GetJxrClass(env, PACKAGE_CORE, "TrackingState");
  jfieldID tracking_state_static_fid = env->GetStaticFieldID(
      tracking_state_enum, "Companion",
      absl::StrFormat("L%s;", GetJxrFullClassName(env, PACKAGE_CORE,
                                                  "TrackingState$Companion"))
          .c_str());
  jmethodID fromOpenXrLocationFlags = env->GetStaticMethodID(
      tracking_state_ext_cls, "fromOpenXrLocationFlags",
      absl::StrFormat(
          "(L%s;I)L%s;",
          GetJxrFullClassName(env, PACKAGE_CORE, "TrackingState$Companion"),
          GetJxrFullClassName(env, PACKAGE_CORE, "TrackingState"))
          .c_str());

  jobject tracking_state_static_obj =
      env->GetStaticObjectField(tracking_state_enum, tracking_state_static_fid);
  return env->CallStaticObjectMethod(
      tracking_state_ext_cls, fromOpenXrLocationFlags,
      tracking_state_static_obj, static_cast<uint32_t>(location_flags));
}

jobject CreateJavaVector2(JNIEnv* env, const XrExtent2Df& xr_extent) {
  jclass vector2_class = GetJxrClass(env, PACKAGE_MATH, "Vector2");
  jmethodID vector2_constructor =
      env->GetMethodID(vector2_class, "<init>", "(FF)V");
  // xr_extent.width => Java Vector2.x
  // xr_extent.height => Java Vector2.y
  return env->NewObject(vector2_class, vector2_constructor, xr_extent.width,
                        xr_extent.height);
}

jobject CreateJavaVector2(JNIEnv* env, const XrVector2f& xr_vector) {
  jclass vector2_class = GetJxrClass(env, PACKAGE_MATH, "Vector2");
  jmethodID vector2_constructor =
      env->GetMethodID(vector2_class, "<init>", "(FF)V");
  return env->NewObject(vector2_class, vector2_constructor, xr_vector.x,
                        xr_vector.y);
}

jobject CreateJavaVector3(JNIEnv* env, const XrVector3f& xr_vector) {
  jclass vector3_class = GetJxrClass(env, PACKAGE_MATH, "Vector3");
  jmethodID vector3_constructor =
      env->GetMethodID(vector3_class, "<init>", "(FFF)V");
  return env->NewObject(vector3_class, vector3_constructor, xr_vector.x,
                        xr_vector.y, xr_vector.z);
}

jobject CreateJavaQuaternion(JNIEnv* env, const XrQuaternionf& xr_quaternion) {
  jclass quaternion_class = GetJxrClass(env, PACKAGE_MATH, "Quaternion");
  jmethodID quaternion_constructor =
      env->GetMethodID(quaternion_class, "<init>", "(FFFF)V");
  return env->NewObject(quaternion_class, quaternion_constructor,
                        xr_quaternion.x, xr_quaternion.y, xr_quaternion.z,
                        xr_quaternion.w);
}

jobject CreateJavaPose(JNIEnv* env, const XrPosef& xr_pose) {
  jclass pose_class = GetJxrClass(env, PACKAGE_MATH, "Pose");
  jmethodID pose_constructor = env->GetMethodID(
      pose_class, "<init>",
      absl::StrFormat("(L%s;L%s;)V",
      GetJxrFullClassName(env, PACKAGE_MATH, "Vector3"),
      GetJxrFullClassName(env, PACKAGE_MATH, "Quaternion")).c_str());

  jobject position = CreateJavaVector3(env, xr_pose.position);
  jobject orientation = CreateJavaQuaternion(env, xr_pose.orientation);
  return env->NewObject(pose_class, pose_constructor, position, orientation);
}

jobject CreateJavaFieldOfView(JNIEnv* env, const XrFovf& fov) {
  jclass fov_class = GetJxrClass(env, PACKAGE_CORE, "FieldOfView");
  jmethodID fov_constructor = env->GetMethodID(fov_class, "<init>", "(FFFF)V");
  return env->NewObject(fov_class, fov_constructor, fov.angleLeft,
                        fov.angleRight, fov.angleUp, fov.angleDown);
}

jobjectArray CreateJavaViewCameraStates(JNIEnv* env, uint32_t view_count,
                                       XrView* views) {
  jclass view_camera_state_class =
      GetJxrClass(env, PACKAGE_OPENXR, "ViewCameraState");
  jmethodID view_camera_state_constructor = env->GetMethodID(
      view_camera_state_class, "<init>",
      absl::StrFormat("(L%s;L%s;)V",
                      GetJxrFullClassName(env, PACKAGE_MATH, "Pose"),
                      GetJxrFullClassName(env, PACKAGE_CORE, "FieldOfView"))
          .c_str());

  jobjectArray view_camera_states =
      env->NewObjectArray(view_count, view_camera_state_class, nullptr);
  for (uint32_t i = 0; i < view_count; ++i) {
    jobject view_camera_state = env->NewObject(
        view_camera_state_class, view_camera_state_constructor,
        CreateJavaPose(env, views[i].pose),
        CreateJavaFieldOfView(env, views[i].fov));
    env->SetObjectArrayElement(view_camera_states, i, view_camera_state);
  }

  return view_camera_states;
}

jobject CreateJavaPlaneState(JNIEnv* env,
                             const XrTrackablePlaneANDROID& xr_plane) {
  jclass plane_state_class = GetJxrClass(env, PACKAGE_OPENXR, "PlaneState");
  jmethodID plane_state_constructor = env->GetMethodID(
      plane_state_class, "<init>",
      absl::StrFormat("(L%s;L%s;L%s;L%s;[L%s;J)V",
                      GetJxrFullClassName(env, PACKAGE_CORE, "TrackingState"),
                      GetJxrFullClassName(env, PACKAGE_RUNTIME, "Plane$Label"),
                      GetJxrFullClassName(env, PACKAGE_MATH, "Pose"),
                      GetJxrFullClassName(env, PACKAGE_MATH, "Vector2"),
                      GetJxrFullClassName(env, PACKAGE_MATH, "Vector2"))
          .c_str());
  jobject tracking_state = CreateJavaTrackingState(env, xr_plane.trackingState);
  jobject plane_label = CreateJavaPlaneLabel(env, xr_plane.planeLabel);
  jobject pose = CreateJavaPose(env, xr_plane.centerPose);
  jobject extents = CreateJavaVector2(env, xr_plane.extents);
  jobjectArray vertices = CreateJavaPlaneVertices(
      env, *xr_plane.vertexCountOutput, xr_plane.vertices);

  jobject plane_state = env->NewObject(
      plane_state_class, plane_state_constructor, tracking_state, plane_label,
      pose, extents, vertices, static_cast<int64_t>(xr_plane.subsumedByPlane));
  return plane_state;
}

jobjectArray CreateJavaPlaneVertices(JNIEnv* env, const uint32_t vertex_count,
                                     const XrVector2f* vertices) {
  jclass vector2_class = GetJxrClass(env, PACKAGE_MATH, "Vector2");

  jmethodID vector2_constructor =
      env->GetMethodID(vector2_class, "<init>", "(FF)V");
  jobjectArray vector2_array =
      env->NewObjectArray(vertex_count, vector2_class, nullptr);
  for (uint32_t i = 0; i < vertex_count; ++i) {
    jobject vector2 = env->NewObject(vector2_class, vector2_constructor,
                                     vertices[i].x, vertices[i].y);
    env->SetObjectArrayElement(vector2_array, i, vector2);
  }
  return vector2_array;
}

jobject CreateJavaPlaneLabel(JNIEnv* env,
                             const XrPlaneLabelANDROID& xr_plane_label) {
  jclass plane_label_ext_cls =
      GetJxrClass(env, PACKAGE_OPENXR, "OpenXrPlaneKt");
  jclass plane_label_enum = GetJxrClass(env, PACKAGE_RUNTIME, "Plane$Label");
  jfieldID plane_label_static_fid = env->GetStaticFieldID(
      plane_label_enum, "Companion",
      absl::StrFormat("L%s;",
      GetJxrFullClassName(env, PACKAGE_RUNTIME, "Plane$Label$Companion"))
      .c_str());
  jmethodID fromOpenXrLabel = env->GetStaticMethodID(
      plane_label_ext_cls, "fromOpenXrLabel",
      absl::StrFormat("(L%s;I)L%s;",
      GetJxrFullClassName(env, PACKAGE_RUNTIME, "Plane$Label$Companion"),
      GetJxrFullClassName(env, PACKAGE_RUNTIME, "Plane$Label")).c_str());

  jobject plane_label_static_obj =
      env->GetStaticObjectField(plane_label_enum, plane_label_static_fid);
  return env->CallStaticObjectMethod(plane_label_ext_cls, fromOpenXrLabel,
                                     plane_label_static_obj,
                                     static_cast<uint32_t>(xr_plane_label));
}

jobject CreateJavaAugmentedObjectState(JNIEnv* env,
                                   const XrTrackableObjectANDROID& xr_object) {
  jclass augmented_object_state_class =
      GetJxrClass(env, PACKAGE_OPENXR, "AugmentedObjectState");
  jmethodID augmented_object_state_constructor = env->GetMethodID(
      augmented_object_state_class, "<init>",
      absl::StrFormat("(L%s;JL%s;L%s;)V",
                      GetJxrFullClassName(env, PACKAGE_CORE, "TrackingState"),
                      GetJxrFullClassName(env, PACKAGE_MATH, "Pose"),
                      GetJxrFullClassName(env, PACKAGE_MATH, "FloatSize3d"))
          .c_str());
  jobject tracking_state =
      CreateJavaTrackingState(env, xr_object.trackingState);
  jobject pose = CreateJavaPose(env, xr_object.centerPose);
  jobject extents = CreateJavaFloatSize3d(env, xr_object.extents);
  jobject augmented_object_state = env->NewObject(
      augmented_object_state_class, augmented_object_state_constructor,
      tracking_state, static_cast<int64_t>(xr_object.objectLabel),
      pose, extents);
  return augmented_object_state;
}

jobject CreateJavaAnchorState(JNIEnv* env,
                              const XrSpaceLocation& anchor_location) {
  jclass anchor_data_class = GetJxrClass(env, PACKAGE_OPENXR, "AnchorState");
  jmethodID anchor_data_constructor = env->GetMethodID(
      anchor_data_class, "<init>",
      absl::StrFormat("(L%s;L%s;)V",
                      GetJxrFullClassName(env, PACKAGE_CORE, "TrackingState"),
                      GetJxrFullClassName(env, PACKAGE_MATH, "Pose"))
          .c_str());

  jobject tracking_state =
      CreateJavaTrackingState(env, anchor_location.locationFlags);
  // Only set the pose if the location from OpenXR is valid, else return pose as
  // null;
  jobject pose = nullptr;
  if ((anchor_location.locationFlags & kPoseValidFlags) == kPoseValidFlags) {
    pose = CreateJavaPose(env, anchor_location.pose);
  }

  return env->NewObject(anchor_data_class, anchor_data_constructor,
                        tracking_state, pose);
}

jobject CreateJavaHitData(JNIEnv* env,
                          const XrRaycastHitResultANDROID& xr_hit_result) {
  jclass hit_data_class = GetJxrClass(env, PACKAGE_OPENXR, "HitData");
  jmethodID hit_data_constructor = env->GetMethodID(
      hit_data_class, "<init>",
      absl::StrFormat("(L%s;J)V",
                      GetJxrFullClassName(env, PACKAGE_MATH, "Pose"))
                      .c_str());

  jobject hit_pose = CreateJavaPose(env, xr_hit_result.pose);
  return env->NewObject(hit_data_class, hit_data_constructor, hit_pose,
                        xr_hit_result.trackable);
}

jobject CreateJavaAnchorPersistenceState(
    JNIEnv* env, const XrAnchorPersistStateANDROID& xr_anchor_persist_state) {
  static_assert(sizeof(XrAnchorPersistStateANDROID) <= sizeof(uint32_t));
  jclass anchor_persistence_state_ext_class =
      GetJxrClass(env, PACKAGE_OPENXR, "OpenXrAnchorKt");
  jclass anchor_persistence_state_enum =
      GetJxrClass(env, PACKAGE_RUNTIME, "Anchor$PersistenceState");
  jfieldID anchor_persistence_state_static_fid = env->GetStaticFieldID(
      anchor_persistence_state_enum, "Companion",
      absl::StrFormat("L%s;",
        GetJxrFullClassName(env, PACKAGE_RUNTIME,
                            "Anchor$PersistenceState$Companion"))
                      .c_str());
  jmethodID fromOpenXrPersistenceState = env->GetStaticMethodID(
      anchor_persistence_state_ext_class, "fromOpenXrPersistenceState",
      absl::StrFormat("(L%s;I)L%s;",
      GetJxrFullClassName(env, PACKAGE_RUNTIME,
                          "Anchor$PersistenceState$Companion"),
      GetJxrFullClassName(env, PACKAGE_RUNTIME,
                          "Anchor$PersistenceState")).c_str());

  jobject anchor_persistence_state_static_obj = env->GetStaticObjectField(
      anchor_persistence_state_enum, anchor_persistence_state_static_fid);
  return env->CallStaticObjectMethod(
      anchor_persistence_state_ext_class, fromOpenXrPersistenceState,
      anchor_persistence_state_static_obj,
      static_cast<uint32_t>(xr_anchor_persist_state));
}

jlong CreateJavaAnchorHandle(XrSpace xr_space) {
  // Ensure conversion is safe bidirectionally, i.e., no truncation or padding.
  static_assert(sizeof(jlong) == sizeof(XrSpace));
  return *reinterpret_cast<jlong*>(&xr_space);
}

jobject CreateJavaFaceState(JNIEnv* env, const XrFaceStateANDROID& xr_face) {
  jclass face_state_class = GetJxrClass(env, PACKAGE_OPENXR, "FaceState");
  jmethodID face_state_constructor = env->GetMethodID(
      face_state_class, "<init>",
      absl::StrFormat("(L%s;Z[F[F)V",
                      GetJxrFullClassName(env, PACKAGE_CORE, "TrackingState"))
          .c_str());
  jobject tracking_state =
      CreateJavaTrackingState(env, xr_face.faceTrackingState);
  jfloatArray parameters = env->NewFloatArray(xr_face.parametersCountOutput);
  env->SetFloatArrayRegion(parameters, /*start=*/0,
                           xr_face.parametersCountOutput, xr_face.parameters);
  jfloatArray region_confidences =
      env->NewFloatArray(xr_face.regionConfidencesCountOutput);
  env->SetFloatArrayRegion(region_confidences, /*start=*/0,
                           xr_face.regionConfidencesCountOutput,
                           xr_face.regionConfidences);
  jobject face_state =
      env->NewObject(face_state_class, face_state_constructor, tracking_state,
                     xr_face.isValid, parameters, region_confidences);
  return face_state;
}

jobject CreateJavaIntSize2d(JNIEnv* env, int width, int height) {
  jclass int_size_2d_class = env->FindClass(
      GetJxrFullClassName(env, PACKAGE_MATH, "IntSize2d").c_str());
  jmethodID int_size_2d_constructor =
      env->GetMethodID(int_size_2d_class, "<init>", "(II)V");
  return env->NewObject(int_size_2d_class, int_size_2d_constructor, width,
                        height);
}

jobject CreateJavaFloatSize3d(JNIEnv* env, const XrExtent3Df& xr_extent) {
  jclass float_size_3d_class = env->FindClass(
      GetJxrFullClassName(env, PACKAGE_MATH, "FloatSize3d").c_str());
  jmethodID float_size_3d_constructor =
      env->GetMethodID(float_size_3d_class, "<init>", "(FFF)V");
  return env->NewObject(float_size_3d_class, float_size_3d_constructor,
                        xr_extent.width, xr_extent.height, xr_extent.depth);
}

}  // namespace androidx::xr::openxr
