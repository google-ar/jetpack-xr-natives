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
#include <openxr/public/all_extensions.h>

#include <cstdint>
#include <string>

#include "absl/strings/str_format.h"
#include "common/namespace_util.h"
#include "openxr/openxr_manager_utils.h"

namespace androidx::xr::openxr {
using ::androidx::xr::common::GetJxrClass;
using ::androidx::xr::common::GetJxrFullClassName;

using ::androidx::xr::common::PACKAGE_ARCORE;
using ::androidx::xr::common::PACKAGE_ARCORE_RUNTIME;
using ::androidx::xr::common::PACKAGE_ARCORE_OPENXR;
using ::androidx::xr::common::PACKAGE_CORE;
using ::androidx::xr::common::PACKAGE_MATH;

namespace {

jclass LoadClassWithClassLoader(JNIEnv* env, jobject class_loader,
                                common::Package package,
                                const char* class_name) {
  std::string full_class_name_str =
      ::androidx::xr::common::GetJxrFullClassName(env, package, class_name);
  jstring full_class_name = env->NewStringUTF(full_class_name_str.c_str());
  jclass class_loader_clazz = env->FindClass("java/lang/ClassLoader");
  jmethodID load_class_method = env->GetMethodID(
      class_loader_clazz, "loadClass", "(Ljava/lang/String;)Ljava/lang/Class;");
  jclass result = (jclass)env->CallObjectMethod(class_loader, load_class_method,
                                                full_class_name);
  env->DeleteLocalRef(full_class_name);
  env->DeleteLocalRef(class_loader_clazz);
  return result;
}

jobject CreateVpsAvailabilityResultInstance(JNIEnv* env, jobject class_loader,
                                            const char* class_name) {
  jclass cls = LoadClassWithClassLoader(env, class_loader,
                                        PACKAGE_ARCORE_RUNTIME, class_name);
  if (cls == nullptr) {
    return nullptr;
  }
  jmethodID constructor = env->GetMethodID(cls, "<init>", "()V");
  if (constructor == nullptr) {
    env->DeleteLocalRef(cls);
    return nullptr;
  }
  jobject instance = env->NewObject(cls, constructor);
  env->DeleteLocalRef(cls);
  return instance;
}
}  // namespace

constexpr XrSpaceLocationFlags kPoseValidFlags =
    XR_SPACE_LOCATION_POSITION_VALID_BIT |
    XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;

jobject CreateJavaTrackingState(JNIEnv* env, TrackingState tracking_state) {
  static constexpr const char* tracking_states[] = {
      "TRACKING", "PAUSED", "STOPPED", "TRACKING_DEGRADED"};
  uint32_t tracking_state_index = static_cast<uint32_t>(tracking_state);
  const char* field_name = (tracking_state_index <= 3)
                               ? tracking_states[tracking_state_index]
                               : "PAUSED";
  jclass cls = GetJxrClass(env, PACKAGE_ARCORE_RUNTIME, "TrackingState");
  jfieldID fid = env->GetStaticFieldID(
      cls, field_name,
      absl::StrFormat("L%s;", GetJxrFullClassName(env, PACKAGE_ARCORE_RUNTIME,
                                                  "TrackingState"))
          .c_str());

  return env->GetStaticObjectField(cls, fid);
}

jobject CreateJavaTrackingState(
    JNIEnv* env, const XrFaceTrackingStateANDROID& xr_face_tracking_state) {
  static constexpr TrackingState kTrackingStates[] = {kPaused, kStopped,
                                                      kTracking};
  uint32_t tracking_state_index = static_cast<uint32_t>(xr_face_tracking_state);
  TrackingState tracking_state = (tracking_state_index < 3)
                                     ? kTrackingStates[tracking_state_index]
                                     : kPaused;
  return CreateJavaTrackingState(env, tracking_state);
}

jobject CreateJavaTrackingState(
    JNIEnv* env, const XrTrackingStateANDROID& xr_tracking_state) {
  static constexpr TrackingState kTrackingStates[] = {kPaused, kStopped,
                                                      kTracking};
  uint32_t tracking_state_index = static_cast<uint32_t>(xr_tracking_state);
  TrackingState tracking_state = (tracking_state_index < 3)
                                     ? kTrackingStates[tracking_state_index]
                                     : kPaused;
  return CreateJavaTrackingState(env, tracking_state);
}

jobject CreateJavaTrackingState(JNIEnv* env,
                                const XrSpaceLocationFlags& location_flags) {
  TrackingState tracking_state = convertTrackingState(location_flags);
  return CreateJavaTrackingState(env, tracking_state);
}

jobject CreateJavaFloatSize2d(JNIEnv* env, const XrExtent2Df& xr_extent) {
  jclass float_size2d_class = GetJxrClass(env, PACKAGE_MATH, "FloatSize2d");
  jmethodID float_size2d_constructor =
      env->GetMethodID(float_size2d_class, "<init>", "(FF)V");
  return env->NewObject(float_size2d_class, float_size2d_constructor,
                        xr_extent.width, xr_extent.height);
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
  jclass fov_class = GetJxrClass(env, PACKAGE_MATH, "FieldOfView");
  jmethodID fov_constructor = env->GetMethodID(fov_class, "<init>", "(FFFF)V");
  return env->NewObject(fov_class, fov_constructor, fov.angleLeft,
                        fov.angleRight, fov.angleUp, fov.angleDown);
}

jobjectArray CreateJavaViewCameraStates(JNIEnv* env, uint32_t view_count,
                                       XrView* views) {
  jclass view_camera_state_class =
      GetJxrClass(env, PACKAGE_ARCORE_OPENXR, "ViewCameraState");
  jmethodID view_camera_state_constructor = env->GetMethodID(
      view_camera_state_class, "<init>",
      absl::StrFormat("(L%s;L%s;)V",
                      GetJxrFullClassName(env, PACKAGE_MATH, "Pose"),
                      GetJxrFullClassName(env, PACKAGE_MATH, "FieldOfView"))
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
  jclass plane_state_class =
      GetJxrClass(env, PACKAGE_ARCORE_OPENXR, "PlaneState");
  jmethodID plane_state_constructor = env->GetMethodID(
      plane_state_class, "<init>",
      absl::StrFormat(
          "(L%s;L%s;L%s;L%s;[L%s;J)V",
          GetJxrFullClassName(env, PACKAGE_ARCORE_RUNTIME, "TrackingState"),
          GetJxrFullClassName(env, PACKAGE_ARCORE_RUNTIME, "Plane$Label"),
          GetJxrFullClassName(env, PACKAGE_MATH, "Pose"),
          GetJxrFullClassName(env, PACKAGE_MATH, "FloatSize2d"),
          GetJxrFullClassName(env, PACKAGE_MATH, "Vector2"))
          .c_str());
  jobject tracking_state = CreateJavaTrackingState(env, xr_plane.trackingState);
  jobject plane_label = CreateJavaPlaneLabel(env, xr_plane.planeLabel);
  jobject pose = CreateJavaPose(env, xr_plane.centerPose);
  jobject extents = CreateJavaFloatSize2d(env, xr_plane.extents);
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
      GetJxrClass(env, PACKAGE_ARCORE_OPENXR, "OpenXrPlaneKt");
  jclass plane_label_enum =
      GetJxrClass(env, PACKAGE_ARCORE_RUNTIME, "Plane$Label");
  jfieldID plane_label_static_fid = env->GetStaticFieldID(
      plane_label_enum, "Companion",
      absl::StrFormat("L%s;", GetJxrFullClassName(env, PACKAGE_ARCORE_RUNTIME,
                                                  "Plane$Label$Companion"))
          .c_str());
  jmethodID fromOpenXrLabel = env->GetStaticMethodID(
      plane_label_ext_cls, "fromOpenXrLabel",
      absl::StrFormat(
          "(L%s;I)L%s;",
          GetJxrFullClassName(env, PACKAGE_ARCORE_RUNTIME,
                              "Plane$Label$Companion"),
          GetJxrFullClassName(env, PACKAGE_ARCORE_RUNTIME, "Plane$Label"))
          .c_str());

  jobject plane_label_static_obj =
      env->GetStaticObjectField(plane_label_enum, plane_label_static_fid);
  return env->CallStaticObjectMethod(plane_label_ext_cls, fromOpenXrLabel,
                                     plane_label_static_obj,
                                     static_cast<uint32_t>(xr_plane_label));
}

jobject CreateJavaAugmentedObjectState(JNIEnv* env,
                                   const XrTrackableObjectANDROID& xr_object) {
  jclass augmented_object_state_class =
      GetJxrClass(env, PACKAGE_ARCORE_OPENXR, "AugmentedObjectState");
  jmethodID augmented_object_state_constructor = env->GetMethodID(
      augmented_object_state_class, "<init>",
      absl::StrFormat(
          "(L%s;JL%s;L%s;)V",
          GetJxrFullClassName(env, PACKAGE_ARCORE_RUNTIME, "TrackingState"),
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
  jclass anchor_data_class =
      GetJxrClass(env, PACKAGE_ARCORE_OPENXR, "AnchorState");
  jmethodID anchor_data_constructor = env->GetMethodID(
      anchor_data_class, "<init>",
      absl::StrFormat(
          "(L%s;L%s;)V",
          GetJxrFullClassName(env, PACKAGE_ARCORE_RUNTIME, "TrackingState"),
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

jobject CreateJavaDeviceState(JNIEnv* env, TrackingState tracking_state,
                              XrPosef head_pose) {
  jclass device_state_class =
      GetJxrClass(env, PACKAGE_ARCORE_OPENXR, "DeviceState");
  jmethodID device_state_constructor = env->GetMethodID(
      device_state_class, "<init>",
      absl::StrFormat(
          "(L%s;L%s;)V",
          GetJxrFullClassName(env, PACKAGE_ARCORE_RUNTIME, "TrackingState"),
          GetJxrFullClassName(env, PACKAGE_MATH, "Pose"))
          .c_str());

  jobject tracking_state_java = CreateJavaTrackingState(env, tracking_state);
  // Only set the pose if the head tracking state is not paused (1),
  // else return pose as null;
  jobject pose = nullptr;
  if (tracking_state == TrackingState::kTracking ||
      tracking_state == TrackingState::kTrackingDegraded) {
    pose = CreateJavaPose(env, head_pose);
  }

  return env->NewObject(device_state_class, device_state_constructor,
                        tracking_state_java, pose);
}

jobject CreateJavaHitData(JNIEnv* env,
                          const XrRaycastHitResultANDROID& xr_hit_result) {
  jclass hit_data_class = GetJxrClass(env, PACKAGE_ARCORE_OPENXR, "HitData");
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
      GetJxrClass(env, PACKAGE_ARCORE_OPENXR, "OpenXrAnchorKt");
  jclass anchor_persistence_state_enum =
      GetJxrClass(env, PACKAGE_ARCORE_RUNTIME, "Anchor$PersistenceState");
  jfieldID anchor_persistence_state_static_fid = env->GetStaticFieldID(
      anchor_persistence_state_enum, "Companion",
      absl::StrFormat("L%s;",
                      GetJxrFullClassName(env, PACKAGE_ARCORE_RUNTIME,
                                          "Anchor$PersistenceState$Companion"))
          .c_str());
  jmethodID fromOpenXrPersistenceState = env->GetStaticMethodID(
      anchor_persistence_state_ext_class, "fromOpenXrPersistenceState",
      absl::StrFormat("(L%s;I)L%s;",
                      GetJxrFullClassName(env, PACKAGE_ARCORE_RUNTIME,
                                          "Anchor$PersistenceState$Companion"),
                      GetJxrFullClassName(env, PACKAGE_ARCORE_RUNTIME,
                                          "Anchor$PersistenceState"))
          .c_str());

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
  jclass face_state_class =
      GetJxrClass(env, PACKAGE_ARCORE_OPENXR, "FaceState");
  jmethodID face_state_constructor = env->GetMethodID(
      face_state_class, "<init>",
      absl::StrFormat(
          "(L%s;Z[F[F)V",
          GetJxrFullClassName(env, PACKAGE_ARCORE_RUNTIME, "TrackingState"))
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

jobject CreateJavaEyeState(JNIEnv* env, const XrEyeStateANDROID& xr_eye_state) {
  jclass eye_state_enum = GetJxrClass(env, PACKAGE_ARCORE_OPENXR, "EyeStatus");
  jclass eye_data_class = GetJxrClass(env, PACKAGE_ARCORE_OPENXR, "EyeDataKt");
  jfieldID eye_state_static_fid = env->GetStaticFieldID(
      eye_state_enum, "Companion",
      absl::StrFormat("L%s;", GetJxrFullClassName(env, PACKAGE_ARCORE_OPENXR,
                                                  "EyeStatus$Companion"))
          .c_str());
  jmethodID fromOpenXrEyeState = env->GetStaticMethodID(
      eye_data_class, "fromOpenXrEyeState",
      absl::StrFormat(
          "(L%s;I)L%s;",
          GetJxrFullClassName(env, PACKAGE_ARCORE_OPENXR, "EyeStatus$Companion"),
          GetJxrFullClassName(env, PACKAGE_ARCORE_OPENXR, "EyeStatus"))
          .c_str());
  jobject eye_state_static_obj =
      env->GetStaticObjectField(eye_state_enum, eye_state_static_fid);
  return env->CallStaticObjectMethod(eye_data_class, fromOpenXrEyeState,
                                     eye_state_static_obj,
                                     static_cast<uint32_t>(xr_eye_state));
}

jobject CreateJavaEyeTrackingState(JNIEnv* env,
                                   const XrEyeTrackingModeANDROID& xr_mode) {
  jclass eye_tracking_mode_enum =
      GetJxrClass(env, PACKAGE_ARCORE_OPENXR, "EyeTrackingState");
  jclass eyes_info_class =
      GetJxrClass(env, PACKAGE_ARCORE_OPENXR, "EyesInfoKt");
  jfieldID eye_tracking_mode_static_fid = env->GetStaticFieldID(
      eye_tracking_mode_enum, "Companion",
      absl::StrFormat("L%s;", GetJxrFullClassName(env, PACKAGE_ARCORE_OPENXR,
                                                  "EyeTrackingState$Companion"))
          .c_str());
  jmethodID fromOpenXrEyeTrackingMode = env->GetStaticMethodID(
      eyes_info_class, "fromOpenXrEyeTrackingMode",
      absl::StrFormat(
          "(L%s;I)L%s;",
          GetJxrFullClassName(env, PACKAGE_ARCORE_OPENXR,
                              "EyeTrackingState$Companion"),
          GetJxrFullClassName(env, PACKAGE_ARCORE_OPENXR, "EyeTrackingState"))
          .c_str());
  jobject eye_tracking_mode_static_obj = env->GetStaticObjectField(
      eye_tracking_mode_enum, eye_tracking_mode_static_fid);
  return env->CallStaticObjectMethod(eyes_info_class, fromOpenXrEyeTrackingMode,
                                     eye_tracking_mode_static_obj,
                                     static_cast<uint32_t>(xr_mode));
}

jobject CreateJavaEye(JNIEnv* env, const XrEyeANDROID& xr_eye) {
  jclass eye_class = env->FindClass(
      GetJxrFullClassName(env, PACKAGE_ARCORE_OPENXR, "EyeData").c_str());
  jmethodID eye_constructor = env->GetMethodID(
      eye_class, "<init>",
      absl::StrFormat("(L%s;L%s;)V",
                      GetJxrFullClassName(env, PACKAGE_ARCORE_OPENXR, "EyeStatus"),
                      GetJxrFullClassName(env, PACKAGE_MATH, "Pose"))
          .c_str());
  return env->NewObject(eye_class, eye_constructor,
                        CreateJavaEyeState(env, xr_eye.eyeState),
                        CreateJavaPose(env, xr_eye.eyePose));
}

jobject CreateJavaEyesInfo(JNIEnv* env, const XrEyesANDROID& xr_eyes) {
  jclass eyes_info_class = env->FindClass(
      GetJxrFullClassName(env, PACKAGE_ARCORE_OPENXR, "EyesInfo").c_str());
  jclass eye_data_class = GetJxrClass(env, PACKAGE_ARCORE_OPENXR, "EyeData");
  jobjectArray eyes_array = env->NewObjectArray(2, eye_data_class, nullptr);
  for (uint32_t i = 0; i < 2; ++i) {
    env->SetObjectArrayElement(eyes_array, i,
                               CreateJavaEye(env, xr_eyes.eyes[i]));
  }
  jobject eye_tracking_mode = CreateJavaEyeTrackingState(env, xr_eyes.mode);
  jmethodID eyes_info_constructor = env->GetMethodID(
      eyes_info_class, "<init>",
      absl::StrFormat(
          "([L%s;L%s;)V",
          GetJxrFullClassName(env, PACKAGE_ARCORE_OPENXR, "EyeData"),
          GetJxrFullClassName(env, PACKAGE_ARCORE_OPENXR, "EyeTrackingState"))
          .c_str());
  return env->NewObject(eyes_info_class, eyes_info_constructor, eyes_array,
                        eye_tracking_mode);
}

jobject CreateJavaGeospatialPose(
    JNIEnv* env, const XrGeospatialPoseANDROID& xr_geospatial_pose) {
  jclass geospatial_pose_class =
      GetJxrClass(env, PACKAGE_MATH, "GeospatialPose");
  jmethodID geospatial_pose_constructor = env->GetMethodID(
      geospatial_pose_class, "<init>",
      absl::StrFormat("(DDDL%s;)V",
                      GetJxrFullClassName(env, PACKAGE_MATH, "Quaternion"))
          .c_str());
  jobject quaternion =
      CreateJavaQuaternion(env, xr_geospatial_pose.eastUpSouthOrientation);
  return env->NewObject(geospatial_pose_class, geospatial_pose_constructor,
                        xr_geospatial_pose.latitude,
                        xr_geospatial_pose.longitude,
                        xr_geospatial_pose.altitude, quaternion);
}

jobject CreateJavaGeospatialPoseResult(
    JNIEnv* env,
    const XrGeospatialPoseResultANDROID& xr_geospatial_pose_result) {
  jclass geospatial_pose_result_class = GetJxrClass(
      env, PACKAGE_ARCORE_RUNTIME, "Geospatial$GeospatialPoseResult");
  jmethodID geospatial_pose_result_constructor = env->GetMethodID(
      geospatial_pose_result_class, "<init>",
      absl::StrFormat("(L%s;DDD)V",
                      GetJxrFullClassName(env, PACKAGE_MATH, "GeospatialPose"))
          .c_str());
  jobject geospatial_pose =
      CreateJavaGeospatialPose(env, xr_geospatial_pose_result.geospatialPose);
  return env->NewObject(geospatial_pose_result_class,
                        geospatial_pose_result_constructor, geospatial_pose,
                        xr_geospatial_pose_result.horizontalAccuracy,
                        xr_geospatial_pose_result.verticalAccuracy,
                        xr_geospatial_pose_result.orientationYawAccuracy);
}

jobject CreateVpsAvailabilityResult(
    JNIEnv* env, jobject class_loader,
    const XrVPSAvailabilityCheckCompletionANDROID& completion) {
  if (completion.futureResult == XR_SUCCESS) {
    switch (completion.availability) {
      case XR_VPS_AVAILABILITY_AVAILABLE_ANDROID:
        return CreateVpsAvailabilityResultInstance(env, class_loader,
                                                   "VpsAvailabilityAvailable");
      case XR_VPS_AVAILABILITY_UNAVAILABLE_ANDROID:
        return CreateVpsAvailabilityResultInstance(
            env, class_loader, "VpsAvailabilityUnavailable");
      default:
        return CreateVpsAvailabilityResultInstance(
            env, class_loader, "VpsAvailabilityErrorInternal");
    }
  } else {
    // Detailed error information is linked in a chained struct from the
    // google_cloud_auth extension.
    const auto* next_struct =
        static_cast<const XrBaseOutStructure*>(completion.next);

    while (next_struct != nullptr &&
           next_struct->type !=
               XR_TYPE_GOOGLE_CLOUD_AUTH_ERROR_RESULT_ANDROID) {
      next_struct = static_cast<const XrBaseOutStructure*>(next_struct->next);
    }

    if (next_struct != nullptr &&
        next_struct->type == XR_TYPE_GOOGLE_CLOUD_AUTH_ERROR_RESULT_ANDROID) {
      const auto* auth_error =
          reinterpret_cast<const XrGoogleCloudAuthErrorResultANDROID*>(
              next_struct);
      switch (auth_error->error) {
        case XR_GOOGLE_CLOUD_AUTH_ERROR_QUOTA_EXCEEDED_ANDROID:
          return CreateVpsAvailabilityResultInstance(
              env, class_loader, "VpsAvailabilityResourceExhausted");
        case XR_GOOGLE_CLOUD_AUTH_ERROR_UNREACHABLE_ANDROID:
          return CreateVpsAvailabilityResultInstance(
              env, class_loader, "VpsAvailabilityNetworkError");
        case XR_GOOGLE_CLOUD_AUTH_ERROR_ANDROID:
          return CreateVpsAvailabilityResultInstance(
              env, class_loader, "VpsAvailabilityNotAuthorized");
        default:
          return CreateVpsAvailabilityResultInstance(
              env, class_loader, "VpsAvailabilityErrorInternal");
      }
    } else {
      return CreateVpsAvailabilityResultInstance(
          env, class_loader, "VpsAvailabilityErrorInternal");
    }
  }
}

jobject CreateJavaDisplayBlendMode(
    JNIEnv* env, const XrEnvironmentBlendMode& xr_blend_mode) {
  jclass blend_mode_ext_cls =
      GetJxrClass(env, PACKAGE_ARCORE_OPENXR, "OpenXrRuntimeKt");
  jclass blend_mode_enum = GetJxrClass(env, PACKAGE_CORE, "DisplayBlendMode");
  jfieldID blend_mode_static_fid = env->GetStaticFieldID(
      blend_mode_enum, "Companion",
      absl::StrFormat("L%s;", GetJxrFullClassName(env, PACKAGE_CORE,
                                                  "DisplayBlendMode$Companion"))
          .c_str());
  jmethodID fromOpenXrEnvironmentBlendMode = env->GetStaticMethodID(
      blend_mode_ext_cls, "fromOpenXrEnvironmentBlendMode",
      absl::StrFormat(
          "(L%s;I)L%s;",
          GetJxrFullClassName(env, PACKAGE_CORE, "DisplayBlendMode$Companion"),
          GetJxrFullClassName(env, PACKAGE_CORE, "DisplayBlendMode"))
          .c_str());
  jobject blend_mode_static_obj =
      env->GetStaticObjectField(blend_mode_enum, blend_mode_static_fid);
  return env->CallStaticObjectMethod(
      blend_mode_ext_cls, fromOpenXrEnvironmentBlendMode, blend_mode_static_obj,
      static_cast<uint32_t>(xr_blend_mode));
}

}  // namespace androidx::xr::openxr
