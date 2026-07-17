// Copyright 2026 Google LLC
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

#ifndef JETPACK_XR_NATIVES_OPENXR_JOBJECT_CREATOR_H_
#define JETPACK_XR_NATIVES_OPENXR_JOBJECT_CREATOR_H_

#include <jni.h>
#include <openxr/openxr.h>
#include <openxr/public/all_extensions.h>

#include <cstdint>

#include "openxr/openxr_manager_utils.h"

namespace androidx::xr::openxr {

// Returns a JVM object of type `androidx.xr.runtime.math.FloatSize2d` from an
// `XrExtent2Df`.
jobject CreateJavaFloatSize2d(JNIEnv* env, const XrExtent2Df& xr_extent);

// Returns a JVM object of type `androidx.xr.runtime.math.Vector2` from an
// `XrVector2f`.
jobject CreateJavaVector2(JNIEnv* env, const XrVector2f& xr_vector);

// Returns a JVM object of type `androidx.xr.runtime.math.Vector3` from an
// `XrVector3f`.
jobject CreateJavaVector3(JNIEnv* env, const XrVector3f& xr_vector);

// Returns a JVM object of type `androidx.xr.runtime.math.Quaternion` from an
// `XrQuaternionf`.
jobject CreateJavaQuaternion(JNIEnv* env, const XrQuaternionf& xr_quaternion);

// Returns a JVM object of type `androidx.xr.runtime.math.Pose` from an
// `XrPosef`.
jobject CreateJavaPose(JNIEnv* env, const XrPosef& xr_pose);

// Returns a JVM object of type `androidx.xr.runtime.math.FieldOfView` from an
// `XrFovf`.
jobject CreateJavaFieldOfView(JNIEnv* env, const XrFovf& fov);

// Returns a JVM object array of type `androidx.xr.openxr.ViewCameraState` from
// an `XrView*` and `uint32_t` array count.
jobjectArray CreateJavaViewCameraStates(JNIEnv* env, uint32_t view_count,
                                       XrView* views);

// Returns a JVM object of type `androidx.xr.openxr.PlaneState` from an
// `XrTrackablePlaneANDROID`.
jobject CreateJavaPlaneState(JNIEnv* env,
                             const XrTrackablePlaneANDROID& xr_plane);

// Returns a JVM object of type `androidx.xr.openxr.OpenXrPlane.Label` from an
// `XrPlaneLabelANDROID`.
jobject CreateJavaPlaneLabel(JNIEnv* env,
                             const XrPlaneLabelANDROID& xr_plane_label);

// Returns a JVM object array of object type `androidx.xr.runtime.math.Vector2`
// from an `XrVector2f*` and `uint32_t` array count.
jobjectArray CreateJavaPlaneVertices(JNIEnv* env, uint32_t vertex_count,
                                     const XrVector2f* vertices);

// Returns a JVM object of type `androidx.xr.openxr.AugmentedObjectState` from
// an `XrTrackableObjectANDROID`.
jobject CreateJavaAugmentedObjectState(JNIEnv* env,
                                     const XrTrackableObjectANDROID& xr_object);

// Returns a JVM object of type `androidx.xr.runtime.openxr.AugmentedImageState`
// from an `XrTrackableImageANDROID`.
jobject CreateJavaAugmentedImageState(
    JNIEnv* env, const XrTrackableImageANDROID& xr_image);

// Returns a JVM object of type `androidx.xr.openxr.AnchorState` from an
// `XrSpaceLocation`.
jobject CreateJavaAnchorState(JNIEnv* env,
                              const XrSpaceLocation& anchor_location);

// Returns a JVM object of type `androidx.xr.openxr.DeviceState`.
jobject CreateJavaDeviceState(JNIEnv* env, TrackingState head_tracking_state,
                              XrPosef pose);

// Returns a JVM object of type `androidx.xr.openxr.HitData` from an
// `XrRaycastHitResultANDROID`.
jobject CreateJavaHitData(JNIEnv* env,
                          const XrRaycastHitResultANDROID& xr_hit_result);

// Returns a JVM object of type `androidx.xr.openxr.Anchor.PersistenceState`
// from an `XrAnchorPersistStateANDROID`.
jobject CreateJavaAnchorPersistenceState(
    JNIEnv* env, const XrAnchorPersistStateANDROID& xr_anchor_persist_state);

// Returns a JVM object of type `long` from an `XrSpace`.
jlong CreateJavaAnchorHandle(XrSpace xr_space);

// Returns a JVM object of type `androidx.xr.runtime.openxr.FaceState` from an
// `XrFaceStateANDROID`.
jobject CreateJavaFaceState(JNIEnv* env, const XrFaceStateANDROID& xr_face);

// Returns a JVM object of type `androidx.xr.runtime.TrackingState` from a
// TrackingState enum.
jobject CreateJavaTrackingState(JNIEnv* env, TrackingState tracking_state);

// Returns a JVM object of type `androidx.xr.runtime.TrackingState` from an
// `XrFaceTrackingStateANDROID`.
jobject CreateJavaTrackingState(
    JNIEnv* env, const XrFaceTrackingStateANDROID& xr_face_tracking_state);

// Returns a JVM object of type `androidx.xr.runtime.TrackingState` from an
// `XrTrackingStateANDROID`.
jobject CreateJavaTrackingState(
    JNIEnv* env, const XrTrackingStateANDROID& xr_tracking_state);

// Returns a JVM object of type `androidx.xr.runtime.TrackingState` from an
// `XrSpaceLocationFlags`.
jobject CreateJavaTrackingState(JNIEnv* env,
                                const XrSpaceLocationFlags& location_flags);

// Prevents dangerous implicit conversions of other C-style enums to int.
template <typename T>
jobject CreateJavaTrackingState(JNIEnv* env, T state) = delete;

// Returns a JVM object of type `androidx.xr.runtime.math.IntSize2d` from two
// `int`s.
jobject CreateJavaIntSize2d(JNIEnv* env, int width, int height);

// Returns a JVM object of type `androidx.xr.runtime.math.FloatSize3d` from
// an `XrExtent3Df`.
jobject CreateJavaFloatSize3d(JNIEnv* env, const XrExtent3Df& xr_extent);

// Returns a JVM object of type `androidx.xr.runtime.openxr.EyeState` from an
// `XrEyeStateANDROID`.
jobject CreateJavaEyeState(JNIEnv* env, const XrEyeStateANDROID& xr_eye_state);

// Returns a JVM object of type `androidx.xr.runtime.openxr.EyeTrackingMode`
// from an `XrEyeTrackingModeANDROID`.
jobject CreateJavaEyeTrackingMode(JNIEnv* env,
                                  const XrEyeTrackingModeANDROID& xr_mode);

// Returns a JVM object of type `androidx.xr.runtime.openxr.Eye` from an
// `XrEyeStateANDROID`.
jobject CreateJavaEye(JNIEnv* env, const XrEyeANDROID& xr_eye);

// Returns a JVM object of type `androidx.xr.runtime.openxr.EyeInfo` from an
// `XrEyesANDROID`.
jobject CreateJavaEyesInfo(JNIEnv* env, const XrEyesANDROID& xr_eyes);

// Returns a JVM object of type `androidx.xr.runtime.math.GeospatialPose` from
// an `XrGeospatialPoseANDROID`.
jobject CreateJavaGeospatialPose(
    JNIEnv* env, const XrGeospatialPoseANDROID& xr_geospatial_pose);

// Returns a JVM object of type
// `androidx.xr.arcore.runtime.GeospatialPoseResult` from an
// `XrGeospatialPoseResultANDROID`.
jobject CreateJavaGeospatialPoseResult(
    JNIEnv* env,
    const XrGeospatialPoseResultANDROID& xr_geospatial_pose_result);

// Returns a JVM object of type `androidx.xr.runtime.XrDevice.DisplayBlendMode`
// from an XrEnvironmentBlendMode.
jobject CreateJavaDisplayBlendMode(JNIEnv* env,
                                   const XrEnvironmentBlendMode& xr_blend_mode);

// Returns a JVM object of type `androidx.xr.runtime.VpsAvailability*` from an
// `XrVPSAvailabilityCheckCompletionANDROID`.
jobject CreateVpsAvailabilityResult(
    JNIEnv* env, jobject class_loader,
    const XrVPSAvailabilityCheckCompletionANDROID& completion);

// Returns a JVM object of type `androidx.xr.openxr.QrCodeState` from an
// `XrTrackableQrCodeANDROID`.
jobject CreateJavaQrCodeState(JNIEnv* env,
                              const XrTrackableQrCodeANDROID& xr_qr_code);

}  // namespace androidx::xr::openxr

#endif  // JETPACK_XR_NATIVES_OPENXR_JOBJECT_CREATOR_H_
