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

#ifndef JETPACK_XR_NATIVES_OPENXR_JOBJECT_CREATOR_H_
#define JETPACK_XR_NATIVES_OPENXR_JOBJECT_CREATOR_H_

#include <jni.h>

#include <cstdint>

#include <openxr/openxr.h>

namespace androidx::xr::openxr {

// Returns a JVM object of type `androidx.xr.math.Vector2` from an
// `XrExtent2Df`.
jobject CreateJavaVector2(JNIEnv* env, const XrExtent2Df& xr_extent);

// Returns a JVM object of type `androidx.xr.math.Vector2` from an
// `XrVector2f`.
jobject CreateJavaVector2(JNIEnv* env, const XrVector2f& xr_vector);

// Returns a JVM object of type `androidx.xr.math.Vector3` from an `XrVector3f`.
jobject CreateJavaVector3(JNIEnv* env, const XrVector3f& xr_vector);

// Returns a JVM object of type `androidx.xr.math.Quaternion` from an
// `XrQuaternionf`.
jobject CreateJavaQuaternion(JNIEnv* env, const XrQuaternionf& xr_quaternion);

// Returns a JVM object of type `androidx.xr.math.Pose` from an `XrPosef`.
jobject CreateJavaPose(JNIEnv* env, const XrPosef& xr_pose);

// Returns a JVM object of type `androidx.xr.openxr.PlaneState` from an
// `XrTrackablePlaneANDROID`.
jobject CreateJavaPlaneState(JNIEnv* env,
                             const XrTrackablePlaneANDROID& xr_plane);

// Returns a JVM object of type `androidx.xr.openxr.OpenXrPlane.Label` from an
// `XrPlaneLabelANDROID`.
jobject CreateJavaPlaneLabel(JNIEnv* env,
                             const XrPlaneLabelANDROID& xr_plane_label);

// Returns a JVM object array of object type `androidx.xr.math.Vector2` from an
// `XrVector2f*` and `uint32_t` array count.
jobjectArray CreateJavaPlaneVertices(JNIEnv* env, uint32_t vertex_count,
                                     const XrVector2f* vertices);

// Returns a JVM object of type `androidx.xr.openxr.AnchorState` from an
// `XrSpaceLocation`.
jobject CreateJavaAnchorState(JNIEnv* env,
                              const XrSpaceLocation& anchor_location);

// Returns a JVM object of type `androidx.xr.openxr.HitData` from an
// `XrRaycastHitResultANDROID`.
jobject CreateJavaHitData(JNIEnv* env,
                          const XrRaycastHitResultANDROID& xr_hit_result);

// Returns a JVM object of type `androidx.xr.openxr.Anchor.PersistenceState`
// from an `XrAnchorPersistStateANDROID`.
jobject CreateJavaAnchorPersistenceState(
    JNIEnv* env, const XrAnchorPersistStateANDROID& xr_anchor_persist_state);

// Returns a JVM object of type `long` from an `XrSpace`.
jlong CreateJavaAnchorHandle(const XrSpace& xr_space);

}  // namespace androidx::xr::openxr

#endif  // JETPACK_XR_NATIVES_OPENXR_JOBJECT_CREATOR_H_
