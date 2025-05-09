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

#ifndef JETPACK_XR_NATIVES_PERCEPTION_LIBRARY_JNI_UTILS_H_
#define JETPACK_XR_NATIVES_PERCEPTION_LIBRARY_JNI_UTILS_H_

#include <jni.h>
#include <openxr/openxr.h>

#include <vector>

namespace vr::realitycore {

// Returns a Java object of type `Pose` from an `XrPosef`.
jobject CreateJavaPose(JNIEnv* env, const XrPosef& xr_pose);

// Returns an `XrPosef` from a Java object of type `Pose`.
XrPosef CreateXrPose(JNIEnv* env, const jobject& java_pose);

// Returns a Java object of type `Fov` from an `XrFovf`.
jobject CreateJavaFov(JNIEnv* env, const XrFovf& fov);

// Returns a Java object of type `ViewProjection` from an `XrView`.
jobject CreateJavaViewProjection(JNIEnv* env, const XrView& view);

// Converts a C++ vector of trackables to a Java ArrayList of Longs.
jobject ConvertVectorToLongArrayList(
    JNIEnv* env, const std::vector<XrTrackableANDROID>& trackable_vector);

// Converts a C++ trackable plane to a Java PlaneData object.
jobject CreatePlaneData(JNIEnv* env, const XrTrackablePlaneANDROID& plane);

// Creates a Java AnchorData object from an anchor.
jobject CreateAnchorDataForAnchor(JNIEnv* env, jobject anchor_token,
                                  XrSpace anchor_id);

// Converts two longs to XrUuidEXT.
XrUuidEXT ConvertLongsToUuid(jlong high_bits, jlong low_bits);

// Converts a C++ enum XrAnchorPersistStateANDROID to a Java enum PersistState.
jobject ConvertXrAnchorPersistStateANDROIDToJavaPersistState(
    JNIEnv* env, XrAnchorPersistStateANDROID persist_state);
}  // namespace vr::realitycore

#endif  // JETPACK_XR_NATIVES_PERCEPTION_LIBRARY_JNI_UTILS_H_
