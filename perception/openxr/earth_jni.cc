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

#include "absl/strings/str_format.h"
#include "common/namespace_util.h"
#include "openxr/jobject_converter.h"
#include "openxr/jobject_creator.h"
#include "openxr/openxr_manager.h"

using ::androidx::xr::common::Package::PACKAGE_ARCORE_RUNTIME;
using ::androidx::xr::openxr::OpenXrManager;

static jobject CreateJavaEarthState(
    JNIEnv* env,
    const androidx::xr::openxr::OpenXrManager::EarthState& earth_state) {
  jclass earth_state_class =
      GetJxrClass(env, PACKAGE_ARCORE_RUNTIME, "Earth$State");
  const char* field_name;
  switch (earth_state) {
    case OpenXrManager::EarthState::kRunning:
      field_name = "RUNNING";
      break;
    case OpenXrManager::EarthState::kStopped:
      field_name = "STOPPED";
      break;
    case OpenXrManager::EarthState::kErrorNotAuthorized:
      field_name = "ERROR_NOT_AUTHORIZED";
      break;
    case OpenXrManager::EarthState::kErrorResourcesExhausted:
      field_name = "ERROR_RESOURCES_EXHAUSTED";
      break;
    case OpenXrManager::EarthState::kErrorInternal:
    default:
      field_name = "ERROR_INTERNAL";
      break;
  }
  jfieldID field_id = env->GetStaticFieldID(
      earth_state_class, field_name,
      absl::StrFormat("L%s;", GetJxrFullClassName(env, PACKAGE_ARCORE_RUNTIME,
                                                  "Earth$State"))
          .c_str());
  return env->GetStaticObjectField(earth_state_class, field_id);
}

static jobject NativeGetEarthState(JNIEnv* env, jlong monotonic_time_ns) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  androidx::xr::openxr::OpenXrManager::EarthState earth_state =
      xr_manager.GetEarthState();
  return CreateJavaEarthState(env, earth_state);
}

static jobject NativeGetGeospatialPose(JNIEnv* env, jlong monotonic_time_ns,
                                       jobject pose) {
  OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  const XrTime time = xr_manager.GetXrTimeFromNanoseconds(monotonic_time_ns);
  const XrPosef xr_pose = androidx::xr::openxr::ConvertToXrPosef(env, pose);

  XrGeospatialPoseResultANDROIDX1 geospatial_pose_result;

  OpenXrManager::GeospatialPoseResult result =
      xr_manager.LocateGeospatialPose(time, xr_pose, &geospatial_pose_result);

  switch (result) {
    case OpenXrManager::GeospatialPoseResult::kSuccess:
      return androidx::xr::openxr::CreateJavaGeospatialPoseResult(
          env, geospatial_pose_result);
      break;
    case OpenXrManager::GeospatialPoseResult::kErrorIllegalState: {
      env->ThrowNew(env->FindClass("java/lang/IllegalStateException"),
                    "Earth not in running state.");
      return nullptr;
    }
    case OpenXrManager::GeospatialPoseResult::kErrorInvalidArgument: {
      env->ThrowNew(env->FindClass("java/lang/IllegalArgumentException"),
                    "Invalid argument provided to get GeospatialPose.");
      return nullptr;
    }
    case OpenXrManager::GeospatialPoseResult::kErrorNotTracking:
      return nullptr;
  }
}

extern "C" {
JNIEXPORT jobject JNICALL
Java_androidx_xr_arcore_openxr_OpenXrEarth_nativeGetEarthState(
    JNIEnv* env, jclass /*clazz*/, jlong monotonic_time_ns) {
  return NativeGetEarthState(env, monotonic_time_ns);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_arcore_openxr_OpenXrEarth_nativeCreateGeospatialPoseFromPose(
    JNIEnv* env, jclass /*clazz*/, jlong monotonic_time_ns, jobject pose) {
  return NativeGetGeospatialPose(env, monotonic_time_ns, pose);
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_arcore_openxr_OpenXrEarth_nativeCreateAnchor(
    JNIEnv* env, jclass /*clazz*/, jdouble latitude, jdouble longitude,
    jdouble altitude, jobject eastUpSouthQuaternion_obj,
    jlong monotonic_time_ns) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();

  // Convert eastUpSouthQuaternion_obj to XrQuaternionf
  XrQuaternionf xr_quaternion = androidx::xr::openxr::ConvertToXrQuaternionf(
      env, eastUpSouthQuaternion_obj);

  XrSpace anchor;
  androidx::xr::openxr::OpenXrManager::CreateAnchorResult result =
      xr_manager.CreateEarthAnchor(static_cast<int64_t>(monotonic_time_ns),
                                   latitude, longitude, altitude, xr_quaternion,
                                   &anchor);

  if (result !=
      androidx::xr::openxr::OpenXrManager::CreateAnchorResult::kSuccess) {
    return static_cast<jlong>(result);
  }
  return androidx::xr::openxr::CreateJavaAnchorHandle(anchor);
}
}  // extern "C"
