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
#include <openxr/public/all_extensions.h>

#include <cstdint>
#include <functional>

#include "absl/strings/str_format.h"
#include "common/namespace_util.h"
#include "openxr/jobject_converter.h"
#include "openxr/jobject_creator.h"
#include "openxr/openxr_manager.h"

using ::androidx::xr::common::Package::PACKAGE_ARCORE_RUNTIME;
using ::androidx::xr::openxr::CreateVpsAvailabilityResult;
using ::androidx::xr::openxr::OpenXrManager;

static jobject CreateJavaGeospatialState(
    JNIEnv* env,
    const androidx::xr::openxr::OpenXrManager::GeospatialState& earth_state) {
  jclass earth_state_class =
      GetJxrClass(env, PACKAGE_ARCORE_RUNTIME, "Geospatial$State");
  const char* field_name;
  switch (earth_state) {
    case OpenXrManager::GeospatialState::kRunning:
      field_name = "RUNNING";
      break;
    case OpenXrManager::GeospatialState::kStopped:
      field_name = "NOT_RUNNING";
      break;
    case OpenXrManager::GeospatialState::kErrorNotAuthorized:
      field_name = "ERROR_NOT_AUTHORIZED";
      break;
    case OpenXrManager::GeospatialState::kErrorAppPreempted:
      field_name = "PAUSED";
      break;
    case OpenXrManager::GeospatialState::kErrorResourcesExhausted:
      field_name = "ERROR_RESOURCE_EXHAUSTED";
      break;
    case OpenXrManager::GeospatialState::kErrorInternal:
    default:
      field_name = "ERROR_INTERNAL";
      break;
  }
  jfieldID field_id = env->GetStaticFieldID(
      earth_state_class, field_name,
      absl::StrFormat("L%s;", GetJxrFullClassName(env, PACKAGE_ARCORE_RUNTIME,
                                                  "Geospatial$State"))
          .c_str());
  return env->GetStaticObjectField(earth_state_class, field_id);
}

static jobject NativeGetGeospatialState(JNIEnv* env, jlong monotonic_time_ns) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  androidx::xr::openxr::OpenXrManager::GeospatialState earth_state =
      xr_manager.GetGeospatialState();
  return CreateJavaGeospatialState(env, earth_state);
}

static jobject NativeGetGeospatialPose(JNIEnv* env, jlong monotonic_time_ns,
                                       jobject pose) {
  OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  const XrTime time = static_cast<int64_t>(monotonic_time_ns);
  const XrPosef xr_pose = androidx::xr::openxr::ConvertToXrPosef(env, pose);

  XrGeospatialPoseResultANDROIDX2 geospatial_pose_result;

  OpenXrManager::GeospatialPoseResult result =
      xr_manager.LocateGeospatialPoseFromPose(time, xr_pose,
                                              &geospatial_pose_result);

  switch (result) {
    case OpenXrManager::GeospatialPoseResult::kSuccess:
      return androidx::xr::openxr::CreateJavaGeospatialPoseResult(
          env, geospatial_pose_result);
      break;
    case OpenXrManager::GeospatialPoseResult::kErrorIllegalState: {
      env->ThrowNew(env->FindClass("java/lang/IllegalStateException"),
                    "Geospatial not in running state.");
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

static jobject NativeLocatePoseFromGeospatialPose(JNIEnv* env,
                                                  jlong monotonic_time_ns,
                                                  jobject geospatial_pose_obj) {
  OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  const XrTime time = static_cast<int64_t>(monotonic_time_ns);
  const XrGeospatialPoseANDROIDX2 xr_geospatial_pose =
      androidx::xr::openxr::ConvertToXrGeospatialPose(env, geospatial_pose_obj);

  XrSpaceLocation location;
  OpenXrManager::GeospatialPoseResult result =
      xr_manager.LocatePoseFromGeospatialPose(time, xr_geospatial_pose,
                                              &location);

  switch (result) {
    case OpenXrManager::GeospatialPoseResult::kSuccess:
      return androidx::xr::openxr::CreateJavaPose(env, location.pose);
    case OpenXrManager::GeospatialPoseResult::kErrorIllegalState: {
      env->ThrowNew(env->FindClass("java/lang/IllegalStateException"),
                    "Geospatial not in running state.");
      return nullptr;
    }
    case OpenXrManager::GeospatialPoseResult::kErrorInvalidArgument: {
      env->ThrowNew(env->FindClass("java/lang/IllegalArgumentException"),
                    "Invalid argument provided to locate Pose.");
      return nullptr;
    }
    case OpenXrManager::GeospatialPoseResult::kErrorNotTracking:
      return nullptr;
  }
}

extern "C" {
JNIEXPORT jobject JNICALL
Java_androidx_xr_arcore_openxr_OpenXrGeospatial_nativeGetGeospatialState(
    JNIEnv* env, jclass /*clazz*/, jlong monotonic_time_ns) {
  return NativeGetGeospatialState(env, monotonic_time_ns);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_arcore_openxr_OpenXrGeospatial_nativeCreateGeospatialPoseFromPose(
    JNIEnv* env, jclass /*clazz*/, jlong monotonic_time_ns, jobject pose) {
  return NativeGetGeospatialPose(env, monotonic_time_ns, pose);
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_arcore_openxr_OpenXrGeospatial_nativeLocatePoseFromGeospatialPose(
    JNIEnv* env, jclass /*clazz*/, jlong monotonic_time_ns,
    jobject geospatial_pose) {
  return NativeLocatePoseFromGeospatialPose(env, monotonic_time_ns,
                                            geospatial_pose);
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_arcore_openxr_OpenXrGeospatial_nativeCreateAnchor(
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
      xr_manager.CreateGeospatialAnchor(static_cast<int64_t>(monotonic_time_ns),
                                        latitude, longitude, altitude,
                                        xr_quaternion, &anchor);

  if (result !=
      androidx::xr::openxr::OpenXrManager::CreateAnchorResult::kSuccess) {
    return static_cast<jlong>(result);
  }
  return androidx::xr::openxr::CreateJavaAnchorHandle(anchor);
}

JNIEXPORT void JNICALL
Java_androidx_xr_arcore_openxr_OpenXrGeospatial_nativeCheckVpsAvailabilityAsync(
    JNIEnv* env, jclass /*clazz*/, jdouble latitude, jdouble longitude,
    jobject java_callback) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();

  JavaVM* jvm;
  env->GetJavaVM(&jvm);
  // Create a global reference to the callback so that it is not garbage
  // collected until the callback is called. The global reference must be
  // deleted in the completion or cancellation callback.
  jobject java_callback_global = env->NewGlobalRef(java_callback);

  std::function<void(const XrVPSAvailabilityCheckCompletionANDROIDX2&)>
      on_complete = [jvm, java_callback_global](
                        const XrVPSAvailabilityCheckCompletionANDROIDX2&
                            completion) {
        JNIEnv* env;
        bool attached = false;
        if (jvm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) ==
            JNI_EDETACHED) {
          jvm->AttachCurrentThread(&env, nullptr);
          attached = true;
        }

        jobject result_obj = CreateVpsAvailabilityResult(env, completion);

        jclass function1_cls = env->FindClass("kotlin/jvm/functions/Function1");
        jmethodID invoke_mid = env->GetMethodID(
            function1_cls, "invoke", "(Ljava/lang/Object;)Ljava/lang/Object;");
        env->CallObjectMethod(java_callback_global, invoke_mid, result_obj);
        env->DeleteGlobalRef(java_callback_global);

        if (attached) {
          jvm->DetachCurrentThread();
        }
      };

  std::function<void()> on_cancel = [jvm, java_callback_global]() {
    JNIEnv* env;
    bool attached = false;
    if (jvm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) ==
        JNI_EDETACHED) {
      jvm->AttachCurrentThread(&env, nullptr);
      attached = true;
    }
    env->DeleteGlobalRef(java_callback_global);
    if (attached) {
      jvm->DetachCurrentThread();
    }
  };

  XrResult result = xr_manager.CheckVpsAvailabilityAsync(
      latitude, longitude, on_complete, on_cancel);
  if (result == XR_ERROR_GEOSPATIAL_COORDINATES_INVALID_ANDROIDX2) {
    env->ThrowNew(env->FindClass("java/lang/IllegalArgumentException"),
                  "Invalid latitude/longitude.");
    return;
  } else if (XR_FAILED(result)) {
    env->ThrowNew(env->FindClass("java/lang/IllegalStateException"),
                  "Check VPS availability failed.");
    return;
  }
}
}  // extern "C"
