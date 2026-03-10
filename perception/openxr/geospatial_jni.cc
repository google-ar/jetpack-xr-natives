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
using ::androidx::xr::openxr::ConvertToXrGeospatialPose;
using ::androidx::xr::openxr::ConvertToXrPosef;
using ::androidx::xr::openxr::ConvertToXrQuaternionf;
using ::androidx::xr::openxr::CreateJavaAnchorHandle;
using ::androidx::xr::openxr::CreateJavaGeospatialPoseResult;
using ::androidx::xr::openxr::CreateJavaPose;
using ::androidx::xr::openxr::CreateVpsAvailabilityResult;
using ::androidx::xr::openxr::OpenXrManager;

static jobject CreateJavaGeospatialState(
    JNIEnv* env, const OpenXrManager::GeospatialState& earth_state) {
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
  OpenXrManager& xr_manager = OpenXrManager::GetOpenXrManager();
  OpenXrManager::GeospatialState earth_state = xr_manager.GetGeospatialState();
  return CreateJavaGeospatialState(env, earth_state);
}

static jobject NativeGetGeospatialPose(JNIEnv* env, jlong monotonic_time_ns,
                                       jobject pose) {
  OpenXrManager& xr_manager = OpenXrManager::GetOpenXrManager();
  const XrTime time = static_cast<int64_t>(monotonic_time_ns);
  const XrPosef xr_pose = ConvertToXrPosef(env, pose);

  XrGeospatialPoseResultANDROID geospatial_pose_result;

  OpenXrManager::GeospatialPoseResult result =
      xr_manager.LocateGeospatialPoseFromPose(time, xr_pose,
                                              &geospatial_pose_result);

  switch (result) {
    case OpenXrManager::GeospatialPoseResult::kSuccess:
      return CreateJavaGeospatialPoseResult(env, geospatial_pose_result);
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
  OpenXrManager& xr_manager = OpenXrManager::GetOpenXrManager();
  const XrTime time = static_cast<int64_t>(monotonic_time_ns);
  const XrGeospatialPoseANDROID xr_geospatial_pose =
      ConvertToXrGeospatialPose(env, geospatial_pose_obj);

  XrSpaceLocation location;
  OpenXrManager::GeospatialPoseResult result =
      xr_manager.LocatePoseFromGeospatialPose(time, xr_geospatial_pose,
                                              &location);

  switch (result) {
    case OpenXrManager::GeospatialPoseResult::kSuccess:
      return CreateJavaPose(env, location.pose);
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
  OpenXrManager& xr_manager = OpenXrManager::GetOpenXrManager();

  // Convert eastUpSouthQuaternion_obj to XrQuaternionf
  XrQuaternionf xr_quaternion =
      ConvertToXrQuaternionf(env, eastUpSouthQuaternion_obj);

  XrSpace anchor;
  OpenXrManager::CreateAnchorResult result = xr_manager.CreateGeospatialAnchor(
      static_cast<int64_t>(monotonic_time_ns), latitude, longitude, altitude,
      xr_quaternion, &anchor);

  if (result != OpenXrManager::CreateAnchorResult::kSuccess) {
    if (result ==
        OpenXrManager::CreateAnchorResult::kErrorGeospatialTrackerNotRunning) {
      env->ThrowNew(env->FindClass("java/lang/IllegalStateException"),
                    "Geospatial not in running state.");
      return 0;
    }
    if (result ==
        OpenXrManager::CreateAnchorResult::kErrorGeospatialCoordinatesInvalid) {
      env->ThrowNew(env->FindClass("java/lang/IllegalArgumentException"),
                    "Invalid latitude/longitude.");
      return 0;
    }

    return static_cast<jlong>(result);
  }
  return CreateJavaAnchorHandle(anchor);
}

JNIEXPORT void JNICALL
Java_androidx_xr_arcore_openxr_OpenXrGeospatial_nativeCheckVpsAvailabilityAsync(
    JNIEnv* env, jclass /*clazz*/, jdouble latitude, jdouble longitude,
    jobject java_callback) {
  OpenXrManager& xr_manager = OpenXrManager::GetOpenXrManager();

  JavaVM* jvm;
  env->GetJavaVM(&jvm);
  // Create a global reference to the callback so that it is not garbage
  // collected until the callback is called. The global reference must be
  // deleted in the completion or cancellation callback.
  jobject java_callback_global = env->NewGlobalRef(java_callback);

  // Cache the class loader, since the default class loader from the attached
  // thread cannot load the Jetpack classes.
  jclass callback_cls = env->GetObjectClass(java_callback);
  jclass class_cls = env->FindClass("java/lang/Class");
  jmethodID get_class_loader_mid = env->GetMethodID(
      class_cls, "getClassLoader", "()Ljava/lang/ClassLoader;");
  jobject class_loader_obj =
      env->CallObjectMethod(callback_cls, get_class_loader_mid);
  jobject class_loader_global = env->NewGlobalRef(class_loader_obj);
  env->DeleteLocalRef(class_loader_obj);
  env->DeleteLocalRef(class_cls);
  env->DeleteLocalRef(callback_cls);

  std::function<void(const XrVPSAvailabilityCheckCompletionANDROID&)>
      on_complete =
          [jvm, java_callback_global, class_loader_global](
              const XrVPSAvailabilityCheckCompletionANDROID& completion) {
            JNIEnv* env;
            bool attached = false;
            if (jvm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) ==
                JNI_EDETACHED) {
              jvm->AttachCurrentThread(&env, nullptr);
              attached = true;
            }

            jobject result_obj = CreateVpsAvailabilityResult(
                env, class_loader_global, completion);

            jclass function1_cls = env->GetObjectClass(java_callback_global);
            jmethodID invoke_mid =
                env->GetMethodID(function1_cls, "invoke",
                                 "(Ljava/lang/Object;)Ljava/lang/Object;");
            env->CallObjectMethod(java_callback_global, invoke_mid, result_obj);
            env->DeleteLocalRef(function1_cls);
            env->DeleteGlobalRef(java_callback_global);
            env->DeleteGlobalRef(class_loader_global);

            if (attached) {
              jvm->DetachCurrentThread();
            }
          };

  std::function<void()> on_cancel = [jvm, java_callback_global,
                                     class_loader_global]() {
    JNIEnv* env;
    bool attached = false;
    if (jvm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) ==
        JNI_EDETACHED) {
      jvm->AttachCurrentThread(&env, nullptr);
      attached = true;
    }
    env->DeleteGlobalRef(java_callback_global);
    env->DeleteGlobalRef(class_loader_global);
    if (attached) {
      jvm->DetachCurrentThread();
    }
  };

  XrResult result = xr_manager.CheckVpsAvailabilityAsync(
      latitude, longitude, on_complete, on_cancel);
  if (XR_FAILED(result)) {
    if (result == XR_ERROR_GEOSPATIAL_COORDINATES_INVALID_ANDROID) {
      env->ThrowNew(env->FindClass("java/lang/IllegalArgumentException"),
                    "Invalid latitude/longitude.");
      return;
    } else if (XR_FAILED(result)) {
      env->ThrowNew(env->FindClass("java/lang/IllegalStateException"),
                    "Check VPS availability failed.");
    }

    env->DeleteGlobalRef(java_callback_global);
    env->DeleteGlobalRef(class_loader_global);
    return;
  }
}

JNIEXPORT void JNICALL
Java_androidx_xr_arcore_openxr_OpenXrGeospatial_nativeCreateSurfaceAnchorAsync(
    JNIEnv* env, jclass /*clazz*/, jint surfaceAnchorType, jdouble latitude,
    jdouble longitude, jdouble altitudeRelativeToSurface,
    jobject eastUpSouthQuaternion_obj, jobject java_callback) {
  OpenXrManager& xr_manager = OpenXrManager::GetOpenXrManager();

  JavaVM* jvm;
  env->GetJavaVM(&jvm);
  // Create a global reference to the callback so that it is not garbage
  // collected until the callback is called. The global reference must be
  // deleted in the completion or cancellation callback.
  jobject java_callback_global = env->NewGlobalRef(java_callback);

  XrQuaternionf xr_quaternion =
      ConvertToXrQuaternionf(env, eastUpSouthQuaternion_obj);

  std::function<void(const XrSurfaceAnchorCreateCompletionANDROID&)>
      on_complete =
          [jvm, java_callback_global](
              const XrSurfaceAnchorCreateCompletionANDROID& completion) {
            JNIEnv* env;
            bool attached = false;
            if (jvm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) ==
                JNI_EDETACHED) {
              jvm->AttachCurrentThread(&env, nullptr);
              attached = true;
            }

            OpenXrManager& xr_manager = OpenXrManager::GetOpenXrManager();

            jlong anchor_handle = 0;
            if (completion.futureResult == XR_SUCCESS) {
              XrSpace anchor_space = XR_NULL_HANDLE;

              OpenXrManager::CreateAnchorResult result =
                  xr_manager.CreateAnchorSpaceFromEntityId(
                      completion.anchorEntityId, &anchor_space);

              if (result == OpenXrManager::CreateAnchorResult::kSuccess) {
                anchor_handle = CreateJavaAnchorHandle(anchor_space);
              } else {
                anchor_handle = static_cast<jlong>(result);
              }
            } else {
              OpenXrManager::CreateAnchorResult result =
                  OpenXrManager::MapAnchorCreateResult(completion.futureResult);
              anchor_handle = static_cast<jlong>(result);
            }

            jclass long_cls = env->FindClass("java/lang/Long");
            jmethodID long_init = env->GetMethodID(long_cls, "<init>", "(J)V");
            jobject result_obj =
                env->NewObject(long_cls, long_init, anchor_handle);

            jclass function1_cls = env->GetObjectClass(java_callback_global);
            jmethodID invoke_mid =
                env->GetMethodID(function1_cls, "invoke",
                                 "(Ljava/lang/Object;)Ljava/lang/Object;");
            env->CallObjectMethod(java_callback_global, invoke_mid, result_obj);
            env->DeleteLocalRef(function1_cls);
            env->DeleteLocalRef(long_cls);
            env->DeleteLocalRef(result_obj);
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

  OpenXrManager::CreateAnchorResult result =
      xr_manager.CreateSurfaceAnchorAsync(
          static_cast<XrSurfaceAnchorTypeANDROID>(surfaceAnchorType), latitude,
          longitude, altitudeRelativeToSurface, xr_quaternion, on_complete,
          on_cancel);

  if (result != OpenXrManager::CreateAnchorResult::kSuccess) {
    env->DeleteGlobalRef(java_callback_global);

    if (result ==
        OpenXrManager::CreateAnchorResult::kErrorGeospatialCoordinatesInvalid) {
      env->ThrowNew(env->FindClass("java/lang/IllegalArgumentException"),
                    "Invalid latitude/longitude.");
      return;
    }

    if (result ==
        OpenXrManager::CreateAnchorResult::kErrorGeospatialTrackerNotRunning) {
      env->ThrowNew(env->FindClass("java/lang/IllegalStateException"),
                    "Geospatial tracker is not running.");
      return;
    }

    if (result == OpenXrManager::CreateAnchorResult::kErrorLimitReached) {
      env->ThrowNew(GetJxrClass(env, PACKAGE_ARCORE_RUNTIME,
                                "AnchorResourcesExhaustedException"),
                    "Unable to create anchor. Anchor resources exhausted.");
      return;
    }

    env->ThrowNew(env->FindClass("java/lang/IllegalStateException"),
                  "Create surface anchor async failed.");
    return;
  }
}

}  // extern "C"
