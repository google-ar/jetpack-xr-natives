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

#include <functional>
#include <vector>

#include "openxr/openxr.h"
#include "openxr/public/all_extensions.h"
#include "absl/log/log.h"
#include "common/pointer_util.h"
#include "openxr/jobject_creator.h"
#include "openxr/openxr_manager.h"

namespace {
void HandleAuthCompletion(const XrFutureCompletionEXT& completion) {
  // Auth is set internally, so there is no app-initiated operation to return
  // the failure result to, so we will just log it instead. Auth-bound
  // operations like geospatial setup should also show useful error messages.
  if (XR_FAILED(completion.futureResult)) {
    if (completion.futureResult == XR_ERROR_KEYLESS_AUTH_FAILED_ANDROID) {
      LOG(ERROR) << "Google Cloud Auth future failed: Keyless Auth failed. The "
                    "application can try again later.";
    } else {
      LOG(ERROR) << "Google Cloud Auth future failed with result: "
                 << completion.futureResult;
    }
    return;
  }
}
}  // namespace

extern "C" {
JNIEXPORT jlong JNICALL
Java_androidx_xr_arcore_openxr_OpenXrManager_nativeGetXrSessionHandle(
  JNIEnv* env, jclass /*clazz*/) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  XrSession xr_session = xr_manager.GetXrSession();
  return androidx::xr::common::PointerToJLong(&xr_session);
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_arcore_openxr_OpenXrManager_nativeGetXrInstanceHandle(
JNIEnv* env, jclass /*clazz*/) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  XrInstance xr_instance = xr_manager.GetXrInstance();
  return androidx::xr::common::PointerToJLong(&xr_instance);
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_arcore_openxr_OpenXrManager_nativeGetPointer(
    JNIEnv* env, jclass /*clazz*/) {
  return androidx::xr::common::PointerToJLong(
      &androidx::xr::openxr::OpenXrManager::GetOpenXrManager());
}

JNIEXPORT jboolean JNICALL
Java_androidx_xr_arcore_openxr_OpenXrManager_nativeInit(
    JNIEnv* env, jclass /*clazz*/, jobject context,
    jboolean start_polling_thread) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  return xr_manager.Init(env, context,
                         XR_REFERENCE_SPACE_TYPE_UNBOUNDED_ANDROID,
                         start_polling_thread);
}

JNIEXPORT void JNICALL
Java_androidx_xr_arcore_openxr_OpenXrManager_nativeDeInit(JNIEnv* env,
                                                           jclass /*clazz*/) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  xr_manager.DeInit(/*stop_polling_thread=*/true);
}

JNIEXPORT jboolean JNICALL
Java_androidx_xr_arcore_openxr_OpenXrManager_nativePause(JNIEnv* env,
                                                          jclass /*clazz*/) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  return xr_manager.PauseSession();
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_arcore_openxr_OpenXrManager_nativeConfigureSession(
    JNIEnv* env, jclass /*clazz*/, jint plane_tracking, jint hand_tracking,
    jint head_tracking, jint depth_estimation, jint anchor_persistence,
    jint face_tracking, jint eye_tracking, jint object_tracking,
    jlongArray object_tracking_labels, jint geospatial_tracking) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();

  androidx::xr::openxr::OpenXrManager::ConfigSettings xr_config = {
      .plane_tracking_mode =
          static_cast<androidx::xr::openxr::OpenXrManager::PlaneTrackingMode>(
              plane_tracking),
      .hand_tracking_mode =
          static_cast<androidx::xr::openxr::OpenXrManager::HandTrackingMode>(
              hand_tracking),
      .head_tracking_mode =
          static_cast<androidx::xr::openxr::OpenXrManager::HeadTrackingMode>(
              head_tracking),
      .depth_estimation_mode =
          static_cast<androidx::xr::openxr::OpenXrManager::DepthEstimationMode>(
              depth_estimation),
      .anchor_persistence_mode = static_cast<
          androidx::xr::openxr::OpenXrManager::AnchorPersistenceMode>(
          anchor_persistence),
      .face_tracking_mode =
          static_cast<androidx::xr::openxr::OpenXrManager::FaceTrackingMode>(
              face_tracking),
      .object_tracking_mode =
          static_cast<androidx::xr::openxr::OpenXrManager::ObjectTrackingMode>(
              object_tracking),
      .eye_tracking_mode =
          static_cast<androidx::xr::openxr::OpenXrManager::EyeTrackingMode>(
              eye_tracking),
      .object_tracking_labels = {},
      .geospatial_mode =
          static_cast<androidx::xr::openxr::OpenXrManager::GeospatialMode>(
              geospatial_tracking),
  };

  if (object_tracking_labels == nullptr) {
    xr_config.object_tracking_mode =
        androidx::xr::openxr::OpenXrManager::ObjectTrackingMode::kDisabled;
  } else {
    const auto labels_length = env->GetArrayLength(object_tracking_labels);
    if (labels_length > 0) {
      xr_config.object_tracking_labels.reserve(labels_length);
      const auto labels = env->GetLongArrayElements(object_tracking_labels,
                                                    /*isCopy=*/nullptr);
      for (auto i = 0u; i < labels_length; ++i) {
        xr_config.object_tracking_labels.push_back(
            static_cast<XrObjectLabelANDROID>(labels[i]));
      }
      env->ReleaseLongArrayElements(object_tracking_labels, labels, JNI_ABORT);
    } else {
      xr_config.object_tracking_mode =
          androidx::xr::openxr::OpenXrManager::ObjectTrackingMode::kDisabled;
    }
  }

  return xr_manager.ConfigureSession(xr_config);
}

JNIEXPORT jboolean JNICALL
Java_androidx_xr_arcore_openxr_OpenXrManager_nativeGetFaceTrackerCalibration(
    JNIEnv* env, jclass /*clazz*/) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  return xr_manager.IsFaceTrackerCalibrated();
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_arcore_openxr_OpenXrRuntime_nativeGetPreferredBlendMode(
    JNIEnv* env, jclass /*clazz*/) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  std::vector<XrEnvironmentBlendMode> blend_modes;

  xr_manager.GetEnvironmentBlendModes(&blend_modes);
  // Return the first blend mode that isn't opaque, or nullptr if none.
  for (auto i = 0u; i < blend_modes.size(); ++i) {
    if (blend_modes[i] != XR_ENVIRONMENT_BLEND_MODE_OPAQUE) {
      return androidx::xr::openxr::CreateJavaDisplayBlendMode(env,
                                                              blend_modes[i]);
    }
  }
  return nullptr;
}

JNIEXPORT jboolean JNICALL
Java_androidx_xr_arcore_openxr_OpenXrRuntime_nativeIsGeospatialSupported(
    JNIEnv* env, jclass /*clazz*/) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();

  return xr_manager.IsGeospatialSupported();
}

JNIEXPORT void JNICALL
Java_androidx_xr_arcore_openxr_OpenXrManager_nativeSetApiKeyAuth(
    JNIEnv* env, jclass /*clazz*/, jstring api_key) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();

  const char* api_key_chars = env->GetStringUTFChars(api_key, nullptr);
  XrGoogleCloudAuthApiKeyANDROID auth_info = {
      .type = XR_TYPE_GOOGLE_CLOUD_AUTH_API_KEY_ANDROID,
      .next = nullptr,
      .apiKey = api_key_chars,
  };

  XrResult result = xr_manager.SetGoogleCloudAuthAsync(
      reinterpret_cast<XrGoogleCloudAuthInfoBaseHeaderANDROID*>(&auth_info),
      HandleAuthCompletion, /*on_cancel=*/nullptr);
  if (XR_FAILED(result) && result != XR_ERROR_FUNCTION_UNSUPPORTED) {
    if (result == XR_ERROR_VALIDATION_FAILURE) {
      LOG(ERROR) << "Failed to set API key auth: API key is invalid.";
    } else {
      LOG(ERROR) << "Failed to set API key auth: " << result;
    }
  }
  env->ReleaseStringUTFChars(api_key, api_key_chars);
}

JNIEXPORT void JNICALL
Java_androidx_xr_arcore_openxr_OpenXrManager_nativeSetAuthTokenAuth(
    JNIEnv* env, jclass /*clazz*/, jstring auth_token) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();

  const char* auth_token_chars = env->GetStringUTFChars(auth_token, nullptr);
  XrGoogleCloudAuthTokenANDROID auth_info = {
      .type = XR_TYPE_GOOGLE_CLOUD_AUTH_TOKEN_ANDROID,
      .next = nullptr,
      .authToken = auth_token_chars,
  };

  XrResult result = xr_manager.SetGoogleCloudAuthAsync(
      reinterpret_cast<XrGoogleCloudAuthInfoBaseHeaderANDROID*>(&auth_info),
      HandleAuthCompletion, /*on_cancel=*/nullptr);
  if (XR_FAILED(result) && result != XR_ERROR_FUNCTION_UNSUPPORTED) {
    if (result == XR_ERROR_VALIDATION_FAILURE) {
      LOG(ERROR) << "Failed to set auth token auth: Auth token is invalid.";
    } else {
      LOG(ERROR) << "Failed to set auth token auth: " << result;
    }
  }
  env->ReleaseStringUTFChars(auth_token, auth_token_chars);
}

JNIEXPORT void JNICALL
Java_androidx_xr_arcore_openxr_OpenXrManager_nativeSetKeylessAuth(
    JNIEnv* env, jclass /*clazz*/) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();

  XrGoogleCloudAuthKeylessANDROID auth_info = {
      .type = XR_TYPE_GOOGLE_CLOUD_AUTH_KEYLESS_ANDROID,
      .next = nullptr,
  };

  XrResult result = xr_manager.SetGoogleCloudAuthAsync(
      reinterpret_cast<XrGoogleCloudAuthInfoBaseHeaderANDROID*>(&auth_info),
      HandleAuthCompletion, /*on_cancel=*/nullptr);
  if (XR_FAILED(result) && result != XR_ERROR_FUNCTION_UNSUPPORTED) {
    LOG(ERROR) << "Failed to set keyless auth: " << result;
  }
}

}  // extern "C"
