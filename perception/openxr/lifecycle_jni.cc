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

#include "openxr/openxr.h"
#include "common/pointer_util.h"
#include "openxr/openxr_manager.h"

static jlong NativeGetPointer(JNIEnv* env) {
  return androidx::xr::common::PointerToJLong(
      &androidx::xr::openxr::OpenXrManager::GetOpenXrManager());
}

static jboolean NativeInit(JNIEnv* env, jobject activity,
                           jboolean start_polling_thread) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  return xr_manager.Init(env, activity,
                         XR_REFERENCE_SPACE_TYPE_UNBOUNDED_ANDROID,
                         start_polling_thread);
}

static void NativeDeInit(JNIEnv* env) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  xr_manager.DeInit(/*stop_polling_thread=*/true);
}

static jboolean NativePause(JNIEnv* env) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  return xr_manager.PauseSession();
}

static jlong NativeConfigureSession(JNIEnv* env, jint plane_tracking,
                                    jint hand_tracking, jint head_tracking,
                                    jint depth_estimation,
                                    jint anchor_persistence,
                                    jint face_tracking,
                                    jint object_tracking,
                                    jlongArray object_tracking_labels) {
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
      .object_tracking_labels = {},
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
        env->ReleaseLongArrayElements(object_tracking_labels, labels,
                                      JNI_ABORT);
      } else {
        xr_config.object_tracking_mode =
            androidx::xr::openxr::OpenXrManager::ObjectTrackingMode::kDisabled;
      }
    }

  return xr_manager.ConfigureSession(xr_config);
}

static jboolean NativeGetFaceTrackerCalibrationState(JNIEnv* env) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  return xr_manager.IsFaceTrackerCalibrated();
}

extern "C" {
JNIEXPORT jlong JNICALL Java_androidx_xr_openxr_OpenXrManager_nativeGetPointer(
    JNIEnv* env, jclass /*clazz*/) {
  return NativeGetPointer(env);
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_runtime_openxr_OpenXrManager_nativeGetPointer(
    JNIEnv* env, jclass /*clazz*/) {
  return NativeGetPointer(env);
}

JNIEXPORT jboolean JNICALL Java_androidx_xr_openxr_OpenXrManager_nativeInit(
    JNIEnv* env, jclass /*clazz*/, jobject activity,
    jboolean start_polling_thread) {
  return NativeInit(env, activity, start_polling_thread);
}

JNIEXPORT jboolean JNICALL
Java_androidx_xr_runtime_openxr_OpenXrManager_nativeInit(
    JNIEnv* env, jclass /*clazz*/, jobject activity,
    jboolean start_polling_thread) {
  return NativeInit(env, activity, start_polling_thread);
}

JNIEXPORT void JNICALL Java_androidx_xr_openxr_OpenXrManager_nativeDeInit(
    JNIEnv* env, jclass /*clazz*/) {
  return NativeDeInit(env);
}

JNIEXPORT void JNICALL
Java_androidx_xr_runtime_openxr_OpenXrManager_nativeDeInit(JNIEnv* env,
                                                           jclass /*clazz*/) {
  return NativeDeInit(env);
}

JNIEXPORT jboolean JNICALL Java_androidx_xr_openxr_OpenXrManager_nativePause(
    JNIEnv* env, jclass /*clazz*/) {
  return NativePause(env);
}

JNIEXPORT jboolean JNICALL
Java_androidx_xr_runtime_openxr_OpenXrManager_nativePause(JNIEnv* env,
                                                          jclass /*clazz*/) {
  return NativePause(env);
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_openxr_OpenXrManager_nativeConfigureSession(
    JNIEnv* env, jclass /*clazz*/, jint plane_tracking, jint hand_tracking,
    jint head_tracking, jint depth_estimation, jint anchor_persistence) {
  return NativeConfigureSession(env, plane_tracking,
                                hand_tracking, head_tracking, depth_estimation,
                                anchor_persistence, /*face_tracking=*/0,
                                /*object_tracking=*/0,
                                /*object_tracking_labels=*/{});
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_runtime_openxr_OpenXrManager_nativeConfigureSession(
    JNIEnv* env, jclass /*clazz*/, jint plane_tracking, jint hand_tracking,
    jint head_tracking, jint depth_estimation, jint anchor_persistence,
    jint face_tracking, jint object_tracking,
    jlongArray object_tracking_labels) {
  return NativeConfigureSession(env, plane_tracking, hand_tracking,
                                head_tracking, depth_estimation,
                                anchor_persistence, face_tracking,
                                object_tracking, object_tracking_labels);
}

JNIEXPORT jboolean JNICALL
Java_androidx_xr_runtime_openxr_OpenXrManager_nativeGetFaceTrackerCalibration(
    JNIEnv* env, jclass /*clazz*/) {
  return NativeGetFaceTrackerCalibrationState(env);
}
}  // extern "C"
