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

#include <string>
#include <vector>

#include "openxr/openxr.h"
#include "common/pointer_util.h"
#include "openxr_runtime/jobject_creator.h"
#include "openxr_runtime/openxr_instance_manager.h"

extern "C" {

using androidx::xr::common::PointerFromJLong;
using androidx::xr::common::PointerToJLong;
using ::androidx::xr::openxr::CreateJavaDisplayBlendMode;
using androidx::xr::openxr::OpenXrInstanceManager;

JNIEXPORT jlong JNICALL
Java_androidx_xr_runtime_openxr_OpenXrInstanceManager_nativeCreateOpenXrInstanceManager__(
    JNIEnv* env, jclass /*clazz*/) {
  return PointerToJLong(new OpenXrInstanceManager());
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_runtime_openxr_OpenXrInstanceManager_nativeCreateOpenXrInstanceManager___3Ljava_lang_String_2(
    JNIEnv* env, jclass /*clazz*/, jobjectArray extension_names) {
  std::vector<std::string> native_extension_names;
  if (extension_names != nullptr) {
    int count = env->GetArrayLength(extension_names);
    for (int i = 0; i < count; ++i) {
      jstring extension_name_jstr =
          (jstring)env->GetObjectArrayElement(extension_names, i);
      if (extension_name_jstr == nullptr) {
        continue;
      }

      const char* native_extension_name_cstr =
          env->GetStringUTFChars(extension_name_jstr, 0);
      if (native_extension_name_cstr == nullptr) {
        env->ThrowNew(env->FindClass("java/lang/OutOfMemoryError"),
                      "Failed to get extension name string.");
        return 0L;
      }
      native_extension_names.push_back(native_extension_name_cstr);
      env->ReleaseStringUTFChars(extension_name_jstr,
                                 native_extension_name_cstr);
    }
  }
  return PointerToJLong(new OpenXrInstanceManager(native_extension_names));
}

JNIEXPORT void JNICALL
Java_androidx_xr_runtime_openxr_OpenXrInstanceManager_nativeDestroyOpenXrInstanceManager(
    JNIEnv* env, jclass /*clazz*/, jlong manager_ptr) {
  delete androidx::xr::common::PointerFromJLong<OpenXrInstanceManager>(
      manager_ptr);
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_runtime_openxr_OpenXrInstanceManager_nativeGetOpenXrInstanceHandle(
    JNIEnv* env, jclass /*clazz*/, jobject context, jlong manager_ptr) {
  XrInstance instance = PointerFromJLong<OpenXrInstanceManager>(manager_ptr)
                            ->GetInstance(env, context);
  return (jlong)instance;
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_runtime_openxr_OpenXrInstanceManager_nativeGetGetInstanceProcAddr(
    JNIEnv* env, jclass /*clazz*/, jlong manager_ptr) {
  PFN_xrGetInstanceProcAddr gipa =
      PointerFromJLong<OpenXrInstanceManager>(manager_ptr)
          ->GetGetInstanceProcAddr();
  return androidx::xr::common::PointerToJLong(reinterpret_cast<void*>(gipa));
}

JNIEXPORT jobject JNICALL
Java_androidx_xr_runtime_openxr_OpenXrDeviceCapabilityProvider_nativeGetPreferredBlendMode(
    JNIEnv* env, jclass /*clazz*/, jlong manager_ptr) {
  OpenXrInstanceManager* manager =
      PointerFromJLong<OpenXrInstanceManager>(manager_ptr);
  XrInstance instance = manager->GetInstance(env, nullptr);
  if (instance == XR_NULL_HANDLE) {
    return nullptr;
  }

  std::vector<XrEnvironmentBlendMode> blend_modes =
      manager->GetEnvironmentBlendModes(instance);

  if (blend_modes.empty()) {
    return nullptr;
  }

  // We ignore OPAQUE as we want only blend modes that describe rendering
  // capabilities with a visible environment.
  for (XrEnvironmentBlendMode blend_mode : blend_modes) {
    if (blend_mode != XR_ENVIRONMENT_BLEND_MODE_OPAQUE) {
      return CreateJavaDisplayBlendMode(env, blend_mode);
    }
  }
  return nullptr;
}

JNIEXPORT jboolean JNICALL
Java_androidx_xr_runtime_openxr_OpenXrDeviceCapabilityProvider_nativeIsHandTrackingSupported(
    JNIEnv* /*env*/, jclass /*clazz*/, jlong manager_ptr) {
  return PointerFromJLong<OpenXrInstanceManager>(manager_ptr)
      ->IsHandTrackingSupported();
}

JNIEXPORT jboolean JNICALL
Java_androidx_xr_runtime_openxr_OpenXrDeviceCapabilityProvider_nativeIsEyeTrackingSupported(
    JNIEnv* /*env*/, jclass /*clazz*/, jlong manager_ptr) {
  return PointerFromJLong<OpenXrInstanceManager>(manager_ptr)
      ->IsEyeTrackingSupported();
}

JNIEXPORT jboolean JNICALL
Java_androidx_xr_runtime_openxr_OpenXrDeviceCapabilityProvider_nativeIsDepthTrackingSupported(
    JNIEnv* /*env*/, jclass /*clazz*/, jlong manager_ptr) {
  return PointerFromJLong<OpenXrInstanceManager>(manager_ptr)
      ->IsDepthTrackingSupported();
}

JNIEXPORT jboolean JNICALL
Java_androidx_xr_runtime_openxr_OpenXrDeviceCapabilityProvider_nativeIsGeospatialSupported(
    JNIEnv* /*env*/, jclass /*clazz*/, jlong manager_ptr) {
  return PointerFromJLong<OpenXrInstanceManager>(manager_ptr)
      ->IsGeospatialSupported();
}

JNIEXPORT jboolean JNICALL
Java_androidx_xr_runtime_openxr_OpenXrDeviceCapabilityProvider_nativeIsRenderingModeSupported(
    JNIEnv* /*env*/, jclass /*clazz*/, jlong manager_ptr, jint mode) {
  return PointerFromJLong<OpenXrInstanceManager>(manager_ptr)
      ->IsRenderingModeSupported(
          static_cast<OpenXrInstanceManager::RenderingMode>(mode));
}

}  // extern "C"
