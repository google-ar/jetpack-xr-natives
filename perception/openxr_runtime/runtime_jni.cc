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

#include "openxr/openxr.h"
#include "common/pointer_util.h"
#include "openxr_runtime/openxr_instance_manager.h"

extern "C" {

using androidx::xr::common::PointerFromJLong;
using androidx::xr::common::PointerToJLong;
using androidx::xr::openxr::OpenXrInstanceManager;

JNIEXPORT jlong JNICALL
Java_androidx_xr_runtime_openxr_OpenXrDeviceCapabilityProvider_nativeCreateOpenXrInstanceManager(
    JNIEnv* env, jclass /*clazz*/) {
  return PointerToJLong(new OpenXrInstanceManager());
}

JNIEXPORT void JNICALL
Java_androidx_xr_runtime_openxr_OpenXrDeviceCapabilityProvider_nativeDestroyOpenXrInstanceManager(
    JNIEnv* env, jclass /*clazz*/, jlong manager_ptr) {
  delete androidx::xr::common::PointerFromJLong<OpenXrInstanceManager>(
      manager_ptr);
}

JNIEXPORT jlong JNICALL
Java_androidx_xr_runtime_openxr_OpenXrDeviceCapabilityProvider_nativeGetOpenXrInstanceHandle(
    JNIEnv* env, jclass /*clazz*/, jlong manager_ptr) {
  XrInstance instance =
      PointerFromJLong<OpenXrInstanceManager>(manager_ptr)->GetInstance();
  return PointerToJLong(&instance);
}

}  // extern "C"

