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

#include <cstdint>
#include <vector>

#include "openxr/jobject_creator.h"
#include "openxr/openxr_manager.h"

extern "C" {

JNIEXPORT jobject JNICALL
Java_androidx_xr_runtime_openxr_OpenXrDepthMap_nativeGetDepthImageWidthAndHeight(
    JNIEnv* env, jclass /*clazz*/) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();

  return androidx::xr::openxr::CreateJavaIntSize2d(
      env, xr_manager.GetDepthImageWidth(), xr_manager.GetDepthImageHeight());
}

}  // extern "C"
