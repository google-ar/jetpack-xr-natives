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

#include "openxr/openxr.h"
#include "openxr/jobject_creator.h"
#include "openxr/openxr_manager.h"

static jobject NativeGetFaceState(JNIEnv* env, jlong monotonic_time_ns) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  XrFaceStateANDROID face_state;
  std::vector<float> blend_shape_values;
  std::vector<float> confidence_values;
  if (XR_FAILED(xr_manager.GetFaceState(static_cast<int64_t>(monotonic_time_ns),
                                        &face_state, blend_shape_values,
                                        confidence_values))) {
    return nullptr;
  }
  return androidx::xr::openxr::CreateJavaFaceState(env, face_state);
}

extern "C" {
JNIEXPORT jobject JNICALL
Java_androidx_xr_runtime_openxr_OpenXrFace_nativeGetFaceState(
    JNIEnv* env, jclass /*clazz*/, jlong monotonic_time_ns) {
  return NativeGetFaceState(env, monotonic_time_ns);
}
}  // extern "C"
