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

#include <cstddef>
#include <cstdint>

#include "openxr/openxr_manager.h"

extern "C" {

JNIEXPORT jobject JNICALL
Java_androidx_xr_arcore_openxr_OpenXrHand_nativeGetHandDataBuffer(
    JNIEnv* env, jclass /*clazz*/, jboolean is_left_hand,
    jlong monotonic_time_ns) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();

  std::byte* buffer = xr_manager.GetHandDataBuffer(
      is_left_hand, static_cast<int64_t>(monotonic_time_ns));
  if (buffer == nullptr) {
    return nullptr;
  }

  return env->NewDirectByteBuffer(buffer, xr_manager.kHandJointsBufferSize);
}

}  // extern "C"
