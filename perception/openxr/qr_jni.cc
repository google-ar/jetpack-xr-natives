// Copyright 2026 Google LLC
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
#include <vector>

#include "openxr/jobject_creator.h"
#include "openxr/openxr_manager.h"

extern "C" {
JNIEXPORT jobject JNICALL
Java_androidx_xr_arcore_openxr_OpenXrQrCode_nativeGetQrCodeState(
    JNIEnv* env, jclass /*clazz*/, jlong qr_code_id, jlong monotonic_time_ns) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();
  std::vector<char> qr_data;
  XrTrackableQrCodeANDROID qr = {
      .type = XR_TYPE_TRACKABLE_QR_CODE_ANDROID,
      .next = nullptr,
      .trackingState = XR_TRACKING_STATE_PAUSED_ANDROID,
      .bufferCapacityInput = 0,
      .bufferCountOutput = 0,
      .buffer = nullptr,
  };
  if (!xr_manager.GetQrCodeState(
          static_cast<XrTrackableANDROID>(qr_code_id),
          XrReferenceSpaceType::XR_REFERENCE_SPACE_TYPE_UNBOUNDED_ANDROID,
          static_cast<int64_t>(monotonic_time_ns), qr, qr_data)) {
    return nullptr;
  }

  return androidx::xr::openxr::CreateJavaQrCodeState(env, qr);
}
}  // extern "C"
