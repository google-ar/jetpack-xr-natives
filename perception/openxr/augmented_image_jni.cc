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

#include "openxr/jobject_creator.h"
#include "openxr/openxr_manager.h"

static jobject NativeGetAugmentedImageState(JNIEnv* env,
                                            jlong augmented_image_id,
                                            jlong monotonic_time_ns) {
  androidx::xr::openxr::OpenXrManager& xr_manager =
      androidx::xr::openxr::OpenXrManager::GetOpenXrManager();

  XrTrackableImageANDROID image = {
      .type = XR_TYPE_TRACKABLE_IMAGE_ANDROID,
      .trackingState = XR_TRACKING_STATE_PAUSED_ANDROID,
  };
  if (!xr_manager.GetAugmentedImageState(
          static_cast<XrTrackableANDROID>(augmented_image_id),
          XrReferenceSpaceType::XR_REFERENCE_SPACE_TYPE_UNBOUNDED_ANDROID,
          static_cast<int64_t>(monotonic_time_ns), image)) {
    return nullptr;
  }

  return androidx::xr::openxr::CreateJavaAugmentedImageState(env, image);
}

extern "C" {
JNIEXPORT jobject JNICALL
Java_androidx_xr_arcore_openxr_OpenXrAugmentedImage_nativeGetAugmentedImageState(
    JNIEnv* env, jclass /*clazz*/, jlong augmented_image_id,
    jlong monotonic_time_ns) {
  return NativeGetAugmentedImageState(env, augmented_image_id,
                                      monotonic_time_ns);
}
}  // extern "C"
