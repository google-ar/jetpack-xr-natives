/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_NDKWRAPPERS_HARDWARE_BUFFER_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_NDKWRAPPERS_HARDWARE_BUFFER_HELPER_H_

#include <android/data_space.h>
#include <android/hardware_buffer.h>

#include <cstdint>

#include "absl/status/status.h"
#include "core/common/robin_map.h"
#include "core/render/android/android_defines.h"

namespace imp::android {

// Function pointer for AHardwareBuffer_getAuxiliaryViewInfo
using FPAHardwareBuffer_getAuxiliaryViewInfo =
    uint32_t (*)(const AHardwareBuffer* buffer);

// Function pointer for AHardwareBuffer_getAuxiliaryBuffer
using FPAHardwareBuffer_getAuxiliaryBuffer =
    const AHardwareBuffer* (*)(AHardwareBuffer * buffer, uint32_t view_mask);

// Function pointer for AHardwareBuffer_getDataSpace
using FPAHardwareBuffer_getDataSpace =
    ADataSpace (*)(const AHardwareBuffer* buffer);

class AHardwareBufferHelper {
 public:
  // Given the primary view hardware buffer, returns a map of the available
  // views. The primary view itself is not necessarily included in the map.
  static std::unique_ptr<RobinMap<SurfaceViewType, const AHardwareBuffer*>>
  GetAvailableViews(AHardwareBuffer* primary_view_ahb);

  static ADataSpace GetDataSpace(AHardwareBuffer* ahb);

  // Loads the runtime symbols for querying and retrieving auxiliary views.
  static absl::Status LoadRuntimeLibraries();

 private:
  // Runtime symbols for AHardwareBuffer functions used for auxiliary views.
  static FPAHardwareBuffer_getAuxiliaryViewInfo
      AHardwareBuffer_getAuxiliaryViewInfo_;
  static FPAHardwareBuffer_getAuxiliaryBuffer
      AHardwareBuffer_getAuxiliaryBuffer_;
  static FPAHardwareBuffer_getDataSpace AHardwareBuffer_getDataSpace_;

  // Whether the MV-HEVC playback API is enabled.
  static bool mvhevc_enabled_;

  // Whether the buffer data space API is enabled.
  static bool data_space_enabled_;
};

}  // namespace imp::android

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_NDKWRAPPERS_HARDWARE_BUFFER_HELPER_H_
