/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_NATIVE_WINDOW_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_NATIVE_WINDOW_H_

#include <optional>

#include "core/async/future.h"
#include "core/math/vec.h"
#include "core/view/platforms/xr_android/openxr_includes.h"

namespace imp {

class XrSessionHost;

// Passed to Filament in place of a native window for XR platforms.
struct XrNativeWindow {
  struct QuadLayerData {
    // Used to override the size of the swapchain.
    int2 size = {0, 0};
    // The sample count to pass to OpenXR to configure the swapchain.
    int sample_count = 1;
    // Resolved asynchronously from the Filament render thread, containing the
    // swapchain handle of the quad layer.
    Future<XrSwapchain> swapchain;
  };

  XrSessionHost* host;
  // If present, this native window represents a quad composition layer, and
  // contains specifications for size, sample count, and the future swapchain
  // handle.
  std::optional<QuadLayerData> quad_layer_data = std::nullopt;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_NATIVE_WINDOW_H_
