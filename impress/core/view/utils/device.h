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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_UTILS_DEVICE_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_UTILS_DEVICE_H_

#include "absl/types/optional.h"
#include "core/math/vec.h"

namespace imp {

// Provides information about the physical device Impress is running on.
class Device {
 public:
  Device();

  // Returns the ratio from virtual pixels to physical pixels on the device's
  // display.
  // On Android:
  //   1 virtual pixel == 1 DP. DP is 1/160 of an inch.
  //   GetPhysicalPixelRatio() == DisplayMetrics.density
  // On iOS & macOS:
  //   1 virtual pixel == 1 Point. Point is 1/163 of an inch.
  //   GetPhysicalPixelRatio() == [UIScreen mainScreen].nativeScale
  // On WASM:
  //   1 virtual pixel == 1 CSS Pixel. CSS Pixel is 1/96 of an inch.
  //   GetPhysicalPixelRatio() = Window.devicePixelRatio
  // On Linux:
  //   1 virtual pixel == 1 window pixel (SDL_GetWindowSize)
  //   GetPhysicalPixelRatio() == SDL_GL_GetDrawableSize / SDL_GetWindowSize
  // TODO: Explore re-naming virtual pixel to something else and
  // cleaning up documentation of pixels throughout Impress.
  // TODO: Explore making an Impress density independent unit that
  // has a consistent ratio to inches on all platforms that we can convert
  // between. This would eliminate the need to be concerned with converting to
  // physical units in most cases.
  float2 GetPhysicalPixelRatio() const;

  // Converts virtual pixels to physical pixels based on the physical pixel
  // ratio.
  float2 PixelsToPhysicalPixels(float2 pixels) const;
  // Converts physical pixels to virtual pixels based on the physical pixel
  // ratio.
  float2 PhysicalPixelsToPixels(float2 physical_pixels) const;
  // Converts virtual pixels to physical pixels based on the average of the
  // physical pixel ratio.
  float PixelsToPhysicalPixels(float pixels) const;
  // Converts physical pixels to virtual pixels based on the average of the
  // physical pixel ratio.
  float PhysicalPixelsToPixels(float physical_pixels) const;

  // Sets the DPI of the device.
  // Must be called before physical pixel ratio is accessed.
  // This is automatically called by the platform integration on each platform.
  void SetPhysicalPixelRatio(float2 physical_pixel_ratio);

  // Returns true if the physical pixel ratio is ready to be used,
  // otherwise returns false.
  //
  // The physical pixels ratio is assigned when the platform's surface or window
  // is created or resized.
  //
  // Before that has occurred, the physical pixel ratio is not accessible.
  bool IsPhysicalPixelRatioAvailable() const;

  // Returns the arithmetic average of the 2D physical pixel ratio array,
  // (width_ratio + height_ratio) / 2.
  // Physical pixel ratio (width_ratio, height_ratio) is the ratio from virtual
  // pixels to physical pixels on the device's display.
  float GetAveragePhysicalPixelRatio() const;

 private:
  absl::optional<float2> physical_pixel_ratio_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_UTILS_DEVICE_H_
