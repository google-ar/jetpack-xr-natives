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

#include "core/render/display_color_space.h"

#include "core/media/media_color_space.h"

namespace imp {

// Helper function to find the target color space from the source color space.
DisplayColorSpace MediaColorSpaceToDisplayColorSpace(
    MediaColorSpace media_color_space) {
  switch (media_color_space.GetStandard()) {
    // This is equivalent to sRGB
    case MediaColorSpace::Standard::kBT709:
    // Those have smaller gamuts than sRGB
    case MediaColorSpace::Standard::kBT601_PAL:
    case MediaColorSpace::Standard::kBT601_525:
      return DisplayColorSpace::kBT709;

    // P3
    case MediaColorSpace::Standard::kDisplayP3:
    case MediaColorSpace::Standard::kDCI_P3:
    // Those have wider gamuts than sRGB
    case MediaColorSpace::Standard::kBT2020:
    case MediaColorSpace::Standard::kAdobeRGB:
      return DisplayColorSpace::kP3;

    // Default to sRGB.
    default:
      return DisplayColorSpace::kBT709;
  }
}
}  // namespace imp
