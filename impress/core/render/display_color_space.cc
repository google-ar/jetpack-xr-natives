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

// TODO: Find a better place for SurfaceColorSpace
#include "core/render/android/platform_android_external_texture_surface.h"

namespace imp {

// Helper function to find the target color space from the source color space.
DisplayColorSpace SurfaceColorSpaceToDisplayColorSpace(
    SurfaceColorSpace surface_color_space) {
  switch (surface_color_space.GetStandard()) {
    // This is equivalent to sRGB
    case SurfaceColorSpace::Standard::kBT709:
    // Those have smaller gamuts than sRGB
    case SurfaceColorSpace::Standard::kBT601_PAL:
    case SurfaceColorSpace::Standard::kBT601_525:
      return DisplayColorSpace::kBT709;

    // P3
    case SurfaceColorSpace::Standard::kDisplayP3:
    case SurfaceColorSpace::Standard::kDCI_P3:
    // Those have wider gamuts than sRGB
    case SurfaceColorSpace::Standard::kBT2020:
    case SurfaceColorSpace::Standard::kAdobeRGB:
      return DisplayColorSpace::kP3;

    // Default to sRGB.
    default:
      return DisplayColorSpace::kBT709;
  }
}
}  // namespace imp
