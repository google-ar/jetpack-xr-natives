// Copyright 2024 Google LLC
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

#include "core/view/utils/device.h"

#include "core/common/log.h"
#include "core/common/platform_helpers.h"
#include "core/config.h"

namespace imp {

Device::Device() : physical_pixel_ratio_(absl::nullopt) {}

float2 Device::GetPhysicalPixelRatio() const {
  if (!physical_pixel_ratio_.has_value()) {
    IMP_LOG(imp::FATAL) << "Device physical pixel ratio value has not been set.";
  }
  return physical_pixel_ratio_.value();
}

void Device::SetPhysicalPixelRatio(float2 physical_pixel_ratio) {
  physical_pixel_ratio_ = physical_pixel_ratio;
}

bool Device::IsPhysicalPixelRatioAvailable() const {
  return physical_pixel_ratio_.has_value();
}

float Device::GetAveragePhysicalPixelRatio() const {
  float2 physical_pixel_ratio = GetPhysicalPixelRatio();
  return (physical_pixel_ratio.x + physical_pixel_ratio.y) / 2;
}

float2 Device::PixelsToPhysicalPixels(float2 pixels) const {
  return pixels * GetPhysicalPixelRatio();
}

float2 Device::PhysicalPixelsToPixels(float2 physical_pixels) const {
  return physical_pixels / GetPhysicalPixelRatio();
}

float Device::PixelsToPhysicalPixels(float pixels) const {
  return pixels * GetAveragePhysicalPixelRatio();
}

float Device::PhysicalPixelsToPixels(float physical_pixels) const {
  return physical_pixels / GetAveragePhysicalPixelRatio();
}

}  // namespace imp
