// Copyright 2025 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "color_conversion_helpers.glsl"

precision highp float;

float sRGBToLinear(float color) {
  return color <= 0.04045 ? color / 12.92 : pow((color + 0.055) / 1.055, 2.4);
}

vec3 sRGBToLinear(vec3 color) {
    return vec3(
        sRGBToLinear(color.r),
        sRGBToLinear(color.g),
        sRGBToLinear(color.b));
}

vec3 sRGBToLinear(
    vec3 color, bool enable_color_conversion, float gamma_to_srgb) {

    // Apply an gamma correction if needed to convert the color to sRGB.
    if (gamma_to_srgb != 1.0) {
      color = pow(color, vec3(gamma_to_srgb));
    }
    // Both images and videos are not linearized, so degamma here assuming that
    // the inputs are in sRGB/BT709
    color = sRGBToLinear(color);

    if (enable_color_conversion) {
      color = applyGlobalMaterialConversionMatrix(color);
    }

    return color;
}
