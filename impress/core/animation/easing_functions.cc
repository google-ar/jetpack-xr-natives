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

#include "core/animation/easing_functions.h"

#include "filament/libs/math/include/math/fast.h"
#include "filament/libs/math/include/math/scalar.h"

namespace imp::animation {

float EaseInSine(float x) {
  // From https://easings.net/#easeInSine
  return 1 - cos((x * filament::math::F_PI) / 2);
}

float EaseOutSine(float x) {
  // From https://easings.net/#easeOutSine
  return sin((x * filament::math::F_PI) / 2);
}

float EaseInOutSine(float x) {
  // From https://easings.net/#easeInOutSine
  return -(cos(filament::math::F_PI * x) - 1) / 2;
}

float EaseInQuad(float x) {
  // From https://easings.net/#easeInQuad
  return x * x;
}

float EaseOutQuad(float x) {
  // From https://easings.net/#easeOutQuad
  return 1 - (1 - x) * (1 - x);
}

float EaseInOutQuad(float x) {
  // From https://easings.net/#easeInOutQuad
  return x < 0.5 ? 2 * x * x : 1 - pow(-2 * x + 2, 2) / 2;
}

float EaseInCubic(float x) {
  // From https://easings.net/#easeInCubic
  return x * x * x;
}

float EaseOutCubic(float x) {
  // From https://easings.net/#easeOutCubic
  return 1 - filament::math::fast::pow(1 - x, 3);
}

float EaseInOutCubic(float x) {
  // From https://easings.net/#easeInOutCubic
  return x < 0.5 ? 4 * x * x * x
                 : 1 - filament::math::fast::pow(-2 * x + 2, 3) / 2;
}

float EaseInBack(float x) {
  // From https://easings.net/#easeInBack
  const float c1 = 1.70158;
  const float c2 = c1 + 1;

  return c2 * x * x * x - c1 * x * x;
}

float EaseOutBack(float x) {
  // From https://easings.net/#easeOutBack
  const float c1 = 1.70158;
  const float c2 = c1 + 1;

  return 1 + c2 * filament::math::fast::pow(x - 1, 3) +
         c1 * filament::math::fast::pow(x - 1, 2);
}

float EaseInOutBack(float x) {
  // From https://easings.net/#easeInOutBack
  const float c1 = 1.70158;
  const float c2 = c1 * 1.525;

  return x < 0.5
             ? (filament::math::fast::pow(2 * x, 2) * ((c2 + 1) * 2 * x - c2)) /
                   2
             : (filament::math::fast::pow(2 * x - 2, 2) *
                    ((c2 + 1) * (x * 2 - 2) + c2) +
                2) /
                   2;
}

}  // namespace imp::animation
