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

#ifndef THIRD_PARTY_IMPRESS_CORE_ANIMATION_EASING_FUNCTIONS_H_
#define THIRD_PARTY_IMPRESS_CORE_ANIMATION_EASING_FUNCTIONS_H_

// Easing functions that map a value representing a fraction of time to a
// value representing a fractional progress. Valid for input from 0 to 1.
// Output begins at 0 and ends at 1. See https://easings.net for more details.
namespace imp::animation {

float EaseInSine(float x);
float EaseOutSine(float x);
float EaseInOutSine(float x);
float EaseInQuad(float x);
float EaseOutQuad(float x);
float EaseInOutQuad(float x);
float EaseInCubic(float x);
float EaseOutCubic(float x);
float EaseInOutCubic(float x);
float EaseInBack(float x);
float EaseOutBack(float x);
float EaseInOutBack(float x);

}  // namespace imp::animation

#endif  // THIRD_PARTY_IMPRESS_CORE_ANIMATION_EASING_FUNCTIONS_H_
