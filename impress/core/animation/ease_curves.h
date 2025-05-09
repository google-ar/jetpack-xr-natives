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

#ifndef THIRD_PARTY_IMPRESS_CORE_ANIMATION_EASE_CURVES_H_
#define THIRD_PARTY_IMPRESS_CORE_ANIMATION_EASE_CURVES_H_

namespace imp::animation {

/**
 * Maps a value representing a fraction of time to a value representing a
 * fractional progress. This is the standard material design easing curve as
 * described at https://material.io/design/motion/speed.html#easing It
 * accelerates quickly but decelerates slowly. Valid for input from 0 to 1,
 * Output begins at 0 and ends at 1.
 */
float FastOutSlowIn(float t);

/**
 * Maps a value representing a fraction of time to a value representing a
 * fractional progress. This is the deceleration only curve, as described at
 * https://material.io/design/motion/speed.html#easing  Valid for input from 0
 * to 1, Output begins at 0 and ends at 1.
 */
float LinearOutSlowIn(float t);

/**
 * Maps a value representing a fraction of time to a value representing a
 * fractional progress. This is the acceleration only curve, as described at
 * https://material.io/design/motion/speed.html#easing
 * Valid for input from 0 to 1, Output begins at 0 and ends
 * at 1.
 */
float FastOutLinearIn(float t);
}  // namespace imp::animation

#endif  // THIRD_PARTY_IMPRESS_CORE_ANIMATION_EASE_CURVES_H_
