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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ANIMATION_ANIMATION_FRAME_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ANIMATION_ANIMATION_FRAME_H_

#include <tuple>
#include <type_traits>
#include <vector>

#include "core/math/math.h"

namespace imp::animation {

template <typename T>
struct Frame {
  using ValueType = T;
  T position;
};

template <typename T>
using EnableIfValidFrame =
    std::enable_if_t<kIsAnyOf<T, float, float2, float3, float4, quatf>, int>;

template <typename ValueType, EnableIfValidFrame<ValueType> = 0>
std::vector<Frame<ValueType>> ToFrame(const std::vector<ValueType>& floats) {
  std::vector<Frame<ValueType>> frames;
  std::transform(floats.begin(), floats.end(), std::back_inserter(frames),
                 [](const ValueType& val) -> Frame<ValueType> {
                   return {.position = val};
                 });
  return std::move(frames);
}

}  // namespace imp::animation

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ANIMATION_ANIMATION_FRAME_H_
