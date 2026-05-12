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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_RENDER_ORDER_CONSTANTS_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_RENDER_ORDER_CONSTANTS_H_

#include <cstdint>

namespace imp {

// Priority constants.
// Filament supports priority values in the range [0, 7].
// 0 is rendered first, 7 is rendered last.
static constexpr uint8_t kMinPriority = 0;
static constexpr uint8_t kMaxPriority = 7;
static constexpr uint8_t kDefaultPriority = 4;

// Channel constants.
// Filament supports channel values in the range [0, 7].
// 0 is rendered first, 7 is rendered last.
static constexpr uint8_t kMinChannel = 0;
static constexpr uint8_t kMaxChannel = 7;
static constexpr uint8_t kDefaultChannel = 2;

// Blend order constants.
static constexpr uint16_t kMinBlendOrder = 0;
static constexpr uint16_t kMaxBlendOrder = 0x7FFF;
static constexpr uint16_t kDefaultBlendOrder = 0;

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_RENDER_ORDER_CONSTANTS_H_
