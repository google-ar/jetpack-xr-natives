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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_IMAGE_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_IMAGE_HELPERS_H_
#include <cstdint>

namespace imp {

// Returns the number of mipmap levels for an image with the given dimensions.
uint8_t GetMipmapLevelCount(uint32_t width, uint32_t height);

}  // namespace imp.
#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_IMAGE_HELPERS_H_
