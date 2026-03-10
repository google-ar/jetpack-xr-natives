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

#ifndef THIRD_PARTY_IMPRESS_CORE_TEXT_SLICED_GLYPH_ATLAS_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_TEXT_SLICED_GLYPH_ATLAS_HELPERS_H_

#include "core/math/vec.h"
#include "core/text/glyph_atlas_slice.h"

namespace imp {
namespace sliced_glyph_atlas {

// Helper method to get the origin (offset) and scale of a given slice's
// sub-region within the grid used by the composite texture.
void GetGridSliceOffsetAndScale(uint2 grid_size, SliceId slice, float2* offset,
                                float2* scale);

}  // namespace sliced_glyph_atlas
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_TEXT_SLICED_GLYPH_ATLAS_HELPERS_H_
