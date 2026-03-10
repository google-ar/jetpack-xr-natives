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

#include "core/text/sliced_glyph_atlas_helpers.h"

#include <cstddef>

#include "core/math/vec.h"
#include "core/text/glyph_atlas_slice.h"

namespace imp {
namespace sliced_glyph_atlas {

void GetGridSliceOffsetAndScale(uint2 grid_size, SliceId slice, float2* offset,
                                float2* scale) {
  auto slice_index = static_cast<SliceId::ValueType>(slice);
  size_t slice_y = slice_index / grid_size.x;
  size_t slice_x = slice_index % grid_size.x;
  *offset = float2(static_cast<float>(slice_x) / grid_size.x,
                   static_cast<float>(slice_y) / grid_size.y);
  *scale = float2(1.0f / grid_size.x, 1.0f / grid_size.y);
}

}  // namespace sliced_glyph_atlas
}  // namespace imp
