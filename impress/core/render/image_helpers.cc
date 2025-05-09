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

#include "core/render/image_helpers.h"

#include <algorithm>
#include <cmath>

namespace imp {

uint8_t GetMipmapLevelCount(uint32_t width, uint32_t height) {
  if (width == 0 && height == 0) {
    return 0;
  }
  return std::floor(log2(std::max(width, height))) + 1;
}

}  // namespace imp.
