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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_VERTEX_ATTRIBUTE_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_VERTEX_ATTRIBUTE_H_

#include <cstdint>

namespace imp {

enum class VertexAttribute : uint8_t {
  POSITION,
  NORMAL,
  TANGENT,
  TEXCOORD_0,
  TEXCOORD_1,
  COLOR_0,
  JOINTS_0,
  WEIGHTS_0,
  MORPH_POSITION_0,
  MORPH_POSITION_1,
  MORPH_POSITION_2,
  MORPH_POSITION_3,
  MORPH_TANGENT_0,
  MORPH_TANGENT_1,
  MORPH_TANGENT_2,
  MORPH_TANGENT_3,
  MORPH_NORMAL_0,
  MORPH_NORMAL_1,
  MORPH_NORMAL_2,
  MORPH_NORMAL_3,
  MIN = POSITION,
  MAX = MORPH_NORMAL_3,
};

class VertexAttributeMask {
 public:
  void Set(VertexAttribute attr) { mask_ |= Mask(attr); }
  bool Test(VertexAttribute attr) { return (mask_ & Mask(attr)) == Mask(attr); }
  bool Any() const { return mask_ != 0; }

 private:
  uint32_t Mask(VertexAttribute attr) { return (1 << static_cast<int>(attr)); }
  uint32_t mask_ = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_VERTEX_ATTRIBUTE_H_
