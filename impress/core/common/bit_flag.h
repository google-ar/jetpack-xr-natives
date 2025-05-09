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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_BIT_FLAG_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_BIT_FLAG_H_

#include <stdint.h>

namespace imp {

using BitFlag = uint32_t;

constexpr inline BitFlag SetBit(BitFlag in, BitFlag bits) { return in | bits; }

constexpr inline BitFlag ClearBit(BitFlag in, BitFlag bits) {
  return (in & ~bits);
}

constexpr inline BitFlag SetBitFromBool(BitFlag in, BitFlag bits, bool state) {
  if (state) {
    return SetBit(in, bits);
  } else {
    return ClearBit(in, bits);
  }
}

constexpr inline bool CheckBit(BitFlag in, BitFlag bits) {
  return (in & bits) != 0;
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_BIT_FLAG_H_
