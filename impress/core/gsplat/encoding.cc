// Copyright 2025 Google LLC
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

#include "core/gsplat/encoding.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "core/math/math.h"
#include "core/math/vec.h"

namespace imp {
namespace {
constexpr uint32_t kMaxValueBits11 = (1 << 11) - 1;
constexpr uint32_t kMaxValueBits10 = (1 << 10) - 1;
constexpr uint32_t kMaxValueBits8 = (1 << 8) - 1;
constexpr uint32_t kMaxValueBits5 = (1 << 5) - 1;
constexpr uint32_t kMaxValueBits6 = (1 << 6) - 1;
constexpr uint32_t kMaxValueBits2 = (1 << 2) - 1;
constexpr float kSqrt2 = 1.41421356237f;

/*****START referenced from third_party/angle/src/common/mathutil.h */
// Need a custom implementation of std::bit_cast due to Android NDK not
// supporting c++20.
template <typename DestT, typename SourceT>
DestT BitCast(const SourceT& source) {
  size_t copy_size = std::min(sizeof(DestT), sizeof(SourceT));
  DestT output;
  memcpy(&output, &source, copy_size);
  return output;
}

inline uint16_t Float32ToFloat16(float fp32) {
  uint32_t fp32i = BitCast<uint32_t>(fp32);
  uint32_t sign = (fp32i & 0x80000000) >> 16;
  uint32_t abs = fp32i & 0x7FFFFFFF;

  if (abs > 0x7F800000) {  // NaN
    return 0x7FFF;
  } else if (abs > 0x47FFEFFF) {  // Infinity
    return static_cast<uint16_t>(sign | 0x7C00);
  } else if (abs < 0x38800000) {  // Denormal
    uint32_t mantissa = (abs & 0x007FFFFF) | 0x00800000;
    int e = 113 - (abs >> 23);

    if (e < 24) {
      abs = mantissa >> e;
    } else {
      abs = 0;
    }

    return static_cast<uint16_t>(sign |
                                 (abs + 0x00000FFF + ((abs >> 13) & 1)) >> 13);
  } else {
    return static_cast<uint16_t>(
        sign | (abs + 0xC8000000 + 0x00000FFF + ((abs >> 13) & 1)) >> 13);
  }
}
/*********END referenced from third_party/angle/src/common/mathutil.h*/
}  // namespace

uint32_t Float3ToNorm11(float3 v) {
  // (broken link)
  // Float->NORM conversion, add 0.5 to round to the nearest integer.
  return (static_cast<uint32_t>(v.x * kMaxValueBits11 + .5)) |
         (static_cast<uint32_t>(v.y * kMaxValueBits11 + .5) << 11) |
         (static_cast<uint32_t>(v.z * kMaxValueBits10 + .5) << 22);
}

float3 Norm11ToFloat3(uint32_t v) {
  return float3(
      (v & kMaxValueBits11) / static_cast<float>(kMaxValueBits11),
      ((v >> 11) & kMaxValueBits11) / static_cast<float>(kMaxValueBits11),
      ((v >> 22) & kMaxValueBits10) / static_cast<float>(kMaxValueBits10));
}

uint32_t Float4ToNorm10(float4 v) {
  // Add 0.5 to round to the nearest integer.
  return (static_cast<uint32_t>(v.x * kMaxValueBits10 + .5)) |
         (static_cast<uint32_t>(v.y * kMaxValueBits10 + .5) << 10) |
         (static_cast<uint32_t>(v.z * kMaxValueBits10 + .5) << 20) |
         (static_cast<uint32_t>(v.w * kMaxValueBits2 + .5) << 30);
}

float4 Norm10ToFloat4(uint32_t v) {
  return float4(
      (v & kMaxValueBits10) / static_cast<float>(kMaxValueBits10),
      ((v >> 10) & kMaxValueBits10) / static_cast<float>(kMaxValueBits10),
      ((v >> 20) & kMaxValueBits10) / static_cast<float>(kMaxValueBits10),
      ((v >> 30) & kMaxValueBits2) / static_cast<float>(kMaxValueBits2));
}

uint32_t Float4ToNorm8(float4 v) {
  // Add 0.5 to round to the nearest integer.
  return (static_cast<uint32_t>(v.x * kMaxValueBits8 + .5)) |
         (static_cast<uint32_t>(v.y * kMaxValueBits8 + .5) << 8) |
         (static_cast<uint32_t>(v.z * kMaxValueBits8 + .5) << 16) |
         (static_cast<uint32_t>(v.w * kMaxValueBits8 + .5) << 24);
}

float4 Norm8ToFloat4(uint32_t v) {
  const float4 extracted((v & kMaxValueBits8), ((v >> 8) & kMaxValueBits8),
                         ((v >> 16) & kMaxValueBits8),
                         ((v >> 24) & kMaxValueBits8));
  return extracted / static_cast<float>(kMaxValueBits8);  // Normalize to [0, 1]
}

uint16_t Float3ToNorm565(float3 v) {
  // Add 0.5 to round to the nearest integer.
  return static_cast<uint16_t>(
      (static_cast<uint32_t>(v.x * kMaxValueBits5 + .5)) |
      (static_cast<uint32_t>(v.y * kMaxValueBits6 + .5) << 5) |
      (static_cast<uint32_t>(v.z * kMaxValueBits5 + .5) << 11));
}

float3 Norm565ToFloat3(uint16_t v) {
  return float3(
      (v & kMaxValueBits5) / static_cast<float>(kMaxValueBits5),
      ((v >> 5) & kMaxValueBits6) / static_cast<float>(kMaxValueBits6),
      ((v >> 11) & kMaxValueBits5) / static_cast<float>(kMaxValueBits5));
}

uint32_t QuatToSmallest3Norm10(const quatf& q) {
  // find biggest component
  float4 abs_q = abs(q.xyzw);
  int max_component_index = 0;
  float max_component_value = abs_q.x;
  if (abs_q.y > max_component_value) {
    max_component_index = 1;
    max_component_value = abs_q.y;
  }
  if (abs_q.z > max_component_value) {
    max_component_index = 2;
    max_component_value = abs_q.z;
  }
  if (abs_q.w > max_component_value) {
    max_component_index = 3;
    max_component_value = abs_q.w;
  }

  // Swizzle the largest component to the w element.
  quatf q_reordered = q;
  if (max_component_index == 0) q_reordered = {float3(q.y, q.z, q.w), q.x};
  if (max_component_index == 1) q_reordered = {float3(q.x, q.z, q.w), q.y};
  if (max_component_index == 2) q_reordered = {float3(q.x, q.y, q.w), q.z};

  // For quaternions, (x,y,z,w) and -(x,y,z,w) represent the same rotation, so
  // rather than storing the sign of the largest component, we always make the w
  // component positive by negating the rest of the quaternion components by the
  // sign of the w component.
  float3 smallest_three = q_reordered.xyz * (q_reordered.w >= 0 ? 1 : -1);
  // Since we are compressing a normalized quaternion, the smallest 3 components
  // by construction are always in the range [-1/sqrt2, +1/sqrt2].
  smallest_three = (smallest_three * kSqrt2) * 0.5f + 0.5f;  // to [0,1] range

  return Float4ToNorm10({smallest_three, max_component_index / 3.0f});
}

quatf Smallest3Norm10ToQuat(uint32_t q_encoded) {
  float4 pq = Norm10ToFloat4(q_encoded);
  uint32_t max_component_index =
      static_cast<uint32_t>(std::round(pq.w * kMaxValueBits2));
  quatf q;
  // Move back to [-1/sqrt2, +1/sqrt2] range
  q.xyz = (pq.xyz - 0.5f) * 2.0f / kSqrt2;
  q.w = std::sqrt(1.0f - std::clamp(dot(q.xyz, q.xyz), 0.0f, 1.0f));
  if (max_component_index == 0) q = {float3(q.w, q.x, q.y), q.z};
  if (max_component_index == 1) q = {float3(q.x, q.w, q.y), q.z};
  if (max_component_index == 2) q = {float3(q.x, q.y, q.w), q.z};
  return q;
}

uint32_t PackHalf2x16(const float f1, const float f2) {
  uint16_t least_significant_bits = static_cast<uint16_t>(Float32ToFloat16(f1));
  uint16_t most_significant_bits = static_cast<uint16_t>(Float32ToFloat16(f2));
  return static_cast<uint32_t>(most_significant_bits) << 16 |
         static_cast<uint32_t>(least_significant_bits);
}

imp::uint2 CalculateTextureSize(int count) {
  // Round up to the nearest power of 2.
  const uint texture_width = std::sqrt(count);
  const uint texture_height =
      std::ceil(static_cast<float>(count) / texture_width);
  return {texture_width, texture_height};
}

}  // namespace imp
