/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_GSPLAT_ENCODING_H_
#define THIRD_PARTY_IMPRESS_CORE_GSPLAT_ENCODING_H_

#include <cstdint>

#include "core/math/quat.h"
#include "core/math/vec.h"

namespace imp {
// Packs a float3 into a uint32_t (little endian) where the x, y, and z
// components are stored in a normalized integer format using the 11 lower bits
// for x, 11 bits for y, and 10 upper bits for z. Assumes all components are in
// the range [0, 1].
uint32_t Float3ToNorm11(float3 v);

// Unpacks a uint32_t containing a Norm11 encoded vector into a float3.
float3 Norm11ToFloat3(uint32_t v);

// Packs a float4 into a uint32_t (little endian) where the x, y, and z
// components are stored in a normalized integer format using the 10 bits for x,
// 10 bits for y, 10 bits for z, and 2 upper bits for w. Assumes all components
// are in the range [0, 1].
uint32_t Float4ToNorm10(float4 v);

// Unpacks a uint32_t containing a Norm10 encoded vector into a float4.
float4 Norm10ToFloat4(uint32_t v);

// Packs a float4 into a uint32_t (little endian) where the x, y, z, and w
// components are stored in a normalized integer format using 8 bits for each
// component. Assumes all components are in the range [0, 1].
uint32_t Float4ToNorm8(float4 v);

// Unpacks a uint32_t containing a Norm8 encoded vector into a float4.
float4 Norm8ToFloat4(uint32_t v);

// Packs a float3 into a uint16_t (little endian) where the x, y, and z
// components are stored in a normalized integer format using the 5 lower bits
// for x, 6 bits for y, and 5 upper bits for z. Assumes all components are in
// the range [0, 1].
uint16_t Float3ToNorm565(float3 v);

// Unpacks a uint16_t containing a Norm565 encoded vector into a float3.
float3 Norm565ToFloat3(uint16_t v);

// Encodes a normalized quaternion into "smallest 3" format and packs it into a
// Norm10 uint32_t.
// Smallest 3 format is  where the largest component of a 4 element vector is
// reconstructed from the three other smaller elements. The w element in this
// resulting vector identifies the index of the largest component being
// reconstructed:
//   * 0: YZW encoded, X reconstructed
//   * 1: XZW encoded, Y reconstructed
//   * 2: XYW encoded, Z reconstructed
//   * 3: XYZ encoded, W reconstructed
uint32_t QuatToSmallest3Norm10(const quatf& q);

// Decodes a quaternion in "smallest 3" format packed in Norm10 uint32_t.
quatf Smallest3Norm10ToQuat(uint32_t q);

// Returns an unsigned integer obtained by converting the two floating-point
// values to the 16-bit floating-point representation found in the OpenGL ES
// Specification, and then packing these two 16-bit integers into a 32-bit
// unsigned integer. f1: The 16 least-significant bits of the result; f2: The 16
// most-significant bits.
// Referenced from third_party/angle/src/common/mathutil.h
uint32_t PackHalf2x16(float f1, float f2);

// Computes the texture size needed to store the given count of data. Attempts
// to make the texture size square +/- 1 row.
uint2 CalculateTextureSize(int count);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_GSPLAT_ENCODING_H_
