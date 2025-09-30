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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATH_FLATBUFFER_SUPPORT_H_
#define THIRD_PARTY_IMPRESS_CORE_MATH_FLATBUFFER_SUPPORT_H_

#include <array>
#include <cstddef>
#include <cstdint>

#include "flatbuffers/array.h"
#include "core/common/schemas/math_generated.h"
#include "core/math/math.h"
#include "core/math/transform.h"

namespace imp::schemas {
struct Bool;
struct Float;
struct Float2;
struct Float3;
struct Float4;
struct Double3;
struct Quatf;
struct Mat2f;
struct Mat3f;
struct Mat4f;
struct Mat4;
struct Transformf;
}  // namespace imp::schemas

namespace flatbuffers {
imp::schemas::Bool Pack(const bool &obj);
bool UnPack(const imp::schemas::Bool &obj);
imp::schemas::Float Pack(const float &obj);
float UnPack(const imp::schemas::Float &obj);
imp::schemas::Float2 Pack(const imp::float2 &obj);
imp::float2 UnPack(const imp::schemas::Float2 &obj);
imp::schemas::Float3 Pack(const imp::float3 &obj);
imp::float3 UnPack(const imp::schemas::Float3 &obj);
imp::schemas::Float4 Pack(const imp::float4 &obj);
imp::float4 UnPack(const imp::schemas::Float4 &obj);
imp::schemas::Double3 Pack(const imp::double3 &obj);
imp::double3 UnPack(const imp::schemas::Double3 &obj);
imp::schemas::Quatf Pack(const imp::quatf &obj);
imp::quatf UnPack(const imp::schemas::Quatf &obj);
imp::schemas::Mat2f Pack(const imp::mat2f& obj);
imp::mat2f UnPack(const imp::schemas::Mat2f& obj);
imp::schemas::Mat3f Pack(const imp::mat3f &obj);
imp::mat3f UnPack(const imp::schemas::Mat3f &obj);
imp::schemas::Mat4f Pack(const imp::mat4f &obj);
imp::mat4f UnPack(const imp::schemas::Mat4f &obj);
imp::schemas::Mat4 Pack(const imp::mat4 &obj);
imp::mat4 UnPack(const imp::schemas::Mat4 &obj);
imp::schemas::Transformf Pack(const imp::Transform<float> &obj);
imp::Transform<float> UnPack(const imp::schemas::Transformf &obj);
imp::schemas::PreciseTransform Pack(const imp::PreciseTransform &obj);
imp::PreciseTransform UnPack(const imp::schemas::PreciseTransform &obj);

template <typename T, uint16_t N>
std::array<T, N> UnPack(const flatbuffers::Array<T, N> *obj) {
  std::array<T, N> result;
  for (std::size_t i = 0; i < N; i++) {
    result[i] = obj->Get(i);
  }
  return result;
}
}  // namespace flatbuffers

#endif  // THIRD_PARTY_IMPRESS_CORE_MATH_FLATBUFFER_SUPPORT_H_
