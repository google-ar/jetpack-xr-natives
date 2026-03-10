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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATH_MAT_H_
#define THIRD_PARTY_IMPRESS_CORE_MATH_MAT_H_

#include <string>
#include <type_traits>

#include "core/common/type_helpers.h"
// IWYU pragma: begin_exports
#include "filament/libs/math/include/math/TMatHelpers.h"
#include "filament/libs/math/include/math/mat2.h"
#include "filament/libs/math/include/math/mat3.h"
#include "filament/libs/math/include/math/mat4.h"
// IWYU pragma: end_exports

namespace imp {

using mat2 = ::filament::math::mat2;
using mat2f = ::filament::math::mat2f;
using mat3 = ::filament::math::mat3;
using mat3f = ::filament::math::mat3f;
using mat4 = ::filament::math::mat4;
using mat4f = ::filament::math::mat4f;

inline constexpr mat2 kIdentityMat2{};
inline constexpr mat2f kIdentityMat2f{};
inline constexpr mat3 kIdentityMat3{};
inline constexpr mat3f kIdentityMat3f{};
inline constexpr mat4 kIdentityMat4{};
inline constexpr mat4f kIdentityMat4f{};

bool IsYUp(const mat4f& m);

std::string ToString(const mat2f& m);
std::string ToString(const mat2& m);
std::string ToString(const mat3f& m);
std::string ToString(const mat3& m);
std::string ToString(const mat4f& m);
std::string ToString(const mat4& m);

template <typename T>
using EnableIfMatrix =
    std::enable_if_t<kIsAnyOf<T, mat2, mat2f, mat3, mat3f, mat4, mat4f>, int>;

}  // namespace imp

namespace filament::math::details {

template <typename Sink, typename T, ::imp::EnableIfMatrix<T> = 0>
void AbslStringify(Sink& sink, const T& mat) {
  sink.Append(::imp::ToString(mat));
}

}  // namespace filament::math::details

#endif  // THIRD_PARTY_IMPRESS_CORE_MATH_MAT_H_
