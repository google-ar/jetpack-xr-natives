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

#include "core/math/mat.h"

#include <string>

#include "absl/strings/str_format.h"
#include "core/math/vec.h"

namespace imp {

namespace {

template <typename T>
std::string Create2x2MatrixString(T m) {
  const auto* a = m.asArray();
  // Column-major order.
  return absl::StrFormat(
      "[ % .3f % .3f  ]\n"
      "[ % .3f % .3f  ]\n",
      a[0], a[2], a[1], a[3]);
}

template <typename T>
std::string Create3x3MatrixString(T m) {
  const auto* a = m.asArray();
  // Column-major order.
  return absl::StrFormat(
      "[ % .3f % .3f % .3f  ]\n"
      "[ % .3f % .3f % .3f  ]\n"
      "[ % .3f % .3f % .3f  ]\n",
      a[0], a[3], a[6],  //
      a[1], a[4], a[7],  //
      a[2], a[5], a[8]);
}

template <typename T>
std::string Create4x4MatrixString(T m) {
  const auto* a = m.asArray();
  // Column-major order.
  return absl::StrFormat(
      "[ % .3f % .3f % .3f % .3f  ]\n"
      "[ % .3f % .3f % .3f % .3f  ]\n"
      "[ % .3f % .3f % .3f % .3f  ]\n"
      "[ % .3f % .3f % .3f % .3f  ]\n",
      a[0], a[4], a[8], a[12],   //
      a[1], a[5], a[9], a[13],   //
      a[2], a[6], a[10], a[14],  //
      a[3], a[7], a[11], a[15]);
}

}  // namespace

bool IsYUp(const mat4f& m) {
  constexpr auto kDotThreshold = 1.0f - 1.0e-3f;
  return dot(imp::kUp, (m * float4(imp::kUp, 0)).xyz) >= kDotThreshold;
}

std::string ToString(const mat2f& m) { return Create2x2MatrixString(m); }
std::string ToString(const mat2& m) { return Create2x2MatrixString(m); }
std::string ToString(const mat3f& m) { return Create3x3MatrixString(m); }
std::string ToString(const mat3& m) { return Create3x3MatrixString(m); }
std::string ToString(const mat4f& m) { return Create4x4MatrixString(m); }
std::string ToString(const mat4& m) { return Create4x4MatrixString(m); }

}  // namespace imp
