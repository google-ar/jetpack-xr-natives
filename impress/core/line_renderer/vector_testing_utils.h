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

#ifndef THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_VECTOR_TESTING_UTILS_H_
#define THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_VECTOR_TESTING_UTILS_H_

#include <ostream>
#include <sstream>
#include <string>

#include "core/math/vec.h"

namespace imp::line_renderer {
// Prints a vector to the given stream.
template <typename T>
std::string VectorToString(const T& v) {
  std::ostringstream ss;
  ss << "(";
  for (int i = 0; i < T::SIZE; i++) {
    ss << std::to_string(v[i]);
    if (i != T::SIZE - 1) {
      ss << ", ";
    }
  }
  ss << ")";
  return ss.str();
}
}  // namespace imp::line_renderer

// Explicit generation of specific print functions.
namespace filament::math::details {
inline void PrintTo(const imp::byte2& v, std::ostream* o) {
  *o << imp::line_renderer::VectorToString(v);
}
}  // namespace filament::math::details

#endif  // THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_VECTOR_TESTING_UTILS_H_
