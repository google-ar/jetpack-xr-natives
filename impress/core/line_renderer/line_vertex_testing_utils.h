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

#ifndef THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_LINE_VERTEX_TESTING_UTILS_H_
#define THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_LINE_VERTEX_TESTING_UTILS_H_

#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "absl/status/status.h"
#include "core/line_renderer/line_mat_types.h"

namespace imp::line_renderer {

// Returns an assert result if the two given vectors are within the given error
// for each dimension.
testing::AssertionResult IsNear(const imp::float3& v1, const imp::float3& v2,
                                const imp::float3& error);

// Expects that the two vertices are approximately equal given the error.
absl::Status ExpectLineVertexNear(const LineVertexAttributes& v1,
                                  const LineVertexAttributes& v2,
                                  const imp::float3& error);

}  // namespace imp::line_renderer

#endif  // THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_LINE_VERTEX_TESTING_UTILS_H_
