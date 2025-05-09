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

#include "core/line_renderer/line_vertex_testing_utils.h"

#include <cmath>
#include <ostream>
#include <sstream>

#include "core/line_renderer/line_mat_types.h"
#include "core/line_renderer/vector_testing_utils.h"

namespace imp::line_renderer {

testing::AssertionResult IsNear(const imp::float3& v1, const imp::float3& v2,
                                const imp::float3& error) {
  imp::float3 diff = v1 - v2;
  if (diff.x < 0) {
    diff.x *= -1;
  }
  if (diff.y < 0) {
    diff.y *= -1;
  }
  if (diff.z < 0) {
    diff.z *= -1;
  }
  if (all(lessThanEqual(diff, error))) {
    return testing::AssertionSuccess();
  } else {
    testing::AssertionResult failure = testing::AssertionFailure();
    failure << "Diff: " << VectorToString(diff) << std::endl;

    failure << "v1: " << VectorToString(v1) << std::endl;

    failure << "v2: " << VectorToString(v2) << std::endl;

    failure << "error: " << VectorToString(error) << std::endl;

    return failure;
  }
}

absl::Status ExpectLineVertexNear(const LineVertexAttributes& v1,
                                  const LineVertexAttributes& v2,
                                  const imp::float3& error) {
  testing::AssertionResult result = IsNear(v1.pos, v2.pos, error);
  if (!result) {
    return absl::UnknownError(result.message());
  }

  result = IsNear(v1.extrusion_vector, v2.extrusion_vector, error);
  if (!result) {
    return absl::UnknownError(result.message());
  }

  result = IsNear(imp::float3(v1.uv0, 0.0f), imp::float3(v2.uv0, 0.0f), error);
  if (!result) {
    return absl::UnknownError(result.message());
  }

  if (v1.style_index() != v2.style_index()) {
    std::ostringstream o;
    o << "v1.style_index != v2.style_index" << std::endl
      << "v1.style_index:" << v1.style_index() << std::endl
      << "v2.style_index:" << v2.style_index();
    return absl::Status(absl::StatusCode::kUnknown, o.str());
  }

  if (v1.consumed_style_index() != v2.consumed_style_index()) {
    std::ostringstream o;
    o << "v1.consumed_style_index != v2.consumed_style_index" << std::endl
      << "v1.consumed_style_index:" << v1.consumed_style_index() << std::endl
      << "v2.consumed_style_index:" << v2.consumed_style_index();
    return absl::Status(absl::StatusCode::kUnknown, o.str());
  }

  if (v1.cap_index() != v2.cap_index()) {
    std::ostringstream o;
    o << "v1.cam_indices != v2.cap_index" << std::endl
      << "v1.cap_index:" << v1.cap_index() << std::endl
      << "v2.cap_index:" << v2.cap_index();
    return absl::Status(absl::StatusCode::kUnknown, o.str());
  }

  if (v1.packed_zoom_range() != v2.packed_zoom_range()) {
    std::ostringstream o;
    o << "v1.packed_zoom_range!= v2.packed_zoom_range" << std::endl
      << "v1.packed_zoom_range:" << v1.packed_zoom_range() << std::endl
      << "v2.packed_zoom_range:" << v2.packed_zoom_range();
    return absl::Status(absl::StatusCode::kUnknown, o.str());
  }

  float error2 = std::fabs(v1.distance - v2.distance);
  if (std::fabs(v1.distance - v2.distance) > error.x) {
    std::ostringstream o;
    o << "v1.distance != v2.distance" << std::endl
      << "v1.distance:" << v1.distance << std::endl
      << "v2.distance:" << v2.distance << std::endl
      << "err: " << error2;
    return absl::Status(absl::StatusCode::kUnknown, o.str());
  }

  result = IsNear(v1.offset_direction, v2.offset_direction, error);
  if (!result) {
    return absl::UnknownError(result.message());
  }

  return absl::Status(absl::StatusCode::kOk, "");
}
}  // namespace imp::line_renderer
