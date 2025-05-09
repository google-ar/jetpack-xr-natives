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

#include "core/ar/ar_face.h"

#include <string>
#include <utility>

#include "core/common/robin_map.h"

namespace imp {
namespace ar {

ArFace::ArFace(ArTrackableId id, TrackingState tracking_state,
               const mat4f& transform, std::vector<float3> vertices,
               std::vector<float2> tex_coords,
               std::vector<uint16_t> triangle_indices,
               RobinMap<FaceBlendShape, float> shapes,
               const mat4f& left_eye_transform,
               const mat4f& right_eye_transform)
    : ArTrackable(id, kType, tracking_state, transform),
      vertices_(std::move(vertices)),
      tex_coords_(std::move(tex_coords)),
      triangle_indices_(std::move(triangle_indices)),
      blend_shapes_(std::move(shapes)),
      left_eye_transform_(left_eye_transform),
      right_eye_transform_(right_eye_transform) {}

absl::Span<const float3> ArFace::GetVertices() const {
  return absl::MakeSpan(vertices_);
}

absl::Span<const float2> ArFace::GetTextureCoordinates() const {
  return absl::MakeSpan(tex_coords_);
}

absl::Span<const uint16_t> ArFace::GetTriangleIndices() const {
  return absl::MakeSpan(triangle_indices_);
}

const RobinMap<FaceBlendShape, float>& ArFace::GetBlendShapes() const {
  return blend_shapes_;
}

const mat4f& ArFace::GetLeftEyeTransform() const { return left_eye_transform_; }

const mat4f& ArFace::GetRightEyeTransform() const {
  return right_eye_transform_;
}

}  // namespace ar
}  // namespace imp
