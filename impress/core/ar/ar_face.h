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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_AR_FACE_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_AR_FACE_H_

#include <string>

#include "core/ar/ar_face.proto.imp.h"
#include "core/ar/ar_trackable.h"
#include "core/common/robin_map.h"
#include "core/math/mat.h"
#include "core/math/vec.h"

namespace imp {
namespace ar {
// The cross-platform representation of a detected face.
class ArFace : public ArTrackable {
  constexpr static TrackableType kType = TrackableType::kFace;

 public:
  ArFace()
      : ArTrackable(ArTrackableId(0), kType, TrackingState::kStopped, mat4f()) {
  }
  // At minimum, we should provide transform and 3D face mesh when instantiating
  // a valid ArFace. We will send blend shapes and eye transforms on platforms
  // where this is available (ARKit for blendshapes and eye transforms and
  // experimental ARCore for blendshapes). Only ARKit is currently supported.
  ArFace(ArTrackableId id, TrackingState tracking_state, const mat4f& transform,
         std::vector<float3> vertices, std::vector<float2> tex_coords,
         std::vector<uint16_t> triangle_indices,
         RobinMap<FaceBlendShape, float> shapes = {},
         const mat4f& left_eye_transform = mat4f(),
         const mat4f& right_eye_transform = mat4f());

  // Array of vertex positions for each point in the 3D mesh representing the
  // detected face. Updates every frame to reflect shape and expression of the
  // user's face.
  absl::Span<const float3> GetVertices() const;

  // Array of texture coordinate values for each point in the 3D face mesh.
  // Constant during the ArSession.
  absl::Span<const float2> GetTextureCoordinates() const;
  // Array of indices describing the triangle mesh formed by the face geometry's
  // vertex data. Constant during the ArSession.
  absl::Span<const uint16_t> GetTriangleIndices() const;

  // Map of facial features to a float [0, 1] indicating the current position of
  // that feature relative to its neutral configuration, where 0 is neutral and
  // 1 is maximum. Only currently available on ARKit and experimental flavors of
  // ARCore.
  const RobinMap<FaceBlendShape, float>& GetBlendShapes() const;
  // Returns map of T to float values of blend shape types. The
  // blend_shape_mapping should be a mapping from FaceBlendShape type
  // to T. This is a convenience function that allows for more easily getting
  // blend shapes in a map of app-specific enums or indices.
  template <typename T>
  RobinMap<T, float> GetMappedBlendShapes(
      const RobinMap<FaceBlendShape, T>& blend_shape_mapping);

  // Position and orientation of the left eye.
  // Only currently valid for ARKit.
  const mat4f& GetLeftEyeTransform() const;
  // Position and orientation of the right eye.
  // Only currently valid for ARKit.
  const mat4f& GetRightEyeTransform() const;

 private:
  std::vector<float3> vertices_;
  std::vector<float2> tex_coords_;
  std::vector<uint16_t> triangle_indices_;
  // Only present on ARKit and experimental ARCore.
  RobinMap<FaceBlendShape, float> blend_shapes_;
  // Only present on ARKit.
  mat4f left_eye_transform_;
  // Only present on ARKit.
  mat4f right_eye_transform_;
};

template <typename T>
RobinMap<T, float> ArFace::GetMappedBlendShapes(
    const RobinMap<FaceBlendShape, T>& blend_shape_mapping) {
  RobinMap<T, float> output;
  for (const auto& kv : blend_shape_mapping) {
    output[kv.second] = blend_shapes_[kv.first];
  }
  return output;
}

// Helper to special case trackable types at compile time.
template <typename T>
constexpr bool IsFace() {
  return std::is_same_v<T, ArFace>;
}

}  // namespace ar
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_AR_FACE_H_
