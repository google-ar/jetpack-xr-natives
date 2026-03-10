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

#include "apibindings/jni_utils.h"

#include <jni.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "apibindings/stereo_surface.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {
// These draw mode values need to stay in sync with JXR.
constexpr int kDrawModeTriangles = 0;
constexpr int kDrawModeTriangleStrip = 1;
// Not supported, falls back to triangles.
constexpr int kDrawModeTriangleFan = 2;

namespace {
absl::StatusOr<std::optional<std::vector<uint32_t>>> ProcessIndices(
    JNIEnv* env, jobject indices_buffer, int vertex_count, int draw_mode) {
  std::optional<std::vector<uint32_t>> indices;
  if (indices_buffer != nullptr) {
    MP_ASSIGN_OR_RETURN(indices, IntBufferToVector(env, indices_buffer));
    // Validate the provided index buffer
    for (uint32_t index : *indices) {
      if (index >= vertex_count) {
        return absl::InvalidArgumentError(
            absl::StrFormat("Index %d is out of bounds, vertex count is %d.",
                            index, vertex_count));
      }
    }
    if (draw_mode == kDrawModeTriangleFan) {
      // Convert the Triangle Fan to a Triangle List; Fans aren't supported by
      // Filament
      std::vector<uint32_t> fan_indices = *indices;
      std::vector<uint32_t> list_indices;
      if (fan_indices.size() >= 3) {
        list_indices.reserve((fan_indices.size() - 2) * 3);
        uint32_t center_index = fan_indices[0];
        for (size_t i = 2; i < fan_indices.size(); ++i) {
          list_indices.push_back(center_index);
          list_indices.push_back(fan_indices[i - 1]);
          list_indices.push_back(fan_indices[i]);
        }
      }
      indices = list_indices;
    }
  } else if (draw_mode == kDrawModeTriangleFan) {
    // Generate a Triangle List index buffer for a Triangle Fan vertex list
    std::vector<uint32_t> new_indices;
    if (vertex_count >= 3) {
      new_indices.reserve((vertex_count - 2) * 3);
      for (int i = 1; i <= vertex_count - 2; ++i) {
        new_indices.push_back(0);
        new_indices.push_back(i);
        new_indices.push_back(i + 1);
      }
    }
    indices = new_indices;
  }
  return indices;
}

absl::StatusOr<int> ValidateMeshAttributes(
    const std::vector<float>& positions, const std::vector<float>& texcoords) {
  if (positions.size() % 3 != 0) {
    return absl::InvalidArgumentError(
        "Position buffer size must be divisible by 3.");
  }
  if (texcoords.size() % 2 != 0) {
    return absl::InvalidArgumentError(
        "Texcoord buffer size must be divisible by 2.");
  }
  const int vertex_count = positions.size() / 3;
  const int tex_coord_count = texcoords.size() / 2;
  if (vertex_count != tex_coord_count) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Position count (%d) does not match texture coordinate count "
        "(%d).",
        vertex_count, tex_coord_count));
  }
  return vertex_count;
}
}  // namespace

absl::StatusOr<std::vector<float>> FloatBufferToVector(JNIEnv* env,
                                                       jobject floatBuffer) {
  // Try to get the raw address. Returns nullptr if buffer is not Direct.
  void* rawAddr = env->GetDirectBufferAddress(floatBuffer);
  if (rawAddr != nullptr) {
    float* ptr = static_cast<float*>(rawAddr);

    // Get capacity (number of elements, not bytes, for a FloatBuffer)
    jlong capacity = env->GetDirectBufferCapacity(floatBuffer);
    // Create vector and copy data in one shot (Constructor Copy)
    // TODO: Investigate returning absl::span to prevent a copy
    return std::vector<float>(ptr, ptr + capacity);
  } else {
    // TODO: Fallback if the buffer is not Direct.
    return absl::InvalidArgumentError("FloatBuffer must be a direct buffer.");
  }
}

absl::StatusOr<std::vector<uint32_t>> IntBufferToVector(JNIEnv* env,
                                                        jobject intBuffer) {
  // Try to get the raw address. Returns nullptr if buffer is not Direct.
  void* rawAddr = env->GetDirectBufferAddress(intBuffer);
  if (rawAddr != nullptr) {
    uint32_t* ptr = static_cast<uint32_t*>(rawAddr);

    // Get capacity (number of ints)
    jlong capacity = env->GetDirectBufferCapacity(intBuffer);
    for (jlong i = 0; i < capacity; ++i) {
      if (ptr[i] < 0) {
        return absl::InvalidArgumentError(absl::StrFormat(
            "Index %d must be non-negative, but got %d.", i, ptr[i]));
      }
    }
    // Create vector and copy data in one shot (Constructor Copy)
    // TODO: Investigate returning absl::span to prevent a copy
    return std::vector<uint32_t>(ptr, ptr + capacity);
  } else {
    // TODO: Fallback if the buffer is not Direct.
    return absl::InvalidArgumentError("IntBuffer must be a direct buffer.");
  }
}

absl::StatusOr<StereoSurface::StereoMesh> BuildStereoMesh(
    JNIEnv* env, jobject left_positions, jobject left_texcoords,
    jobject left_indices, jobject right_positions, jobject right_texcoords,
    jobject right_indices, jint draw_mode) {
  if (left_positions == nullptr) {
    return absl::InvalidArgumentError("left_positions must be non-null.");
  }
  if (left_texcoords == nullptr) {
    return absl::InvalidArgumentError("left_texcoords must be non-null.");
  }

  StereoSurface::StereoMesh mesh;
  // Use temporary variables to work around potential static assertion issues
  // when ASSIGN_OR_RETURN is used directly with struct members.
  MP_ASSIGN_OR_RETURN(mesh.left_positions,
                   FloatBufferToVector(env, left_positions));

  MP_ASSIGN_OR_RETURN(mesh.left_texcoords,
                   FloatBufferToVector(env, left_texcoords));

  MP_ASSIGN_OR_RETURN(
      const int left_vertex_count,
      ValidateMeshAttributes(mesh.left_positions, mesh.left_texcoords));
  MP_ASSIGN_OR_RETURN(
      mesh.left_indices,
      ProcessIndices(env, left_indices, left_vertex_count, draw_mode));

  // Right positions, texcoords, and indices are optional.
  if (right_positions != nullptr && right_texcoords != nullptr) {
    MP_ASSIGN_OR_RETURN(mesh.right_positions,
                     FloatBufferToVector(env, right_positions));
    MP_ASSIGN_OR_RETURN(mesh.right_texcoords,
                     FloatBufferToVector(env, right_texcoords));
    MP_ASSIGN_OR_RETURN(
        const int right_vertex_count,
        ValidateMeshAttributes(*mesh.right_positions, *mesh.right_texcoords));
    MP_ASSIGN_OR_RETURN(
        mesh.right_indices,
        ProcessIndices(env, right_indices, right_vertex_count, draw_mode));
  }

  switch (draw_mode) {
    case kDrawModeTriangles:
    // fans are not supported, fallback to triangles.
    case kDrawModeTriangleFan:
      mesh.draw_mode = filament::RenderableManager::PrimitiveType::TRIANGLES;
      break;
    case kDrawModeTriangleStrip:
      mesh.draw_mode =
          filament::RenderableManager::PrimitiveType::TRIANGLE_STRIP;
      break;
    default:
      return absl::InvalidArgumentError(
          absl::StrFormat("Invalid draw mode: %d.", draw_mode));
  }
  return mesh;
}

}  // namespace imp
