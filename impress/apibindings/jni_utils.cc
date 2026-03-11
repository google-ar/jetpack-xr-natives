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

#include <cstdint>
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

absl::StatusOr<StereoSurface::CustomMesh> BuildCustomMesh(
    JNIEnv* env, jobject left_positions, jobject left_texcoords,
    jobject left_indices, jobject right_positions, jobject right_texcoords,
    jobject right_indices, jint draw_mode) {
  if (left_positions == nullptr) {
    return absl::InvalidArgumentError("left_positions must be non-null.");
  }
  if (left_texcoords == nullptr) {
    return absl::InvalidArgumentError("left_texcoords must be non-null.");
  }

  StereoSurface::CustomMesh mesh;
  // Use temporary variables to work around potential static assertion issues
  // when ASSIGN_OR_RETURN is used directly with struct members.
  MP_ASSIGN_OR_RETURN(mesh.left_positions,
                   FloatBufferToVector(env, left_positions));

  MP_ASSIGN_OR_RETURN(mesh.left_texcoords,
                   FloatBufferToVector(env, left_texcoords));

  if (left_indices != nullptr) {
    MP_ASSIGN_OR_RETURN(mesh.left_indices, IntBufferToVector(env, left_indices));
    for (uint32_t index : *mesh.left_indices) {
      if (index >= mesh.left_positions.size() / 3) {
        return absl::InvalidArgumentError(absl::StrFormat(
            "Left index %d is out of bounds, vertex count is %d.", index,
            mesh.left_positions.size() / 3));
      }
    }
  }

  // Right positions, texcoords, and indices are optional.
  if (right_positions != nullptr && right_texcoords != nullptr) {
    MP_ASSIGN_OR_RETURN(mesh.right_positions,
                     FloatBufferToVector(env, right_positions));
    MP_ASSIGN_OR_RETURN(mesh.right_texcoords,
                     FloatBufferToVector(env, right_texcoords));
    if (right_indices != nullptr) {
      MP_ASSIGN_OR_RETURN(mesh.right_indices,
                       IntBufferToVector(env, right_indices));
      for (uint32_t index : *mesh.right_indices) {
        if (index >= mesh.right_positions->size() / 3) {
          return absl::InvalidArgumentError(absl::StrFormat(
              "Right index %d is out of bounds, vertex count is %d.", index,
              mesh.right_positions->size() / 3));
        }
      }
    }
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
