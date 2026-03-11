/*
 * Copyright 2025 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_JNI_UTILS_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_JNI_UTILS_H_

#include <jni.h>

#include <cstdint>
#include <vector>

#include "absl/status/statusor.h"
#include "apibindings/stereo_surface.h"

namespace imp {

// Converts a jobject FloatBuffer to a std::vector<float>.
// TODO: Investigate returning absl::span to prevent a copy
absl::StatusOr<std::vector<float>> FloatBufferToVector(JNIEnv* env,
                                                       jobject floatBuffer);

// Converts a jobject IntBuffer to a std::vector<uint32_t>.
// TODO: Investigate returning absl::span to prevent a copy
absl::StatusOr<std::vector<uint32_t>> IntBufferToVector(JNIEnv* env,
                                                        jobject intBuffer);

// Creates a CustomMesh struct from JNI buffers.
absl::StatusOr<StereoSurface::CustomMesh> BuildCustomMesh(
    JNIEnv* env, jobject left_positions, jobject left_texcoords,
    jobject left_indices, jobject right_positions, jobject right_texcoords,
    jobject right_indices, jint draw_mode);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_JNI_UTILS_H_
