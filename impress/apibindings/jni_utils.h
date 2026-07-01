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

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "apibindings/stereo_surface.h"

namespace imp {

// Converts a jobject FloatBuffer to an absl::Span<const float>.
absl::StatusOr<absl::Span<const float>> FloatBufferToSpan(JNIEnv* env,
                                                          jobject floatBuffer);

// Converts a jobject IntBuffer to an absl::Span<const uint32_t>.
absl::StatusOr<absl::Span<const uint32_t>> IntBufferToSpan(JNIEnv* env,
                                                           jobject intBuffer);

// Creates a StereoMesh struct from JNI buffers.
absl::StatusOr<StereoSurface::StereoMesh> BuildStereoMesh(
    JNIEnv* env, jobject left_positions, jobject left_texcoords,
    jobject left_indices, jobject right_positions, jobject right_texcoords,
    jobject right_indices, jint draw_mode);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_JNI_UTILS_H_
