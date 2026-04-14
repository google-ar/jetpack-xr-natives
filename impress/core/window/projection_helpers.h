// Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_WINDOW_PROJECTION_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_WINDOW_PROJECTION_HELPERS_H_

#include "filament/filament/include/filament/Engine.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/render_passes/texture_pipeline_renderer_projection_quad.h"

namespace imp {

// Computes a projection matrix for a camera at `eye_pos`, with its rotation
// the same as the rotation in `quad_trs` so that:
// - The view direction is aligned with the quad's normal (i.e., the Z-axis in
//   `quad_trs`).
// - The horizontal and vertical axes of the camera also aligned with the quad's
//   horizontal and vertical edges.
// With that, we can then adjust the frustum's 4 planes to fit the quad
// perfectly.
// See (broken link) for an illustration.
mat4 ComputeProjectionMatrixToFitQuad(const double3& eye_pos,
                                      const mat4& quad_trs,
                                      const float2& quad_size, float near_clip,
                                      float far_clip);

// Aims camera at a virtual quad defined in world space.
// Overwrites camera's projection and view matrices so that the Quad fills
// the viewport. Does not support stereoscopic cameras.
void AimCameraToFitQuad(
    filament::Engine* engine, filament::Camera* camera,
    const TexturePipelineRendererProjectionQuad& quad_in_world);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_WINDOW_PROJECTION_HELPERS_H_
