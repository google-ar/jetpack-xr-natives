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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_PROJECTION_QUAD_CAMERA_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_PROJECTION_QUAD_CAMERA_HELPER_H_

#include "absl/status/status.h"
#include "core/camera/camera_component.h"
#include "core/ncsb/component_handle.h"
#include "core/render_passes/texture_pipeline_renderer_projection_quad.h"

namespace imp {

// Sets the eye model matrices and projection matrices for a projection on the
// given `quad_in_world`. Stereoscopic eyes are supported.
absl::Status AimPassCameraEyesAtProjectionQuad(
    ComponentHandle<CameraComponent> pass_camera,
    const TexturePipelineRendererProjectionQuad& quad_in_world);

// Repositions a camera to match the source camera and copies the source
// camera's view and per eye projection matrices to the destination camera.
// Stereoscopic configurations are supported.
absl::Status CopyModelAndEyeProjectionToPassCamera(
    ComponentHandle<CameraComponent> source_camera,
    ComponentHandle<CameraComponent> destination_camera);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_PROJECTION_QUAD_CAMERA_HELPER_H_
