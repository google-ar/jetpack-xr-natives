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

#include "core/window/projection_helpers.h"

#include "filament/filament/include/filament/Camera.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/TransformManager.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/render_passes/texture_pipeline_renderer_projection_quad.h"

namespace imp {
namespace {}  // namespace

mat4 ComputeProjectionMatrixToFitQuad(const double3& eye_pos,
                                      const mat4& quad_trs,
                                      const float2& quad_size, float near_clip,
                                      float far_clip) {
  mat3 quad_rot = quad_trs.upperLeft();
  double3 quad_center = quad_trs[3].xyz;
  double width = quad_size.x;
  double height = quad_size.y;

  // 1. Define the Target's Basis Vectors in World Space
  // targetRotInWorld columns: 0=Right, 1=Up, 2=Normal (Forward)
  double3 right = quad_rot[0];
  double3 up = quad_rot[1];
  double3 normal = quad_rot[2];

  // 2. Project Eye Position into Target Local Space
  double3 eye_rel = eye_pos - quad_center;

  // Calculate distance from eye to target plane (perpendicular distance)
  // eye_distance = dot product of (eye to target center) and target normal
  double eye_distance = dot(eye_rel, normal);

  // Safeguard: if the eye is behind the target or too close, abort or clamp
  if (eye_distance < near_clip) eye_distance = near_clip;

  // 3. Calculate the Local 2D offsets of the eye relative to the target
  // center
  double2 eye_offsets = double2(dot(eye_rel, right), dot(eye_rel, up));

  // 4. Calculate l, r, b, t boundaries at the Target Plane
  double left_w = (-width / 2.0f) - eye_offsets.x;
  double right_w = (width / 2.0f) - eye_offsets.x;
  double bottom_w = (-height / 2.0f) - eye_offsets.y;
  double top_w = (height / 2.0f) - eye_offsets.y;

  // 5. Map to the Near Clipping Plane
  double scale = near_clip / eye_distance;

  double l = left_w * scale;
  double r = right_w * scale;
  double b = bottom_w * scale;
  double t = top_w * scale;

  return mat4::frustum(l, r, b, t, near_clip, far_clip);
}

void AimCameraToFitQuad(
    filament::Engine* engine, filament::Camera* camera,
    const TexturePipelineRendererProjectionQuad& quad_in_world) {
  const filament::TransformManager& transform_manager =
      engine->getTransformManager();
  const utils::Entity camera_entity = camera->getEntity();
  const mat4 world_from_view = transform_manager.getTransformAccurate(
      transform_manager.getInstance(camera_entity));

  const mat4 world_from_quad =
      mat4::translation(quad_in_world.center) * mat4(quad_in_world.rotation);

  const double near_clip = camera->getNear();
  const double far_clip = camera->getCullingFar();

  // Logic for setting up the camera through the projection quad:
  // - The camera position is unchanged.
  // - The camera optical axis (i.e., perpendicular to the image plane) is
  //   parallel to the quad normal.
  // - The camera roll is set so that the image quad is parallel to the
  //   boundary of the quad. This allows us to move the 4
  //   frustum edges to exactly match the 4 edges of the quad.
  // - The camera frustum is set up to be asymmetric / off-axis, i.e., it is
  //   not symmetrical to the optical axis, and each of the 4 sides falls
  //   directly on the 4 sides of the quad.
  // For more details, see (broken link)

  // Set camera model matrix to (CameraPos, QuadRot)
  Transform<double> world_from_view_transform(world_from_view);
  world_from_view_transform.rotation = quat(quad_in_world.rotation);
  camera->setModelMatrix(world_from_view_transform.AsMat4());

  mat4 projection = ComputeProjectionMatrixToFitQuad(
      world_from_view_transform.translation, world_from_quad,
      quad_in_world.size, near_clip, far_clip);

  camera->setCustomProjection(projection, near_clip, far_clip);
}

}  // namespace imp
