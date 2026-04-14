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

#include "core/view/framework/collision/collision_system.h"

#include "absl/base/optimization.h"
#include "core/camera/camera_component.h"
#include "core/camera/camera_manager.h"
#include "core/collision/ray.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/view/base_view.h"

namespace imp {
namespace details {

Ray GetWorldRayFromPixelPosition(BaseView& view, const float2 screen_pos) {
  const ComponentHandle<CameraComponent> camera =
      view.GetCameraManager().GetCamera();
  // If there is no camera, return a zero ray.
  if (ABSL_PREDICT_FALSE(!camera)) {
    return Ray(kZero3, kZero3);
  }
  return camera->WorldRayFromPixelPoint(screen_pos);
}

DoubleRay GetWorldRayFromPixelPositionPrecise(BaseView& view,
                                              const float2 screen_pos) {
  const ComponentHandle<CameraComponent> camera =
      view.GetCameraManager().GetCamera();
  // If there is no camera, return a zero ray.
  if (ABSL_PREDICT_FALSE(!camera)) {
    return DoubleRay(kZero3, kZero3);
  }
  return camera->WorldRayFromPixelPointPrecise(screen_pos);
}

}  // namespace details

}  // namespace imp
