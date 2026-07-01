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

#include "core/render_passes/projection_quad_camera_helper.h"

#include "absl/container/inlined_vector.h"
#include "absl/status/status.h"
#include "filament/filament/include/filament/Camera.h"
#include "filament/filament/include/filament/Engine.h"
#include "core/camera/camera_component.h"
#include "core/math/mat.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/render_passes/texture_pipeline_renderer_projection_quad.h"
#include "core/view/base_view.h"
#include "core/window/projection_helpers.h"

#ifdef __ANDROID__
#include "core/view/platforms/xr_android/xr_session_host.h"
#endif  // __ANDROID__

namespace imp {

absl::Status AimPassCameraEyesAtProjectionQuad(
    ComponentHandle<CameraComponent> pass_camera,
    const TexturePipelineRendererProjectionQuad& quad_in_world) {
  if (!pass_camera) {
    return absl::InvalidArgumentError(
        "AimPassCameraEyesAtProjectionQuad failed: pass_camera is null.");
  }

#if defined(__ANDROID__)
  BaseView& view = pass_camera->GetView();
  if (view.GetHost()->IsInXr()) {
    XrSessionHost* xr_host = static_cast<XrSessionHost*>(view.GetHost());
    absl::StatusOr<XrSessionHost::ViewInfo> latest_views;
    MP_ASSIGN_OR_RETURN(latest_views, xr_host->GetLatestViews());

    SetCameraEyesForProjectionQuad(BaseView::GetSharedEngine(),
                                   pass_camera->GetCamera(), quad_in_world,
                                   latest_views->views);
    return absl::OkStatus();
  }
#endif  // __ANDROID__

  AimCameraToFitQuad(BaseView::GetSharedEngine(), pass_camera->GetCamera(),
                     quad_in_world);
  return absl::OkStatus();
}

absl::Status CopyModelAndEyeProjectionToPassCamera(
    ComponentHandle<CameraComponent> source_camera,
    ComponentHandle<CameraComponent> destination_camera) {
  if (!source_camera || !destination_camera) {
    return absl::InvalidArgumentError(
        "CopyCameraModelAndProjection failed: source_camera or "
        "destination_camera is null.");
  }

  destination_camera->GetNode()->SetWorldTrsPrecise(
      source_camera->GetNode()->GetWorldTrsPrecise());

  // TODO: (broken link) - The ability to use a subset of eyes would
  // improve performance.  When the Pass Camera is used for offscreen rendering
  // purpose it does not need the potential quad-view configuration.
  BaseView& view = source_camera->GetView();
  const int eye_count =
      view.GetHost()->GetEngine()->getConfig().stereoscopicEyeCount;

  filament::Camera* source_filament_camera = source_camera->GetCamera();
  // OpenXR can change these matrices per frame.
  absl::InlinedVector<mat4, 4> eye_projections;
  eye_projections.reserve(eye_count);
  for (int i = 0; i < eye_count; ++i) {
    // Note: getProjectionMatrix() here does not return same values as were
    // set.  Filament modifies the far plane.  This is OK because
    // setCustomEyeProjection will overwrite the far.
    eye_projections.push_back(source_filament_camera->getProjectionMatrix(i));
  };
  destination_camera->GetCamera()->setCustomEyeProjection(
      eye_projections.data(), eye_count,
      source_filament_camera->getCullingProjectionMatrix(),
      source_filament_camera->getNear(),
      source_filament_camera->getCullingFar());
  return absl::OkStatus();
}
}  // namespace imp
