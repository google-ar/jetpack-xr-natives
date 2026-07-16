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

#include "apibindings/testing/marshalling/test_stereo_surface_manager.h"

#include <cstdint>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "apibindings/impress_api_view.h"
#include "apibindings/stereo_surface.h"
#include "core/math/math.h"
#include "core/media/media_color_space.h"
#include "core/media/media_type.h"
#include "core/render/content_security_level.h"
#include "core/view/platforms/android/wrappers/surface.h"

namespace imp {

TestStereoSurfaceManager::TestStereoSurfaceManager(ImpressApiView& view) {}

absl::StatusOr<int32_t> TestStereoSurfaceManager::CreateStereoSurfaceEntity(
    MediaStereoMode stereo_mode, MediaBlendingMode blending_mode,
    ContentSecurityLevel content_security_level, bool use_super_sampling) {
  return absl::UnimplementedError(
      "TestStereoSurfaceManager::CreateStereoSurfaceEntity unimplemented");
}

absl::Status TestStereoSurfaceManager::SetStereoSurfaceEntityCanvasShape(
    int32_t node_id, StereoSurface::CanvasShape canvas_shape) {
  return absl::UnimplementedError(
      "TestStereoSurfaceManager::SetStereoSurfaceEntityCanvasShape "
      "unimplemented");
}

absl::Status TestStereoSurfaceManager::SetStereoSurfaceEntityColliderEnabled(
    int32_t node_id, bool enable_collider) {
  return absl::UnimplementedError(
      "TestStereoSurfaceManager::SetStereoSurfaceEntityColliderEnabled "
      "unimplemented");
}

absl::StatusOr<android::Surface*>
TestStereoSurfaceManager::GetSurfaceFromStereoSurfaceEntity(int32_t node_id) {
  return absl::UnimplementedError(
      "TestStereoSurfaceManager::GetSurfaceFromStereoSurfaceEntity "
      "unimplemented");
}

absl::Status
TestStereoSurfaceManager::SetSurfaceDimensionsForStereoSurfaceEntity(
    int32_t node_id, int32_t width, int32_t height) {
  return absl::UnimplementedError(
      "TestStereoSurfaceManager::SetSurfaceDimensionsForStereoSurfaceEntity "
      "unimplemented");
}

absl::Status TestStereoSurfaceManager::SetFeatherRadiusForStereoSurfaceEntity(
    int32_t node_id, const float2& feather_radius) {
  return absl::UnimplementedError(
      "TestStereoSurfaceManager::SetFeatherRadiusForStereoSurfaceEntity "
      "unimplemented");
}

absl::Status TestStereoSurfaceManager::SetStereoModeForStereoSurfaceEntity(
    int32_t node_id, MediaStereoMode stereo_mode) {
  return absl::UnimplementedError(
      "TestStereoSurfaceManager::SetStereoModeForStereoSurfaceEntity "
      "unimplemented");
}

absl::Status TestStereoSurfaceManager::SetBlendingModeForStereoSurfaceEntity(
    int32_t node_id, MediaBlendingMode blending_mode) {
  return absl::UnimplementedError(
      "TestStereoSurfaceManager::SetBlendingModeForStereoSurfaceEntity "
      "unimplemented");
}

absl::Status
TestStereoSurfaceManager::SetPrimaryAlphaMaskForStereoSurfaceEntity(
    int32_t node_id, int64_t alpha_mask_token) {
  return absl::UnimplementedError(
      "TestStereoSurfaceManager::SetPrimaryAlphaMaskForStereoSurfaceEntity "
      "unimplemented");
}

absl::Status
TestStereoSurfaceManager::SetAuxiliaryAlphaMaskForStereoSurfaceEntity(
    int32_t node_id, int64_t alpha_mask_token) {
  return absl::UnimplementedError(
      "TestStereoSurfaceManager::SetAuxiliaryAlphaMaskForStereoSurfaceEntity "
      "unimplemented");
}

absl::Status
TestStereoSurfaceManager::SetContentColorMetadataForStereoSurfaceEntity(
    int32_t node_id, MediaColorSpace color_space) {
  return absl::UnimplementedError(
      "TestStereoSurfaceManager::SetContentColorMetadataForStereoSurfaceEntity "
      "unimplemented");
}

absl::Status TestStereoSurfaceManager::SetSubViewConfigForStereoSurfaceEntity(
    int32_t node_id, float left_bottom, float left_left, float left_right,
    float left_top, float right_bottom, float right_left, float right_right,
    float right_top) {
  return absl::UnimplementedError(
      "TestStereoSurfaceManager::SetSubViewConfigForStereoSurfaceEntity "
      "unimplemented");
}

}  // namespace imp
