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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEST_STEREO_SURFACE_MANAGER_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEST_STEREO_SURFACE_MANAGER_H_

#include <cstdint>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "apibindings/impress_api_view.h"
#include "apibindings/stereo_surface.h"
#include "apibindings/stereo_surface_manager.h"
#include "core/math/math.h"
#include "core/media/media_color_space.h"
#include "core/media/media_type.h"
#include "core/render/content_security_level.h"
#include "core/view/platforms/android/wrappers/surface.h"

namespace imp {

// Inherits from the real StereoSurfaceManager for testing purposes.
class TestStereoSurfaceManager : public StereoSurfaceManager {
 public:
  explicit TestStereoSurfaceManager(ImpressApiView& view);
  ~TestStereoSurfaceManager() override = default;

  absl::StatusOr<int32_t> CreateStereoSurfaceEntity(
      MediaStereoMode stereo_mode, MediaBlendingMode blending_mode,
      ContentSecurityLevel content_security_level,
      bool use_super_sampling) override;
  absl::Status SetStereoSurfaceEntityCanvasShape(
      int32_t node_id, StereoSurface::CanvasShape canvas_shape) override;
  absl::Status SetStereoSurfaceEntityColliderEnabled(
      int32_t node_id, bool enable_collider) override;
  absl::StatusOr<android::Surface*> GetSurfaceFromStereoSurfaceEntity(
      int32_t node_id) override;
  absl::Status SetSurfaceDimensionsForStereoSurfaceEntity(
      int32_t node_id, int32_t width, int32_t height) override;
  absl::Status SetFeatherRadiusForStereoSurfaceEntity(
      int32_t node_id, const float2& feather_radius) override;
  absl::Status SetStereoModeForStereoSurfaceEntity(
      int32_t node_id, MediaStereoMode stereo_mode) override;
  absl::Status SetBlendingModeForStereoSurfaceEntity(
      int32_t node_id, MediaBlendingMode blending_mode) override;
  absl::Status SetPrimaryAlphaMaskForStereoSurfaceEntity(
      int32_t node_id, int64_t alpha_mask_token) override;
  absl::Status SetAuxiliaryAlphaMaskForStereoSurfaceEntity(
      int32_t node_id, int64_t alpha_mask_token) override;
  absl::Status SetContentColorMetadataForStereoSurfaceEntity(
      int32_t node_id, MediaColorSpace color_space = {}) override;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEST_STEREO_SURFACE_MANAGER_H_
