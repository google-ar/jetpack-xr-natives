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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_STEREO_SURFACE_MANAGER_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_STEREO_SURFACE_MANAGER_H_

#include <cstdint>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "apibindings/stereo_surface.h"
#include "core/math/math.h"
#include "core/media/media_color_space.h"
#include "core/media/media_type.h"
#include "core/render/content_security_level.h"
#include "core/view/platforms/android/wrappers/surface.h"

namespace imp {

// Manages Jetpack XR SurfaceEntities and their properties.
class StereoSurfaceManager {
 public:
  virtual ~StereoSurfaceManager() = default;

  // Creates a new Impress node and attaches a StereoSurface component to it.
  virtual absl::StatusOr<int32_t> CreateStereoSurfaceEntity(
      MediaStereoMode stereo_mode, MediaBlendingMode blending_mode,
      ContentSecurityLevel content_security_level, bool use_super_sampling) = 0;

  // Sets the canvas shape of a stereo surface using its entity ID.
  virtual absl::Status SetStereoSurfaceEntityCanvasShape(
      int32_t node_id, StereoSurface::CanvasShape canvas_shape) = 0;

  // Attaches or detaches a collider based on enable_collider.
  virtual absl::Status SetStereoSurfaceEntityColliderEnabled(
      int32_t node_id, bool enable_collider) = 0;

  // Returns the surface associated with a stereo surface entity.
  virtual absl::StatusOr<android::Surface*> GetSurfaceFromStereoSurfaceEntity(
      int32_t node_id) = 0;

  // Updates the Surface Dimensions of a stereo surface entity.
  virtual absl::Status SetSurfaceDimensionsForStereoSurfaceEntity(
      int32_t node_id, int32_t width, int32_t height) = 0;

  // Sets the Left/Right and Top/Bottom feather radius of a surface entity.
  virtual absl::Status SetFeatherRadiusForStereoSurfaceEntity(
      int32_t node_id, const float2& feather_radius) = 0;

  // Sets the stereo mode of a stereo surface entity.
  virtual absl::Status SetStereoModeForStereoSurfaceEntity(
      int32_t node_id, MediaStereoMode stereo_mode) = 0;

  // Sets the blending mode of a stereo surface entity.
  virtual absl::Status SetBlendingModeForStereoSurfaceEntity(
      int32_t node_id, MediaBlendingMode blending_mode) = 0;

  // Sets an alpha mask on an stereo surface entity.
  virtual absl::Status SetPrimaryAlphaMaskForStereoSurfaceEntity(
      int32_t node_id, int64_t alpha_mask_token) = 0;

  // Sets an auxiliary alpha mask on an stereo surface entity.
  virtual absl::Status SetAuxiliaryAlphaMaskForStereoSurfaceEntity(
      int32_t node_id, int64_t alpha_mask_token) = 0;

  // Configures the color space metadata for content on the stereo surface.
  virtual absl::Status SetContentColorMetadataForStereoSurfaceEntity(
      int32_t node_id, MediaColorSpace color_space = {}) = 0;

  // Sets the subview configuration of a stereo surface entity.
  virtual absl::Status SetSubViewConfigForStereoSurfaceEntity(
      int32_t node_id, float bottom, float left, float right, float top) = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_STEREO_SURFACE_MANAGER_H_
