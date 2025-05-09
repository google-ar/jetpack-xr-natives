/*
 * Copyright 2025 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef VR_THIRD_PARTY_SPLIT_ENGINE_MATERIALS_PHOTOS_TEXTURE_3D_MATERIAL_PARAMS_H_
#define VR_THIRD_PARTY_SPLIT_ENGINE_MATERIALS_PHOTOS_TEXTURE_3D_MATERIAL_PARAMS_H_

#include "absl/strings/string_view.h"
#include "core/math/mat.h"
#include "core/math/vec.h"

// Helper structs to ensure type safety when setting parameters on the
// PhotosTexture3DMaterial.
namespace params {
struct ShowVideo {
  static constexpr absl::string_view kName = "show_video";
  using ValueType = bool;
};
struct ThumbnailMix {
  static constexpr absl::string_view kName = "thumbnail_mix";
  using ValueType = float;
};
struct BlurMix {
  static constexpr absl::string_view kName = "blur_mix";
  using ValueType = float;
};
struct BlurFallbackColor {
  static constexpr absl::string_view kName = "blur_fallback_color";
  using ValueType = imp::float3;
};
struct IsStereo {
  static constexpr absl::string_view kName = "is_stereo";
  using ValueType = bool;
};
struct IsStereoUsingAuxiliaryTexture {
  static constexpr absl::string_view kName =
      "is_stereo_using_auxiliary_texture";
  using ValueType = bool;
};
struct StereoAxis {
  static constexpr absl::string_view kName = "stereo_axis";
  using ValueType = float;
};
struct StereoDisparityAdjustment {
  static constexpr absl::string_view kName = "stereo_disparity_adjustment";
  using ValueType = float;
};
struct UvTransform {
  static constexpr absl::string_view kName = "uv_transform";
  using ValueType = imp::mat3f;
};
struct ParallaxAmount {
  static constexpr absl::string_view kName = "parallax_amount";
  using ValueType = float;
};
struct ExtraZoomAfterParallax {
  static constexpr absl::string_view kName = "extra_zoom_after_parallax";
  using ValueType = float;
};
struct MaxWindowPopDistance {
  static constexpr absl::string_view kName = "max_window_pop_distance";
  using ValueType = float;
};
struct MaxMediaInsetDistance {
  static constexpr absl::string_view kName = "max_media_inset_distance";
  using ValueType = float;
};
struct EdgeFadeAmount {
  static constexpr absl::string_view kName = "edge_fade_amount";
  using ValueType = float;
};
struct WindowEdgeFadeThickness {
  static constexpr absl::string_view kName = "window_edge_fade_thickness";
  using ValueType = float;
};
struct InsetMediaEdgeFadeThickness {
  static constexpr absl::string_view kName = "inset_media_edge_fade_thickness";
  using ValueType = float;
};
struct CornerRadius {
  static constexpr absl::string_view kName = "corner_radius";
  using ValueType = float;
};
struct CurveParams {
  static constexpr absl::string_view kName = "curve_params";
  using ValueType = imp::float3;
};
struct BlurCenterClearAmount {
  static constexpr absl::string_view kName = "blur_center_clear_amount";
  using ValueType = float;
};
struct BlurCenterEdgeFadeThickness {
  static constexpr absl::string_view kName = "blur_center_edge_fade_thickness";
  using ValueType = float;
};
struct BlurCenterMaskFlip {
  static constexpr absl::string_view kName = "blur_center_mask_flip";
  using ValueType = bool;
};
struct NoiseInBlurMix {
  static constexpr absl::string_view kName = "noise_in_blur_mix";
  using ValueType = float;
};
struct FrameTimeSeconds {
  static constexpr absl::string_view kName = "frame_time_seconds";
  using ValueType = float;
};
struct TimeScale {
  static constexpr absl::string_view kName = "time_scale";
  using ValueType = float;
};
struct PulseXYScale {
  static constexpr absl::string_view kName = "pulse_xy_scale";
  using ValueType = float;
};
struct PulseExponent {
  static constexpr absl::string_view kName = "pulse_exponent";
  using ValueType = imp::float4;
};
struct PulseMultiplier {
  static constexpr absl::string_view kName = "pulse_multiplier";
  using ValueType = imp::float4;
};
struct GrainXYScale {
  static constexpr absl::string_view kName = "grain_xy_scale";
  using ValueType = float;
};
struct GrainMaskExponentAndMultiplier {
  static constexpr absl::string_view kName =
      "grain_mask_exponent_and_multiplier";
  using ValueType = imp::float2;
};
struct GrainDimAmount {
  static constexpr absl::string_view kName = "grain_dim_amount";
  using ValueType = float;
};
struct GrainExponentAndMultiplier {
  static constexpr absl::string_view kName = "grain_exponent_and_multiplier";
  using ValueType = imp::float2;
};
struct IsPano {
  static constexpr absl::string_view kName = "is_pano";
  using ValueType = bool;
};
struct PanoFovDegrees {
  static constexpr absl::string_view kName = "pano_fov_degrees";
  using ValueType = imp::float3;
};
struct PanoSphericalAmount {
  static constexpr absl::string_view kName = "pano_spherical_amount";
  using ValueType = float;
};
struct PanoSphereRadius {
  static constexpr absl::string_view kName = "pano_sphere_radius";
  using ValueType = float;
};
struct PanoBlurFadeToGrayExponent {
  static constexpr absl::string_view kName = "pano_blur_fade_to_gray_exponent";
  using ValueType = imp::float2;
};
struct PanoBlurFadeToGraySubtractor {
  static constexpr absl::string_view kName =
      "pano_blur_fade_to_gray_subtractor";
  using ValueType = imp::float2;
};
struct PanoMediaEdgeFadeExponent {
  static constexpr absl::string_view kName = "pano_media_edge_fade_exponent";
  using ValueType = float;
};
struct PanoCornerRadiusDecreaseExponent {
  static constexpr absl::string_view kName =
      "pano_corner_radius_decrease_exponent";
  using ValueType = float;
};
struct GammaToSrgb {
  static constexpr absl::string_view kName = "gamma_to_srgb";
  using ValueType = float;
};
struct TintColor {
  static constexpr absl::string_view kName = "tint_color";
  using ValueType = imp::float3;
};
struct Opacity {
  static constexpr absl::string_view kName = "opacity";
  using ValueType = float;
};
}  // namespace params

#endif  // VR_THIRD_PARTY_SPLIT_ENGINE_MATERIALS_PHOTOS_TEXTURE_3D_MATERIAL_PARAMS_H_
