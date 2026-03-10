// Copyright 2025 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// The available stereo encoding types for visual stereo media.
// LINT.IfChange
const int kMonoscopic = 0;
const int kTopBottom = 1;
const int kLeftRight = 2;
const int kStereoMesh = 3;
const int kInterleavedLeftPrimary = 4;
const int kInterleavedRightPrimary = 5;
const int kInterleavedLeftPrimaryWithDepth = 6;
const int kInterleavedRightPrimaryWithDepth = 7;
// LINT.ThenChange(//depot/google3/third_party/impress/core/xr/media/xr_media_viewer_state.proto)

/**
 * These values map to the RenderEyeTarget enum in media_type.h.
 */
// This mesh should be rendered in both eyes.
const int kRenderEyeTargetBoth = 0;
// This mesh should be rendered only in the left eye.
const int kRenderEyeTargetLeft = 1;
// This mesh should be rendered only in the right eye.
const int kRenderEyeTargetRight = 2;

float sRGBToLinear(float color) {
  return color <= 0.04045 ? color / 12.92 : pow((color + 0.055) / 1.055, 2.4);
}

vec4 sRGBToLinear(vec4 color) {
  return vec4(
    sRGBToLinear(color.r),
    sRGBToLinear(color.g),
    sRGBToLinear(color.b),
    sRGBToLinear(color.a));
}

// Calculate new stereo UVs using the given UVs and stereo encoding.
// encodingType values match the constants provides above.
// isLeftEye is true if the left eye is being rendered.
highp float2 CalculateStereoUvs(highp float2 originalUvs, int encodingType, bool isLeftEye) {
  highp float2 newUvs = float2(originalUvs.x, originalUvs.y);
  if (encodingType == kLeftRight) {
    highp float eyeUvOffset = isLeftEye ? 0.0 : 0.5;
    newUvs.x = (newUvs.x * 0.5) + eyeUvOffset;
  } else if (encodingType == kTopBottom) {
    // The [top, bottom] halves should be routed to the [left, right] eyes.
    highp float eyeUvOffset = isLeftEye ? 0.5 : 0.0;
    newUvs.y = (newUvs.y * 0.5) + eyeUvOffset;
  }
  return newUvs;
}

// Calculate new stereo UVs using the given UVs and a subview configuration.
// subViewConfig is a float4(x, y, width, height) containing the subview
// configuration, which maps 0-1 UVs to the sub-rectangle of the texture
// corresponding to the current eye.
highp float2 CalculateStereoUvs(highp float2 originalUvs, highp vec4 subViewConfig) {
  return originalUvs * subViewConfig.zw + subViewConfig.xy;
}

// Calculate the alpha value for a given UV position, using a feather radius and
// a corner radius.
// The alpha value is 1.0 outside the feather radius, and smoothly transitions
// to 0.0 inside the feather radius.
float getEdgeFeatheredAlpha(highp float2 uv, float2 feather_radius,
                            float2 corner_radius) {
  float4 edges = float4(0.0, 0.0, 1.0, 1.0); // Left, bottom, right, top

  if (corner_radius.x > 0.0 && corner_radius.y > 0.0) {
    // Transform the coordinates to make the corner radius square.
    float aspect_ratio = corner_radius.x / corner_radius.y;
    uv.y *= aspect_ratio;
    feather_radius.y *= aspect_ratio;
    edges.w *= aspect_ratio;

    float2 uv_half_size = float2(0.5, 0.5 * aspect_ratio);
    // After scaling, the corner radius is the same for both dimensions.
    float  radius = corner_radius.x;
    // Calculate the distance from the inner rectangle that is formed by the
    // corner radius.
    float2 dist = max(abs(uv - uv_half_size) - (uv_half_size - radius), 0.0);
    // Distance squared and radius squared must be highp to avoid flushing to
    // zero for small values of corner_radius (b/475301410)
    highp float2 dist2 = dist * dist;
    highp float radius2 = radius * radius;
    // Calculate the inward offset from the main (0.0 or 1.0) edge.
    // This uses the circular arc equation.
    float corner_u = radius - sqrt(max(0.0, radius2 - dist2.y));
    float corner_v = radius - sqrt(max(0.0, radius2 - dist2.x));
    edges += float4(corner_u, corner_v, -corner_u, -corner_v);
  }

  // Calculate the distance from the nearest edge
  float2 dist_from_edge = min(uv - edges.xy, edges.zw - uv);
  // Normalize the distance by the feather radius.
  feather_radius = max(feather_radius, 1e-4);
  dist_from_edge /= feather_radius;
  // Calculate the alpha value based on the normalized distance.
  float2 alpha = smoothstep(0.0, 1.0, dist_from_edge);
  // Combine the alpha values for the two UVs.
  float feathered_alpha = alpha.x * alpha.y;

  return feathered_alpha;
}
