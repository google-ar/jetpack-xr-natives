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

// Calculate the alpha value for a given UV position, using a feather radius.
// The alpha value is 1.0 outside the feather radius, and smoothly transitions
// to 0.0 inside the feather radius.
float getEdgeFeatheredAlpha(highp float2 uv, vec2 feather_radius) {
  // Calculate the distance from the nearest edge (0.0 or 1.0)
  float2 dist_from_edge = min(uv, 1.0 - uv);
  // Normalize the distance by the feather radius.
  dist_from_edge /= max(feather_radius, 1e-3);
  // Calculate the alpha value based on the normalized distance.
  float2 alpha = smoothstep(0.0, 1.0, dist_from_edge);
  // Combine the alpha values for the two UVs.
  return alpha.x * alpha.y;
}
