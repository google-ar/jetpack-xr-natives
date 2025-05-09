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
float2 CalculateStereoUvs(float2 originalUvs, int encodingType, bool isLeftEye) {
  float2 newUvs = float2(originalUvs.x, originalUvs.y);
  float eyeUvOffset = isLeftEye ? 0.0 : 0.5;
  if (encodingType == kLeftRight) {
    newUvs.x = (newUvs.x * 0.5) + eyeUvOffset;
  } else if (encodingType == kTopBottom) {
    newUvs.y = (newUvs.y * 0.5) + eyeUvOffset;
  }
  return newUvs;
}

// Calculate the alpha value for a given UV position, using a feather radius.
// The alpha value is 1.0 outside the feather radius, and smoothly transitions
// to 0.0 inside the feather radius.
float getEdgeFeatheredAlpha(highp float2 uv, vec2 feather_radius) {
  // TODO: b/399922916 - Consider polishing this shader for performance and to
  //                     improve the look around the corners of the canvas.
  float alpha_x = 1.0f;
  float alpha_y = 1.0f;

  if (uv.x < feather_radius.x) {
    alpha_x = smoothstep(0.0, 1.0, uv.x / feather_radius.x);
  } else if (uv.x > 1.0f - feather_radius.x) {
    alpha_x = smoothstep(0.0, 1.0, (1.0f - uv.x) / feather_radius.x);
  }

  if (uv.y < feather_radius.y) {
    alpha_y = smoothstep(0.0, 1.0, uv.y / feather_radius.y);
  } else if (uv.y > 1.0f - feather_radius.y) {
    alpha_y = smoothstep(0.0, 1.0, (1.0f - uv.y) / feather_radius.y);
  }

  return alpha_x * alpha_y;
}
