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

#include "occlusion_helpers_common.glsl"

vec2 getBackgroundUvFromWorldPosition(vec3 worldPosition) {
  mat4 uvFromWorldTransform = transposeCompat(mat4(
         vec4(materialParams.backgroundUvFromNdc0.xy, 0, materialParams.backgroundUvFromNdc0.z),
         vec4(materialParams.backgroundUvFromNdc1.xy, 0, materialParams.backgroundUvFromNdc1.z),
         vec4(0,0,1,0),
         vec4(0,0,0,1)))
       * getClipFromWorldMatrix();
  vec4 positionClip = uvFromWorldTransform * vec4(worldPosition, 1.0);
  return positionClip.xy / positionClip.w;
}

vec2 getDepthAndMinVisibility(vec2 backgroundUv) {
  // Depth is packed into the red and green components of its texture.
  // The texture is a normalized format, storing millimeters.
#if MATERIAL_FEATURE_LEVEL == 0
  vec3 packedDepthAndVisibility =
      texture(materialParams_estimatedDepthTexture, backgroundUv).xyz;
#else
  vec3 packedDepthAndVisibility =
      textureLod(materialParams_estimatedDepthTexture, backgroundUv, 0.0).xyz;
#endif
  return vec2(
      dot(packedDepthAndVisibility.xy, vec2(255, 256 * 255)),
      packedDepthAndVisibility.z);
}

float GetVisibility(vec2 depthAndMinVisibility, vec2 assetDepthFactor) {
  float depth_mm = depthAndMinVisibility.x;
  float minVisibility = depthAndMinVisibility.y;
  float uOcclusionAlpha = 1.0 - materialParams.maximumOcclusionFactor;

  // Instead of a hard z-buffer test, allows the asset to fade into the
  // background along a 2 * uDepthTolerancePerMm * asset_depth_mm
  // range centered on the background depth.
  float visibility_occlusion = depth_mm * assetDepthFactor.x + assetDepthFactor.y;

  return clamp(visibility_occlusion, max(minVisibility, uOcclusionAlpha), 1.0);
}

vec2 getAssetDepthFactor(float depthTolerancePerMm, float assetDepthMm) {
  return vec2(
      0.5 / (depthTolerancePerMm * assetDepthMm),
      -0.5 / depthTolerancePerMm + 0.5);
}

float GetBlurredVisibilityAroundUV(vec2 uv, float uv_step,
                                   vec2 assetDepthFactor) {
  // Kernel used:
  // 1   4   7   4   1
  // 4   16  26  16  4
  // 7   26  41  26  7
  // 4   16  26  16  4
  // 1   4   7   4   1
  const float kKernelTotalWeights = 273.0;
  float sum = 0.0;

  vec2 step =  vec2(uv_step, uv_step * materialParams.depthTextureAspectRatio);

  sum += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(-3.0, -3.0) * step), assetDepthFactor);
  sum += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(+3.0, -3.0) * step), assetDepthFactor);
  sum += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(-3.0, +3.0) * step), assetDepthFactor);
  sum += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(+3.0, +3.0) * step), assetDepthFactor);

  float current = 0.0;

  current += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(-2.0, -3.0) * step), assetDepthFactor);
  current += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(+2.0, -3.0) * step), assetDepthFactor);
  current += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(-2.0, +3.0) * step), assetDepthFactor);
  current += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(+2.0, +3.0) * step), assetDepthFactor);
  current += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(-3.0, +2.0) * step), assetDepthFactor);
  current += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(+3.0, +2.0) * step), assetDepthFactor);
  current += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(-3.0, -2.0) * step), assetDepthFactor);
  current += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(+3.0, -2.0) * step), assetDepthFactor);
  sum += current * 4.0;

  current = 0.0;
  current += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(-3.0, -0.0) * step), assetDepthFactor);
  current += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(+3.0, +0.0) * step), assetDepthFactor);
  current += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(+0.0, +3.0) * step), assetDepthFactor);
  current += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(-0.0, -3.0) * step), assetDepthFactor);
  sum += current * 7.0;

  current = 0.0;
  current += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(-1.0, -1.0) * step), assetDepthFactor);
  current += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(+1.0, -1.0) * step), assetDepthFactor);
  current += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(-1.0, +1.0) * step), assetDepthFactor);
  current += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(+1.0, +1.0) * step), assetDepthFactor);
  sum += current * 16.0;

  current = 0.0;
  current += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(+0.0, +1.0) * step), assetDepthFactor);
  current += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(-0.0, -1.0) * step), assetDepthFactor);
  current += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(-1.0, -0.0) * step), assetDepthFactor);
  current += GetVisibility(
      getDepthAndMinVisibility(uv + vec2(+1.0, +0.0) * step), assetDepthFactor);
  sum += current * 26.0;

  sum +=
      GetVisibility(getDepthAndMinVisibility(uv), assetDepthFactor)
      * 41.0;

  return sum / kKernelTotalWeights;
}

float getVisibility(float renderedDepth, vec2 backgroundUv) {
  float asset_depth_mm = renderedDepth * 1000.0;
  vec2 depthAndMinVisibility = getDepthAndMinVisibility(backgroundUv);
  vec2 assetDepthFactor = getAssetDepthFactor(
      materialParams.depthTolerancePerMm, asset_depth_mm);
  float visibility = materialParams.edgeBlur > 0.0 ?
      GetBlurredVisibilityAroundUV(backgroundUv, materialParams.edgeBlur,
                                   assetDepthFactor) :
      GetVisibility(depthAndMinVisibility, assetDepthFactor);
  return visibility;
}
