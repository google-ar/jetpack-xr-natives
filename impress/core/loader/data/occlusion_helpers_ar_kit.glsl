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

float getVisibility(float renderedDepth, vec2 backgroundUv) {
  // While iOS never needs to generate ESSL 1.0 code, this file is included by
  // both Feature Level 0 and 1 materials, and Feature Level 0 shaders are
  // always validated against ESSL 1.0.
  float4 depth_texture_sample =
    texture(materialParams_estimatedDepthTexture, backgroundUv);
  // x channel contains the contents of the ARKit Dilated Depth texture.
  float depth = depth_texture_sample.x;
  // y channel contains the contents of the ARKit matte texture.
  float alpha = depth_texture_sample.y;

  // People occlusion occurs if the depth map and alpha map indicate that a body
  // is present that is closer than rendered depth.
  // TODO: We can potentially improve on this to remove flickering
  // around edges and z-fighting at close depths; this is a baseline comparison
  // that matches Apple's sample code and QuickLook people occlusions behavior.
  bool shouldOcclude = depth > 0.0 && alpha > 0.0 && depth < renderedDepth;
  return shouldOcclude ? 1.0 - alpha : 1.0;
}
