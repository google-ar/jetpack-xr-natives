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

#ifndef THIRD_PARTY_ARCORE_AR_IMP_CORE_TEXT_TEXT_MATERIAL_COMMON_H_
#define THIRD_PARTY_ARCORE_AR_IMP_CORE_TEXT_TEXT_MATERIAL_COMMON_H_

#include "text_colors.glsl"

vec4 getGlyphSample(highp vec2 uv) {
  // This file is included by both FL0 and 1 materials.
#if FILAMENT_EFFECTIVE_VERSION == 100
  return texture2D(materialParams_GlyphAtlas, abs(uv));
#else
  return texture(materialParams_GlyphAtlas, abs(uv));
#endif
}

// This function returns the glyph color with alpha pre-multiplied.
vec4 getGlyphColor(highp vec2 uv, vec4 glyph_sample) {
  vec4 color;
  if (uv.y < 0.0) {
    // If the y coord is negative, then it is an emoji and we should respect the
    // original color from the atlas. It's in sRGB space so needs to be
    // converted to linear.
    // color = vec4(glyph_sample.rgb, 1.0) * glyph_sample.a;
    color = glyph_sample;
    color.rgb = srgbToLinear(color.rgb);
    color *= materialParams.TextColorFactor.a;
    return color;
  }

  if (uv.x < 0.0) {
    // If the x coord is negative, then this is the stroke. The red channel is
    // used as the stroke alpha.
    color = materialParams.StrokeColorFactor;
    color.a *= glyph_sample.r;
  } else {
    // If the x coord is positive, then this is the fill. The green channel is
    // used as the fill alpha.
    color = materialParams.TextColorFactor;
    color.a *= glyph_sample.g;
  }
  color.rgb = srgbToLinear(color.rgb);
  color.rgb *= color.a;
  return color;
}

#endif  // THIRD_PARTY_ARCORE_AR_IMP_CORE_TEXT_TEXT_MATERIAL_COMMON_H_
