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

// This file is included by both FL0 and 1 materials.
#if FILAMENT_EFFECTIVE_VERSION == 100
#define TEX texture2D
#else
#define TEX texture
#endif

vec4 getGlyphSample(highp vec2 uv_neg) {
  vec2 uv = abs(uv_neg);
  if (materialParams.ShouldSuperSample) {
    // Value here is a constant close enough to dFdx(uv) * 0.4
    vec2 _ss_x = vec2(1.0 / 1024.0 * 0.4, 0);
    vec4 _ss_a = TEX(materialParams_GlyphAtlas, uv);
    vec4 _ss_b = TEX(materialParams_GlyphAtlas, uv + _ss_x);
    vec4 _ss_c = TEX(materialParams_GlyphAtlas, uv - _ss_x);
    vec4 _ss_d = TEX(materialParams_GlyphAtlas, uv);
    vec4 _ss_e = TEX(materialParams_GlyphAtlas, uv);
    return (_ss_a + _ss_b + _ss_c + _ss_d + _ss_e) / 5.0;
  }
  return TEX(materialParams_GlyphAtlas, uv);
}

// This function returns the glyph color with alpha pre-multiplied. Alpha must
// be multiplied here in the color space of the glyph atlas (sRGB) for text to
// render correctly.
vec4 getGlyphColor(highp vec2 uv, vec4 glyph_sample) {
  vec4 color;
  if (uv.y < 0.0) {
    // When emojis (or unseparable texts that contain emojis) are rendered to
    // the canvas, we want to sample the colors with a single pass, otherwise if
    // the color is semi-opaque, we would be applying the color onto itself
    // twice.
    if (uv.x < 0.0) {
      return float4(0.0);
    }
    // If the y coord is negative, then it is an emoji and we should respect the
    // original color from the atlas. It's in sRGB space so needs to be
    // converted to linear.
    color = vec4(srgbToLinear(glyph_sample.rgb), 1.0) * glyph_sample.a;
    color *= materialParams.TextColorFactor.a;
    return color;
  }

  if (uv.x < 0.0) {
    // If the x coord is negative, then this is the stroke. The red channel is
    // used as the stroke alpha.
    color = materialParams.StrokeColorFactor;
    color.a *= glyph_sample.r * glyph_sample.a;
  } else {
    // If the x coord is positive, then this is the fill. The green channel is
    // used as the fill alpha.
    color = materialParams.TextColorFactor;
    color.a *= glyph_sample.g * glyph_sample.a;
  }
  color.rgb = srgbToLinear(color.rgb);
  color.rgb *= color.a;
  return color;
}

#endif  // THIRD_PARTY_ARCORE_AR_IMP_CORE_TEXT_TEXT_MATERIAL_COMMON_H_
