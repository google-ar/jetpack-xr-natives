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

// Converts a scalar in srgb color space to linear color space.
float srgbToLinear(float color) {
  return color <= 0.04045
      ? color / 12.92
      : pow((color + 0.055) / 1.055, 2.4);
}

// Converts a vec3 in srgb color space to linear color space.
vec3 srgbToLinear(vec3 color) {
  return vec3(
      srgbToLinear(color.x),
      srgbToLinear(color.y),
      srgbToLinear(color.z));
}

// Returns the final uv coordinates for a given sampler index.
//
// Picks if UV0 or UV1 should be used based on samplers_uv_bitflags bitflags.
// Used to implement "texcoord" properties from glTF spec.
//
// Transforms the uvs using the matrix in samplers_uv_matrices.
// Used to implement KHR_texture_transform glTF Extension.
highp vec2 uvForSampler(lowp int samplerIndex) {
#if FILAMENT_EFFECTIVE_VERSION == 100
  // ESSL 1.0 doesn't support bit shifting operations.
  bool isUV1 = mod(float(materialParams.samplers_uv_bitflags) /
                   pow(2.0, float(samplerIndex)), 1.0) != 0.0;
#else
  bool isUV1 = (materialParams.samplers_uv_bitflags & (1 << samplerIndex)) > 0;
#endif

  highp vec2 uv = isUV1 ? getUV1() : getUV0();
  highp mat3 rhs = materialParams.samplers_uv_matrices[samplerIndex];
  rhs[2] = vec3(0.0, 0.0, 0.0);
  return (vec3(uv, 1.0) * rhs).xy;
}

// Returns vec4 representing the color white.
// Used as a fallback for unassigned textures.
highp vec4 whiteFallbackSample() {
  return vec4(1.0, 1.0, 1.0, 1.0);
}

// Returns vec4 representing the normals.
// Used as a fallback for unassigned textures.
highp vec4 normalFallbackSample() {
  return vec4(0.498, 0.498, 1.0, 1.0);
}
