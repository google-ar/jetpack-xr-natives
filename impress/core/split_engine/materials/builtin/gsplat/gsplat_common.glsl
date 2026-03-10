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

ivec2 getSampleCoord(int index, int stride, highp sampler2D dataTexture) {
  int dataWidth = textureSize(dataTexture, 0).x;
  return ivec2(index * stride % dataWidth, index * stride / dataWidth);
}

ivec2 getSampleCoord(int index, int stride, highp usampler2D dataTexture) {
  int dataWidth = textureSize(dataTexture, 0).x;
  return ivec2(index * stride % dataWidth, index * stride / dataWidth);
}

// unpacks the 3d covariance matrix from it's packed uint3 representation.
// expects the uint3 to encode the upper half of the 3d covariance matrix
// via half precision floats.
mat3 unpackCov3d(uint3 cov3dPacked) {
  vec2 cov3dMat0 = unpackHalf2x16(cov3dPacked.x);
  vec2 cov3dMat1 = unpackHalf2x16(cov3dPacked.y);
  vec2 cov3dMat2 = unpackHalf2x16(cov3dPacked.z);
  return mat3(cov3dMat0.x, cov3dMat0.y, cov3dMat1.x,
              cov3dMat0.y, cov3dMat1.y, cov3dMat2.x,
              cov3dMat1.x, cov3dMat2.x, cov3dMat2.y);
}

// Computes cov2d from cov3d, use eq(5) in the 3DGS paper.
vec3 computeCov2d(vec3 eyePos, mat3 eyeFromModel, mat4 clipFromEye, mat4 eyeFromClip, mat3 cov3d, vec2 resolution) {
  vec2 focal = resolution * vec2(clipFromEye[0][0], clipFromEye[1][1]) / 2.0;

  mat3 jacobian;
  const float epsilon = 1e-5;
  if (abs(clipFromEye[3][3]) > epsilon) {
    // Orthographic Projection
    jacobian = mat3(
        focal.x, 0,           0,
        0,       focal.y,     0,
        0,       0,           0
    );
  }
  else {
    // Perspective Projection
    // this is needed in order for splats that are visible in view but clipped "quite a lot" to work
    vec2 tanFov = vec2(eyeFromClip[0][0], eyeFromClip[1][1]);
    vec2 limit = 1.3 * tanFov;
    eyePos.xy = clamp(eyePos.xy / eyePos.z, -limit, limit) * eyePos.z;
    jacobian = mat3(
        focal.x / eyePos.z, 0,                  -(focal.x * eyePos.x) / (eyePos.z * eyePos.z),
        0,                  focal.y / eyePos.z, -(focal.y * eyePos.y) / (eyePos.z * eyePos.z),
        0,                  0,                  0
    );
  }

  mat3 jw = transpose(eyeFromModel) * jacobian;
  mat3 cov = transpose(jw) * cov3d * jw;

  // Low pass filter to make each splat at least 1px size.
  // TODO: Remove this logic and update Scuba tests.
  cov[0][0] += 0.3;
  cov[1][1] += 0.3;
  // Only need upper half of matrix since it's diagonal
  return vec3(cov[0][0], cov[0][1], cov[1][1]);
}

// Computes scale and rotation from 2D covariance via eigenvalue/vector decomposition
// Will clamp the size of each gSplat to a specifed minimum and maximum size
void scaleAndRotationFromCov2D(vec3 cov2d, vec2 minSize, vec2 maxSize,
                               out vec2 basisAxisX, out float aspectRatio) {
  float diag1 = cov2d.x, diag2 = cov2d.z, offDiag = cov2d.y;
  float mid = 0.5 * (diag1 + diag2);
  float radius = length(vec2((diag1 - diag2) / 2.0, offDiag));
  float2 lambda = float2(mid + radius, max(mid - radius, 0.1));
  vec2 diagVec = normalize(vec2(offDiag, lambda.x - diag1));
  float2 size = min(max(sqrt(2.0 * lambda), minSize), maxSize);
  basisAxisX = size.x * diagVec;
  aspectRatio = size.y / size.x;
}

// computes the clip space position, x-axis basis for quad in NDC space, and
// aspect ratio from the splat data. The y-axis basis can be computed from the
// aspect ratio and the x-axis basis.
// eyeIndex of -1 indicates mono mode.
// minSize and maxSize clamp are used to clamp each gSplat by a minimum and
// maximum bounds
void computeSplatData(int eyeIndex, vec4 position, mat3 cov3d,
                      mat4 worldFromModelMatrix, vec2 resolution, vec2 minSize,
                      vec2 maxSize, out vec4 clipPos, out vec2 basisAxisX,
                      out float aspectRatio) {
  mat4 eyeFromModel;
  mat4 clipFromEye;
  mat4 eyeFromClip;

  if (eyeIndex >= 0) {
    eyeFromModel =
      getEyeFromViewMatrix(eyeIndex) * getViewFromWorldMatrix() * worldFromModelMatrix;
    clipFromEye =
      getClipFromWorldMatrix(eyeIndex) * inverse(getEyeFromViewMatrix(eyeIndex) * getViewFromWorldMatrix());
    eyeFromClip = inverse(clipFromEye);
  } else {
    // When rendering in mono mode, we can use a simpler construction to avoid
    // an extra matrix multiplication.
    eyeFromModel = getViewFromWorldMatrix() * worldFromModelMatrix;
    clipFromEye = getClipFromViewMatrix();
    eyeFromClip = getViewFromClipMatrix();
  }
  vec4 eyePos = eyeFromModel * position;

  // TODO : Profile performance and see if reducing divergence by removing this check is worth it.
  // If behind the camera discard.
  if (eyePos.z > 0.0)
  {
    clipPos = vec4(0.0, 0.0, 0.0, 0.0);
    basisAxisX = vec2(0.0, 0.0);
    aspectRatio = 0.0;
    return;
  }

  // Compute cov2d from cov3d.
  vec3 cov2d = computeCov2d(eyePos.xyz, mat3(eyeFromModel), clipFromEye,
                            eyeFromClip, cov3d, resolution);
  // Get basis axes for quad in NDC space (which encodes the scale and
  // rotation) from cov2d.
  scaleAndRotationFromCov2D(cov2d, minSize, maxSize, basisAxisX, aspectRatio);
  // Get the center of the quad associated with this splat.
  clipPos = clipFromEye * eyePos;
  // perspective divide ahead of time to avoid having to do this later in the
  // rendering vertex shader.
  clipPos /= clipPos.w;
}

// The clip space convention used by Impress getClip*() functions is what's
// defined as "DX Convention", where depth/z is in the range [1, 0].
// However, when the material is set to vertexDomain : Device, clip space is
// expected to be in "GL Convention" which has a depth/z range of [-1, 1].
// So we need to apply a conversion to the depth/z value of our clipPos
// from "DX Convention" [1, 0] to "GL Convention" [-1, 1]
mat4 createClipSpaceTransform(vec4 clipPos) {
  clipPos.z = -2.0 * (clipPos.z - 0.5);
  return mat4(vec4(0), vec4(0), vec4(0), clipPos);
}

// Returns the vertex coordinate for the given vertex index of the splat being
// rendered. If useTrianglesForSplats is true, the vertex coordinates are
// computed for a triangle, otherwise they are computed for a 2 triangle quad.
vec2 getVertexCoord(int vertIndex, bool useTrianglesForSplats) {
  if (useTrianglesForSplats) {
    const vec2 triCoords[3] = vec2[3](
      // 3 vertices per splat to create a triangle that inscribes a unit circle.
      vec2( 0.000,   1.0) * 2.0,
      vec2( sqrt(3.0) * 0.5,  -0.5) * 2.0,
      vec2(-sqrt(3.0) * 0.5,  -0.5) * 2.0
    );
    return triCoords[vertIndex % 3];
  }
  else {
    const vec2 quadCoords[6] = vec2[6](
      // 6 vertices per splat to make a 2 triangle quad.
      vec2(-1.0,  1.0),
      vec2( 1.0,  1.0),
      vec2(-1.0, -1.0),
      vec2(-1.0, -1.0),
      vec2( 1.0,  1.0),
      vec2( 1.0, -1.0)
    );
    return quadCoords[vertIndex % 6];
  }
}

// Mono version of fetchPrecomputedSplatData
// eyeIndex is unused, but kept so the function signature matches the 3D version.
uint4 fetchPrecomputedSplatData(int splatIndex, int splatDataWidth, int /*eyeIndex*/, const highp sampler2D dataTexture) {
  ivec2 sampleCoord = ivec2(0);
  sampleCoord.x = splatIndex % splatDataWidth;
  sampleCoord.y = splatIndex / splatDataWidth;
  highp vec4 splatDataPrecomputedFloats = texelFetch(dataTexture, sampleCoord, 0);
  return floatBitsToUint(splatDataPrecomputedFloats);
}

// Stereo version of fetchPrecomputedSplatData
uint4 fetchPrecomputedSplatData(int splatIndex, int splatDataWidth, int eyeIndex, const highp sampler2DArray dataTexture) {
  ivec3 sampleCoord = ivec3(0);
  sampleCoord.x = splatIndex % splatDataWidth;
  sampleCoord.y = splatIndex / splatDataWidth;
  // eyeIndex is used to index depth into the 3D texture.
  sampleCoord.z = eyeIndex;
  highp vec4 splatDataPrecomputedFloats = texelFetch(dataTexture, sampleCoord, 0);
  return floatBitsToUint(splatDataPrecomputedFloats);
}
