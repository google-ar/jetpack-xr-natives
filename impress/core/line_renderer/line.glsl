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

#ifndef MAPS_SHARED_MAPCORE_RENDERER_INTERNAL_MATERIALS_LINE_
#define MAPS_SHARED_MAPCORE_RENDERER_INTERNAL_MATERIALS_LINE_


// The anti-alisas Y coordinate max value will be clamped to this value. This
// was choosen empirically and is dependent on the anti-aliasing asset.
const float kMaxAntialiasCoordY = 0.9;

#ifdef VERTEX
// Camera viewing angle (tilt) range for dynamic widening.
// No scaling is applied at angles below kWidenLowTiltRadians.
// Full scaling is applied at angles higher than kWidenHighTiltRadians.
// Intermediate values have the scale factor linearly interpolated.
// Note: These values were chosen empirically.
const float kWidenLowTiltRadians = 0.685;
const float kWidenHighTiltRadians = 0.785;

// Camera zoom range for dynamic widening.
// No scaling is applied at zooms below kWidenLowZoom.
// Full scaling is applied at zooms higher than kWidenHighZoom.
// Intermediate values have the scale factor linearly interpolated.
// Note: These values were chosen empirically, and are closesly tied to the
// viewing angles allowed for the camera. Any camera angle changes would
// require corresponding changes to these values.
const float kWidenLowZoom = 13.f;
const float kWidenHighZoom = 15.f;

// Depth range for determining closer vs. farther lines. Lines farther away
// get widened more.
// Note: These values were chosen empirically.
const float kWidenLinearDepthWidth = 1.66;
const float kWidenSquaredDepthWidth = 1.f;

float3 GetExtrusionVector() {
  return getCustom0().xyz;
}

float3 GetDirection() {
  return getCustom2().xyz;
}

float3 GetOffsetDirection() {
  return getCustom4().xyz;
}

// Returns the distance to the current vertex from the start of the line after
// extrusion in model coordinates.
float GetDistance(float lineWidthModel) {
  float projectionModel =
    dot(GetDirection(), GetExtrusionVector() * lineWidthModel);
  return getCustom3().x + projectionModel;
}

// Returns the additional width (extrusion length) to apply.
// This is used to dynamically boost the visibility of distant lines that
// are orthogonal to the view when the camera is sufficiently tilted. This
// gets applied everywhere, but is particularly beneficial on low-resolution
// screens like those used with CarPlay.
float additionalWidth(float3 position) {
  if(materialParams.cameraTiltRadians < kWidenLowTiltRadians ||
      materialParams.zoom < kWidenLowZoom) {
    return 0.f;
  }

  // Base additional width is a function of depth.
  mat4 mvp = getClipFromWorldMatrix() * getWorldFromModelMatrix();
  float4 posClip = mulMat4x4Float3(mvp, position);
  // [-1, 1], 0 is center of the screen.
  float depth = max((posClip.y / posClip.w)
    * tan(materialParams.cameraTiltRadians), 0.f);
  if (depth < 0.01) {
    return 0.f;
  }

  float width = (depth * kWidenLinearDepthWidth)
    + (depth * depth * kWidenSquaredDepthWidth);

  // Amount to widen varies with camera tilt.
  float tiltFactor = smoothstep(kWidenLowTiltRadians, kWidenHighTiltRadians,
                                materialParams.cameraTiltRadians);
  width *= tiltFactor;

  // Amount to widen varies with camera zoom.
  float zoomFactor = smoothstep(kWidenLowZoom, kWidenHighZoom,
                                materialParams.zoom);
  width *= zoomFactor;

  // Decrease additional width for lines that are more parallel to camera
  // direction.
  float angleFactor = 1.f - abs(dot(GetDirection(),
                                    materialParams.cameraDirectionWorld));
  width *= angleFactor;
  return width * materialParams.widenIntensity;
}

#endif

#endif  // MAPS_SHARED_MAPCORE_RENDERER_INTERNAL_MATERIALS_LINE_
