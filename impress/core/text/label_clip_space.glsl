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

#ifndef THIRD_PARTY_ARCORE_AR_IMP_CORE_TEXT_LABELS_CLIP_SPACE_H_
#define THIRD_PARTY_ARCORE_AR_IMP_CORE_TEXT_LABELS_CLIP_SPACE_H_

#ifdef VERTEX

// The input position is expected to be model space "pixel units",
// so a value of 1.0 means "one pixel to the right".
// The world space anchor is expected to be encoded in the mvp transform.
// If snap_to_pixel is true, the position will be pixel snapped only in the y
// direction.
vec4 getClipSpacePosition(vec4 modelPosition, bool snap_to_pixel) {
  vec3 pos = modelPosition.xyz;

  // Experimentally derived constant to get labels a reasonable size.
  // There would need to be math between this and the model coords to
  // acheive the correct point sized text.
  vec2 pixScale = 2.0f * getResolution().zw;
  pos.xy = pos.xy * pixScale;
  pos.z = 0.0;

  // Add in the clip-space translation.
  mat4 mvp = getClipFromWorldMatrix() * getWorldFromModelMatrix();
  vec3 modelToClipOffset = mvp[3].xyz / mvp[3].w;

  if (snap_to_pixel) {
    vec4 clipPosition = vec4(pos + modelToClipOffset, 1);
    vec2 screenPosition = clipPosition.xy / pixScale;
    // On odd resolution screens the position needs to be snapped to a half
    // pixel in order to align to the device pixel.
    screenPosition.y =
        floor(screenPosition.y + 0.5) + mod(getResolution().y, 2.0) * 0.5;
    return vec4(screenPosition.xy * pixScale, clipPosition.z, 1);
  }

  // Affine transformation must have 1 in the w component of translation part,
  // otherwise it defaults to platform specific fallbacks (or errors).
  return vec4(pos + modelToClipOffset, 1);
}

#endif  // VERTEX
#endif  // THIRD_PARTY_ARCORE_AR_IMP_CORE_TEXT_LABELS_CLIP_SPACE_H_
