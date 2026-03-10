/*
 * Copyright 2026 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "core/split_engine/materials/builtin/jxr_media_material_utils.h"

#include "core/math/vec.h"
#include "core/media/media_type.h"

namespace imp::split_engine {
constexpr float4 kFullImageRect = float4(0.0f, 0.0f, 1.0f, 1.0f);
constexpr SubviewRects kMonoscopicSubviewRects = {
    .left_rect = kFullImageRect,
    .right_rect = kFullImageRect,
};

bool StereoTypeToSubviewRects(int stereo_type, SubviewRects& subview_rects) {
  switch (static_cast<imp::MediaStereoMode>(stereo_type)) {
    case imp::MediaStereoMode::kMonoscopic:
      // Both eyes see the full image.
      subview_rects = kMonoscopicSubviewRects;
      return true;
    case imp::MediaStereoMode::kTopBottom:
      // Left Eye sees the top half of the image
      // Right eye sees the bottom half
      subview_rects.left_rect = float4(0.0f, 0.5f, 1.0f, 0.5f);
      subview_rects.right_rect = float4(0.0f, 0.0f, 1.0f, 0.5f);
      return true;
    case imp::MediaStereoMode::kLeftRight:
      // Left eye sees the left half of the image.
      // Right eye sees the right half.
      subview_rects.left_rect = float4(0.0f, 0.0f, 0.5f, 1.0f);
      subview_rects.right_rect = float4(0.5f, 0.0f, 0.5f, 1.0f);
      return true;
    case imp::MediaStereoMode::kInterleavedLeftPrimary:
    case imp::MediaStereoMode::kInterleavedRightPrimary:
      // Each eye sees the full image, but the shader knows to sample
      // different textures based on the eye.
      subview_rects = kMonoscopicSubviewRects;
      return true;
    case imp::MediaStereoMode::kUnknown:
    case imp::MediaStereoMode::kStereoMesh:
    case imp::MediaStereoMode::kInterleavedLeftPrimaryWithDepth:
    case imp::MediaStereoMode::kInterleavedRightPrimaryWithDepth:
    default:
      subview_rects = kMonoscopicSubviewRects;
      // JXR doesn't support this value as an option.
      return false;
  }
}

}  // namespace imp::split_engine
