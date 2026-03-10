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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_JXR_MEDIA_MATERIAL_UTILS_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_JXR_MEDIA_MATERIAL_UTILS_H_

#include "core/math/math.h"

namespace imp::split_engine {

// Each float4 here represents the subRectangle of the media texture to be
// displayed: [uMin, vMin, width, height]. These are used to populate
// builtin_jxr_media.mat (leftEyeViewRect and rightEyeViewRect)
struct SubviewRects {
  float4 left_rect;
  float4 right_rect;
};

// Returns rectangles which support the default layout for a given stereo type.
// This allows the jxr_media_material to rely on view rects being set, even if
// talking to an application built against an older version of JXR.
// Returns false if the stereo type is unsupported.
bool StereoTypeToSubviewRects(int stereo_type, SubviewRects& subview_rects);

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_JXR_MEDIA_MATERIAL_UTILS_H_
