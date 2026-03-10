/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_CANVAS_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_CANVAS_H_

#include <jni.h>

#include "absl/strings/string_view.h"
#include "core/common/context.h"
#include "core/common/jni_helpers.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/view/platforms/android/wrappers/paint.h"

namespace imp::android {

// JNI wrapper for the Android Paint class.
class Canvas : public JavaWrapper {
 public:
  Canvas(JNIEnv* env, JniUniquePtr<jobject> j_canvas);

  void DrawColor(float3 color);
  void DrawColor(float4 color);
  void Clear();

  void DrawRect(const imp::Rect& rect, Paint& paint);
  void DrawRoundRect(const imp::Rect& rect, float2 corner_radius, Paint& paint);

  void DrawText(absl::string_view text, float2 pos, Paint& paint);

  void DrawPicture(jobject picture);

 private:
  JniHandle draw_color_;
  JniHandle draw_color_with_mode_;
  JniHandle draw_rect_;
  JniHandle draw_round_rect_;
  JniHandle draw_text_;
  JniHandle draw_picture_;
  jobject clear_mode_;
};

}  // namespace imp::android

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_CANVAS_H_
