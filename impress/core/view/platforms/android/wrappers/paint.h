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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_PAINT_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_PAINT_H_

#include <memory>
#include <optional>
#include <vector>

#include "absl/strings/string_view.h"
#include "core/common/context.h"
#include "core/common/jni_helpers.h"
#include "core/math/vec.h"
#include "core/view/platforms/android/wrappers/rect.h"

namespace imp::android {

// JNI wrapper for the Android Paint class.
class Paint : public JavaWrapper {
 public:
  // (broken link)
  enum class Align { kLeft, kCenter, kRight };
  // (broken link)
  enum class Style { kFill, kStroke, kFillAndStroke };

  class FontMetrics : public JavaWrapper {
   public:
    FontMetrics(JNIEnv* env, jobject j_font_metrics);

    float Leading();

   private:
    JniHandle leading_;
  };

  explicit Paint(const Context& context);

  void SetTextSize(float text_size);
  void SetLetterSpacing(float text_tracking);
  void SetStrokeWidth(float stroke_width);
  void SetStyle(Style style);
  void SetTextAlign(Align align);
  void SetColor(float4 color);
  void SetTypeface(jobject typeface);
  void SetAntiAlias(bool anti_alias);

  std::unique_ptr<Rect> GetTextBounds(absl::string_view text);
  float MeasureText(absl::string_view text);
  float GetRunAdvance(absl::string_view text, int start, int end,
                      int contextStart, int contetxEnd, bool isRtl, int offset);
  std::vector<float> GetTextWidths(absl::string_view text);

  float GetFontSpacing();

  float Ascent();
  float Descent();
  std::unique_ptr<FontMetrics> GetFontMetrics();

 private:
  JniHandle set_text_size_;
  JniHandle set_letter_spacing_;
  JniHandle set_stroke_width_;
  JniHandle set_style_;
  JniHandle set_text_align_;
  JniHandle set_color_;
  JniHandle set_typeface_;
  JniHandle set_anti_alias_;
  JniHandle get_text_bounds_;
  JniHandle measure_text_;
  JniHandle get_font_spacing_;
  JniHandle ascent_;
  JniHandle descent_;
  JniHandle get_font_metrics_;
  JniHandle get_run_advance_;
  JniHandle get_text_widths_;

  std::optional<float> last_text_size_;
  std::optional<float> last_text_tracking_;
  std::optional<float> last_stroke_width_;
  std::optional<Style> last_style_;
  std::optional<Align> last_align_;
  std::optional<float4> last_color_;
  std::optional<jobject> last_typeface_;
  std::optional<bool> last_anti_alias_;
};

}  // namespace imp::android

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_PAINT_H_
