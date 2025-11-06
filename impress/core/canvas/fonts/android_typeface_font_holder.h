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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_FONTS_ANDROID_TYPEFACE_FONT_HOLDER_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_FONTS_ANDROID_TYPEFACE_FONT_HOLDER_H_

#include <memory>

#include "absl/strings/string_view.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/fonts/font_params.proto.imp.h"
#include "core/view/platforms/android/wrappers/typeface.h"

namespace imp {

// Helper implementation of FontHolder for Android, wrapping the Typeface class.
// See also AndroidFontFontHolder, which wraps the Font class instead.
class AndroidTypefaceFontHolder : public FontHolder {
 public:
  explicit AndroidTypefaceFontHolder(
      std::unique_ptr<android::Typeface> typeface,
      FontWeight font_weight = FontWeight::FONT_WEIGHT_NORMAL,
      TextStyle text_style = TextStyle::TEXT_STYLE_NORMAL);

  void* GetPlatformFont() override;

  absl::string_view GetFontName() const override;

  FontWeight GetFontWeight() const override;

  TextStyle GetTextStyle() const override;

  bool IsAndroidTypeface() const override { return true; }

 private:
  std::unique_ptr<android::Typeface> typeface_;
  FontWeight font_weight_;
  TextStyle text_style_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_FONTS_ANDROID_TYPEFACE_FONT_HOLDER_H_
