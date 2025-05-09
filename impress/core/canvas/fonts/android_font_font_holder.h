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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_FONTS_ANDROID_FONT_FONT_HOLDER_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_FONTS_ANDROID_FONT_FONT_HOLDER_H_

#include <memory>
#include <string>

#include "absl/strings/string_view.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/fonts/font_params.proto.imp.h"
#include "core/view/platforms/android/wrappers/font.h"

namespace imp {

// Helper implementation of FontHolder for Android, wrapping the Font class.
// See also AndroidFontTypefaceHolder, which wraps the Typeface class instead.
class AndroidFontFontHolder : public FontHolder {
 public:
  explicit AndroidFontFontHolder(std::unique_ptr<android::Font> font);

  void* GetPlatformFont() override { return font_->WeakReference(); }

  absl::string_view GetFontName() const override { return font_name_; }

  FontWeight GetFontWeight() const override { return font_weight_; }

  TextStyle GetTextStyle() const override { return text_style_; }

 private:
  std::unique_ptr<android::Font> font_;
  std::string font_name_;
  FontWeight font_weight_;
  TextStyle text_style_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_FONTS_ANDROID_FONT_FONT_HOLDER_H_
