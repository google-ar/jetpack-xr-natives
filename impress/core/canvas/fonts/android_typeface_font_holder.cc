// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/canvas/fonts/android_typeface_font_holder.h"

#include <memory>
#include <utility>

#include "absl/strings/string_view.h"
#include "core/canvas/fonts/font_params.proto.imp.h"
#include "core/view/platforms/android/wrappers/typeface.h"

namespace imp {

AndroidTypefaceFontHolder::AndroidTypefaceFontHolder(
    std::unique_ptr<android::Typeface> typeface, FontWeight font_weight,
    TextStyle text_style)
    : typeface_(std::move(typeface)),
      font_weight_(font_weight),
      text_style_(text_style) {}

void* AndroidTypefaceFontHolder::GetPlatformFont() {
  return static_cast<void*>(typeface_->WeakReference());
}

absl::string_view AndroidTypefaceFontHolder::GetFontName() const {
  return typeface_->GetFontFamilyName();
}

FontWeight AndroidTypefaceFontHolder::GetFontWeight() const {
  return font_weight_;
}

TextStyle AndroidTypefaceFontHolder::GetTextStyle() const {
  return text_style_;
}

}  // namespace imp
