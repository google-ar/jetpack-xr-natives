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

#include "core/canvas/fonts/android_system_font_provider.h"

#include <memory>
#include <string>
#include <utility>

#include "absl/base/attributes.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "core/canvas/fonts/android_typeface_font_holder.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/fonts/font_params.proto.imp.h"
#include "core/common/context.h"
#include "core/view/platforms/android/wrappers/typeface.h"

namespace imp {
namespace {
// Constants are from
// https://developer.android.com/reference/android/graphics/Typeface#constants_1
constexpr int kFontWeightBold = 1;
constexpr int kTextStyleItalic = 2;
}  // namespace

std::unique_ptr<FontHolder> LoadSystemAndroidFont(const Context& context,
                                                  absl::string_view family_name,
                                                  FontWeight font_weight,
                                                  TextStyle text_style) {
  uint android_text_style = 0;
  if (font_weight == FontWeight::FONT_WEIGHT_BOLD) {
    android_text_style |= kFontWeightBold;
  }
  if (text_style == TextStyle::TEXT_STYLE_ITALIC) {
    android_text_style |= kTextStyleItalic;
  }

  std::unique_ptr<android::Typeface> typeface;
  if (!family_name.empty()) {
    std::string family_name_with_weight;
    switch (font_weight) {
      case FontWeight::FONT_WEIGHT_LIGHT:
        family_name_with_weight = absl::StrCat(family_name, "-light");
        break;
      case FontWeight::FONT_WEIGHT_NORMAL:
        // Bold is a font style rather than a weight on Android so set it below.
        ABSL_FALLTHROUGH_INTENDED;
      case FontWeight::FONT_WEIGHT_BOLD:
        family_name_with_weight = family_name;
        break;
      case FontWeight::FONT_WEIGHT_MEDIUM:
        family_name_with_weight = absl::StrCat(family_name, "-medium");
        break;
    }

    typeface = std::make_unique<android::Typeface>(
        context.GetJniEnv(), family_name_with_weight, android_text_style);
  } else {
    // No family name specified -- ask the OS to look up a variant
    // of Typeface.DEFAULT that matches the requested style.
    typeface = std::make_unique<android::Typeface>(context.GetJniEnv(),
                                                   android_text_style);
  }

  if (!typeface->WeakReference()) {
    return {};
  }

  return std::make_unique<AndroidTypefaceFontHolder>(std::move(typeface),
                                                     font_weight, text_style);
}

}  // namespace imp
