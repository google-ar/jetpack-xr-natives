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

#if defined(__ANDROID__)
#include <android/api-level.h>
#endif

#include <memory>
#include <utility>

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
constexpr int kStyleBold = 1;
constexpr int kStyleItalic = 2;

// https://developer.android.com/reference/android/graphics/Typeface#create(android.graphics.Typeface,%20int,%20boolean)
constexpr int kWeightLight = 300;
constexpr int kWeightNormal = 400;
constexpr int kWeightMedium = 500;
constexpr int kWeightBold = 700;

// While this file is only ever included in Android builds, linting tools
// require this guard.
static inline bool SupportsFontWeight() {
#if defined(__ANDROID__)
  return android_get_device_api_level() >= 28;
#else
  return true;
#endif
}
}  // namespace

std::unique_ptr<FontHolder> LoadSystemAndroidFont(const Context& context,
                                                  absl::string_view family_name,
                                                  FontWeight font_weight,
                                                  TextStyle text_style) {
  uint android_text_style = 0;
  int weight;
  bool italic;
  switch (font_weight) {
    case FontWeight::FONT_WEIGHT_LIGHT:
      weight = kWeightLight;
      break;
    case FontWeight::FONT_WEIGHT_NORMAL:
      weight = kWeightNormal;
      break;
    case FontWeight::FONT_WEIGHT_MEDIUM:
      weight = kWeightMedium;
      break;
    case FontWeight::FONT_WEIGHT_BOLD:
      android_text_style |= kStyleBold;
      weight = kWeightBold;
      break;
  }
  switch (text_style) {
    case TextStyle::TEXT_STYLE_ITALIC:
      android_text_style |= kStyleItalic;
      italic = true;
      break;
    case TextStyle::TEXT_STYLE_NORMAL:
      italic = false;
      break;
  }

  std::unique_ptr<android::Typeface> typeface;

  if (SupportsFontWeight()) {
    // We probably don't need to actually pass android_text_style into the first
    // Typeface.create() call, but I'm paranoid.
    android::Typeface family =
        !family_name.empty()
            ? android::Typeface(context.GetJniEnv(), family_name,
                                android_text_style)
            : android::Typeface(context.GetJniEnv(), android_text_style);
    if (!family.WeakReference()) {
      return nullptr;
    }
    // Can you believe there wasn't an API to specify font weight until
    // Android 9?
    typeface = std::make_unique<android::Typeface>(context.GetJniEnv(), family,
                                                   weight, italic);
  } else {
    typeface = !family_name.empty()
                   ? std::make_unique<android::Typeface>(
                         context.GetJniEnv(), family_name, android_text_style)
                   : std::make_unique<android::Typeface>(context.GetJniEnv(),
                                                         android_text_style);
  }
  if (!typeface->WeakReference()) {
    return {};
  }

  return std::make_unique<AndroidTypefaceFontHolder>(std::move(typeface),
                                                     font_weight, text_style);
}

}  // namespace imp
