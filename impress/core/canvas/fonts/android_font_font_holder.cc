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

#include "core/canvas/fonts/android_font_font_holder.h"

#include <memory>
#include <utility>

#include "core/common/log.h"
#include "core/canvas/fonts/font_params.proto.imp.h"
#include "core/common/file_helpers.h"
#include "core/common/platform_helpers.h"
#include "core/view/platforms/android/wrappers/file.h"
#include "core/view/platforms/android/wrappers/font.h"
#include "core/view/platforms/android/wrappers/font_style.h"

namespace imp {

AndroidFontFontHolder::AndroidFontFontHolder(
    std::unique_ptr<android::Font> font)
    : font_(std::move(font)),
      font_name_(RemoveExtensionFromFilename(font_->GetFile().GetName())) {
  android::FontStyle style = font_->GetStyle();

  switch (style.GetSlant()) {
    case android::FontStyle::Slant::kUpright:
      text_style_ = TextStyle::TEXT_STYLE_NORMAL;
      break;
    case android::FontStyle::Slant::kItalic:
      text_style_ = TextStyle::TEXT_STYLE_ITALIC;
      break;
    default:
      IMP_LOG(imp::FATAL) << "Unknown FontStyle slant.";
  }

  int weight = style.GetWeight();
  if (weight <= static_cast<int>(android::FontStyle::Weight::kLight)) {
    font_weight_ = FontWeight::FONT_WEIGHT_LIGHT;
  } else if (weight <= static_cast<int>(android::FontStyle::Weight::kNormal)) {
    font_weight_ = FontWeight::FONT_WEIGHT_NORMAL;
  } else if (weight <= static_cast<int>(android::FontStyle::Weight::kMedium)) {
    font_weight_ = FontWeight::FONT_WEIGHT_MEDIUM;
  } else {
    font_weight_ = FontWeight::FONT_WEIGHT_BOLD;
  }
}

}  // namespace imp
