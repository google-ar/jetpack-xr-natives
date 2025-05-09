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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_FONTS_DESKTOP_FONT_HOLDER_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_FONTS_DESKTOP_FONT_HOLDER_H_

#include <optional>
#include <string>

#include "absl/strings/string_view.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/fonts/font_params.proto.imp.h"
#include "third_party/skia/HEAD/include/core/SkFontMgr.h"
#include "third_party/skia/HEAD/include/core/SkRefCnt.h"
#include "third_party/skia/HEAD/modules/skparagraph/include/FontCollection.h"

namespace imp {

using skia::textlayout::FontCollection;

// Helper implementation of FontHolder for Desktop.
class DesktopFontHolder : public FontHolder {
 public:
  explicit DesktopFontHolder();
  explicit DesktopFontHolder(
      std::optional<absl::string_view> family_name,
      FontWeight font_weight = FontWeight::FONT_WEIGHT_NORMAL,
      TextStyle text_style = TextStyle::TEXT_STYLE_NORMAL);

  void* GetPlatformFont() override;

  absl::string_view GetFontName() const override;
  FontWeight GetFontWeight() const override;
  TextStyle GetTextStyle() const override;

 private:
  sk_sp<SkFontMgr> GetFontMgr();
  sk_sp<FontCollection> font_collection_;
  std::string font_name_;
  FontWeight font_weight_;
  TextStyle text_style_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_FONTS_DESKTOP_FONT_HOLDER_H_
