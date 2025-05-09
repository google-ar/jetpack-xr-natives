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

#import "core/canvas/fonts/ios_font_holder.h"

#include "third_party/absl/strings/match.h"
#include "core/canvas/fonts/font_params.proto.imp.h"

namespace imp {

IosFontHolder::IosFontHolder(UIFont* ui_font)
    : ui_font_(ui_font), font_name_(std::string([ui_font.fontName UTF8String])) {
  // Determine the text style because the passed in font should already have the style information
  // baked into the selected font
  UIFontDescriptor* font_descriptor = ui_font_.fontDescriptor;
  UIFontDescriptorSymbolicTraits symbolic_traits = font_descriptor.symbolicTraits;
  bool italic = (symbolic_traits & UIFontDescriptorTraitItalic) != 0;
  text_style_ = italic ? TextStyle::TEXT_STYLE_ITALIC : TextStyle::TEXT_STYLE_NORMAL;

  // UIFont has no good way to get the font weight so try getting it from the font name.
  bool light = absl::StrContains(font_name_, "Light");
  bool medium = absl::StrContains(font_name_, "Medium");
  bool bold = (symbolic_traits & UIFontDescriptorTraitBold) != 0;
  if (light) {
    font_weight_ = FontWeight::FONT_WEIGHT_LIGHT;
  } else if (medium) {
    font_weight_ = FontWeight::FONT_WEIGHT_MEDIUM;
  } else if (bold) {
    font_weight_ = FontWeight::FONT_WEIGHT_BOLD;
  } else {
    font_weight_ = FontWeight::FONT_WEIGHT_NORMAL;
  }
}

void* IosFontHolder::GetPlatformFont() { return (__bridge void*)ui_font_; }

absl::string_view IosFontHolder::GetFontName() const { return font_name_; };

FontWeight IosFontHolder::GetFontWeight() const { return font_weight_; }

TextStyle IosFontHolder::GetTextStyle() const { return text_style_; }
}  // namespace imp
