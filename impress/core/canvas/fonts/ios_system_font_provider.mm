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

#import "core/canvas/fonts/ios_system_font_provider.h"

#import <UIKit/UIKit.h>

#include "third_party/absl/memory/memory.h"
#include "core/canvas/fonts/font_params.proto.imp.h"
#include "core/canvas/fonts/ios_font_holder.h"

namespace imp {

std::unique_ptr<FontHolder> LoadSystemIosFont(std::optional<absl::string_view> font_name,
                                              FontWeight font_weight, TextStyle text_style) {
  NSString* font_family;
  if (font_name.has_value() && !font_name->empty()) {
    font_family = [NSString stringWithUTF8String:std::string(*font_name).c_str()];
  }

  UIFontWeight weight;
  switch (font_weight) {
    case FontWeight::FONT_WEIGHT_LIGHT:
      weight = UIFontWeightLight;
      break;
    case FontWeight::FONT_WEIGHT_NORMAL:
      weight = UIFontWeightRegular;
      break;
    case FontWeight::FONT_WEIGHT_MEDIUM:
      weight = UIFontWeightMedium;
      break;
    case FontWeight::FONT_WEIGHT_BOLD:
      weight = UIFontWeightBold;
      break;
  }

  UIFont* ui_font;
  if (font_family) {
    NSDictionary* font_attributes = @{
      UIFontDescriptorTraitsAttribute : @{UIFontWeightTrait : @(weight)},
      UIFontDescriptorFamilyAttribute : font_family
    };

    UIFontDescriptor* font_family_descriptor =
        [UIFontDescriptor fontDescriptorWithFontAttributes:font_attributes];
    ui_font = [UIFont fontWithDescriptor:font_family_descriptor size:1.0];
  } else {
    ui_font = [UIFont systemFontOfSize:1.0 weight:weight];
  }

  if (text_style == TextStyle::TEXT_STYLE_ITALIC) {
    UIFontDescriptorSymbolicTraits traits = UIFontDescriptorTraitItalic;
    if (font_weight == FontWeight::FONT_WEIGHT_BOLD) {
      traits |= UIFontDescriptorTraitBold;
    }
    UIFontDescriptor* style_descriptor =
        [ui_font.fontDescriptor fontDescriptorWithSymbolicTraits:traits];
    ui_font = [UIFont fontWithDescriptor:style_descriptor size:1.0];
  }

  if (!ui_font) {
    return {};
  }

  return absl::make_unique<IosFontHolder>(ui_font);
}

}  // namespace imp
