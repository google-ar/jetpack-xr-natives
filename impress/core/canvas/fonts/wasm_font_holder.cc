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

#include "core/canvas/fonts/wasm_font_holder.h"

#include <string>

#include "absl/strings/string_view.h"
#include "core/canvas/fonts/font_params.proto.imp.h"

namespace imp {

WasmFontHolder::WasmFontHolder(std::string font_family, FontWeight font_weight,
                               TextStyle text_style)
    : font_family_(font_family),
      font_weight_(font_weight),
      text_style_(text_style) {}

void* WasmFontHolder::GetPlatformFont() {
  return static_cast<void*>(&font_family_);
}

absl::string_view WasmFontHolder::GetFontName() const { return font_family_; }

FontWeight WasmFontHolder::GetFontWeight() const { return font_weight_; }

TextStyle WasmFontHolder::GetTextStyle() const { return text_style_; }

}  // namespace imp
