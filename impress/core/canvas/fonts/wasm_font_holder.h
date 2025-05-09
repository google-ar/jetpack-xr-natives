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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_FONTS_WASM_FONT_HOLDER_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_FONTS_WASM_FONT_HOLDER_H_

#include <string>

#include "absl/strings/string_view.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/fonts/font_params.proto.imp.h"

namespace imp {

// Helper implementation of FontHolder for Wasm. Stores the font family used for
// drawing text on the canvas.
class WasmFontHolder : public FontHolder {
 public:
  explicit WasmFontHolder(
      std::string font_family,
      FontWeight font_weight = FontWeight::FONT_WEIGHT_NORMAL,
      TextStyle text_style = TextStyle::TEXT_STYLE_NORMAL);

  void* GetPlatformFont() override;

  absl::string_view GetFontName() const override;

  FontWeight GetFontWeight() const override;

  TextStyle GetTextStyle() const override;

 private:
  std::string font_family_;
  FontWeight font_weight_;
  TextStyle text_style_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_FONTS_WASM_FONT_HOLDER_H_
