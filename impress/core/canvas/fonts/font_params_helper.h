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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_FONTS_FONT_PARAMS_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_FONTS_FONT_PARAMS_HELPER_H_

#include <cstddef>

#include "absl/hash/hash.h"
#include "core/canvas/fonts/font_params.proto.imp.h"

namespace imp {

struct SystemFontParamsKeyHash {
  size_t operator()(const SystemFontParams &fp) const {
    return absl::HashOf(fp.font_weight, fp.text_style, fp.android_family_name,
                        fp.ios_family_name, fp.wasm_family_name,
                        fp.desktop_family_name);
  }
};

struct SystemFontParamsKeyEquals {
  bool operator()(const SystemFontParams &fp1,
                  const SystemFontParams &fp2) const {
    return fp1.font_weight == fp2.font_weight &&
           fp1.text_style == fp2.text_style &&
           fp1.android_family_name == fp2.android_family_name &&
           fp1.ios_family_name == fp2.ios_family_name &&
           fp1.wasm_family_name == fp2.wasm_family_name &&
           fp1.desktop_family_name == fp2.desktop_family_name;
  }
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_FONTS_FONT_PARAMS_HELPER_H_
