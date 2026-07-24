/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_FONTS_ICON_FONT_H_
#define THIRD_PARTY_IMPRESS_CORE_FONTS_ICON_FONT_H_

#include "dear_imgui/imgui.h"

// Glyph mapping for Material Symbols
// (broken link)
// MaterialSymbolsOutined-Regular.ttf

// Extract only necessary icon font glyphs from
// github.com/juliettef/IconFontCppHeaders/blob/main/IconsMaterialSymbols.h
// NOTE: using #defines to allow direct copying from IconsMaterialSymbols.h
// when adding additional icons.
// TODO: Add center toolbar icons.

// UTF-8 unicode mapping for printing icons as characters.
#define ICON_MS_SORT "\xee\x85\xa4"             // U+e164
#define ICON_MS_ARROW_DROP_DOWN "\xee\x97\x85"  // U+e5c5
#define ICON_MS_CLOSE "\xee\x97\x8d"            // U+e5cd
#define ICON_MS_ARROW_RIGHT "\xee\x97\x9f"      // U+e5df
#define ICON_MS_DELETE "\xee\xa4\xae"           // U+e92e
#define ICON_MS_KEEP "\xef\x80\xa6"             // U+f026

// clang-format off
// Glyph ranges for loading select icons.
static const ImWchar kIconGlyphRanges[] = {
    0xe164, 0xe164,  // ICON_MS_SORT
    0xe5c5, 0xe5c5,  // ICON_MS_ARROW_DROP_DOWN
    0xe5cd, 0xe5cd,  // ICON_MS_CLOSE
    0xe5df, 0xe5df,  // ICON_MS_ARROW_RIGHT
    0xe92e, 0xe92e,  // ICON_MS_DELETE
    0xf026, 0xf026,  // ICON_MS_KEEP
    0
};
// clang-format on

#endif  // THIRD_PARTY_IMPRESS_CORE_FONTS_ICON_FONT_H_
