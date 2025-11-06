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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_FONTS_FONT_HOLDER_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_FONTS_FONT_HOLDER_H_

#include "absl/strings/string_view.h"
#include "core/canvas/fonts/font_params.proto.imp.h"
namespace imp {

//  Holds a platform specific representation of a font and provides it to an
//  Impress CanvasSource so that the font can be used when drawing text to a
//  CanvasSource.
//
// The FontHolder is expected to own the memory of the underlying font.
//
// Impress provides the header system_font_provider.h for loading default system
// fonts into a FontHolder.
//
// For custom fonts, there are many different ways for applications to bundle
// and load fonts, which are often application-specific. To do this, users can
// write custom font providers similar to system_font_provider.h.
//
// Impress provides implementations of FontHolder named AndroidFontHolder
// and IosFontHolder to help with authoring custom font providers.
struct FontHolder {
  virtual ~FontHolder() {}

  // Returns the platform specific representation of a font.
  //
  // Android:
  //   The returned pointer should be a jobject representing an android
  //   Typeface.
  //
  // iOS:
  //   The returned pointer should be a UIFont.
  // WASM:
  //   The returned pointer should be a string of the font name.
  //
  // Desktop:
  //   The returned pointer should be a FontCollection.
  virtual void* GetPlatformFont() = 0;

  // Returns the name of the font in this FontHolder.
  virtual absl::string_view GetFontName() const = 0;

  // Returns the font weight of the font in this FontHolder.
  virtual FontWeight GetFontWeight() const = 0;

  // Returns the text style of the font in this FontHolder.
  virtual TextStyle GetTextStyle() const = 0;

  // Returns true if this is an Android Typeface object.
  //
  // On Android, there are Typefaces and there are Fonts. Typefaces are
  // specified with Paint.setTypeface(). We pass this Paint object to
  // TextRunShaper.shapeTextRun(), which returns a list of glyphs that each have
  // their own Font. Both this Font and the original Paint is then passed to
  // Canvas.drawGlyphs(), which *ignores* the Typeface on the Paint and uses
  // only the per-glyph Font.
  //
  // This is why there's two implementations of FontHolder used on Android:
  // AndroidTypefaceFontHolder and AndroidFontFontHolder.
  virtual bool IsAndroidTypeface() const { return false; }
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_FONTS_FONT_HOLDER_H_
