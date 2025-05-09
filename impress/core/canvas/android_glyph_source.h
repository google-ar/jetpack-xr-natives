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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_ANDROID_GLYPH_SOURCE_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_ANDROID_GLYPH_SOURCE_H_

#include <jni.h>

#include "core/canvas/scoped_canvas.h"
#include "core/common/context.h"
#include "core/common/jni_helpers.h"
#include "core/math/vec.h"
#include "core/view/platforms/android/wrappers/canvas.h"

namespace imp {

/**
 * Wrapper around com.google.ar.imp.core.glyph.GlyphSource for Android.
 *
 * Android glyph rendering requires a non-trivial amount of Android API
 * interaction. As such, it's mostly implemented at the JVM layer. This class
 * encapsulates all glyph-specific methods of CanvasSource and ScopedCanvas for
 * Android.
 */
class AndroidGlyphSource : public JavaWrapper {
 public:
  /** What method to use when rendering glyphs. */
  enum class Method {
    /**
     * Pick based on API level: prefer Shaper-based on API level 31+,
     * Path-based otherwise.
     */
    kAuto,
    /**
     * Split text into glyphs using a workaround based on Android Path objects.
     */
    kPath,
    /**
     * Use the TextRunShaper API to directly reference individual glyphs in
     * fonts. Only works on API level 31+.
     */
    kShaper,
  };

  explicit AndroidGlyphSource(const Context& context, Method method);

  /**
   * HACK: Don't crash clients which forget to include the
   * corresponding Java dependency.
   */
  bool IsAvailable() const;

  /**
   * See CanvasSource::GetGlyphMetrics().
   */
  ScopedCanvas::TextMetrics GetGlyphMetrics(int glyph_id, FontHolder* font,
                                            int size, float stroke_width,
                                            float text_tracking);

  /**
   * See CanvasSource::GetTextGlyphs().
   */
  std::vector<ScopedCanvas::GlyphAdvance> GetTextGlyphs(absl::string_view text,
                                                        int font_size,
                                                        float text_tracking);

  /**
   * See CanvasSource::DrawGlyph().
   */
  void DrawGlyph(android::Canvas& canvas, int glyph_id, float x, float y,
                 FontHolder* font, int font_size, float stroke_width,
                 float4 fill_color, float4 stroke_color, float text_tracking);

 private:
  JniHandle get_glyph_metrics_;
  JniHandle get_text_glyphs_;
  JniHandle draw_glyph_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_ANDROID_GLYPH_SOURCE_H_
