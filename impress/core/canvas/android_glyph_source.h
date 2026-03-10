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

#include <cstdint>
#include <vector>

#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/context.h"
#include "core/common/jni_helpers.h"
#include "core/text/text_metrics.proto.h"
#include "core/view/platforms/android/wrappers/canvas.h"
#include "core/view/platforms/android/wrappers/paint.h"

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

  explicit AndroidGlyphSource(const Context& context, Method method,
                              int cache_size_bytes,
                              bool force_individual_glyph_source_instances);

  ~AndroidGlyphSource() override;

  /**
   * HACK: Don't crash clients which forget to include the
   * corresponding Java dependency.
   */
  bool IsAvailable() const;

  /**
   * See CanvasSource::GetGlyphMetrics().
   */
  TextMetrics GetGlyphMetrics(int glyph_id, FontHolder* font,
                              float stroke_width, android::Paint& paint);

  /**
   * See CanvasSource::GetTextGlyphs().
   */
  std::vector<ScopedCanvas::GlyphAdvance> GetTextGlyphs(absl::string_view text,
                                                        android::Paint& paint);

  /**
   * See CanvasSource::ReleaseTextGlyphs().
   */
  void ReleaseTextGlyphs(absl::Span<int> glyph_ids);

  /**
   * See CanvasSource::GetCombinedCharacterGroups().
   */
  std::vector<ScopedCanvas::GlyphGroup> GetCombinedCharacterGroups(
      absl::string_view text, android::Paint& paint);

  /**
   * See CanvasSource::DrawGlyph().
   */
  void DrawGlyph(android::Canvas& canvas, int glyph_id, float x, float y,
                 FontHolder* font, float stroke_width,
                 android::Paint& fillPaint, android::Paint& strokePaint);

 private:
  JniHandle get_glyph_metrics_;
  JniHandle get_text_glyphs_;
  JniHandle release_text_glyphs_;
  JniHandle release_text_glyph_;
  JniHandle get_combined_character_groups_;
  JniHandle draw_glyph_;
  JniHandle dispose_;

  JniUniquePtr<jclass> glyph_advance_class_;
  jmethodID glyph_advance_get_id_;
  jmethodID glyph_advance_get_width_;
  jmethodID glyph_advance_get_font_;
  jmethodID glyph_advance_is_emoji_;

  int GlyphAdvanceGetId(jobject object);
  float GlyphAdvanceGetWidth(jobject object);
  jobject GlyphAdvanceGetFont(jobject object);
  bool GlyphAdvanceIsEmoji(jobject object);
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_ANDROID_GLYPH_SOURCE_H_
