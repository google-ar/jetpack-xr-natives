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

package com.google.ar.imp.core.glyph

import android.graphics.Canvas
import android.graphics.Paint
import android.os.Build
import com.google.android.filament.proguard.UsedByNative
import com.google.common.flogger.GoogleLogger

private val logger = GoogleLogger.forEnclosingClass()

/**
 * Implements a rough subset of functions of CanvasSource and ScopedCanvas specific to rendering
 * text by glyphs. See android_glyph_source.h.
 */
@UsedByNative("android_glyph_source.cc")
class GlyphSource @UsedByNative("android_glyph_source.cc") constructor(method: Method) {
  @UsedByNative("android_glyph_source.cc")
  enum class Method {
    AUTO,
    PATH,
    SHAPER,
  }

  private val inner =
    when (method) {
      Method.AUTO ->
        if (Build.VERSION.SDK_INT >= 31) {
          ShaperGlyphSource()
        } else {
          PathGlyphSource()
        }
      Method.PATH -> PathGlyphSource()
      Method.SHAPER -> ShaperGlyphSource()
    }

  /**
   * Roughly analogous to GetTextOrigin and GetTextSize.
   *
   * @return a float array of size 8. See android_glyph_source.cc as the reference implementation.
   */
  @UsedByNative("android_glyph_source.cc")
  fun getGlyphMetrics(glyphId: Int, font: Any?, paint: Paint): FloatArray = withExceptionsLogged {
    inner.getGlyphMetrics(glyphId, font, paint)
  }

  /**
   * Analogous to GetTextGlyphs.
   *
   * Out parameters must be arrays of length of text or greater, though only the first N elements
   * will be set, where N is the number of glyphs.
   *
   * A glyph ID is a function of `paint`. Different configurations of `paint` can and will yield
   * different glyph IDs.
   *
   * @return the number of glyphs
   */
  @UsedByNative("android_glyph_source.cc")
  fun getTextGlyphs(text: String, paint: Paint) = withExceptionsLogged {
    inner.getTextGlyphs(text, paint)
  }

  /**
   * Analogous to GetCombinedCharacterGroups.
   *
   * Out parameter must be array of length of text or greater, though only the first N elements will
   * be set, where N is the number of glyphs.
   *
   * @return the number of glyphs
   */
  @UsedByNative("android_glyph_source.cc")
  fun getCombinedCharacterGroups(text: String, paint: Paint) = withExceptionsLogged {
    inner.getCombinedCharacterGroups(text, paint)
  }

  /**
   * Analogous to DrawGlyph.
   *
   * The `glyphId`, `fontSize`, and `font` parameters must correspond exactly with the results of
   * [getTextGlyphs].
   */
  @UsedByNative("android_glyph_source.cc")
  fun drawGlyph(
    canvas: Canvas,
    glyphId: Int,
    x: Float,
    y: Float,
    font: Any?,
    strokeWidth: Float,
    fillPaint: Paint,
    strokePaint: Paint,
  ) = withExceptionsLogged {
    inner.drawGlyph(canvas, glyphId, x, y, font, strokeWidth, fillPaint, strokePaint)
  }
}

internal interface IGlyphSource {
  fun getGlyphMetrics(glyphId: Int, font: Any?, paint: Paint): FloatArray

  fun getTextGlyphs(text: String, paint: Paint): Array<GlyphAdvance>

  fun getCombinedCharacterGroups(text: String, paint: Paint): IntArray

  fun drawGlyph(
    canvas: Canvas,
    glyphId: Int,
    x: Float,
    y: Float,
    font: Any?,
    strokeWidth: Float,
    fillPaint: Paint,
    strokePaint: Paint,
  )
}

private inline fun <T> withExceptionsLogged(body: () -> T): T {
  try {
    return body()
  } catch (e: Throwable) {
    logger.atSevere().withCause(e).log("Exception in GlyphSource")
    throw e
  }
}
