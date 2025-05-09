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
import android.os.Build
import com.google.android.filament.proguard.UsedByNative

/**
 * Implements a rough subset of functions of CanvasSource and ScopedCanvas specific to rendering
 * text by glyphs. See android_glyph_source.h.
 */
@UsedByNative("android_glyph_source.cc")
class GlyphSource(method: Method) {
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
   * Analogous to GetTextOrigin and GetTextSize.
   *
   * @param out An array of minimum size 4. The first two items are set to the X and Y origin, while
   *   the final two items are set to the width and height.
   */
  @UsedByNative("android_glyph_source.cc")
  fun getGlyphMetrics(
    glyphId: Int,
    font: Any?,
    fontSize: Int,
    strokeWidth: Float,
    textTracking: Float,
    out: FloatArray,
  ) = inner.getGlyphMetrics(glyphId, font, fontSize, strokeWidth, textTracking, out)

  /**
   * Analogous to GetTextGlyphs.
   *
   * Out parameters must be arrays of length of text or greater, though only the first N elements
   * will be set, where N is the number of glyphs.
   *
   * A glyph ID is a function of `fontSize`. Different values of `fontSize` can and will yield
   * different glyph IDs.
   *
   * @return the number of glyphs
   */
  @UsedByNative("android_glyph_source.cc")
  fun getTextGlyphs(
    text: String,
    fontSize: Int,
    textTracking: Float,
    outIds: IntArray,
    outWidths: FloatArray,
    outFonts: Array<Any?>,
    outIsEmoji: BooleanArray,
  ) = inner.getTextGlyphs(text, fontSize, textTracking, outIds, outWidths, outFonts, outIsEmoji)

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
    fontSize: Int,
    strokeWidth: Float,
    fillColor: Int,
    strokeColor: Int,
    textTracking: Float,
  ) =
    inner.drawGlyph(
      canvas,
      glyphId,
      x,
      y,
      font,
      fontSize,
      strokeWidth,
      fillColor,
      strokeColor,
      textTracking,
    )
}

internal interface IGlyphSource {
  fun getGlyphMetrics(
    glyphId: Int,
    font: Any?,
    fontSize: Int,
    strokeWidth: Float,
    textTracking: Float,
    out: FloatArray,
  )

  fun getTextGlyphs(
    text: String,
    fontSize: Int,
    textTracking: Float,
    outIds: IntArray,
    outWidths: FloatArray,
    outFonts: Array<Any?>,
    outIsEmoji: BooleanArray,
  ): Int

  fun drawGlyph(
    canvas: Canvas,
    glyphId: Int,
    x: Float,
    y: Float,
    font: Any?,
    fontSize: Int,
    strokeWidth: Float,
    fillColor: Int,
    strokeColor: Int,
    textTracking: Float,
  )
}
