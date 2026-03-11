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
import android.graphics.Path
import android.graphics.RectF
import android.graphics.fonts.Font
import android.graphics.text.TextRunShaper
import androidx.annotation.RequiresApi
import java.io.File

/**
 * ShaperGlyphSource provides an interface around the [TextRunShaper] set of APIs for per-glyph
 * rendering, which is only available on API levels 31 and later.
 */
@RequiresApi(31)
internal class ShaperGlyphSource : IGlyphSource {
  // Reusable buffers.
  private val glyphIdPtr = IntArray(1)
  private val positionPtr = FloatArray(2)
  private val boundingBoxF = RectF()
  private val fontMetrics = Paint.FontMetrics()
  private val glyphMetrics = FloatArray(8)

  // HACK: No proper way to determine if a glyph is a color emoji using the TextRunShaper API.
  // Instead, try turning some test emoji into paths; if the resulting paths are empty, this
  // indicates they belong to a color emoji fallback font. If the path is not empty, it's either
  // being rendered as an old-school black and white emoji or a tofu indicating a missing character;
  // in either case, we know for sure that the font returned is not a color emoji font.
  //
  // On Pixel devices, and likely many other OEMs, color emoji can be split across several files.
  // Specifically, Pixel uses a separate font for country flags.
  private val emojiFontFiles: Set<File> =
    mutableSetOf<File>().apply {
      val paint = Paint()
      var emojiPath = Path()
      // These two emoji were introduced specifically as emoji in Unicode Version 6.0.
      // Swiss flag chosen arbitrarily for neutrality.
      for (emoji in listOf("🐢", "🇨🇭")) {
        paint.getTextPath(
          emoji,
          /*start=*/ 0,
          /*count=*/ emoji.length,
          /*x=*/ 0f,
          /*y=*/ 0f,
          emojiPath,
        )
        if (emojiPath.isEmpty) {
          val glyphs =
            TextRunShaper.shapeTextRun(
              emoji,
              /*start=*/ 0,
              /*count=*/ emoji.length,
              /*contextStart=*/ 0,
              /*contextCount=*/ emoji.length,
              /*xOffset=*/ 0f,
              /*yOffset=*/ 0f,
              /*isRtl=*/ false,
              paint,
            )
          for (i in 0 until glyphs.glyphCount()) {
            glyphs.getFont(i).file?.let { add(it) }
          }
        } else {
          emojiPath = Path()
        }
      }
    }

  override fun getGlyphMetrics(glyphId: Int, font: Any?, paint: Paint): FloatArray {
    val actualFont = font as Font

    val advanceWidth = actualFont.getGlyphBounds(glyphId, paint, boundingBoxF)
    actualFont.getMetrics(paint, fontMetrics)
    val fontDescent = maxFontDescent(boundingBoxF, fontMetrics)
    val fontAscent = maxFontAscent(boundingBoxF, fontMetrics)

    // Origin does not include the stroke width.
    glyphMetrics[0] = if (emojiFontFiles.contains(actualFont.file)) 0f else 1f
    glyphMetrics[1] = boundingBoxF.left
    glyphMetrics[2] = -boundingBoxF.bottom
    glyphMetrics[3] = boundingBoxF.width()
    glyphMetrics[4] = boundingBoxF.height()
    glyphMetrics[5] = advanceWidth
    glyphMetrics[6] = -fontDescent
    glyphMetrics[7] = fontAscent + fontDescent
    return glyphMetrics
  }

  override fun getTextGlyphs(text: String, paint: Paint): Array<GlyphAdvance> {
    require(!text.any { it == '\n' }) { "Text must not contain newlines" }

    val bidi = BidiRuns.create(text)
    val result: MutableList<GlyphAdvance> = mutableListOf()

    var x = 0f

    for (run in bidi) {
      val isRtl = bidi.isRtl(run)
      val runStart = bidi.getRunStart(run)
      val runLimit = bidi.getRunLimit(run)

      val glyphs =
        TextRunShaper.shapeTextRun(
          text,
          runStart,
          runLimit - runStart,
          /*contextStart=*/ 0,
          /*contextCount=*/ text.length,
          x,
          /*yOffset=*/ 0f,
          isRtl,
          paint,
        )

      for (i in 0 until glyphs.glyphCount()) {
        val nextX =
          if (i + 1 >= glyphs.glyphCount()) {
            x + glyphs.advance
          } else {
            glyphs.getGlyphX(i + 1)
          }
        val font = glyphs.getFont(i)

        result.add(
          GlyphAdvance(
            id = glyphs.getGlyphId(i),
            width = nextX - glyphs.getGlyphX(i),
            font = font,
            isEmoji = emojiFontFiles.contains(font.file),
          )
        )
      }
      x += glyphs.advance
    }

    return result.toTypedArray()
  }

  override fun releaseTextGlyph(glyphId: Int) {
    // Shaper doesn't cache anything.
  }

  override fun getCombinedCharacterGroups(text: String, paint: Paint): IntArray {
    val bidi = BidiRuns.create(text)
    var outIndex = 0
    var x = 0f
    var glyphIdx = 0
    val result = mutableListOf<Int>()

    for (run in bidi) {
      val isRtl = bidi.isRtl(run)
      val runStart = bidi.getRunStart(run)
      val runLimit = bidi.getRunLimit(run)

      val glyphs =
        TextRunShaper.shapeTextRun(
          text,
          runStart,
          runLimit - runStart,
          /*contextStart=*/ 0,
          /*contextCount=*/ text.length,
          x,
          /*yOffset=*/ 0f,
          isRtl,
          paint,
        )

      var prevX = x
      for (i in 0 until glyphs.glyphCount()) {
        val nextX =
          if (i + 1 >= glyphs.glyphCount()) {
            x + glyphs.advance
          } else {
            glyphs.getGlyphX(i + 1)
          }

        if (i > 0 && prevX < nextX) {
          glyphIdx++
        }

        result.add(glyphIdx)
        prevX = nextX
        outIndex++
      }
      x += glyphs.advance
    }

    return result.toIntArray()
  }

  override fun drawGlyph(
    canvas: Canvas,
    glyphId: Int,
    x: Float,
    y: Float,
    font: Any?,
    strokeWidth: Float,
    fillPaint: Paint,
    strokePaint: Paint,
  ) {
    val actualFont = font as Font

    actualFont.getGlyphBounds(glyphId, fillPaint, boundingBoxF)
    actualFont.getMetrics(fillPaint, fontMetrics)
    val fontAscent = maxFontAscent(boundingBoxF, fontMetrics)

    glyphIdPtr[0] = glyphId
    positionPtr[0] = x + (strokeWidth / 2) - boundingBoxF.left
    positionPtr[1] = y + (strokeWidth / 2) + fontAscent

    if (strokeWidth > 0f) {
      canvas.drawGlyphs(
        glyphIdPtr,
        /*glyphIdOffset=*/ 0,
        positionPtr,
        /*positionOffset=*/ 0,
        /*glyphCount=*/ 1,
        actualFont,
        strokePaint,
      )
    }
    canvas.drawGlyphs(
      glyphIdPtr,
      /*glyphIdOffset=*/ 0,
      positionPtr,
      /*positionOffset=*/ 0,
      /*glyphCount=*/ 1,
      actualFont,
      fillPaint,
    )
  }
}
