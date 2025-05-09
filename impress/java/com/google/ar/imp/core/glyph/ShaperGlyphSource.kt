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

private const val EMOJI = "🀄" // E0.6 standard emoji.

/**
 * ShaperGlyphSource provides an interface around the [TextRunShaper] set of APIs for per-glyph
 * rendering, which is only available on API levels 31 and later.
 */
@RequiresApi(31)
internal class ShaperGlyphSource : IGlyphSource {
  // Reusable buffers.
  private val paint = Paint().apply { setAntiAlias(true) }
  private val glyphIdPtr = IntArray(1)
  private val positionPtr = FloatArray(2)
  private val boundingBoxF = RectF()

  // HACK: No proper way to determine if a glyph is a color emoji using the TextRunShaper API.
  // Instead, try turning a test emoji into a path; if the path is empty, it indicates that the
  // device is capable of rendering color emoji. Record the font file used to render color emoji by
  // shaping that single emoji and reading its font.
  private val emojiFontFile: File? = run {
    val emojiPath = Path()
    paint.getTextPath(
      EMOJI,
      /*start=*/ 0,
      /*count=*/ EMOJI.length,
      /*x=*/ 0f,
      /*y=*/ 0f,
      emojiPath,
    )
    if (emojiPath.isEmpty) {
      val glyphs =
        TextRunShaper.shapeTextRun(
          EMOJI,
          /*start=*/ 0,
          /*count=*/ EMOJI.length,
          /*contextStart=*/ 0,
          /*contextCount=*/ EMOJI.length,
          /*xOffset=*/ 0f,
          /*yOffset=*/ 0f,
          /*isRtl=*/ false,
          paint,
        )
      glyphs.getFont(0).file
    } else {
      null
    }
  }

  override fun getGlyphMetrics(
    glyphId: Int,
    font: Any?,
    fontSize: Int,
    strokeWidth: Float,
    textTracking: Float,
    out: FloatArray,
  ) {
    require(out.size == 7)
    val actualFont = font as Font

    paint.textSize = fontSize.toFloat()
    paint.letterSpacing = textTracking

    val advanceWidth = actualFont.getGlyphBounds(glyphId, paint, boundingBoxF)

    val padding =
      if (emojiFontFile != null && actualFont.file == emojiFontFile) {
        0f
      } else {
        // Stroke staddles the font, half in and half out.
        strokeWidth
      }

    // Origin does not include the stroke width.
    out[0] = boundingBoxF.left
    out[1] = -boundingBoxF.bottom
    out[2] = boundingBoxF.width() + padding
    out[3] = boundingBoxF.height() + padding
    out[4] = advanceWidth
    // TODO: Return proper metrics here
    out[5] = -boundingBoxF.bottom
    out[6] = boundingBoxF.height() + padding
  }

  override fun getTextGlyphs(
    text: String,
    fontSize: Int,
    textTracking: Float,
    outIds: IntArray,
    outWidths: FloatArray,
    outFonts: Array<Any?>,
    outIsEmoji: BooleanArray,
  ): Int {
    require(!text.any { it == '\n' }) { "Text must not contain newlines" }
    require(outIds.size >= text.length)
    require(outWidths.size >= text.length)
    require(outFonts.size >= text.length)
    require(outIsEmoji.size >= text.length)

    paint.textSize = fontSize.toFloat()
    paint.letterSpacing = textTracking

    val bidi = BidiRuns.create(text)

    var outIndex = 0
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

        outIds[outIndex] = glyphs.getGlyphId(i)
        outWidths[outIndex] = nextX - glyphs.getGlyphX(i)
        outFonts[outIndex] = font
        outIsEmoji[outIndex] = emojiFontFile != null && font.file == emojiFontFile
        outIndex++
      }
      x += glyphs.advance
    }

    return outIndex
  }

  override fun drawGlyph(
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
  ) {
    val actualFont = font as Font

    paint.textSize = fontSize.toFloat()
    paint.letterSpacing = textTracking

    actualFont.getGlyphBounds(glyphId, paint, boundingBoxF)

    glyphIdPtr[0] = glyphId
    positionPtr[0] = x + (strokeWidth / 2) - boundingBoxF.left
    positionPtr[1] = y + (strokeWidth / 2) - boundingBoxF.top

    paint.style = Paint.Style.STROKE
    paint.color = strokeColor
    paint.strokeWidth = strokeWidth
    canvas.drawGlyphs(
      glyphIdPtr,
      /*glyphIdOffset=*/ 0,
      positionPtr,
      /*positionOffset=*/ 0,
      /*glyphCount=*/ 1,
      actualFont,
      paint,
    )

    paint.style = Paint.Style.FILL
    paint.color = fillColor
    canvas.drawGlyphs(
      glyphIdPtr,
      /*glyphIdOffset=*/ 0,
      positionPtr,
      /*positionOffset=*/ 0,
      /*glyphCount=*/ 1,
      actualFont,
      paint,
    )
  }
}
