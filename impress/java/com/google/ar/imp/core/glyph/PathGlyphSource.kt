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
import android.graphics.Rect
import android.graphics.RectF
import androidx.graphics.path.PathIterator
import androidx.graphics.path.PathSegment

private const val NULL_GLYPH_MESSAGE = "Attempted to reference a non-existent glyph"
private const val UNKNOWN_GLYPH_MESSAGE = "Unexpected data type in glyph store"
private const val UNEXPECTED_VERB_MESSAGE = "Font path contains unexpected verb"

/** This value is the same prime used in AutoValue's hashing algorithm. */
private const val HASH_PRIME = 1000003

private class PathWithMetrics(
  val path: Path,
  val x: Float,
  val y: Float,
  val width: Float,
  val height: Float,
  val fontOriginY: Float,
  val fontSizeY: Float,
  val typographicalWidth: Float,
)

/** Used for whitespace characters. */
private class Blank(val typographicWidth: Float)

/**
 * PathGlyphSource provides a workaround for Android API levels 30 and below, in which there are
 * extremely limited options for directly interacting with fonts on a per-glyph basis. It operates
 * by splitting the result of [Paint.getTextPath] along the glyph boundaries reported by
 * [Paint.getTextWidths].
 */
internal class PathGlyphSource : IGlyphSource {
  // Map of glyph IDs to either Strings, Paths, or the Blank singleton.
  private val map = mutableMapOf<Int, Any>()

  // Reusable buffers.
  private val boundingBox = Rect()
  private val boundingBoxF = RectF()
  private val fontMetrics = Paint.FontMetrics()

  override fun getGlyphMetrics(
    glyphId: Int,
    font: Any?,
    strokeWidth: Float,
    paint: Paint,
    out: FloatArray,
  ) {
    require(out.size == 7)
    val glyph = map[glyphId]
    when (glyph) {
      is String -> {
        // Ignore stroke; this is a colored emoji.
        paint.getTextBounds(glyph, /* index= */ 0, glyph.length, boundingBox)
        val typographicalWidths = FloatArray(glyph.length)
        paint.getTextWidths(glyph, typographicalWidths)
        out[0] = boundingBox.left.toFloat()
        out[1] = -boundingBox.bottom.toFloat()
        out[2] = boundingBox.width().toFloat()
        out[3] = boundingBox.height().toFloat()
        out[4] = typographicalWidths.sum()

        // In some scripts, in particular, emoji, some characters exceed the boundaries of the font
        // metrics. Expand the font metrics to include the actual bounding box in those cases.
        paint.getFontMetrics(fontMetrics)
        val fontDescent = maxFontDescent(boundingBox, fontMetrics)
        val fontAscent = maxFontAscent(boundingBox, fontMetrics)
        out[5] = -fontDescent
        out[6] = fontAscent + fontDescent
      }
      is PathWithMetrics -> {
        val padding = strokeWidth
        // Origin does not include the stroke width.
        out[0] = glyph.x
        out[1] = glyph.y
        // Stroke straddles the font, half in and half out.
        out[2] = glyph.width + padding
        out[3] = glyph.height + padding
        // Typographical width
        out[4] = glyph.typographicalWidth
        // Font-relative metrics.
        out[5] = glyph.fontOriginY
        out[6] = glyph.fontSizeY + padding
      }
      is Blank -> {
        out[0] = 0f
        out[1] = 0f
        out[2] = 1f
        out[3] = 1f
        out[4] = glyph.typographicWidth
        out[5] = 0f
        out[6] = 1f
      }
      null -> throw IllegalStateException(NULL_GLYPH_MESSAGE)
      else -> throw IllegalStateException(UNKNOWN_GLYPH_MESSAGE)
    }
  }

  override fun getTextGlyphs(text: String, paint: Paint): Array<GlyphAdvance> {
    require(!text.any { it == '\n' }) { "Text must not contain newlines" }

    val glyphBuilders = createGlyphBuilders(text, paint)
    if (glyphBuilders.isEmpty()) {
      return arrayOf()
    }
    val firstGlyphBuilder = glyphBuilders.first()
    val lastGlyphBuilder = glyphBuilders.last()

    paint.getFontMetrics(fontMetrics)

    val fullPath = Path()
    paint.getTextPath(text, 0, text.length, /* x= */ 0f, /* y= */ 0f, fullPath)

    val closedPath = mutableListOf<PathSegment>()
    var totalX = 0f

    for (segment in PathIterator(fullPath, PathIterator.ConicEvaluation.AsConic)) {
      when (segment.type) {
        PathSegment.Type.Done -> check(closedPath.isEmpty())
        PathSegment.Type.Close -> {
          check(closedPath.isNotEmpty())
          val averageX = totalX / closedPath.size
          val builder =
            when {
              averageX <= firstGlyphBuilder.left -> firstGlyphBuilder
              averageX >= lastGlyphBuilder.right -> lastGlyphBuilder
              else -> glyphBuilders.first { averageX >= it.left && averageX <= it.right }
            }
          builder.addClosedPath(closedPath)
          // Reset closed path information.
          totalX = 0f
          closedPath.clear()
        }
        PathSegment.Type.Move,
        PathSegment.Type.Line,
        PathSegment.Type.Quadratic,
        PathSegment.Type.Cubic -> {
          totalX += segment.points.last().x
          closedPath.add(segment)
        }
        PathSegment.Type.Conic -> throw IllegalStateException(UNEXPECTED_VERB_MESSAGE)
      }
    }

    // Add new glyphs to our store and return their ids/positions.
    val result: MutableList<GlyphAdvance> = mutableListOf()
    for (builder in glyphBuilders) {
      val hash: Int
      val isEmoji: Boolean
      if (builder.isEmptyPath) {
        // Either a colored emoji or a blank character such as a space.
        val glyph = builder.asString(text)
        hash = glyph.hashCode()
        if (glyph.isBlank()) {
          isEmoji = false
          map.getOrPut(hash) { Blank(builder.width) }
        } else {
          isEmoji = true
          map.getOrPut(hash) { glyph }
        }
      } else {
        hash = builder.hash
        isEmoji = false
        map.getOrPut(hash) {
          val path = builder.asPath()
          // computeBounds(RectF) locked behind a feature flag?
          @Suppress("Deprecation") path.computeBounds(boundingBoxF, /* exact= */ true)

          // In some scripts, some characters exceed the boundaries of the font metrics. Expand the
          // font metrics to include the actual bounding box in those cases.
          val fontDescent = maxFontDescent(boundingBoxF, fontMetrics)
          val fontAscent = maxFontAscent(boundingBoxF, fontMetrics)

          // Offset path such that drawGlyph doesn't need to do it later.
          path.offset(-boundingBoxF.left, fontAscent)

          PathWithMetrics(
            path,
            x = boundingBoxF.left,
            y = -boundingBoxF.bottom,
            width = boundingBoxF.width(),
            height = boundingBoxF.height(),
            typographicalWidth = builder.width,
            fontOriginY = -fontDescent,
            fontSizeY = fontAscent + fontDescent,
          )
        }
      }

      result.add(GlyphAdvance(id = hash, width = builder.width, font = null, isEmoji = isEmoji))
    }
    return result.toTypedArray()
  }

  override fun getCombinedCharacterGroups(text: String, paint: Paint): IntArray {
    // The path calculations already handle combined character glyphs by calculating their
    // centroids, thus this method does not need to be implemented.
    return IntArray(0)
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
    val glyph = map[glyphId]
    when (glyph) {
      is String -> {
        // Align at top-left.
        fillPaint.getTextBounds(glyph, /* index= */ 0, glyph.length, boundingBox)
        canvas.drawText(
          glyph,
          x + (strokeWidth / 2) - boundingBox.left,
          y + (strokeWidth / 2) - boundingBox.top,
          fillPaint,
        )
      }
      is PathWithMetrics -> {
        val xOffset = x + (strokeWidth / 2)
        val yOffset = y + (strokeWidth / 2)
        glyph.path.offset(xOffset, yOffset)
        try {
          if (strokeWidth > 0f) {
            canvas.drawPath(glyph.path, strokePaint)
          }
          canvas.drawPath(glyph.path, fillPaint)
        } finally {
          glyph.path.offset(-xOffset, -yOffset)
        }
      }
      is Blank -> {}
      null -> throw IllegalStateException(NULL_GLYPH_MESSAGE)
      else -> throw IllegalStateException(UNKNOWN_GLYPH_MESSAGE)
    }
  }
}

/** Create glyph builders in order from LEFT to RIGHT. Handles BiDi text. */
private fun createGlyphBuilders(text: String, paint: Paint): List<GlyphBuilder> {
  val seed = paint.seed
  val widths = FloatArray(text.length)
  val textWidthCount = paint.getTextWidths(text, widths)
  check(textWidthCount == text.length)

  val bidi = BidiRuns.create(text)

  val result = mutableListOf<GlyphBuilder>()
  var position = 0f
  for (run in bidi) {
    val runStart = bidi.getRunStart(run)
    val runLimit = bidi.getRunLimit(run)
    if (bidi.isRtl(run)) {
      // Add glyphs in reverse order.
      var i = runLimit
      while (i > runStart) {
        val glyphEnd = i--
        while (i > runStart && widths[i] == 0f) {
          i--
        }
        val width = widths[i]
        if (width != 0f) {
          result.add(GlyphBuilder(seed, position, width, i, glyphEnd - i))
          position += width
        }
      }
    } else {
      // Add glyphs in forward order.
      var i = runStart
      // Skip past any invisible characters.
      while (i < runLimit && widths[i] == 0f) {
        i++
      }
      while (i < runLimit) {
        val width = widths[i]
        val glyphStart = i++
        // If the next width is 0, that means the next character belongs to this glyph.
        while (i < runLimit && widths[i] == 0f) {
          i++
        }
        result.add(GlyphBuilder(seed, position, width, glyphStart, i - glyphStart))
        position += width
      }
    }
  }
  return result
}

private class GlyphBuilder(
  /** Seed used to initialize the hash. See Paint.seed */
  seed: Int,
  /** X position of the left side of the glyph in pixels. */
  val left: Float,
  /** Width of the glyph in pixels. */
  val width: Float,
  /** Position of the starting index in the string. */
  val start: Int,
  /** Number of chars in the string. */
  val count: Int,
) {
  var hash = seed
    private set

  private val segmentVerbs = mutableListOf<PathSegment.Type>()
  private val segmentValues = mutableListOf<Float>()

  val isEmptyPath: Boolean
    get() = segmentVerbs.isEmpty()

  /** X position of the right side of the glyph in pixels. */
  val right: Float
    get() = left + width

  fun asString(text: String): String = text.substring(start, start + count)

  fun asPath(): Path =
    Path().apply {
      val iter = segmentValues.iterator()
      for (type in segmentVerbs) {
        when (type) {
          PathSegment.Type.Move -> moveTo(iter.next(), iter.next())
          PathSegment.Type.Line -> lineTo(iter.next(), iter.next())
          PathSegment.Type.Quadratic -> quadTo(iter.next(), iter.next(), iter.next(), iter.next())
          PathSegment.Type.Cubic ->
            cubicTo(iter.next(), iter.next(), iter.next(), iter.next(), iter.next(), iter.next())
          PathSegment.Type.Close -> close()
          PathSegment.Type.Conic,
          PathSegment.Type.Done -> throw IllegalStateException(UNEXPECTED_VERB_MESSAGE)
        }
      }
    }

  fun addClosedPath(segments: Iterable<PathSegment>) {
    for (segment in segments) {
      addVerb(segment.type)
      when (segment.type) {
        PathSegment.Type.Move -> {
          addX(segment.points[0].x)
          addY(segment.points[0].y)
        }
        PathSegment.Type.Line -> {
          addX(segment.points[1].x)
          addY(segment.points[1].y)
        }
        PathSegment.Type.Quadratic -> {
          addX(segment.points[1].x)
          addY(segment.points[1].y)
          addX(segment.points[2].x)
          addY(segment.points[2].y)
        }
        PathSegment.Type.Cubic -> {
          addX(segment.points[1].x)
          addY(segment.points[1].y)
          addX(segment.points[2].x)
          addY(segment.points[2].y)
          addX(segment.points[3].x)
          addY(segment.points[3].y)
        }
        PathSegment.Type.Close,
        PathSegment.Type.Conic,
        PathSegment.Type.Done -> throw IllegalStateException(UNEXPECTED_VERB_MESSAGE)
      }
    }
    addVerb(PathSegment.Type.Close)
  }

  // Each of the following add* functions both record the necessary values to reconstruct the Path
  // object and compute its hash. We use the same hashing algorithm as AutoValue.

  private fun addVerb(verb: PathSegment.Type) {
    segmentVerbs.add(verb)
    hash *= HASH_PRIME
    hash = hash xor verb.hashCode()
  }

  private fun addX(x: Float) {
    addValue(x - left)
  }

  private fun addY(y: Float) {
    addValue(y)
  }

  private fun addValue(value: Float) {
    segmentValues.add(value)
    hash *= HASH_PRIME
    hash = hash xor value.getMantissaBits()
  }
}

/** Return a hash seed for this paint. */
private val Paint.seed: Int
  get() =
    if (typeface !== null) {
      typeface.hashCode() * HASH_PRIME
    } else {
      0
    } xor textSize.toBits()

/** Return the N most significant mantissa bits of the given float. */
private fun Float.getMantissaBits(n: Int = 4): Int {
  require(n >= 1 && n <= 23)
  return (toBits() and 0x007fffff) shr (23 - n)
}
