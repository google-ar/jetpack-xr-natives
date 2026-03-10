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
import android.util.LruCache
import androidx.graphics.path.PathIterator
import androidx.graphics.path.PathSegment
import com.google.android.libraries.performance.primes.flogger.logargs.NonSensitiveLogParameterFactory
import com.google.common.flogger.GoogleLogger
import kotlin.math.max

private val logger = GoogleLogger.forEnclosingClass()

private abstract class ReferenceCounted {
  var referenceCount: Int = 0
}

private class Glyph(val path: Path, val sizeBytes: Int, val metrics: FloatArray) :
  ReferenceCounted() {}

private class ReferenceCountedString(val string: String) : ReferenceCounted() {}

private class GlyphCache(cacheSizeBytes: Int) {
  private val glyphs = mutableMapOf<Int, ReferenceCounted>()
  // Orphaned glyphs which are kept around in case they are reused soon.
  private val orphanedGlyphs =
    object : LruCache<Int, Glyph>(cacheSizeBytes) {
      protected override fun sizeOf(key: Int, value: Glyph) = value.sizeBytes
    }

  operator fun get(glyphId: Int) = glyphs.get(glyphId)

  fun acquireString(glyphId: Int, string: String) {
    glyphs.getOrPut(glyphId) { ReferenceCountedString(string) }.referenceCount++
  }

  inline fun acquireGlyph(glyphId: Int, defaultValue: () -> Glyph) {
    glyphs.getOrPut(glyphId) { orphanedGlyphs.remove(glyphId) ?: defaultValue() }.referenceCount++
  }

  fun release(glyphId: Int) {
    val glyph = glyphs[glyphId] ?: throw IllegalStateException(PathGlyphSource.NULL_GLYPH_MESSAGE)

    if (--glyph.referenceCount <= 0) {
      glyphs.remove(glyphId)
      if (glyph is Glyph) {
        orphanedGlyphs.put(glyphId, glyph)
      }
    }
  }
}

/**
 * PathGlyphSource provides a workaround for Android API levels 30 and below, in which there are
 * extremely limited options for directly interacting with fonts on a per-glyph basis. It operates
 * by splitting the result of [Paint.getTextPath] along the glyph boundaries reported by
 * [Paint.getTextWidths].
 */
internal class PathGlyphSource(cacheSizeBytes: Int) : IGlyphSource {
  companion object {
    const val NULL_GLYPH_MESSAGE = "Attempted to reference a non-existent glyph"
    const val UNKNOWN_GLYPH_MESSAGE = "Unexpected data type in glyph store"

    /**
     * Initial capacity of the temporary path buffer, used to buffer a single closed path. 512 is
     * more than enough for almost anything.
     */
    private const val CLOSED_PATH_BUFFER_CAPACITY = 512

    /**
     * Upper limit of the number of characters in a string owned by GlyphCache, i.e. a
     * multi-codepoint emoji.
     *
     * 32 is more than enough to handle even the most complex emoji.
     */
    private const val MAX_STRING_LENGTH = 32

    /**
     * Flag which determines if this glyph ID is stored in the cache.
     *
     * If set, the glyph ID is actually a single UTF-32 codepoint.
     */
    // Internal for testing.
    internal const val GLYPH_IS_UTF32_MASK = 1 shl 31
  }

  private val glyphs = GlyphCache(cacheSizeBytes)

  // Reusable buffers.
  private val tempPath = Path()
  private val boundingBox = Rect()
  private val boundingBoxF = RectF()
  private val fontMetrics = Paint.FontMetrics()
  private val chars = CharArray(2) // one Unicode codepoint can be at most two UTF-16 codepoints.
  private val typographicalWidths = FloatArray(MAX_STRING_LENGTH)
  private val stringGlyphMetrics = FloatArray(8)
  private val closedPath = PathBuffer(CLOSED_PATH_BUFFER_CAPACITY)

  override fun getGlyphMetrics(glyphId: Int, font: Any?, paint: Paint): FloatArray {
    if (glyphId and GLYPH_IS_UTF32_MASK != 0) {
      val len = Character.toChars(glyphId and GLYPH_IS_UTF32_MASK.inv(), chars, 0)
      val numWidths =
        paint.withNoLetterSpacing {
          paint.getFontMetrics(fontMetrics)
          paint.getTextBounds(chars, /* index= */ 0, len, boundingBox)
          paint.getTextWidths(chars, /* index= */ 0, len, typographicalWidths)
        }
      setStringGlyphMetrics(numWidths)
      return stringGlyphMetrics
    }

    val glyph = glyphs[glyphId]
    when (glyph) {
      is ReferenceCountedString -> {
        val numWidths =
          paint.withNoLetterSpacing {
            paint.getFontMetrics(fontMetrics)
            paint.getTextBounds(glyph.string, /* index= */ 0, glyph.string.length, boundingBox)
            paint.getTextWidths(glyph.string, typographicalWidths)
          }
        setStringGlyphMetrics(numWidths)
        return stringGlyphMetrics
      }
      is Glyph -> return glyph.metrics
      null -> throw IllegalArgumentException(NULL_GLYPH_MESSAGE)
      else -> throw IllegalArgumentException(UNKNOWN_GLYPH_MESSAGE)
    }
  }

  private fun setStringGlyphMetrics(numWidths: Int) {
    // Ignore stroke; this is either a colored emoji or a blank character.
    stringGlyphMetrics[1] = boundingBox.left.toFloat()
    stringGlyphMetrics[2] = -boundingBox.bottom.toFloat()
    stringGlyphMetrics[3] = boundingBox.width().toFloat()
    stringGlyphMetrics[4] = boundingBox.height().toFloat()
    stringGlyphMetrics[5] = typographicalWidths.take(numWidths).sum()

    // In some scripts, in particular, emoji, some characters exceed the boundaries of the font
    // metrics. Expand the font metrics to include the actual bounding box in those cases.
    val fontDescent = maxFontDescent(boundingBox, fontMetrics)
    val fontAscent = maxFontAscent(boundingBox, fontMetrics)
    stringGlyphMetrics[6] = -fontDescent
    stringGlyphMetrics[7] = fontAscent + fontDescent
  }

  override fun getTextGlyphs(text: String, paint: Paint): Array<GlyphAdvance> {
    if (text.any { it == '\n' }) {
      // TODO: (broken link) - throw exception once we fix in production.
      logger.atSevere().log("Text must not contain newlines")
      return emptyArray()
    }

    if (text.isEmpty()) {
      return arrayOf()
    }

    paint.withNoLetterSpacing { letterSpacing ->
      val glyphBuilders = createGlyphBuilders(text, paint)
      if (glyphBuilders.isEmpty()) {
        return arrayOf()
      }

      try {
        val firstGlyphBuilder = glyphBuilders.first()
        val lastGlyphBuilder = glyphBuilders.last()

        var totalX = 0f
        paint.getTextPath(text, 0, text.length, /* x= */ 0f, /* y= */ 0f, tempPath)

        for (segment in PathIterator(tempPath, PathIterator.ConicEvaluation.AsConic)) {
          when (segment.type) {
            PathSegment.Type.Done -> check(closedPath.isEmpty())
            PathSegment.Type.Close -> {
              check(!closedPath.isEmpty())
              val averageX = totalX / closedPath.verbCount
              val builder =
                when {
                  averageX <= firstGlyphBuilder.left -> firstGlyphBuilder
                  averageX >= lastGlyphBuilder.right -> lastGlyphBuilder
                  else -> glyphBuilders.first { averageX >= it.left && averageX <= it.right }
                }
              closedPath.add(segment)
              builder.pathBuffer.append(closedPath)
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
            PathSegment.Type.Conic ->
              throw IllegalStateException(PathBuffer.UNEXPECTED_VERB_MESSAGE)
          }
        }
      } catch (e: Throwable) {
        // TODO: (broken link) - throw exception once we fix in production.
        logger
          .atSevere()
          .withCause(e)
          .log(
            "Exception %s shaping text string \"%s\"",
            NonSensitiveLogParameterFactory.fromClassName(e::class.java),
            text,
          )
        return emptyArray()
      } finally {
        tempPath.reset()
        closedPath.clear()
      }

      // Convert GlyphBuilders to GlyphAdvances.
      try {
        val tracking = letterSpacing * paint.textSize
        paint.getFontMetrics(fontMetrics)
        return Array<GlyphAdvance>(glyphBuilders.size) {
          glyphBuilders[it].toGlyphAdvance(glyphs, fontMetrics, boundingBoxF, tracking)
        }
      } catch (e: Throwable) {
        logger
          .atSevere()
          .withCause(e)
          .log(
            "Exception %s converting glyph builders to glyph advances in text string \"%s\"",
            NonSensitiveLogParameterFactory.fromClassName(e::class.java),
            text,
          )
        return emptyArray()
      }
    }
  }

  override fun releaseTextGlyph(glyphId: Int) {
    if (glyphId and GLYPH_IS_UTF32_MASK == 0) {
      glyphs.release(glyphId)
    }
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
    if (glyphId and GLYPH_IS_UTF32_MASK != 0) {
      val len = Character.toChars(glyphId and GLYPH_IS_UTF32_MASK.inv(), chars, 0)
      fillPaint.withNoLetterSpacing {
        // Align at top-left.
        fillPaint.getTextBounds(chars, /* index= */ 0, len, boundingBox)
        canvas.drawText(
          chars,
          /*index=*/ 0,
          /*count=*/ len,
          x + (strokeWidth / 2) - boundingBox.left,
          y + (strokeWidth / 2) - boundingBox.top,
          fillPaint,
        )
      }
      return
    }

    val glyph = glyphs[glyphId]
    when (glyph) {
      is ReferenceCountedString ->
        fillPaint.withNoLetterSpacing {
          // Align at top-left.
          fillPaint.getTextBounds(glyph.string, /* index= */ 0, glyph.string.length, boundingBox)
          canvas.drawText(
            glyph.string,
            x + (strokeWidth / 2) - boundingBox.left,
            y + (strokeWidth / 2) - boundingBox.top,
            fillPaint,
          )
        }
      is Glyph -> {
        canvas.save()
        try {
          canvas.translate(x + (strokeWidth / 2), y + (strokeWidth / 2))
          if (strokeWidth > 0f) {
            canvas.drawPath(glyph.path, strokePaint)
          }
          canvas.drawPath(glyph.path, fillPaint)
        } finally {
          canvas.restore()
        }
      }
      null -> throw IllegalArgumentException(NULL_GLYPH_MESSAGE)
      else -> throw IllegalArgumentException(UNKNOWN_GLYPH_MESSAGE)
    }
  }
}

/** Buffer for Path segments that we're not ready to turn into a real Path yet. */
private class PathBuffer(capacity: Int) {
  companion object {
    const val UNEXPECTED_VERB_MESSAGE = "Font path contains unexpected verb"

    // These values are identical to the ones defined in PathIterator.
    // https://developer.android.com/reference/android/graphics/PathIterator
    private const val VERB_MOVE = 0
    private const val VERB_LINE = 1
    private const val VERB_QUAD = 2
    private const val VERB_CONIC = 3
    private const val VERB_CUBIC = 4
    private const val VERB_CLOSE = 5
    private const val VERB_DONE = 6

    /** Amount to grow path buffers when they run out. */
    private const val PATH_BUFFER_GROW_AMOUNT = 512

    /** This value is the same prime used in AutoValue's hashing algorithm. */
    const val HASH_PRIME = 1000003
  }

  var verbCount = 0
    private set

  var pointCount = 0
    private set

  private var hash = 0
  private var size = 0
  private var data = IntArray(capacity)

  // Two floats per point, plus one byte (verb) per point. Internally Path is an SkPath, which
  // theoretically should let you reserve verbs/points independently, but Android doesn't expose
  // this.
  val sizeBytes: Int
    get() = pointCount * 4 * 2 + pointCount

  /**
   * Note that the hash generated by this function is not associative.
   *
   * An object constructed with only add() calls will have a different hash than an object
   * constructed with only append() calls, even if the paths are identical.
   */
  override fun hashCode() = hash

  fun add(segment: PathSegment) {
    when (segment.type) {
      PathSegment.Type.Move -> {
        pointCount += 1
        addVerb(VERB_MOVE)
        addX(segment.points[0].x)
        addY(segment.points[0].y)
      }
      PathSegment.Type.Line -> {
        pointCount += 1
        addVerb(VERB_LINE)
        addX(segment.points[1].x)
        addY(segment.points[1].y)
      }
      PathSegment.Type.Quadratic -> {
        pointCount += 2
        addVerb(VERB_QUAD)
        addX(segment.points[1].x)
        addY(segment.points[1].y)
        addX(segment.points[2].x)
        addY(segment.points[2].y)
      }
      PathSegment.Type.Cubic -> {
        pointCount += 3
        addVerb(VERB_CUBIC)
        addX(segment.points[1].x)
        addY(segment.points[1].y)
        addX(segment.points[2].x)
        addY(segment.points[2].y)
        addX(segment.points[3].x)
        addY(segment.points[3].y)
      }
      PathSegment.Type.Close -> {
        addVerb(VERB_CLOSE)
      }
      PathSegment.Type.Conic,
      PathSegment.Type.Done -> throw IllegalStateException(UNEXPECTED_VERB_MESSAGE)
    }
    verbCount++
  }

  fun append(other: PathBuffer) {
    val minimumCapacity = size + other.size
    if (minimumCapacity > data.size) {
      // Grow to fit the other buffer in increments of GROW_AMOUNT.
      grow((max(0, (minimumCapacity - 1)) / PATH_BUFFER_GROW_AMOUNT + 1) * PATH_BUFFER_GROW_AMOUNT)
    }
    System.arraycopy(other.data, 0, data, size, other.size)
    verbCount += other.verbCount
    pointCount += other.pointCount
    size += other.size
    hash = (hash * HASH_PRIME) xor other.hashCode()
  }

  fun clear() {
    verbCount = 0
    pointCount = 0
    size = 0
    hash = 0
  }

  fun isEmpty() = size == 0

  fun toPath(): Path {
    val path = Path().apply { incReserve(pointCount) }

    var i = 0
    fun nextVerb() = data[i++]
    fun nextFloat() = Float.fromBits(data[i++])

    while (i < size) {
      when (nextVerb()) {
        VERB_MOVE -> path.moveTo(nextFloat(), nextFloat())
        VERB_LINE -> path.lineTo(nextFloat(), nextFloat())
        VERB_QUAD -> path.quadTo(nextFloat(), nextFloat(), nextFloat(), nextFloat())
        VERB_CUBIC ->
          path.cubicTo(nextFloat(), nextFloat(), nextFloat(), nextFloat(), nextFloat(), nextFloat())
        VERB_CLOSE -> path.close()
        else -> throw IllegalStateException(UNEXPECTED_VERB_MESSAGE)
      }
    }

    return path
  }

  // Each of the following add* functions both record the necessary values to reconstruct the Path
  // object and compute its hash. We use the same hashing algorithm as AutoValue.

  private fun addVerb(verb: Int) {
    addValue(verb)
    hash = (hash * HASH_PRIME) xor verb
  }

  private fun addX(x: Float) {
    addValue(x.toBits())
  }

  private fun addY(y: Float) {
    addValue(y.toBits())
    hash = (hash * HASH_PRIME) xor y.toInt()
  }

  private fun addValue(value: Int) {
    data[size++] = value
    if (size >= data.size) {
      grow(data.size + PATH_BUFFER_GROW_AMOUNT)
    }
  }

  private fun grow(capacity: Int) {
    require(capacity > data.size)
    val newData = IntArray(capacity)
    System.arraycopy(data, 0, newData, 0, data.size)
    data = newData
  }
}

/** Create glyph builders in order from LEFT to RIGHT. Handles BiDi text. */
private fun createGlyphBuilders(text: String, paint: Paint): List<GlyphBuilder> {
  val seed = paint.seed
  val widths = FloatArray(text.length)
  val textWidthCount = paint.getTextWidths(text, widths)
  check(textWidthCount == text.length)

  val bidi = BidiRuns.create(text)

  val result: MutableList<GlyphBuilder> = ArrayList(text.length)
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
          result.add(GlyphBuilder(seed, position, width, text.substring(i, glyphEnd)))
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
        result.add(GlyphBuilder(seed, position, width, text.substring(glyphStart, i)))
        position += width
      }
    }
  }
  return result
}

private class GlyphBuilder(
  /** Seed used to initialize the hash. See Paint.seed */
  val seed: Int,
  /** X position of the left side of the glyph in pixels. */
  val left: Float,
  /** Width of the glyph in pixels. */
  val width: Float,
  /** String which this glyph corresponds to. */
  val text: String,
) {
  companion object {
    /** Value chosen empirically; 1024 is enough for almost all CJK characters. */
    private const val GLYPH_BUILDER_PATH_BUFFER_CAPACITY = 1024
  }

  /** X position of the right side of the glyph in pixels. */
  val right: Float
    get() = left + width

  val pathBuffer = PathBuffer(GLYPH_BUILDER_PATH_BUFFER_CAPACITY)

  fun toGlyphAdvance(
    glyphs: GlyphCache,
    fontMetrics: Paint.FontMetrics,
    boundingBoxF: RectF,
    tracking: Float,
  ): GlyphAdvance {
    val hash: Int
    val isEmoji: Boolean

    if (pathBuffer.isEmpty()) {
      // Either a colored emoji or a blank character such as a space.
      isEmoji = !text.isBlank()
      if (text.codePointCount(0, text.length) == 1) {
        hash = text.codePointAt(0) or PathGlyphSource.GLYPH_IS_UTF32_MASK
      } else {
        hash = text.hashCode() and PathGlyphSource.GLYPH_IS_UTF32_MASK.inv()
        glyphs.acquireString(hash, text)
      }
    } else {
      hash =
        ((((seed * PathBuffer.HASH_PRIME) xor text.hashCode()) * PathBuffer.HASH_PRIME) xor
          pathBuffer.hashCode()) and PathGlyphSource.GLYPH_IS_UTF32_MASK.inv()
      isEmoji = false
      glyphs.acquireGlyph(hash) {
        val path = pathBuffer.toPath()
        path.offset(-left, 0f)

        // computeBounds(RectF) locked behind a feature flag?
        @Suppress("Deprecation") path.computeBounds(boundingBoxF, /* exact= */ true)

        // In some scripts, some characters exceed the boundaries of the font metrics. Expand the
        // font metrics to include the actual bounding box in those cases.
        val fontDescent = maxFontDescent(boundingBoxF, fontMetrics)
        val fontAscent = maxFontAscent(boundingBoxF, fontMetrics)

        // Offset path such that drawGlyph doesn't need to do it later.
        path.offset(-boundingBoxF.left, fontAscent)

        Glyph(
          path,
          // Approximate size in bytes of this object. Include the size of the eight floats. Of
          // course, there's extra memory being taken up by memory allocations and the Glyph
          // object
          // itself, so this is just a very rough approximation.
          pathBuffer.sizeBytes + 8 * 4,
          floatArrayOf(
            1f, // padding
            boundingBoxF.left, // x
            -boundingBoxF.bottom, // y
            boundingBoxF.width(), // width
            boundingBoxF.height(), // height
            width, // typographical width
            -fontDescent, // font origin Y
            fontAscent + fontDescent, // font size Y
          ),
        )
      }
    }

    return GlyphAdvance(id = hash, width = width + tracking, font = null, isEmoji = isEmoji)
  }
}

/** Return a hash seed for this paint. */
private val Paint.seed: Int
  get() =
    if (typeface !== null) {
      typeface.hashCode() * PathBuffer.HASH_PRIME
    } else {
      0
    } xor textSize.toBits()

/** Temporarily set letter spacing to 0. */
private inline fun <T> Paint.withNoLetterSpacing(body: (letterSpacing: Float) -> T): T {
  val letterSpacing = letterSpacing
  try {
    this.letterSpacing = 0f
    return body(letterSpacing)
  } finally {
    this.letterSpacing = letterSpacing
  }
}
