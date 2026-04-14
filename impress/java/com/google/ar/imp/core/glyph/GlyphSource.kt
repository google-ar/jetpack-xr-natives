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
import com.google.android.libraries.performance.primes.flogger.logargs.NonSensitiveLogParameterFactory
import com.google.common.flogger.GoogleLogger
import com.google.errorprone.annotations.CompileTimeConstant
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.withLock

private val logger = GoogleLogger.forEnclosingClass()

/**
 * Implements a rough subset of functions of CanvasSource and ScopedCanvas specific to rendering
 * text by glyphs. See android_glyph_source.h.
 */
@UsedByNative("android_glyph_source.cc")
class GlyphSource
@UsedByNative("android_glyph_source.cc")
constructor(
  method: Method,
  cacheSizeBytes: Int,
  private val forceIndividualGlyphSourceInstances: Boolean = false,
) {
  @UsedByNative("android_glyph_source.cc")
  enum class Method {
    AUTO,
    PATH,
    SHAPER,
  }

  // TODO: Remove the shared glyph source implementation once
  // forceIndividualGlyphSourceInstances is fully launched.
  companion object {
    internal val lock = ReentrantLock()

    private var instance: IGlyphSource? = null
    private var referenceCount = 0

    private fun createGlyphSource(method: Method, cacheSizeBytes: Int): IGlyphSource =
      when (method) {
        Method.AUTO ->
          if (Build.VERSION.SDK_INT >= 31) {
            ShaperGlyphSource()
          } else {
            PathGlyphSource(cacheSizeBytes, useLocalCache = false)
          }
        Method.PATH -> {
          PathGlyphSource(cacheSizeBytes, useLocalCache = false)
        }
        Method.SHAPER -> {
          ShaperGlyphSource()
        }
      }

    internal fun acquireGlyphSource(method: Method, cacheSizeBytes: Int): IGlyphSource {
      referenceCount++
      return instance ?: createGlyphSource(method, cacheSizeBytes).also { instance = it }
    }

    internal fun releaseGlyphSource() {
      if (--referenceCount == 0) {
        instance = null
      }
    }
  }

  private var impl: IGlyphSource? =
    if (forceIndividualGlyphSourceInstances) {
      createGlyphSource(method, cacheSizeBytes)
    } else {
      lock.withLock { acquireGlyphSource(method, cacheSizeBytes) }
    }

  /** Free resources associated with this GlyphSource. */
  @UsedByNative("android_glyph_source.cc")
  fun dispose() {
    if (forceIndividualGlyphSourceInstances) {
      impl = null
    } else {
      lock.withLock {
        releaseGlyphSource()
        impl = null
      }
    }
  }

  /**
   * Roughly analogous to GetTextOrigin and GetTextSize.
   *
   * @return a float array of size 8. See android_glyph_source.cc as the reference implementation.
   */
  @UsedByNative("android_glyph_source.cc")
  fun getGlyphMetrics(glyphId: Int, font: Any?, paint: Paint): FloatArray =
    withImpl("getGlyphMetrics") { it.getGlyphMetrics(glyphId, font, paint) }

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
  fun getTextGlyphs(text: String, paint: Paint) =
    withImpl("getTextGlyphs") { it.getTextGlyphs(text, paint) }

  /**
   * Release glyph IDs.
   *
   * The path glyph method caches some information when new glyphs are gotten by getTextGlyphs().
   * For every time getTextGlyphs() is called, releaseTextGlyphs() should be called with that glyph
   * ID.
   */
  @UsedByNative("android_glyph_source.cc")
  fun releaseTextGlyphs(glyphIds: IntArray) =
    withImpl("releaseTextGlyphs") {
      for (glyphId in glyphIds) {
        it.releaseTextGlyph(glyphId)
      }
    }

  /** Release a single glyph ID. */
  @UsedByNative("android_glyph_source.cc")
  fun releaseTextGlyph(glyphId: Int) = withImpl("releaseTextGlyph") { it.releaseTextGlyph(glyphId) }

  /**
   * Analogous to GetCombinedCharacterGroups.
   *
   * Out parameter must be array of length of text or greater, though only the first N elements will
   * be set, where N is the number of glyphs.
   *
   * @return the number of glyphs
   */
  @UsedByNative("android_glyph_source.cc")
  fun getCombinedCharacterGroups(text: String, paint: Paint) =
    withImpl("getCombinedCharacterGroups") { it.getCombinedCharacterGroups(text, paint) }

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
  ) =
    withImpl("drawGlyph") {
      it.drawGlyph(canvas, glyphId, x, y, font, strokeWidth, fillPaint, strokePaint)
    }

  private inline fun <T> withImpl(@CompileTimeConstant name: String, body: (IGlyphSource) -> T): T {
    lock.withLock {
      val impl = impl
      check(impl !== null) { "Attempted to call method on disposed GlyphSource" }
      try {
        return body(impl)
      } catch (e: Throwable) {
        logger
          .atSevere()
          .withCause(e)
          .log(
            "Exception %s in GlyphSource.%s",
            NonSensitiveLogParameterFactory.fromConstantString(name),
            NonSensitiveLogParameterFactory.fromClassName(e::class.java),
          )
        throw e
      }
    }
  }
}

internal interface IGlyphSource {
  fun getGlyphMetrics(glyphId: Int, font: Any?, paint: Paint): FloatArray

  fun getTextGlyphs(text: String, paint: Paint): Array<GlyphAdvance>

  fun releaseTextGlyph(glyphId: Int)

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
