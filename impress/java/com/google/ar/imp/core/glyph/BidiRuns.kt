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

import java.text.Bidi

/**
 * Simple wrapper around [Bidi] to properly reorder text into left-to-right order.
 *
 * The behavior of this class *must exactly match* the reordering behavior of
 * [android.graphics.Paint.getTextWidths].
 */
internal class BidiRuns(
  private val bidi: Bidi,
  private val levels: ByteArray,
  private val runs: Array<Int>,
) : Iterable<Int> {
  override operator fun iterator() = runs.iterator()

  fun isRtl(run: Int) = levels[run] % 2 == 1

  fun getRunStart(run: Int) = bidi.getRunStart(run)

  fun getRunLimit(run: Int) = bidi.getRunLimit(run)

  companion object {
    fun create(text: String): BidiRuns {
      val bidi = Bidi(text, Bidi.DIRECTION_DEFAULT_LEFT_TO_RIGHT)

      val runCount = bidi.runCount
      val levels = ByteArray(runCount) { bidi.getRunLevel(it).toByte() }
      val runs = Array<Int>(runCount) { it }
      Bidi.reorderVisually(levels, /* levelStart= */ 0, runs, /* runStart= */ 0, runCount)

      return BidiRuns(bidi, levels, runs)
    }
  }
}
