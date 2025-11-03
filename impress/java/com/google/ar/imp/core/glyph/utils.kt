/*
 * Copyright 2025 Google LLC
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

import android.graphics.Paint
import android.graphics.Rect
import android.graphics.RectF
import kotlin.math.max
import kotlin.math.min

// In some scripts, some characters exceed the boundaries of the font metrics. Expand the font
// metrics to include the actual bounding box in those cases.
internal fun maxFontDescent(boundingBoxF: RectF, fontMetrics: Paint.FontMetrics) =
  max(boundingBoxF.bottom, fontMetrics.descent)

internal fun maxFontAscent(boundingBoxF: RectF, fontMetrics: Paint.FontMetrics) =
  -min(boundingBoxF.top, fontMetrics.ascent)

internal fun maxFontDescent(boundingBox: Rect, fontMetrics: Paint.FontMetrics) =
  max(boundingBox.bottom.toFloat(), fontMetrics.descent)

internal fun maxFontAscent(boundingBox: Rect, fontMetrics: Paint.FontMetrics) =
  -min(boundingBox.top.toFloat(), fontMetrics.ascent)
