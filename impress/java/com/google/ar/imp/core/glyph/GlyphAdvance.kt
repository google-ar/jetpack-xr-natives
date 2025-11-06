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

import com.google.android.filament.proguard.UsedByNative

/** Corresponds to ScopedCanvas::GlyphAdvance */
@UsedByNative("android_glyph_source.cc")
data class GlyphAdvance(
  @get:UsedByNative("android_glyph_source.cc") val id: Int,
  @get:UsedByNative("android_glyph_source.cc") val width: Float,
  @get:UsedByNative("android_glyph_source.cc") val font: Any?,
  @get:UsedByNative("android_glyph_source.cc") val isEmoji: Boolean,
)
