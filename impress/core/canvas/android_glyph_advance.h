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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_ANDROID_GLYPH_ADVANCE_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_ANDROID_GLYPH_ADVANCE_H_

#include <jni.h>

#include "core/common/jni_helpers.h"

namespace imp {

/**
 * Wrapper around com.google.ar.imp.core.glyph.GlyphAdvance for Android.
 */
class AndroidGlyphAdvance : public JavaWrapper {
 public:
  AndroidGlyphAdvance(JNIEnv* env, jobject java_glyph_advance);

  int GetId();
  float GetWidth();
  jobject GetFont();
  bool IsEmoji();

 private:
  JniHandle get_id_;
  JniHandle get_width_;
  JniHandle get_font_;
  JniHandle is_emoji_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_ANDROID_GLYPH_ADVANCE_H_
