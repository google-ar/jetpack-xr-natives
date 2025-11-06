// Copyright 2025 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/canvas/android_glyph_advance.h"

#include <jni.h>

#include <cassert>

#include "core/common/jni_helpers.h"

namespace imp {

AndroidGlyphAdvance::AndroidGlyphAdvance(JNIEnv* env,
                                         jobject java_glyph_advance)
    : JavaWrapper(env, WrapJni(env, java_glyph_advance),
                  "com/google/ar/imp/core/glyph/GlyphAdvance") {
  get_id_ = GetMethodHandle("getId", "()I");
  get_width_ = GetMethodHandle("getWidth", "()F");
  get_font_ = GetMethodHandle("getFont", "()Ljava/lang/Object;");
  is_emoji_ = GetMethodHandle("isEmoji", "()Z");
}

int AndroidGlyphAdvance::GetId() { return CallIntMethod(get_id_); }

float AndroidGlyphAdvance::GetWidth() { return CallFloatMethod(get_width_); }

jobject AndroidGlyphAdvance::GetFont() { return CallObjectMethod(get_font_); }

bool AndroidGlyphAdvance::IsEmoji() { return CallBooleanMethod(is_emoji_); }

}  // namespace imp
