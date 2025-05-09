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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_FONT_STYLE_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_FONT_STYLE_H_

#include <jni.h>

#include "core/common/jni_helpers.h"

namespace imp::android {

// JNI wrapper for the Android FontStyle class.
class FontStyle : public JavaWrapper {
 public:
  /**
   * Corresponds with FONT_SLANT values in android.graphics.fonts.FontStyle.
   *
   * https://developer.android.com/reference/android/graphics/fonts/FontStyle
   */
  enum class Slant : int {
    kUpright = 0,
    kItalic = 1,
  };

  /**
   * Corresponds with FONT_WEIGHT values in android.graphics.fonts.FontStyle.
   *
   * https://developer.android.com/reference/android/graphics/fonts/FontStyle
   */
  enum class Weight : int {
    kThin = 100,
    kExtraLight = 200,
    kLight = 300,
    kNormal = 400,
    kMedium = 500,
    kSemiBold = 600,
    kBold = 700,
    kExtraBold = 800,
    kBlack = 900,
  };

  // Constructs a Font by wrapping an existing FontStyle jobject.
  FontStyle(JNIEnv* env, jobject j_font_style);

  Slant GetSlant();
  int GetWeight();

 private:
  JniHandle get_slant_;
  JniHandle get_weight_;
};

}  // namespace imp::android

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_FONT_STYLE_H_
