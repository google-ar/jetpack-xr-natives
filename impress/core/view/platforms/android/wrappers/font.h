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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_FONT_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_FONT_H_

#include <jni.h>

#include "core/common/jni_helpers.h"
#include "core/view/platforms/android/wrappers/file.h"
#include "core/view/platforms/android/wrappers/font_style.h"

namespace imp::android {

// JNI wrapper for the Android Font class.
class Font : public JavaWrapper {
 public:
  // Constructs a Font by wrapping an existing Font jobject.
  Font(JNIEnv* env, jobject j_font);

  File GetFile();
  FontStyle GetStyle();

 private:
  JniHandle get_file_;
  JniHandle get_style_;
};

}  // namespace imp::android

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_FONT_H_
