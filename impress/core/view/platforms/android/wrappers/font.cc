// Copyright 2024 Google LLC
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

#include "core/view/platforms/android/wrappers/font.h"

#include <jni.h>

#include <utility>

#include "core/common/jni_helpers.h"
#include "core/view/platforms/android/wrappers/file.h"
#include "core/view/platforms/android/wrappers/font_style.h"

namespace imp::android {

Font::Font(JNIEnv* env, jobject j_font) : JavaWrapper(env, j_font) {
  get_file_ = GetMethodHandle("getFile", "()Ljava/io/File;");
  get_style_ =
      GetMethodHandle("getStyle", "()Landroid/graphics/fonts/FontStyle;");
}

File Font::GetFile() {
  JniUniquePtr<jobject> file = CallObjectMethod(get_file_);
  return File(Env(), std::move(file));
}

FontStyle Font::GetStyle() {
  JniUniquePtr<jobject> style = CallObjectMethod(get_style_);
  return FontStyle(Env(), style.get());
}

}  // namespace imp::android
