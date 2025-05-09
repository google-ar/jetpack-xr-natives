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

#include "core/view/platforms/android/wrappers/font_style.h"

#include <jni.h>

#include "core/common/jni_helpers.h"

namespace imp::android {

FontStyle::FontStyle(JNIEnv* env, jobject j_font_style)
    : JavaWrapper(env, j_font_style) {
  get_slant_ = GetMethodHandle("getSlant", "()I");
  get_weight_ = GetMethodHandle("getWeight", "()I");
}

FontStyle::Slant FontStyle::GetSlant() {
  return static_cast<FontStyle::Slant>(CallIntMethod(get_slant_));
}

int FontStyle::GetWeight() { return CallIntMethod(get_weight_); }

}  // namespace imp::android
