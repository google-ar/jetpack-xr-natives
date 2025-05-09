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

#include "core/view/platforms/android/wrappers/rect.h"

namespace imp::android {

Rect::Rect(JNIEnv* env) : JavaWrapper(env, "android/graphics/Rect", "()V") {
  bottom_ = GetFieldHandle("bottom", "I");
  left_ = GetFieldHandle("left", "I");
  right_ = GetFieldHandle("right", "I");
  top_ = GetFieldHandle("top", "I");

  width_ = GetMethodHandle("width", "()I");
  height_ = GetMethodHandle("height", "()I");
}

int Rect::GetBottom() { return Env()->GetIntField(Self(), ToFieldID(bottom_)); }
int Rect::GetLeft() { return Env()->GetIntField(Self(), ToFieldID(left_)); }
int Rect::GetRight() { return Env()->GetIntField(Self(), ToFieldID(right_)); }
int Rect::GetTop() { return Env()->GetIntField(Self(), ToFieldID(top_)); }

int Rect::GetWidth() { return CallIntMethod(width_); }
int Rect::GetHeight() { return CallIntMethod(height_); }

}  // namespace imp::android
