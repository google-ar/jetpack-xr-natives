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

#include "core/input/android/android_keyboard_controller.h"

namespace imp {

AndroidKeyboardController::AndroidKeyboardController(const Context& context,
                                                     BaseView* view_handle)
    : JavaWrapper(context.GetJniEnv(),
                  "com/google/ar/imp/core/input/ImpKeyboardController",
                  "(Landroid/content/Context;J)V", context.GetActivityContext(),
                  ToJava<BaseView>(view_handle)) {}

void AndroidKeyboardController::OpenKeyboard() {
  open_keyboard_ = GetMethodHandle("openKeyboard", "()V");
  CallVoidMethod(open_keyboard_);
}

void AndroidKeyboardController::CloseKeyboard() {
  close_keyboard_ = GetMethodHandle("closeKeyboard", "()V");
  CallVoidMethod(close_keyboard_);
}

}  // namespace imp
