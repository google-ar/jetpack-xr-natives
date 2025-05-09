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

#include "core/input/keyboard_controller.h"

#include <memory>
#include <utility>

#include "core/input/platform_keyboard_controller.h"
#if IMP_PLATFORM(ANDROID)
#include "core/input/android/android_keyboard_controller.h"
#elif IMP_PLATFORM(IOS)
#include "core/input/ios/ios_keyboard_controller.h"
#endif

namespace imp {

std::unique_ptr<KeyboardController> KeyboardController::Create(
    const Context &context, BaseView *view) {
#if IMP_PLATFORM(ANDROID)
  if (view->GetContext().GetActivityContext()) {
        return {};
  } else {
    return {};
  }
#elif IMP_PLATFORM(IOS)
  if (view->GetContext().GetOwningUIView()) {
    auto keyboard_controller =
        std::make_unique<iOSKeyboardController>(view->GetContext(), view);
    return absl::WrapUnique(
        new KeyboardController(std::move(keyboard_controller)));
  } else {
    return {};
  }
#else
  return {};
#endif
}

KeyboardController::KeyboardController(
    std::unique_ptr<PlatformKeyboardController> platform_keyboard_controller)
    : keyboard_controller_(std::move(platform_keyboard_controller)) {}

void KeyboardController::SetKeyboardShown(bool show_keyboard) {
  if (show_keyboard && !is_soft_keyboard_shown_) {
    keyboard_controller_->OpenKeyboard();
    is_soft_keyboard_shown_ = true;
  } else if (!show_keyboard && is_soft_keyboard_shown_) {
    keyboard_controller_->CloseKeyboard();
    is_soft_keyboard_shown_ = false;
  }
}

}  //  namespace imp
