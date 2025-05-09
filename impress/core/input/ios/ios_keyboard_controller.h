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

#ifndef THIRD_PARTY_IMPRESS_CORE_INPUT_IOS_IOS_KEYBOARD_CONTROLLER_H_
#define THIRD_PARTY_IMPRESS_CORE_INPUT_IOS_IOS_KEYBOARD_CONTROLLER_H_

#endif

#include <memory>

#include "core/common/context.h"
#include "core/input/platform_keyboard_controller.h"
#include "core/view/base_view.h"

namespace imp {

class iOSKeyboardController : public PlatformKeyboardController {
 public:
  iOSKeyboardController(const Context& context, BaseView* view);
  ~iOSKeyboardController() override;

  void OpenKeyboard() override;
  void CloseKeyboard() override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace imp
