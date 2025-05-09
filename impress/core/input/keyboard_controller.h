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

#ifndef THIRD_PARTY_IMPRESS_CORE_INPUT_KETBOARD_CONTROLLER_H_
#define THIRD_PARTY_IMPRESS_CORE_INPUT_KETBOARD_CONTROLLER_H_

#include <memory>

#include "core/common/context.h"
#include "core/config.h"
#include "core/input/platform_keyboard_controller.h"
#include "core/view/base_view.h"

namespace imp {

class KeyboardController {
 public:
  static std::unique_ptr<KeyboardController> Create(const Context& context,
                                                    BaseView* view);
  void SetKeyboardShown(bool show_keyboard);

 private:
  explicit KeyboardController(
      std::unique_ptr<PlatformKeyboardController> platform_keyboard_controller);

  std::unique_ptr<PlatformKeyboardController> keyboard_controller_;
  bool is_soft_keyboard_shown_ = false;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_INPUT_KETBOARD_CONTROLLER_H_
