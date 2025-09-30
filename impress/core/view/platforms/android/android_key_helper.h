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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_ANDROID_KEY_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_ANDROID_KEY_HELPER_H_

#include "core/input/key_codes.h"
#include "core/input/keyboard_event.h"
#include "core/view/base_view.h"

namespace imp {

VirtualKeyCode getVkCode(int key_code);

KeyboardEventType getAction(int action);

void ProcessKeyboardEvent(BaseView* view, int char_code, int key_code,
                          int action, int modifiers);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_ANDROID_KEY_HELPER_H_
