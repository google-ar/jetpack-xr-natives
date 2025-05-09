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

#ifndef THIRD_PARTY_IMPRESS_CORE_INPUT_TEXTINPUT_EVENT_H_
#define THIRD_PARTY_IMPRESS_CORE_INPUT_TEXTINPUT_EVENT_H_

#endif  // THIRD_PARTY_IMPRESS_CORE_INPUT_TEXTINPUT_EVENT_H_

#include <string>

#include "absl/strings/string_view.h"

namespace imp {

// Represents a text input event user interaction.
// The difference between a "TextInputEvent" and a "KeyboardEvent" is that
// "TextInputEvent" is for you to type in texts, like "Hellow World!" or "3.14"
// "KeyboardEvent" is for trigger certain functionalities, like "TAB", "Page Up"
// and "'S' for open search bar".
struct TextInputEvent {
  explicit TextInputEvent(absl::string_view contents)
      : text(std::string(contents)) {}
  std::string text;
};

}  // namespace imp
