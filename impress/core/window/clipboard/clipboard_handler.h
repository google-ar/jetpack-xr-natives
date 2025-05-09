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

#ifndef THIRD_PARTY_IMPRESS_CORE_WINDOW_CLIPBOARD_CLIPBOARD_HANDLER_H_
#define THIRD_PARTY_IMPRESS_CORE_WINDOW_CLIPBOARD_CLIPBOARD_HANDLER_H_

#include "absl/strings/string_view.h"

// ClipboardHandler serves as an interface to platform specific handlers.
//
// Handlers should be able to communicate with OS clipboard and be able to
// retrieve/send data from/to OS clipboard.
class ClipboardHandler {
 public:
  virtual ~ClipboardHandler() = default;

  // Set the OS clipboard text to be `text`.
  virtual void SetClipboardText(absl::string_view text) = 0;
  // Get the OS clipboard text.
  virtual absl::string_view GetClipboardText() = 0;
};

#endif  // THIRD_PARTY_IMPRESS_CORE_WINDOW_CLIPBOARD_CLIPBOARD_HANDLER_H_
