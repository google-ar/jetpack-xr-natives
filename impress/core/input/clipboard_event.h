/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_INPUT_CLIPBOARD_EVENT_H_
#define THIRD_PARTY_IMPRESS_CORE_INPUT_CLIPBOARD_EVENT_H_

#include <cstdint>
#include <string>

#include "absl/strings/string_view.h"

namespace imp {

enum class ClipboardEventType : uint8_t {
  kCopy = 0,
  kPaste = 1,
};

/**
 * A clipboard event is triggered when a copy or paste operation is requested.
 */
class ClipboardEvent {
 public:
  explicit ClipboardEvent(const ClipboardEventType type,
                          absl::string_view content)
      : type_(type), content_(content) {}

  ClipboardEventType type_;
  std::string content_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_INPUT_CLIPBOARD_EVENT_H_
