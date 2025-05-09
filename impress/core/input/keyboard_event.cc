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

#include "core/input/keyboard_event.h"

namespace imp {
KeyboardEvent::KeyboardEvent()
    : type(KeyboardEventType::kNone),
      key(Key()),
      elapsed_time(absl::ZeroDuration()) {}

KeyboardEvent::KeyboardEvent(const KeyboardEventType& type, const Key& key,
                             absl::Duration elapsed_time)
    : type(type), key(key), elapsed_time(elapsed_time) {}

}  // namespace imp
